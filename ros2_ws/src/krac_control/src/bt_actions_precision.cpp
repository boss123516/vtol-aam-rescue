#include "krac_control/bt/bt_actions_precision.hpp"
#include "krac_control/bt/bt_conditions.hpp"
#include <algorithm>
#include <cmath>

namespace krac_control::bt
{
namespace
{
constexpr int MAV_LANDED_STATE_ON_GROUND = 1;
constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double wrapPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
GPSPoint gpsFromLocalOffset(const GPSPoint& origin, double east_m, double north_m, double alt_m)
{
  if (!origin.valid) return GPSPoint{};
  const double lat_rad = origin.lat * DEG2RAD;
  const double lat = origin.lat + (north_m / EARTH_RADIUS_M) * RAD2DEG;
  const double lon = origin.lon + (east_m / (EARTH_RADIUS_M * std::max(0.1, std::cos(lat_rad)))) * RAD2DEG;
  return GPSPoint{lat, lon, alt_m, true};
}
void gpsErrorToEnu(const GPSPoint& current, const GPSPoint& target, double& east_m, double& north_m)
{
  const double lat_ref = current.lat * DEG2RAD;
  north_m = (target.lat - current.lat) * DEG2RAD * EARTH_RADIUS_M;
  east_m = (target.lon - current.lon) * DEG2RAD * EARTH_RADIUS_M * std::cos(lat_ref);
}
}

IsWaypointReached::IsWaypointReached(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsWaypointReached::providedPorts() { return {BT::InputPort<int>("seq", 0, "")}; }
BT::NodeStatus IsWaypointReached::tick()
{
  int seq; getInput("seq", seq);
  // >= not ==: waypoints placed close together can all be reached within a
  // single tick period, so the mission may have already moved past `seq` by
  // the time this is checked. lastReachedWaypointSeq() tracks the highest
  // seq ever seen, so ">=" still correctly reports "yes, we passed it".
  return ctx_->lastReachedWaypointSeq() >= seq ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

WaitForWaypointReached::WaitForWaypointReached(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList WaitForWaypointReached::providedPorts()
{
  return {
    BT::InputPort<int>("seq", 0, ""),
    BT::InputPort<double>("timeout_sec", 300.0, ""),
    BT::InputPort<std::string>("match", "at_or_after", "")
  };
}
BT::NodeStatus WaitForWaypointReached::onStart()
{
  getInput("seq", seq_);
  getInput("timeout_sec", timeout_sec_);
  getInput("match", match_);
  start_time_ = ctx_->node()->now();
  start_event_count_ = ctx_->waypointReachedEventCount();
  // See IsWaypointReached::tick() for why this is ">=" rather than "==".
  if (match_ == "exact") {
    if (ctx_->latestReachedWaypointSeq() == seq_) return BT::NodeStatus::SUCCESS;
  } else if (match_ == "after_start") {
    return BT::NodeStatus::RUNNING;
  } else if (ctx_->lastReachedWaypointSeq() >= seq_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus WaitForWaypointReached::onRunning()
{
  if (match_ == "exact") {
    if (ctx_->latestReachedWaypointSeq() == seq_) return BT::NodeStatus::SUCCESS;
  } else if (match_ == "after_start") {
    if (ctx_->waypointReachedEventCount() > start_event_count_ &&
        ctx_->latestReachedWaypointSeq() == seq_) {
      return BT::NodeStatus::SUCCESS;
    }
  } else if (ctx_->lastReachedWaypointSeq() >= seq_) {
    return BT::NodeStatus::SUCCESS;
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::RUNNING;
}
void WaitForWaypointReached::onHalted() {}

SetMissionCurrentWaypoint::SetMissionCurrentWaypoint(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList SetMissionCurrentWaypoint::providedPorts()
{
  return {BT::InputPort<int>("seq", 0, ""), BT::InputPort<double>("timeout_sec", 5.0, "")};
}
BT::NodeStatus SetMissionCurrentWaypoint::onStart()
{
  getInput("seq", seq_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  request_sent_ = false;
  auto client = ctx_->missionSetCurrentClient();
  if (!client->wait_for_service(std::chrono::milliseconds(200))) return BT::NodeStatus::RUNNING;
  auto req = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
  req->wp_seq = seq_;
  future_ = client->async_send_request(req);
  request_sent_ = true;
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus SetMissionCurrentWaypoint::onRunning()
{
  if (!request_sent_) {
    auto client = ctx_->missionSetCurrentClient();
    if (client->wait_for_service(std::chrono::milliseconds(50))) {
      auto req = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
      req->wp_seq = seq_;
      future_ = client->async_send_request(req);
      request_sent_ = true;
    }
  } else if (future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    return future_.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::RUNNING;
}
void SetMissionCurrentWaypoint::onHalted() {}

EnablePrecisionLander::EnablePrecisionLander(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList EnablePrecisionLander::providedPorts() { return {BT::InputPort<bool>("enable", true, "")}; }
BT::NodeStatus EnablePrecisionLander::tick()
{
  bool enable; getInput("enable", enable);
  ctx_->setPrecisionLanderEnabled(enable);
  return BT::NodeStatus::SUCCESS;
}

IsPrecisionTargetDetected::IsPrecisionTargetDetected(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), ctx_(globalContext()) {}
BT::PortsList IsPrecisionTargetDetected::providedPorts() { return {BT::InputPort<double>("max_age_sec", 0.5, "")}; }
BT::NodeStatus IsPrecisionTargetDetected::tick()
{
  double age; getInput("max_age_sec", age);
  return ctx_->targetErrorFresh(age) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

WaitForPrecisionTarget::WaitForPrecisionTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList WaitForPrecisionTarget::providedPorts()
{
  return {
    BT::InputPort<double>("max_age_sec", 2.0, ""),
    BT::InputPort<double>("timeout_sec", 30.0, "")
  };
}
BT::NodeStatus WaitForPrecisionTarget::onStart()
{
  getInput("max_age_sec", max_age_sec_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  return onRunning();
}
BT::NodeStatus WaitForPrecisionTarget::onRunning()
{
  if (ctx_->targetErrorFresh(max_age_sec_)) {
    return BT::NodeStatus::SUCCESS;
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "WaitForPrecisionTarget timed out after %.1fs without fresh target", timeout_sec_);
    return BT::NodeStatus::FAILURE;
  }
  ctx_->publishZeroVelocity();
  return BT::NodeStatus::RUNNING;
}
void WaitForPrecisionTarget::onHalted() { ctx_->publishZeroVelocity(); }

PrecisionLandOnTarget::PrecisionLandOnTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList PrecisionLandOnTarget::providedPorts()
{
  return {
    BT::InputPort<double>("target_altitude_m", 0.3, ""),
    BT::InputPort<double>("timeout_sec", 60.0, ""),
    BT::InputPort<double>("target_fresh_max_age_sec", 3.0, ""),
    BT::InputPort<double>("close_range_lock_alt_m", 2.0, ""),
    BT::InputPort<bool>("require_landed_state", false,
      "When true, keep the lander active until PX4 reports ON_GROUND")
  };
}
BT::NodeStatus PrecisionLandOnTarget::onStart()
{
  getInput("target_altitude_m", target_altitude_m_);
  getInput("timeout_sec", timeout_sec_);
  getInput("target_fresh_max_age_sec", target_fresh_max_age_sec_);
  getInput("close_range_lock_alt_m", close_range_lock_alt_m_);
  getInput("require_landed_state", require_landed_state_);
  start_time_ = ctx_->node()->now();
  ctx_->setPrecisionLanderEnabled(true);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus PrecisionLandOnTarget::onRunning()
{
  if (ctx_->landedState() == MAV_LANDED_STATE_ON_GROUND) {
    ctx_->setPrecisionLanderEnabled(false);
    RCLCPP_INFO(ctx_->node()->get_logger(),
      "PrecisionLandOnTarget: PX4 reports ON_GROUND; landing committed.");
    return BT::NodeStatus::SUCCESS;
  }

  if (!require_landed_state_ && ctx_->relativeAltitude() <= target_altitude_m_) {
    ctx_->setPrecisionLanderEnabled(false);
    return BT::NodeStatus::SUCCESS;
  }

  if (require_landed_state_ && ctx_->relativeAltitude() <= target_altitude_m_) {
    RCLCPP_INFO_THROTTLE(
      ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
      "PrecisionLandOnTarget: low altitude %.2fm; waiting for PX4 ON_GROUND.",
      ctx_->relativeAltitude());
  }
  if (!ctx_->targetErrorFresh(target_fresh_max_age_sec_)) {
    // 실측 확인: 구조 박스에 근접(대략 2m 이하)하면 정렬이 거의 완벽해도
    // YOLO/ArUco가 예외 없이 타겟을 놓친다 (근접 시야각/스케일 한계로 추정).
    // 이미 그 근접 구간까지 들어온 상태라면 즉시 실패시키지 않고 계속
    // RUNNING을 유지한다 - precision_lander 노드가 마지막 정렬을 신뢰하고
    // 눈 감고 하강을 이어가서(blind descent), 위의 고도 기준 SUCCESS
    // 조건(비전과 무관)에 도달하게 한다. 아직 근접 구간 밖에서 놓쳤다면
    // (정렬 자체가 안 됐거나 실제 이탈) 기존처럼 즉시 실패 -> BT가 REP로
    // 복귀시켜 재시도하게 둔다.
    if (ctx_->relativeAltitude() <= close_range_lock_alt_m_) {
      RCLCPP_WARN_THROTTLE(
        ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
        "Precision landing target stale near ground (alt<=%.1fm); trusting last alignment, continuing.",
        close_range_lock_alt_m_);
      return BT::NodeStatus::RUNNING;
    }
    ctx_->setPrecisionLanderEnabled(false);
    RCLCPP_WARN_THROTTLE(
      ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
      "Precision landing target lost or stale; aborting precision descent for local recovery.");
    return BT::NodeStatus::FAILURE;
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    ctx_->setPrecisionLanderEnabled(false);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void PrecisionLandOnTarget::onHalted() { ctx_->setPrecisionLanderEnabled(false); }

FlyToLocalPoint::FlyToLocalPoint(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList FlyToLocalPoint::providedPorts()
{
  return {
    BT::InputPort<double>("x_m", 0.0, ""),
    BT::InputPort<double>("y_m", 0.0, ""),
    BT::InputPort<double>("z_m", 10.0, ""),
    BT::InputPort<double>("xy_tolerance_m", 2.0, ""),
    BT::InputPort<double>("z_tolerance_m", 2.0, ""),
    BT::InputPort<double>("max_xy_speed_mps", 4.0, ""),
    BT::InputPort<double>("max_z_speed_mps", 1.0, ""),
    BT::InputPort<double>("timeout_sec", 45.0, "")
  };
}
BT::NodeStatus FlyToLocalPoint::onStart()
{
  getInput("x_m", x_m_);
  getInput("y_m", y_m_);
  getInput("z_m", z_m_);
  getInput("xy_tolerance_m", xy_tolerance_m_);
  getInput("z_tolerance_m", z_tolerance_m_);
  getInput("max_xy_speed_mps", max_xy_speed_mps_);
  getInput("max_z_speed_mps", max_z_speed_mps_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  target_gps_ = GPSPoint{};
  RCLCPP_INFO(
    ctx_->node()->get_logger(),
    "FlyToLocalPoint target local ENU x=%.1f y=%.1f z=%.1f, tolerance xy=%.1f z=%.1f",
    x_m_, y_m_, z_m_, xy_tolerance_m_, z_tolerance_m_);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus FlyToLocalPoint::onRunning()
{
  double current_x = 0.0;
  double current_y = 0.0;
  double current_z = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  const auto home = ctx_->homePoint();
  const auto current_gps = ctx_->currentGps();
  if (current_gps.valid && home.valid) {
    if (!target_gps_.valid) {
      target_gps_ = gpsFromLocalOffset(home, x_m_, y_m_, z_m_);
      RCLCPP_INFO(
        ctx_->node()->get_logger(),
        "FlyToLocalPoint set OFFBOARD global target for REP: lat=%.8f lon=%.8f alt=%.1f",
        target_gps_.lat, target_gps_.lon, target_gps_.alt);
    }
    ctx_->setHoldGps(target_gps_);
    gpsErrorToEnu(current_gps, target_gps_, dx, dy);
    current_x = x_m_ - dx;
    current_y = y_m_ - dy;
    current_z = current_gps.alt;
    dz = z_m_ - current_gps.alt;
  } else if (ctx_->hasLocalPose()) {
    const auto pose = ctx_->localPose();
    const auto& pos = pose.pose.position;
    current_x = pos.x;
    current_y = pos.y;
    current_z = pos.z;
    dx = x_m_ - pos.x;
    dy = y_m_ - pos.y;
    dz = z_m_ - pos.z;
  } else {
    if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
    RCLCPP_WARN_THROTTLE(
      ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
      "FlyToLocalPoint waiting for local pose or GPS input.");
    return BT::NodeStatus::RUNNING;
  }

  const double xy_dist = std::hypot(dx, dy);

  if (xy_dist <= xy_tolerance_m_ && std::abs(dz) <= z_tolerance_m_) {
    ctx_->publishZeroVelocity();
    RCLCPP_INFO(
      ctx_->node()->get_logger(),
      "FlyToLocalPoint reached target: current x=%.1f y=%.1f z=%.1f, xy_dist=%.2f dz=%.2f",
      current_x, current_y, current_z, xy_dist, dz);
    return BT::NodeStatus::SUCCESS;
  }

  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    ctx_->publishZeroVelocity();
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "FlyToLocalPoint timed out: current x=%.1f y=%.1f z=%.1f, target x=%.1f y=%.1f z=%.1f, xy_dist=%.2f dz=%.2f",
      current_x, current_y, current_z, x_m_, y_m_, z_m_, xy_dist, dz);
    return BT::NodeStatus::FAILURE;
  }

  if (target_gps_.valid) {
    RCLCPP_INFO_THROTTLE(
      ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
      "FlyToLocalPoint holding OFFBOARD global REP setpoint: current x=%.1f y=%.1f z=%.1f, xy_dist=%.1f dz=%.1f",
      current_x, current_y, current_z, xy_dist, dz);
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::Twist cmd;
  if (xy_dist > 1e-3) {
    const double speed_xy = std::min(max_xy_speed_mps_, std::max(0.4, xy_dist * 0.6));
    cmd.linear.x = dx / xy_dist * speed_xy;
    cmd.linear.y = dy / xy_dist * speed_xy;
  }
  cmd.linear.z = clamp(dz * 0.5, -max_z_speed_mps_, max_z_speed_mps_);
  ctx_->publishVelocity(cmd);

  RCLCPP_INFO_THROTTLE(
    ctx_->node()->get_logger(), *ctx_->node()->get_clock(), 1000,
    "FlyToLocalPoint moving: current x=%.1f y=%.1f z=%.1f, xy_dist=%.1f dz=%.1f, cmd=(%.2f, %.2f, %.2f)",
    current_x, current_y, current_z, xy_dist, dz, cmd.linear.x, cmd.linear.y, cmd.linear.z);
  return BT::NodeStatus::RUNNING;
}
void FlyToLocalPoint::onHalted() { ctx_->publishZeroVelocity(); }

AlignHeadingToWaypoint::AlignHeadingToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList AlignHeadingToWaypoint::providedPorts()
{
  return {BT::InputPort<int>("seq", 0, ""), BT::InputPort<double>("tolerance_rad", 0.1, ""), BT::InputPort<double>("timeout_sec", 8.0, "")};
}
BT::NodeStatus AlignHeadingToWaypoint::onStart()
{
  getInput("seq", seq_);
  getInput("tolerance_rad", tolerance_rad_);
  getInput("timeout_sec", timeout_sec_);
  target_yaw_rad_ = ctx_->computeBearingToWaypoint(seq_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AlignHeadingToWaypoint::onRunning()
{
  const double yaw_err = wrapPi(target_yaw_rad_ - ctx_->currentYaw());
  if (std::abs(yaw_err) < tolerance_rad_) {
    ctx_->publishZeroVelocity();
    return BT::NodeStatus::SUCCESS;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = clamp(yaw_err * 0.8, -0.5, 0.5);
  ctx_->publishVelocity(cmd);
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    ctx_->publishZeroVelocity();
    RCLCPP_WARN(ctx_->node()->get_logger(),
      "AlignHeadingToWaypoint timed out; refusing FW transition.");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void AlignHeadingToWaypoint::onHalted() { ctx_->publishZeroVelocity(); }

SetMissionPhase::SetMissionPhase(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList SetMissionPhase::providedPorts() { return {BT::InputPort<int>("phase", 0, "")}; }
BT::NodeStatus SetMissionPhase::tick()
{
  int phase; getInput("phase", phase);
  ctx_->publishMissionPhase(static_cast<uint8_t>(phase));
  return BT::NodeStatus::SUCCESS;
}

OpenGripper::OpenGripper(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList OpenGripper::providedPorts()
{
  return {
    BT::InputPort<std::string>("service", "/gripper/open", ""),
    BT::InputPort<std::string>("left_topic", "/model/standard_vtol_0/servo_5", ""),
    BT::InputPort<std::string>("right_topic", "/model/standard_vtol_0/servo_6", ""),
    BT::InputPort<std::string>("detach_topic", "/survivor_tray_rep/detach", ""),
    BT::InputPort<double>("left_position", -1.2, ""),
    BT::InputPort<double>("right_position", -1.2, ""),
    BT::InputPort<double>("settle_sec", 1.0, ""),
    BT::InputPort<double>("publish_rate_hz", 20.0, ""),
    BT::InputPort<double>("timeout_sec", 5.0, "")
  };
}
BT::NodeStatus OpenGripper::onStart()
{
  getInput("left_topic", left_topic_);
  getInput("right_topic", right_topic_);
  getInput("detach_topic", detach_topic_);
  getInput("left_position", left_position_);
  getInput("right_position", right_position_);
  getInput("settle_sec", settle_sec_);
  getInput("publish_rate_hz", publish_rate_hz_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  last_publish_time_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  detached_ = false;
  if (ctx_->gripperStubSuccess()) {
    ctx_->setGripperClosed(false);
    RCLCPP_WARN(ctx_->node()->get_logger(), "gripper_stub_success=true; OpenGripper returns SUCCESS without real service.");
    return BT::NodeStatus::SUCCESS;
  }
  left_pub_ = ctx_->node()->create_publisher<std_msgs::msg::Float64>(left_topic_, 10);
  right_pub_ = ctx_->node()->create_publisher<std_msgs::msg::Float64>(right_topic_, 10);
  detach_pub_ = ctx_->node()->create_publisher<std_msgs::msg::Empty>(detach_topic_, 10);
  RCLCPP_INFO(ctx_->node()->get_logger(), "Opening gripper via %s=%.2f, %s=%.2f, detach_topic=%s",
              left_topic_.c_str(), left_position_, right_topic_.c_str(), right_position_, detach_topic_.c_str());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus OpenGripper::onRunning()
{
  const auto now = ctx_->node()->now();
  const double min_period = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.05;
  if (last_publish_time_.nanoseconds() == 0 || (now - last_publish_time_).seconds() >= min_period) {
    std_msgs::msg::Float64 msg;
    msg.data = left_position_;
    left_pub_->publish(msg);
    msg.data = right_position_;
    right_pub_->publish(msg);
    last_publish_time_ = now;
  }
  if ((now - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  if ((now - start_time_).seconds() >= settle_sec_) {
    if (!detached_) {
      std_msgs::msg::Empty msg;
      detach_pub_->publish(msg);
      detached_ = true;
      RCLCPP_INFO(ctx_->node()->get_logger(), "Requested payload detach via %s", detach_topic_.c_str());
    }
    ctx_->setGripperClosed(false);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void OpenGripper::onHalted() {}

}  // namespace krac_control::bt
