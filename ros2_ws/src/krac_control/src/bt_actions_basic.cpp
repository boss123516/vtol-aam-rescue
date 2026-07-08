#include "krac_control/bt/bt_actions_basic.hpp"
#include "krac_control/bt/bt_conditions.hpp"

#include <chrono>
#include <thread>

namespace krac_control::bt
{
namespace
{
constexpr int MAV_CMD_DO_VTOL_TRANSITION = 3000;
constexpr int MAV_VTOL_STATE_MC = 3;
constexpr int MAV_VTOL_STATE_FW = 4;
}

StartOffboardSetpointStream::StartOffboardSetpointStream(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList StartOffboardSetpointStream::providedPorts()
{
  return {BT::InputPort<std::string>("mode", "hold_current_pose", ""), BT::InputPort<double>("rate_hz", 20.0, "")};
}
BT::NodeStatus StartOffboardSetpointStream::tick()
{
  std::string mode; double rate;
  getInput("mode", mode); getInput("rate_hz", rate);
  ctx_->startOffboardSetpointStream(rate, mode);
  return BT::NodeStatus::SUCCESS;
}

SetFlightMode::SetFlightMode(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList SetFlightMode::providedPorts()
{
  return {BT::InputPort<std::string>("mode", "OFFBOARD", ""), BT::InputPort<double>("timeout_sec", 5.0, "")};
}
BT::NodeStatus SetFlightMode::onStart()
{
  getInput("mode", requested_mode_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  request_sent_ = false;

  if (ctx_->mode() == requested_mode_) return BT::NodeStatus::SUCCESS;
  auto client = ctx_->setModeClient();
  if (!client->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(ctx_->node()->get_logger(), "SetMode service unavailable");
    return BT::NodeStatus::RUNNING;
  }
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = requested_mode_;
  client->async_send_request(req);
  request_sent_ = true;
  RCLCPP_INFO(ctx_->node()->get_logger(), "Requested flight mode: %s", requested_mode_.c_str());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus SetFlightMode::onRunning()
{
  if (ctx_->mode() == requested_mode_) return BT::NodeStatus::SUCCESS;
  if (!request_sent_) {
    auto client = ctx_->setModeClient();
    if (client->wait_for_service(std::chrono::milliseconds(50))) {
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = requested_mode_;
      client->async_send_request(req);
      request_sent_ = true;
    }
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "SetFlightMode timed out: requested=%s current=%s",
      requested_mode_.c_str(),
      ctx_->mode().c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void SetFlightMode::onHalted() {}

ArmVehicle::ArmVehicle(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList ArmVehicle::providedPorts()
{
  return {BT::InputPort<bool>("arm", true, ""), BT::InputPort<double>("timeout_sec", 5.0, "")};
}
BT::NodeStatus ArmVehicle::onStart()
{
  getInput("arm", target_arm_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  if (ctx_->armed() == target_arm_) return BT::NodeStatus::SUCCESS;
  auto client = ctx_->armingClient();
  if (!client->wait_for_service(std::chrono::milliseconds(500))) return BT::NodeStatus::RUNNING;
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = target_arm_;
  client->async_send_request(req);
  RCLCPP_INFO(ctx_->node()->get_logger(), "Requested arm=%s", target_arm_ ? "true" : "false");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ArmVehicle::onRunning()
{
  if (ctx_->armed() == target_arm_) return BT::NodeStatus::SUCCESS;
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "ArmVehicle timed out: requested=%s current=%s",
      target_arm_ ? "true" : "false",
      ctx_->armed() ? "true" : "false");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void ArmVehicle::onHalted() {}

CommandVTOLTransition::CommandVTOLTransition(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList CommandVTOLTransition::providedPorts()
{
  return {BT::InputPort<std::string>("target_state", "MC", ""), BT::InputPort<double>("timeout_sec", 20.0, "")};
}
BT::NodeStatus CommandVTOLTransition::onStart()
{
  getInput("target_state", target_state_);
  getInput("timeout_sec", timeout_sec_);
  target_vtol_state_ = (target_state_ == "FW") ? MAV_VTOL_STATE_FW : MAV_VTOL_STATE_MC;
  start_time_ = ctx_->node()->now();
  if (ctx_->vtolState() == target_vtol_state_) return BT::NodeStatus::SUCCESS;
  auto client = ctx_->commandClient();
  if (!client->wait_for_service(std::chrono::milliseconds(500))) return BT::NodeStatus::RUNNING;
  auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  req->command = MAV_CMD_DO_VTOL_TRANSITION;
  req->param1 = static_cast<float>(target_vtol_state_);
  req->confirmation = 0;
  client->async_send_request(req);
  RCLCPP_INFO(ctx_->node()->get_logger(), "Requested VTOL transition to %s", target_state_.c_str());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus CommandVTOLTransition::onRunning()
{
  if (ctx_->vtolState() == target_vtol_state_) return BT::NodeStatus::SUCCESS;
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "CommandVTOLTransition timed out: requested=%s current_state=%d",
      target_state_.c_str(),
      ctx_->vtolState());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void CommandVTOLTransition::onHalted() {}

ClearMissionPlan::ClearMissionPlan(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList ClearMissionPlan::providedPorts() { return {BT::InputPort<bool>("optional", true, "")}; }
BT::NodeStatus ClearMissionPlan::onStart()
{
  getInput("optional", optional_);
  start_time_ = ctx_->node()->now();
  auto client = ctx_->wpClearClient();
  if (!client->wait_for_service(std::chrono::milliseconds(500))) {
    return optional_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  auto req = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
  client->async_send_request(req);
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus ClearMissionPlan::onRunning() { return optional_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }
void ClearMissionPlan::onHalted() {}

UploadMissionPlan::UploadMissionPlan(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList UploadMissionPlan::providedPorts()
{
  return {BT::InputPort<std::string>("plan_path", std::string(""), ""), BT::InputPort<std::string>("mission_name", std::string(""), ""), BT::InputPort<std::string>("expected_end", std::string(""), ""), BT::InputPort<double>("timeout_sec", 30.0, "")};
}
BT::NodeStatus UploadMissionPlan::onStart()
{
  getInput("plan_path", plan_path_);
  getInput("mission_name", mission_name_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  future_ = std::async(std::launch::async, [this]() {
    return ctx_->runExternalMissionLoader(plan_path_, mission_name_, timeout_sec_);
  });
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus UploadMissionPlan::onRunning()
{
  if (future_.valid() && future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    return future_.get() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_ + 2.0) {
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "UploadMissionPlan timed out: plan_path=%s mission_name=%s timeout_sec=%.1f",
      plan_path_.c_str(),
      mission_name_.c_str(),
      timeout_sec_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}
void UploadMissionPlan::onHalted() {}

ActivateYOLO::ActivateYOLO(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList ActivateYOLO::providedPorts()
{
  return {BT::InputPort<std::string>("target", std::string(""), ""), BT::InputPort<std::string>("mode", "start_or_set_target", "")};
}
BT::NodeStatus ActivateYOLO::tick()
{
  std::string target; getInput("target", target);
  ctx_->publishVisionTarget(target);
  return BT::NodeStatus::SUCCESS;
}

DeactivateYOLO::DeactivateYOLO(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList DeactivateYOLO::providedPorts() { return {BT::InputPort<std::string>("target", std::string(""), "")}; }
BT::NodeStatus DeactivateYOLO::tick()
{
  ctx_->publishVisionTarget("none");
  return BT::NodeStatus::SUCCESS;
}

CloseGripper::CloseGripper(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList CloseGripper::providedPorts()
{
  return {
    BT::InputPort<std::string>("service", "/gripper/close", ""),
    BT::InputPort<std::string>("left_topic", "/model/standard_vtol_0/servo_5", ""),
    BT::InputPort<std::string>("right_topic", "/model/standard_vtol_0/servo_6", ""),
    BT::InputPort<std::string>("attach_topic", "/survivor_tray_rep/attach", ""),
    BT::InputPort<double>("left_position", 0.5, ""),
    BT::InputPort<double>("right_position", 0.5, ""),
    BT::InputPort<double>("settle_sec", 1.0, ""),
    BT::InputPort<double>("publish_rate_hz", 20.0, ""),
    BT::InputPort<double>("timeout_sec", 5.0, "")
  };
}
BT::NodeStatus CloseGripper::onStart()
{
  getInput("left_topic", left_topic_);
  getInput("right_topic", right_topic_);
  getInput("attach_topic", attach_topic_);
  getInput("left_position", left_position_);
  getInput("right_position", right_position_);
  getInput("settle_sec", settle_sec_);
  getInput("publish_rate_hz", publish_rate_hz_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  last_publish_time_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  attached_ = false;
  if (ctx_->gripperStubSuccess()) {
    ctx_->setGripperClosed(true);
    RCLCPP_WARN(ctx_->node()->get_logger(), "gripper_stub_success=true; CloseGripper returns SUCCESS without real service.");
    return BT::NodeStatus::SUCCESS;
  }
  left_pub_ = ctx_->node()->create_publisher<std_msgs::msg::Float64>(left_topic_, 10);
  right_pub_ = ctx_->node()->create_publisher<std_msgs::msg::Float64>(right_topic_, 10);
  attach_pub_ = ctx_->node()->create_publisher<std_msgs::msg::Empty>(attach_topic_, 10);
  RCLCPP_INFO(ctx_->node()->get_logger(), "Closing gripper via %s=%.2f, %s=%.2f, attach_topic=%s",
              left_topic_.c_str(), left_position_, right_topic_.c_str(), right_position_, attach_topic_.c_str());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus CloseGripper::onRunning()
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
    if (!attached_) {
      std_msgs::msg::Empty msg;
      attach_pub_->publish(msg);
      attached_ = true;
      RCLCPP_INFO(ctx_->node()->get_logger(), "Requested payload attach via %s", attach_topic_.c_str());
    }
    ctx_->setGripperClosed(true);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void CloseGripper::onHalted() {}

}  // namespace krac_control::bt
