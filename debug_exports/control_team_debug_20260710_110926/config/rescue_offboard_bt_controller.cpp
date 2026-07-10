#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

/*
 * BT adapter for cjfgus814123/krac24:
 *   src/krac_control/src/offboard.cpp
 *
 * Preserved rescue-control logic:
 *   - camera/set_target = basket
 *   - /camera/target_error Point(dx, dy, detected)
 *   - smooth search pattern
 *   - P-control visual alignment
 *   - one-metre step descent down to 1.5 m
 *   - manual/automatic proceed
 *   - ascend to safe altitude
 *
 * Removed from the original monolithic node:
 *   - mission waypoint ownership
 *   - end-waypoint landing logic
 *   - AUTO.MISSION resume logic
 *
 * Those responsibilities remain in the Behavior Tree.  This node only owns
 * the rescue-control interval between BT enable=true and result=SUCCESS/FAILURE.
 */

class Krac24OffboardBtAdapter final : public rclcpp::Node
{
public:
  Krac24OffboardBtAdapter()
  : Node("krac24_offboard_bt_adapter")
  {
    declare_parameter<std::string>("enable_topic", "/krac/rescue_module/enable");
    declare_parameter<std::string>("ready_topic", "/krac/rescue_module/ready");
    declare_parameter<std::string>("result_topic", "/krac/rescue_module/result");
    declare_parameter<std::string>("vision_error_topic", "/camera/target_error");
    declare_parameter<std::string>("target_label_topic", "/camera/set_target");
    declare_parameter<std::string>(
      "velocity_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped");
    declare_parameter<std::string>("target_label", "basket");

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("handover_prestream_sec", 1.0);
    declare_parameter<double>("vision_fresh_sec", 0.5);
    declare_parameter<double>("vision_p_gain", 0.005);
    declare_parameter<double>("max_align_speed_mps", 0.5);
    declare_parameter<double>("descend_speed_mps", -0.3);
    declare_parameter<double>("search_speed_mps", 0.2);
    declare_parameter<double>("search_altitude_m", 3.0);
    declare_parameter<double>("minimum_hover_altitude_m", 1.5);
    declare_parameter<double>("hover_step_m", 1.0);
    declare_parameter<double>("center_tolerance_px", 30.0);
    declare_parameter<double>("descent_hold_error_px", 100.0);
    declare_parameter<double>("auto_proceed_after_hover_sec", 3.0);
    declare_parameter<double>("ascend_speed_mps", 0.5);
    declare_parameter<double>("ascend_target_altitude_m", 5.0);
    declare_parameter<double>("ascend_stable_sec", 1.0);
    declare_parameter<double>("max_stable_vertical_speed_mps", 0.35);
    declare_parameter<double>("execution_timeout_sec", 240.0);
    declare_parameter<double>("offboard_retry_sec", 1.0);

    enable_topic_ = get_parameter("enable_topic").as_string();
    ready_topic_ = get_parameter("ready_topic").as_string();
    result_topic_ = get_parameter("result_topic").as_string();
    vision_error_topic_ = get_parameter("vision_error_topic").as_string();
    target_label_topic_ = get_parameter("target_label_topic").as_string();
    velocity_topic_ = get_parameter("velocity_topic").as_string();
    target_label_ = get_parameter("target_label").as_string();

    control_rate_hz_ = std::max(10.0, get_parameter("control_rate_hz").as_double());
    handover_prestream_sec_ =
      std::max(0.5, get_parameter("handover_prestream_sec").as_double());
    vision_fresh_sec_ = std::max(0.1, get_parameter("vision_fresh_sec").as_double());
    vision_p_gain_ = get_parameter("vision_p_gain").as_double();
    max_align_speed_mps_ =
      std::max(0.05, get_parameter("max_align_speed_mps").as_double());
    descend_speed_mps_ = get_parameter("descend_speed_mps").as_double();
    search_speed_mps_ = get_parameter("search_speed_mps").as_double();
    search_altitude_m_ = get_parameter("search_altitude_m").as_double();
    minimum_hover_altitude_m_ =
      get_parameter("minimum_hover_altitude_m").as_double();
    hover_step_m_ = std::max(0.1, get_parameter("hover_step_m").as_double());
    center_tolerance_px_ =
      std::max(1.0, get_parameter("center_tolerance_px").as_double());
    descent_hold_error_px_ =
      std::max(center_tolerance_px_,
               get_parameter("descent_hold_error_px").as_double());
    auto_proceed_after_hover_sec_ =
      get_parameter("auto_proceed_after_hover_sec").as_double();
    ascend_speed_mps_ = get_parameter("ascend_speed_mps").as_double();
    ascend_target_altitude_m_ =
      get_parameter("ascend_target_altitude_m").as_double();
    ascend_stable_sec_ =
      std::max(0.2, get_parameter("ascend_stable_sec").as_double());
    max_stable_vertical_speed_mps_ =
      std::max(0.05, get_parameter("max_stable_vertical_speed_mps").as_double());
    execution_timeout_sec_ =
      std::max(10.0, get_parameter("execution_timeout_sec").as_double());
    offboard_retry_sec_ =
      std::max(0.2, get_parameter("offboard_retry_sec").as_double());

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10,
      std::bind(&Krac24OffboardBtAdapter::enableCallback, this, std::placeholders::_1));

    ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, 10);
    result_pub_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", sensor_qos,
      std::bind(&Krac24OffboardBtAdapter::stateCallback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", sensor_qos,
      std::bind(&Krac24OffboardBtAdapter::poseCallback, this, std::placeholders::_1));
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", sensor_qos,
      std::bind(&Krac24OffboardBtAdapter::velocityCallback, this, std::placeholders::_1));
    vision_sub_ = create_subscription<geometry_msgs::msg::Point>(
      vision_error_topic_, 10,
      std::bind(&Krac24OffboardBtAdapter::visionCallback, this, std::placeholders::_1));

    velocity_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(velocity_topic_, 10);
    target_label_pub_ =
      create_publisher<std_msgs::msg::String>(target_label_topic_, 10);

    set_mode_client_ =
      create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    proceed_service_ = create_service<std_srvs::srv::Trigger>(
      "/cmd/mission_proceed",
      std::bind(
        &Krac24OffboardBtAdapter::manualProceedCallback,
        this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&Krac24OffboardBtAdapter::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "KRAC24 offboard rescue BT adapter ready. "
      "Waiting for %s=true.",
      enable_topic_.c_str());
  }

private:
  enum class Phase
  {
    IDLE,
    PREPARE_OFFBOARD,
    SEARCH_RESCUE,
    VISION_ALIGN_RESCUE,
    ASCEND_WITH_VICTIM,
    RESULT_HOLD
  };

  static const char * phaseName(Phase phase)
  {
    switch (phase) {
      case Phase::IDLE: return "IDLE";
      case Phase::PREPARE_OFFBOARD: return "PREPARE_OFFBOARD";
      case Phase::SEARCH_RESCUE: return "SEARCH_RESCUE";
      case Phase::VISION_ALIGN_RESCUE: return "VISION_ALIGN_RESCUE";
      case Phase::ASCEND_WITH_VICTIM: return "ASCEND_WITH_VICTIM";
      case Phase::RESULT_HOLD: return "RESULT_HOLD";
    }
    return "UNKNOWN";
  }

  double clampVelocity(double velocity) const
  {
    return std::clamp(velocity, -max_align_speed_mps_, max_align_speed_mps_);
  }

  double elapsed(const rclcpp::Time & start) const
  {
    if (start.nanoseconds() == 0) {
      return 0.0;
    }
    return (now() - start).seconds();
  }

  void setPhase(Phase phase)
  {
    phase_ = phase;
    phase_started_ = now();
    centered_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    stable_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "Phase -> %s", phaseName(phase_));
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      if (active_) {
        RCLCPP_INFO(get_logger(), "BT handback received: enable=false.");
      }
      resetIdle();
      return;
    }

    if (active_) {
      return;
    }

    active_ = true;
    ready_sent_ = false;
    result_sent_ = false;
    manual_proceed_ = false;
    search_step_ = 0;
    target_hover_altitude_m_ =
      pose_received_ ? current_pose_.pose.position.z : search_altitude_m_;
    request_started_ = now();
    last_ready_publish_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_result_publish_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_mode_request_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publishTarget(target_label_);
    setPhase(Phase::PREPARE_OFFBOARD);

    RCLCPP_INFO(
      get_logger(),
      "BT rescue enable=true. Starting preserved KRAC24 search/align/descent logic.");
  }

  void resetIdle()
  {
    active_ = false;
    ready_sent_ = false;
    result_sent_ = false;
    manual_proceed_ = false;
    publishTarget("disabled");

    geometry_msgs::msg::Twist stop;
    velocity_pub_->publish(stop);

    setPhase(Phase::IDLE);
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
    state_received_ = true;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
    pose_received_ = true;
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    vertical_speed_mps_ = msg->twist.linear.z;
  }

  void visionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    vision_error_ = *msg;
    if (msg->z > 0.5) {
      last_vision_time_ = now();
    }
  }

  void manualProceedCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    manual_proceed_ = true;
    response->success = true;
    response->message = "Force Proceed Triggered";
    RCLCPP_INFO(get_logger(), "Manual proceed requested.");
  }

  void publishTarget(const std::string & label)
  {
    std_msgs::msg::String msg;
    msg.data = label;
    target_label_pub_->publish(msg);
  }

  void publishReady()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    ready_pub_->publish(msg);
    last_ready_publish_ = now();
    ready_sent_ = true;
  }

  void publishResult(const std::string & result)
  {
    std_msgs::msg::String msg;
    msg.data = result;
    result_pub_->publish(msg);
    last_result_publish_ = now();
    result_sent_ = true;
  }

  bool objectDetected() const
  {
    return
      last_vision_time_.nanoseconds() > 0 &&
      elapsed(last_vision_time_) < vision_fresh_sec_;
  }

  void ensureOffboard()
  {
    if (state_received_ && current_state_.mode == "OFFBOARD") {
      return;
    }

    if (
      last_mode_request_.nanoseconds() != 0 &&
      elapsed(last_mode_request_) < offboard_retry_sec_)
    {
      return;
    }

    if (!set_mode_client_->service_is_ready()) {
      set_mode_client_->wait_for_service(50ms);
    }

    if (set_mode_client_->service_is_ready()) {
      auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      request->base_mode = 0;
      request->custom_mode = "OFFBOARD";
      set_mode_client_->async_send_request(request);
      last_mode_request_ = now();
    }
  }

  void executeSmoothSearchPattern(geometry_msgs::msg::Twist & command)
  {
    if (search_step_ == 0) {
      search_started_ = now();
      search_step_ = 1;
    }

    const double seconds = elapsed(search_started_);

    if (seconds < 3.0) {
      command.linear.y = -search_speed_mps_;
    } else if (seconds < 4.0) {
      // hold
    } else if (seconds < 7.0) {
      command.linear.x = -search_speed_mps_;
    } else if (seconds < 8.0) {
      // hold
    } else if (seconds < 11.0) {
      command.linear.y = search_speed_mps_;
    } else if (seconds < 12.0) {
      // hold
    } else if (seconds < 15.0) {
      command.linear.x = search_speed_mps_;
    } else {
      search_step_ = 0;
    }
  }

  void executeStepDescent(
    geometry_msgs::msg::Twist & command,
    double & target_altitude)
  {
    const double current_altitude = current_pose_.pose.position.z;
    const double pixel_distance =
      std::hypot(vision_error_.x, vision_error_.y);

    // Preserved signs from krac24/offboard.cpp.
    command.linear.x = clampVelocity(-vision_error_.y * vision_p_gain_);
    command.linear.y = clampVelocity(vision_error_.x * vision_p_gain_);

    if (current_altitude < target_altitude - 0.1) {
      target_altitude = current_altitude;
    }

    if (std::abs(current_altitude - target_altitude) < 0.2) {
      if (pixel_distance < center_tolerance_px_) {
        if (target_altitude > minimum_hover_altitude_m_) {
          target_altitude -= hover_step_m_;
          target_altitude =
            std::max(target_altitude, minimum_hover_altitude_m_);
          RCLCPP_INFO(
            get_logger(),
            "Aligned. Step descent target -> %.2f m.",
            target_altitude);
        }
      }
    } else if (current_altitude > target_altitude) {
      command.linear.z =
        pixel_distance > descent_hold_error_px_ ? 0.0 : descend_speed_mps_;
    }
  }

  void fail(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "Rescue adapter failed: %s", reason.c_str());
    publishResult("FAILURE:" + reason);
    setPhase(Phase::RESULT_HOLD);
  }

  void controlLoop()
  {
    if (!active_) {
      return;
    }

    geometry_msgs::msg::Twist command;

    if (elapsed(request_started_) > execution_timeout_sec_) {
      if (phase_ != Phase::RESULT_HOLD) {
        fail("controller_timeout");
      }
    }

    if (!pose_received_) {
      velocity_pub_->publish(command);
      if (elapsed(request_started_) > 10.0 && phase_ != Phase::RESULT_HOLD) {
        fail("no_local_pose");
      }
      return;
    }

    ensureOffboard();

    switch (phase_) {
      case Phase::IDLE:
        return;

      case Phase::PREPARE_OFFBOARD:
        if (
          elapsed(phase_started_) >= handover_prestream_sec_ &&
          state_received_ &&
          current_state_.mode == "OFFBOARD")
        {
          publishReady();
          setPhase(Phase::SEARCH_RESCUE);
        }
        break;

      case Phase::SEARCH_RESCUE:
        publishTarget(target_label_);
        if (objectDetected()) {
          target_hover_altitude_m_ = current_pose_.pose.position.z;
          RCLCPP_INFO(
            get_logger(),
            "Object found: dx=%.1f px, dy=%.1f px.",
            vision_error_.x, vision_error_.y);
          setPhase(Phase::VISION_ALIGN_RESCUE);
        } else if (current_pose_.pose.position.z < search_altitude_m_) {
          executeSmoothSearchPattern(command);
        } else {
          command.linear.z = descend_speed_mps_;
        }
        break;

      case Phase::VISION_ALIGN_RESCUE:
        if (!objectDetected()) {
          RCLCPP_WARN(get_logger(), "Lost object. Returning to search.");
          search_step_ = 0;
          setPhase(Phase::SEARCH_RESCUE);
          break;
        }

        if (
          current_pose_.pose.position.z < minimum_hover_altitude_m_ + 0.1 &&
          target_hover_altitude_m_ <= minimum_hover_altitude_m_)
        {
          const double pixel_distance =
            std::hypot(vision_error_.x, vision_error_.y);
          command.linear.x =
            clampVelocity(-vision_error_.y * vision_p_gain_);
          command.linear.y =
            clampVelocity(vision_error_.x * vision_p_gain_);

          if (pixel_distance < center_tolerance_px_) {
            if (centered_started_.nanoseconds() == 0) {
              centered_started_ = now();
            }
          } else {
            centered_started_ =
              rclcpp::Time(0, 0, get_clock()->get_clock_type());
          }

          const bool automatic_proceed =
            auto_proceed_after_hover_sec_ >= 0.0 &&
            centered_started_.nanoseconds() > 0 &&
            elapsed(centered_started_) >= auto_proceed_after_hover_sec_;

          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Hovering over target. error=%.1f px. "
            "manual=%s auto=%s",
            pixel_distance,
            manual_proceed_ ? "true" : "false",
            automatic_proceed ? "true" : "false");

          if (manual_proceed_ || automatic_proceed) {
            manual_proceed_ = false;
            setPhase(Phase::ASCEND_WITH_VICTIM);
          }
        } else {
          executeStepDescent(command, target_hover_altitude_m_);
        }
        break;

      case Phase::ASCEND_WITH_VICTIM:
        command.linear.z = ascend_speed_mps_;

        if (current_pose_.pose.position.z >= ascend_target_altitude_m_) {
          command.linear.z = 0.0;

          if (std::abs(vertical_speed_mps_) <= max_stable_vertical_speed_mps_) {
            if (stable_started_.nanoseconds() == 0) {
              stable_started_ = now();
            }
          } else {
            stable_started_ =
              rclcpp::Time(0, 0, get_clock()->get_clock_type());
          }

          if (
            stable_started_.nanoseconds() > 0 &&
            elapsed(stable_started_) >= ascend_stable_sec_)
          {
            publishResult("SUCCESS");
            setPhase(Phase::RESULT_HOLD);
          }
        }
        break;

      case Phase::RESULT_HOLD:
        if (result_sent_ && elapsed(last_result_publish_) >= 0.5) {
          std_msgs::msg::String result;
          result.data = "SUCCESS";
          result_pub_->publish(result);
          last_result_publish_ = now();
        }
        break;
    }

    velocity_pub_->publish(command);

    if (
      ready_sent_ &&
      phase_ != Phase::RESULT_HOLD &&
      elapsed(last_ready_publish_) >= 0.5)
    {
      std_msgs::msg::Bool ready;
      ready.data = true;
      ready_pub_->publish(ready);
      last_ready_publish_ = now();
    }
  }

  std::string enable_topic_;
  std::string ready_topic_;
  std::string result_topic_;
  std::string vision_error_topic_;
  std::string target_label_topic_;
  std::string velocity_topic_;
  std::string target_label_;

  double control_rate_hz_{20.0};
  double handover_prestream_sec_{1.0};
  double vision_fresh_sec_{0.5};
  double vision_p_gain_{0.005};
  double max_align_speed_mps_{0.5};
  double descend_speed_mps_{-0.3};
  double search_speed_mps_{0.2};
  double search_altitude_m_{3.0};
  double minimum_hover_altitude_m_{1.5};
  double hover_step_m_{1.0};
  double center_tolerance_px_{30.0};
  double descent_hold_error_px_{100.0};
  double auto_proceed_after_hover_sec_{3.0};
  double ascend_speed_mps_{0.5};
  double ascend_target_altitude_m_{5.0};
  double ascend_stable_sec_{1.0};
  double max_stable_vertical_speed_mps_{0.35};
  double execution_timeout_sec_{240.0};
  double offboard_retry_sec_{1.0};

  Phase phase_{Phase::IDLE};
  bool active_{false};
  bool ready_sent_{false};
  bool result_sent_{false};
  bool manual_proceed_{false};
  bool state_received_{false};
  bool pose_received_{false};

  int search_step_{0};
  double target_hover_altitude_m_{3.0};
  double vertical_speed_mps_{0.0};

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Point vision_error_;

  rclcpp::Time request_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time search_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time centered_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stable_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_vision_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ready_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_result_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mode_request_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_label_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr proceed_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Krac24OffboardBtAdapter>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "krac24_offboard_bt_adapter exception: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
