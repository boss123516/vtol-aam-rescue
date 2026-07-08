#include "krac_control/bt/bt_actions_vision.hpp"
#include "krac_control/bt/bt_conditions.hpp"
#include <algorithm>
#include <cmath>

namespace krac_control::bt
{
namespace
{
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}

ExecuteSearchPattern::ExecuteSearchPattern(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList ExecuteSearchPattern::providedPorts()
{
  return {BT::InputPort<std::string>("target", std::string(""), ""), BT::InputPort<std::string>("pattern", "spiral_or_lawnmower", ""), BT::InputPort<double>("max_duration_sec", 45.0, "")};
}
BT::NodeStatus ExecuteSearchPattern::onStart()
{
  getInput("max_duration_sec", max_duration_sec_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ExecuteSearchPattern::onRunning()
{
  if (ctx_->objectDetected(0.5)) { ctx_->publishZeroVelocity(); return BT::NodeStatus::SUCCESS; }
  const double t = (ctx_->node()->now() - start_time_).seconds();
  if (t > max_duration_sec_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::FAILURE; }
  geometry_msgs::msg::Twist cmd;
  const double speed = 0.2;
  const int phase = static_cast<int>(t / 4.0) % 4;
  if (phase == 0) cmd.linear.y = speed;
  else if (phase == 1) cmd.linear.x = speed;
  else if (phase == 2) cmd.linear.y = -speed;
  else cmd.linear.x = -speed;
  ctx_->publishVelocity(cmd);
  return BT::NodeStatus::RUNNING;
}
void ExecuteSearchPattern::onHalted() { ctx_->publishZeroVelocity(); }

AlignToTarget::AlignToTarget(const std::string& name, const BT::NodeConfiguration& config, const std::string& target_name)
: BT::StatefulActionNode(name, config), ctx_(globalContext()), target_name_(target_name) {}
BT::PortsList AlignToTarget::providedPorts()
{
  return {BT::InputPort<double>("pixel_tolerance_px", 25.0, ""), BT::InputPort<double>("yaw_tolerance_rad", 0.1, ""), BT::InputPort<double>("stable_duration_sec", 1.0, ""), BT::InputPort<double>("timeout_sec", 35.0, "")};
}
BT::NodeStatus AlignToTarget::onStart()
{
  getInput("pixel_tolerance_px", pixel_tolerance_px_);
  getInput("stable_duration_sec", stable_duration_sec_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus AlignToTarget::onRunning()
{
  if (!ctx_->objectDetected(0.5)) {
    ctx_->publishZeroVelocity();
    if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
  }
  auto err = ctx_->visionError();
  const double pixel_error = std::sqrt(err.x * err.x + err.y * err.y);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(-err.y * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  cmd.linear.y = clamp(-err.x * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  ctx_->publishVelocity(cmd);

  if (pixel_error <= pixel_tolerance_px_) {
    if (stable_start_.nanoseconds() == 0) stable_start_ = ctx_->node()->now();
    if ((ctx_->node()->now() - stable_start_).seconds() >= stable_duration_sec_) {
      ctx_->publishZeroVelocity();
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_start_ = rclcpp::Time(0, 0, ctx_->node()->get_clock()->get_clock_type());
  }
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::RUNNING;
}
void AlignToTarget::onHalted() { ctx_->publishZeroVelocity(); }

DescendWithAlignment::DescendWithAlignment(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList DescendWithAlignment::providedPorts()
{
  return {BT::InputPort<std::string>("target", "basket", ""), BT::InputPort<double>("descent_step_m", 0.3, ""), BT::InputPort<double>("min_confidence", 0.55, ""), BT::InputPort<double>("max_lost_time_sec", 0.7, ""), BT::InputPort<double>("timeout_sec", 60.0, ""), BT::InputPort<double>("target_altitude_m", 0.8, "")};
}
BT::NodeStatus DescendWithAlignment::onStart()
{
  getInput("target", target_); getInput("min_confidence", min_confidence_);
  getInput("max_lost_time_sec", max_lost_time_sec_); getInput("timeout_sec", timeout_sec_);
  getInput("target_altitude_m", target_altitude_m_);
  start_time_ = ctx_->node()->now();
  last_seen_time_ = start_time_;
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus DescendWithAlignment::onRunning()
{
  const bool seen = ctx_->objectDetected(min_confidence_);
  if (seen) last_seen_time_ = ctx_->node()->now();
  if (!seen && (ctx_->node()->now() - last_seen_time_).seconds() > max_lost_time_sec_) {
    ctx_->publishZeroVelocity();
    return BT::NodeStatus::FAILURE;
  }
  if (ctx_->relativeAltitude() <= target_altitude_m_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::SUCCESS; }
  auto err = ctx_->visionError();
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(-err.y * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  cmd.linear.y = clamp(-err.x * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  cmd.linear.z = -std::abs(ctx_->descendSpeed());
  ctx_->publishVelocity(cmd);
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::RUNNING;
}
void DescendWithAlignment::onHalted() { ctx_->publishZeroVelocity(); }

LandOnZone::LandOnZone(const std::string& name, const BT::NodeConfiguration& config, bool basket)
: BT::StatefulActionNode(name, config), ctx_(globalContext()), basket_(basket) {}
BT::PortsList LandOnZone::providedPorts()
{
  return {BT::InputPort<double>("target_altitude_m", 0.35, ""), BT::InputPort<double>("handoff_altitude_m", 1.0, ""), BT::InputPort<double>("max_horizontal_error_m", 0.25, ""), BT::InputPort<double>("timeout_sec", 30.0, "")};
}
BT::NodeStatus LandOnZone::onStart()
{
  if (basket_) getInput("target_altitude_m", target_altitude_m_);
  else getInput("handoff_altitude_m", target_altitude_m_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus LandOnZone::onRunning()
{
  if (ctx_->relativeAltitude() <= target_altitude_m_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::SUCCESS; }
  geometry_msgs::msg::Twist cmd;
  if (ctx_->objectDetected(0.4)) {
    auto err = ctx_->visionError();
    cmd.linear.x = clamp(-err.y * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
    cmd.linear.y = clamp(-err.x * ctx_->alignPGain(), -ctx_->maxAlignSpeed(), ctx_->maxAlignSpeed());
  }
  cmd.linear.z = -std::abs(ctx_->descendSpeed()) * 0.6;
  ctx_->publishVelocity(cmd);
  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) { ctx_->publishZeroVelocity(); return BT::NodeStatus::FAILURE; }
  return BT::NodeStatus::RUNNING;
}
void LandOnZone::onHalted() { ctx_->publishZeroVelocity(); }

VerifyBasketPicked::VerifyBasketPicked(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config), ctx_(globalContext()) {}
BT::PortsList VerifyBasketPicked::providedPorts()
{
  return {BT::InputPort<std::string>("method", "gripper_feedback_or_visual_invariant", ""), BT::InputPort<double>("lift_test_height_m", 0.5, ""), BT::InputPort<double>("timeout_sec", 10.0, "")};
}
BT::NodeStatus VerifyBasketPicked::onStart()
{
  getInput("lift_test_height_m", lift_test_height_m_);
  getInput("timeout_sec", timeout_sec_);
  start_time_ = ctx_->node()->now();
  if (!ctx_->gripperClosed() && !ctx_->gripperStubSuccess()) {
    RCLCPP_WARN(ctx_->node()->get_logger(), "VerifyBasketPicked started before gripper close was confirmed.");
    return BT::NodeStatus::FAILURE;
  }
  start_altitude_m_ = ctx_->relativeAltitude();
  target_altitude_m_ = start_altitude_m_ + std::max(0.0, lift_test_height_m_);
  ctx_->setHoldCurrentPosition();
  ctx_->setHoldAltitude(target_altitude_m_);
  RCLCPP_INFO(
    ctx_->node()->get_logger(),
    "Verifying basket pickup with lift test: start_alt=%.2f target_alt=%.2f timeout=%.1f",
    start_altitude_m_, target_altitude_m_, timeout_sec_);
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus VerifyBasketPicked::onRunning()
{
  if (!ctx_->gripperClosed() && !ctx_->gripperStubSuccess()) {
    return BT::NodeStatus::FAILURE;
  }

  ctx_->setHoldAltitude(target_altitude_m_);
  if (ctx_->relativeAltitude() >= target_altitude_m_ - 0.15) {
    ctx_->markRescueCompleted();
    RCLCPP_INFO(ctx_->node()->get_logger(), "Basket pickup lift test passed at alt=%.2f", ctx_->relativeAltitude());
    return BT::NodeStatus::SUCCESS;
  }

  if ((ctx_->node()->now() - start_time_).seconds() > timeout_sec_) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::RUNNING;
}
void VerifyBasketPicked::onHalted() {}

}  // namespace krac_control::bt
