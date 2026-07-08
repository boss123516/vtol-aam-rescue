#include "krac_control/bt/bt_actions_recovery.hpp"
#include "krac_control/bt/bt_conditions.hpp"

#include <algorithm>
#include <cmath>

namespace krac_control::bt
{

IsEmergencyDetected::IsEmergencyDetected(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config),
  ctx_(globalContext())
{
}

BT::PortsList IsEmergencyDetected::providedPorts()
{
  return {
    BT::InputPort<std::string>("sources", std::string(""), "")
  };
}

BT::NodeStatus IsEmergencyDetected::tick()
{
  // v1: emergency는 아직 외부 abort/failsafe와 연결하지 않음.
  // SafetyGuard 실패는 GlobalMissionRecovery 쪽에서 처리.
  return BT::NodeStatus::FAILURE;
}

RecoveryAction::RecoveryAction(
  const std::string& name,
  const BT::NodeConfiguration& config,
  const std::string& label)
: BT::StatefulActionNode(name, config),
  ctx_(globalContext()),
  label_(label)
{
}

BT::PortsList RecoveryAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("policy", std::string("hold"), ""),
    BT::InputPort<std::string>("reason", std::string(""), ""),
    BT::InputPort<double>("climb_altitude_m", 1.0, ""),
    BT::InputPort<double>("timeout_sec", 20.0, "")
  };
}

BT::NodeStatus RecoveryAction::onStart()
{
  getInput("policy", policy_);
  getInput("climb_altitude_m", climb_altitude_m_);
  getInput("timeout_sec", timeout_sec_);

  const double current_alt = ctx_->relativeAltitude();
  target_altitude_m_ = std::max(current_alt, current_alt + std::max(0.0, climb_altitude_m_));
  start_time_ = ctx_->node()->now();
  ctx_->setPrecisionLanderEnabled(false);
  ctx_->setHoldCurrentPosition();
  ctx_->setHoldAltitude(target_altitude_m_);
  ctx_->startOffboardSetpointStream(20.0, "hold_current_pose");
  RCLCPP_WARN(
    ctx_->node()->get_logger(),
    "%s started. policy=%s current_alt=%.2f target_alt=%.2f",
    label_.c_str(),
    policy_.c_str(),
    current_alt,
    target_altitude_m_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecoveryAction::onRunning()
{
  ctx_->setHoldAltitude(target_altitude_m_);
  const double alt_err = std::abs(ctx_->relativeAltitude() - target_altitude_m_);
  if (alt_err <= 0.7 || (ctx_->node()->now() - start_time_).seconds() > timeout_sec_) {
    ctx_->publishZeroVelocity();
    RCLCPP_WARN(
      ctx_->node()->get_logger(),
      "%s completed. alt=%.2f target=%.2f",
      label_.c_str(),
      ctx_->relativeAltitude(),
      target_altitude_m_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void RecoveryAction::onHalted()
{
  ctx_->publishZeroVelocity();
}

HoldRecoveryAction::HoldRecoveryAction(
  const std::string& name,
  const BT::NodeConfiguration& config,
  const std::string& label)
: BT::StatefulActionNode(name, config),
  ctx_(globalContext()),
  label_(label),
  policy_("hold")
{
}

BT::PortsList HoldRecoveryAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("policy", std::string("hold"), ""),
    BT::InputPort<std::string>("reason", std::string(""), ""),
    BT::InputPort<double>("climb_altitude_m", 1.0, ""),
    BT::InputPort<double>("timeout_sec", 20.0, "")
  };
}

BT::NodeStatus HoldRecoveryAction::onStart()
{
  getInput("policy", policy_);

  RCLCPP_WARN(
    ctx_->node()->get_logger(),
    "%s started. policy=%s. Holding instead of finishing the BT.",
    label_.c_str(),
    policy_.c_str());

  ctx_->publishZeroVelocity();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldRecoveryAction::onRunning()
{
  RCLCPP_WARN_THROTTLE(
    ctx_->node()->get_logger(),
    *ctx_->node()->get_clock(),
    2000,
    "%s active. policy=%s. Mission is held.",
    label_.c_str(),
    policy_.c_str());

  ctx_->publishZeroVelocity();

  return BT::NodeStatus::RUNNING;
}

void HoldRecoveryAction::onHalted()
{
  RCLCPP_WARN(
    ctx_->node()->get_logger(),
    "%s halted.",
    label_.c_str());
}

}  // namespace krac_control::bt
