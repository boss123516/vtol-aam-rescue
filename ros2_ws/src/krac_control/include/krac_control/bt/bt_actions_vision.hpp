#pragma once

#include "krac_control/bt/mission_context.hpp"
#include <behaviortree_cpp_v3/action_node.h>

namespace krac_control::bt
{

class ExecuteSearchPattern : public BT::StatefulActionNode
{
public:
  ExecuteSearchPattern(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double max_duration_sec_{45.0};
  rclcpp::Time start_time_;
};

class AlignToTarget : public BT::StatefulActionNode
{
public:
  AlignToTarget(const std::string& name, const BT::NodeConfiguration& config, const std::string& target_name);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  std::string target_name_;
  double pixel_tolerance_px_{25.0};
  double stable_duration_sec_{1.0};
  double timeout_sec_{35.0};
  rclcpp::Time start_time_;
  rclcpp::Time stable_start_;
};

class AlignToBasket : public AlignToTarget
{
public:
  AlignToBasket(const std::string& name, const BT::NodeConfiguration& config) : AlignToTarget(name, config, "basket") {}
};

class AlignToLandingMarker : public AlignToTarget
{
public:
  AlignToLandingMarker(const std::string& name, const BT::NodeConfiguration& config) : AlignToTarget(name, config, "landing_marker") {}
};

class DescendWithAlignment : public BT::StatefulActionNode
{
public:
  DescendWithAlignment(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  std::string target_;
  double min_confidence_{0.55};
  double max_lost_time_sec_{0.7};
  double timeout_sec_{60.0};
  double target_altitude_m_{0.8};
  rclcpp::Time start_time_;
  rclcpp::Time last_seen_time_;
};

class LandOnZone : public BT::StatefulActionNode
{
public:
  LandOnZone(const std::string& name, const BT::NodeConfiguration& config, bool basket);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  bool basket_;
  double target_altitude_m_{0.35};
  double timeout_sec_{30.0};
  rclcpp::Time start_time_;
};

class LandOnBasketZone : public LandOnZone
{
public:
  LandOnBasketZone(const std::string& name, const BT::NodeConfiguration& config) : LandOnZone(name, config, true) {}
};

class LandOnLandingZone : public LandOnZone
{
public:
  LandOnLandingZone(const std::string& name, const BT::NodeConfiguration& config) : LandOnZone(name, config, false) {}
};

class VerifyBasketPicked : public BT::StatefulActionNode
{
public:
  VerifyBasketPicked(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  double timeout_sec_{10.0};
  rclcpp::Time start_time_;
};

}  // namespace krac_control::bt
