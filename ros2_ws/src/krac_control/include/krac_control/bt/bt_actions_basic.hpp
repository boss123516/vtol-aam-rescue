#pragma once

#include "krac_control/bt/mission_context.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <future>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>

namespace krac_control::bt
{

class StartOffboardSetpointStream : public BT::SyncActionNode
{
public:
  StartOffboardSetpointStream(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class SetFlightMode : public BT::StatefulActionNode
{
public:
  SetFlightMode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  std::string requested_mode_;
  rclcpp::Time start_time_;
  double timeout_sec_{5.0};
  bool request_sent_{false};
};

class ArmVehicle : public BT::StatefulActionNode
{
public:
  ArmVehicle(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  bool target_arm_{true};
  rclcpp::Time start_time_;
  double timeout_sec_{5.0};
};

class CommandVTOLTransition : public BT::StatefulActionNode
{
public:
  CommandVTOLTransition(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  std::string target_state_;
  int target_vtol_state_{3};
  rclcpp::Time start_time_;
  double timeout_sec_{20.0};
};

class ClearMissionPlan : public BT::StatefulActionNode
{
public:
  ClearMissionPlan(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  bool optional_{true};
  rclcpp::Time start_time_;
};

class UploadMissionPlan : public BT::StatefulActionNode
{
public:
  UploadMissionPlan(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  std::future<bool> future_;
  std::string mission_name_;
  std::string plan_path_;
  double timeout_sec_{30.0};
  rclcpp::Time start_time_;
};

class ActivateYOLO : public BT::SyncActionNode
{
public:
  ActivateYOLO(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class DeactivateYOLO : public BT::SyncActionNode
{
public:
  DeactivateYOLO(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  std::shared_ptr<MissionContext> ctx_;
};

class CloseGripper : public BT::StatefulActionNode
{
public:
  CloseGripper(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  std::string left_topic_;
  std::string right_topic_;
  std::string attach_topic_;
  double timeout_sec_{5.0};
  double settle_sec_{1.0};
  double publish_rate_hz_{20.0};
  double left_position_{0.5};
  double right_position_{0.5};
  bool attached_{false};
  rclcpp::Time start_time_;
  rclcpp::Time last_publish_time_;
};

}  // namespace krac_control::bt
