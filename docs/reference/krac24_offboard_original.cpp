#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/waypoint_reached.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

// FSM 상태 정의: 미션 비행과 오프보드 제어를 혼합
enum MissionPhase {
    PHASE_MISSION_FLYING = 0,         // AUTO.MISSION 비행 중
    PHASE_PREPARE_OFFBOARD = 1,       // OFFBOARD 전환 대기 (제어값 스트리밍)
    PHASE_SEARCH_RESCUE = 2,          // 구조물 탐색
    PHASE_VISION_ALIGN_RESCUE = 3,    // 구조물 비전 정렬 및 하강
    PHASE_ASCEND_WITH_VICTIM = 4,     // 파지 후 상승
    PHASE_RESUME_MISSION = 5,         // 다시 AUTO.MISSION으로 복귀
    PHASE_VISION_ALIGN_LAND = 6,      // 끝점 도착 후 비전 기반 정밀 착륙
    PHASE_LANDING = 7                 // AUTO.LAND 전환
};

class RescueMissionNode : public rclcpp::Node
{
public:
    RescueMissionNode() : Node("rescue_mission_node")
    {
        auto qos = rclcpp::SensorDataQoS();
        last_vision_time_ = this->now();

        // 파라미터: 웨이포인트 시퀀스 번호 지정 (시작 0, 구조 1, 끝점 2)
        this->declare_parameter("rescue_wp_seq", 1);
        this->declare_parameter("end_wp_seq", 2);

        rescue_wp_seq_ = this->get_parameter("rescue_wp_seq").as_int();
        end_wp_seq_ = this->get_parameter("end_wp_seq").as_int();

        // 구독자 설정
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", qos, std::bind(&RescueMissionNode::state_cb, this, std::placeholders::_1));

        mission_reached_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
            "mavros/mission/reached", 10, std::bind(&RescueMissionNode::mission_reached_cb, this, std::placeholders::_1));

        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "mavros/local_position/pose", qos, std::bind(&RescueMissionNode::local_pose_cb, this, std::placeholders::_1));

        vision_error_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "camera/target_error", 10, std::bind(&RescueMissionNode::vision_cb, this, std::placeholders::_1));

        // 발행자 설정
        vel_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        target_label_pub_ = this->create_publisher<std_msgs::msg::String>(
            "camera/set_target", 10);

        // 클라이언트 & 서비스
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        set_mission_current_client_ = this->create_client<mavros_msgs::srv::WaypointSetCurrent>("mavros/mission/set_current");

        manual_trigger_server_ = this->create_service<std_srvs::srv::Trigger>(
            "cmd/mission_proceed", std::bind(&RescueMissionNode::manual_proceed_cb, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(50ms, std::bind(&RescueMissionNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 Waypoint-based Offboard Rescue Node Started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for AUTO.MISSION to reach WP %d...", rescue_wp_seq_);
    }

private:
    MissionPhase current_phase_ = PHASE_MISSION_FLYING;
    MissionPhase next_phase_after_prep_ = PHASE_MISSION_FLYING;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_local_pose_;
    geometry_msgs::msg::Point vision_error_;
    rclcpp::Time last_vision_time_;

    bool manual_ok_ = false;
    bool offboard_enabled_ = false;
    int last_processed_wp_ = -1;
    int offboard_prep_counter_ = 0;
    
    int rescue_wp_seq_;
    int end_wp_seq_;

    const double VISION_P_GAIN = 0.005;
    const double MAX_ALIGN_SPEED = 0.5;
    const double DESCEND_SPEED = -0.3;
    const double HOVER_STEP = 1.0; 
    const double SEARCH_SPEED = 0.2; 

    int search_step_ = 0;
    rclcpp::Time search_start_time_;
    double target_hover_alt_ = 0.0;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr mission_reached_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_error_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_label_pub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr set_mission_current_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_trigger_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
    void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_local_pose_ = *msg; }
    void vision_cb(const geometry_msgs::msg::Point::SharedPtr msg) { 
        vision_error_ = *msg; 
        if(msg->z > 0.5) last_vision_time_ = this->now();
    }

    void set_vision_label(std::string label) {
        std_msgs::msg::String msg; msg.data = label; target_label_pub_->publish(msg);
    }

    void manual_proceed_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        manual_ok_ = true;
        res->success = true;
        res->message = "Force Proceed Triggered";
        RCLCPP_INFO(this->get_logger(), "✅ User Triggered Force Proceed.");
    }

    double clamp_velocity(double vel) {
        return std::clamp(vel, -MAX_ALIGN_SPEED, MAX_ALIGN_SPEED);
    }

    // 미션 웨이포인트 도달 콜백 (vtol_fsm 방식)
    void mission_reached_cb(const mavros_msgs::msg::WaypointReached::SharedPtr msg) {
        if (msg->wp_seq == last_processed_wp_) return; 
        last_processed_wp_ = msg->wp_seq; 

        if (current_phase_ != PHASE_MISSION_FLYING) return;

        RCLCPP_INFO(this->get_logger(), "🎯 Waypoint Reached: [WP %d]", msg->wp_seq);

        if (msg->wp_seq == rescue_wp_seq_) { 
            RCLCPP_WARN(this->get_logger(), "🚨 구조 지점 WP(%d번) 도달! 오프보드 강제 진입 시작.", rescue_wp_seq_);
            set_vision_label("basket"); 
            offboard_enabled_ = true;
            offboard_prep_counter_ = 0;
            next_phase_after_prep_ = PHASE_SEARCH_RESCUE;
            current_phase_ = PHASE_PREPARE_OFFBOARD;
        } 
        else if (msg->wp_seq == end_wp_seq_) { 
            RCLCPP_WARN(this->get_logger(), "🛬 끝점 WP(%d번) 도달! 착륙 오프보드 진입.", end_wp_seq_);
            set_vision_label("vertiport");
            offboard_enabled_ = true;
            offboard_prep_counter_ = 0;
            next_phase_after_prep_ = PHASE_VISION_ALIGN_LAND;
            current_phase_ = PHASE_PREPARE_OFFBOARD;
        }
    }

    void execute_smooth_search_pattern(geometry_msgs::msg::Twist &vel_cmd) {
        if (search_step_ == 0) {
            search_start_time_ = this->now();
            search_step_ = 1;
        }
        double elapsed = (this->now() - search_start_time_).seconds();
        
        if (elapsed < 3.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = -SEARCH_SPEED; } 
        else if (elapsed < 4.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; } 
        else if (elapsed < 7.0) { vel_cmd.linear.x = -SEARCH_SPEED; vel_cmd.linear.y = 0.0; }
        else if (elapsed < 8.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; }
        else if (elapsed < 11.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = SEARCH_SPEED; }
        else if (elapsed < 12.0) { vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; }
        else if (elapsed < 15.0) { vel_cmd.linear.x = SEARCH_SPEED; vel_cmd.linear.y = 0.0; }
        else { search_step_ = 0; }
        vel_cmd.linear.z = 0.0; vel_cmd.angular.z = 0.0; 
    }

    void execute_step_descent(geometry_msgs::msg::Twist &vel_cmd, double &target_alt) {
        double current_alt = current_local_pose_.pose.position.z;
        double dist_pixel = std::sqrt(vision_error_.x * vision_error_.x + vision_error_.y * vision_error_.y);

        vel_cmd.linear.x = clamp_velocity(-vision_error_.y * VISION_P_GAIN);
        vel_cmd.linear.y = clamp_velocity(vision_error_.x * VISION_P_GAIN);
        vel_cmd.angular.z = 0.0;

        if (current_alt < target_alt - 0.1) target_alt = current_alt;

        if (std::abs(current_alt - target_alt) < 0.2) {
            if (dist_pixel < 30.0) { 
                if (target_alt > 1.5) {
                     target_alt -= HOVER_STEP;
                     if (target_alt < 1.5) target_alt = 1.5; 
                     RCLCPP_INFO(this->get_logger(), "Aligned! Stepping down to %.1f m", target_alt);
                }
            } else {
                vel_cmd.linear.z = 0.0;
            }
        } else if (current_alt > target_alt) {
            if (dist_pixel > 100.0) vel_cmd.linear.z = 0.0;
            else vel_cmd.linear.z = DESCEND_SPEED;
        } else {
            vel_cmd.linear.z = 0.0;
        }
    }

    void control_loop() {
        // 1. 미션 비행 중이거나 이착륙 중에는 이 노드가 제어할 필요가 없음
        if (current_phase_ == PHASE_MISSION_FLYING) return;

        // 2. Offboard 활성화 상태에서 모드가 풀렸을 때 강제 복구 (착륙, 미션 복귀 상태 제외)
        if (offboard_enabled_ && current_state_.mode != "OFFBOARD" && 
            current_phase_ != PHASE_LANDING && current_phase_ != PHASE_RESUME_MISSION) 
        {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(req);
        }

        geometry_msgs::msg::Twist vel_cmd;
        bool publish_vel = true; 
        bool object_detected = (last_vision_time_.nanoseconds() > 0 && 
                               (this->now() - last_vision_time_).seconds() < 0.5); 

        // 사용자 강제 진행 오버라이드
        if (manual_ok_) {
            manual_ok_ = false; 
            if (current_phase_ == PHASE_VISION_ALIGN_RESCUE) {
                 RCLCPP_INFO(this->get_logger(), "Force Proceed -> ASCEND WITH VICTIM");
                 current_phase_ = PHASE_ASCEND_WITH_VICTIM;
            } else if (current_phase_ == PHASE_VISION_ALIGN_LAND) {
                 RCLCPP_INFO(this->get_logger(), "Force Proceed -> LANDING");
                 current_phase_ = PHASE_LANDING;
            }
        }

        switch (current_phase_) {
            case PHASE_PREPARE_OFFBOARD:
                // 오프보드 모드 전환 전 반드시 제어값 스트리밍이 필요
                vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; vel_cmd.linear.z = 0.0; vel_cmd.angular.z = 0.0;
                offboard_prep_counter_++;
                if (offboard_prep_counter_ >= 20) { // 1초 대기 후 실제 로직으로 천이
                    current_phase_ = next_phase_after_prep_;
                }
                break;

            case PHASE_SEARCH_RESCUE:
                if (object_detected) {
                    RCLCPP_INFO(this->get_logger(), "Object Found! Aligning...");
                    target_hover_alt_ = current_local_pose_.pose.position.z; 
                    current_phase_ = PHASE_VISION_ALIGN_RESCUE;
                } else {
                    if (current_local_pose_.pose.position.z < 3.0) execute_smooth_search_pattern(vel_cmd);
                    else { vel_cmd.linear.z = -0.3; vel_cmd.angular.z = 0.0; }
                }
                break;

            case PHASE_VISION_ALIGN_RESCUE:
                if (object_detected) {
                    if (current_local_pose_.pose.position.z < 1.6 && target_hover_alt_ <= 1.5) {
                        vel_cmd.linear.z = 0.0;
                        vel_cmd.linear.x = clamp_velocity(-vision_error_.y * VISION_P_GAIN);
                        vel_cmd.linear.y = clamp_velocity(vision_error_.x * VISION_P_GAIN);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hovering. Send trigger to grab and ascend...");
                    } else {
                        execute_step_descent(vel_cmd, target_hover_alt_);
                    }
                } else { 
                    RCLCPP_WARN(this->get_logger(), "Lost Object! Searching...");
                    current_phase_ = PHASE_SEARCH_RESCUE;
                    search_step_ = 0; 
                }
                break;

            case PHASE_ASCEND_WITH_VICTIM:
                vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0;
                vel_cmd.linear.z = 0.5; // 상승
                if (current_local_pose_.pose.position.z > 5.0) { // 안전 고도 도달
                    RCLCPP_INFO(this->get_logger(), "Ascent Complete. Switching back to AUTO.MISSION.");
                    current_phase_ = PHASE_RESUME_MISSION;
                }
                break;

            case PHASE_RESUME_MISSION:
                {
                    publish_vel = false; // 제어값 스트리밍 중단
                    
                    // 다음 웨이포인트(끝점)로 Current 지정
                    auto wp_req = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
                    wp_req->wp_seq = end_wp_seq_;
                    set_mission_current_client_->async_send_request(wp_req);

                    // 다시 미션 모드로 전환
                    auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    mode_req->custom_mode = "AUTO.MISSION";
                    set_mode_client_->async_send_request(mode_req);
                    
                    offboard_enabled_ = false;
                    current_phase_ = PHASE_MISSION_FLYING;
                }
                break;

            case PHASE_VISION_ALIGN_LAND:
                if (object_detected) {
                    double dist_pixel = std::sqrt(vision_error_.x * vision_error_.x + vision_error_.y * vision_error_.y);
                    
                    vel_cmd.linear.x = clamp_velocity(-vision_error_.y * VISION_P_GAIN);
                    vel_cmd.linear.y = clamp_velocity(vision_error_.x * VISION_P_GAIN);
                    vel_cmd.angular.z = 0.0;

                    if (dist_pixel < 100.0) vel_cmd.linear.z = -0.4; 
                    else vel_cmd.linear.z = -0.1; 

                    if (current_local_pose_.pose.position.z < 0.5) {
                        RCLCPP_INFO(this->get_logger(), "Near Ground. Switching to AUTO.LAND.");
                        current_phase_ = PHASE_LANDING;
                    }
                } else {
                     vel_cmd.linear.x = 0.0; vel_cmd.linear.y = 0.0; vel_cmd.linear.z = -0.2;
                     vel_cmd.angular.z = 0.0;
                }
                break;

            case PHASE_LANDING:
                {
                    publish_vel = false;
                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "AUTO.LAND";
                    set_mode_client_->async_send_request(req);
                    offboard_enabled_ = false;
                }
                break;
                
            default:
                break;
        }

        if (publish_vel) {
            vel_setpoint_pub_->publish(vel_cmd);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RescueMissionNode>());
    rclcpp::shutdown();
    return 0;
}
