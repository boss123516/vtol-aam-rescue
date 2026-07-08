#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <krac_interfaces/msg/target_error.hpp> // 커스텀 메시지
#include <std_srvs/srv/set_bool.hpp>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class PrecisionLander : public rclcpp::Node {
public:
    PrecisionLander() : Node("precision_lander_node") {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/precision_lander/cmd_vel", 10);
        vision_sub_ = this->create_subscription<krac_interfaces::msg::TargetError>("/vision/target_error", 10, std::bind(&PrecisionLander::vision_cb, this, std::placeholders::_1));

        auto qos = rclcpp::SensorDataQoS();
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&PrecisionLander::pose_cb, this, std::placeholders::_1));

        enable_srv_ = this->create_service<std_srvs::srv::SetBool>("/precision_lander/enable", std::bind(&PrecisionLander::enable_cb, this, std::placeholders::_1, std::placeholders::_2));
        timer_ = this->create_wall_timer(50ms, std::bind(&PrecisionLander::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🛬 정밀 착륙(Precision Lander) 가동! (0도/180도 최단거리 정렬 적용)");
    }

private:
    bool enabled_ = false;
    krac_interfaces::msg::TargetError vision_err_;
    rclcpp::Time last_detection_time_;
    double current_alt_ = 0.0;

    // 실제 카메라 캘리브레이션을 1024x1024 해상도 비율로 변환한 값
    const double fx_ = 582.5;
    const double fy_ = 1036.7;

    // PID 제어기 변수
    double prev_err_x_ = 0.0;
    double prev_err_y_ = 0.0;
    double sum_x_ = 0.0;
    double sum_y_ = 0.0;

    // 요동침(Oscillation) 방지를 위해 PID 게인 및 최대 속도 대폭 하향 조정
    const double KP_XY = 0.25;      // 부드럽게 반응
    const double KI_XY = 0.005;
    const double KD_XY = 0.02;      // 노이즈 증폭 및 떨림 방지
    const double MAX_XY_VEL = 0.5;  // 급발진 방지

    // 헤딩(Yaw) 정렬 파라미터 (회전 떨림도 방지하기 위해 하향)
    const double KP_YAW = 0.3;
    const double MAX_YAW_VEL = 0.2;

    // 고도 강하 파라미터
    const double SEARCH_ALTITUDE = 7.0;
    const double BLIND_DESCENT_SPEED = -1.5;
    const double PRECISION_DESCEND_SPEED = -0.4;
    const double TARGET_FRESH_MAX_AGE_SEC = 3.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<krac_interfaces::msg::TargetError>::SharedPtr vision_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    void vision_cb(const krac_interfaces::msg::TargetError::SharedPtr msg) {
        vision_err_ = *msg;
        if (msg->is_detected) {
            last_detection_time_ = this->now();
        }
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_alt_ = msg->pose.position.z;
    }

    void enable_cb(const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
        enabled_ = req->data;
        res->success = true;

        if (enabled_) {
            sum_x_ = 0.0; sum_y_ = 0.0;
            prev_err_x_ = 0.0; prev_err_y_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "✅ Lander 상태: ON (안정화 제어 시작)");
        } else {
            RCLCPP_INFO(this->get_logger(), "⛔ Lander 상태: OFF");
        }
    }

    void control_loop() {
        if (!enabled_) return;

        geometry_msgs::msg::Twist vel_cmd;

        // =======================================================
        // 1. 타겟이 보이지 않을 때의 예외 처리
        // =======================================================
        const bool target_fresh =
            vision_err_.is_detected &&
            last_detection_time_.nanoseconds() > 0 &&
            (this->now() - last_detection_time_).seconds() <= TARGET_FRESH_MAX_AGE_SEC;

        if (!target_fresh) {
            vel_cmd.linear.x = 0.0;
            vel_cmd.linear.y = 0.0;
            vel_cmd.linear.z = 0.0;
            vel_cmd.angular.z = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[Target LOST] holding position; BT recovery must reacquire before descent.");
            vel_pub_->publish(vel_cmd);
            return;
        }

        // =======================================================
        // 🎯 2. 타겟 감지 시 정밀 정렬 (요동침 방지 적용)
        // =======================================================
        double alt_factor = std::max(current_alt_, 0.5);
        double err_x_m = vision_err_.pixel_err_x * (alt_factor / fx_);
        double err_y_m = vision_err_.pixel_err_y * (alt_factor / fy_);

        // PID 연산
        sum_x_ = std::clamp(sum_x_ + err_x_m * 0.05, -1.0, 1.0);
        sum_y_ = std::clamp(sum_y_ + err_y_m * 0.05, -1.0, 1.0);

        double v_x = (KP_XY * err_x_m) + (KI_XY * sum_x_) + (KD_XY * (err_x_m - prev_err_x_) / 0.05);
        double v_y = (KP_XY * err_y_m) + (KI_XY * sum_y_) + (KD_XY * (err_y_m - prev_err_y_) / 0.05);

        prev_err_x_ = err_x_m;
        prev_err_y_ = err_y_m;

        // 데드밴드(Deadband): 오차가 10cm 이내면 X, Y 이동 명령을 무시하여 제자리 떨림 방지
        if (std::abs(err_x_m) < 0.1) v_x = 0.0;
        if (std::abs(err_y_m) < 0.1) v_y = 0.0;

        // 앞/뒤(X축) 방향 오류 수정을 위해 v_y 앞에 다시 마이너스(-)를 붙임.
        vel_cmd.linear.x = std::clamp(-v_y, -MAX_XY_VEL, MAX_XY_VEL); // 앞뒤 제어
        vel_cmd.linear.y = std::clamp(v_x, -MAX_XY_VEL, MAX_XY_VEL);  // 좌우 제어

        // =======================================================
        // 🔄 3. OBB 기반 헤딩(Yaw) 0도 또는 180도 최단거리 정렬
        // =======================================================
        double target_yaw_err = vision_err_.yaw_err_rad;

        // 💡 [핵심 추가] 각도 오차를 [-90도, +90도] 구간으로 강제 매핑(래핑)합니다.
        // 이를 통해 드론이 180도를 빙 돌지 않고, 가장 가까운 직사각형 축으로 바로 맞춥니다.
        while (target_yaw_err > M_PI / 2.0) {
            target_yaw_err -= M_PI;
        }
        while (target_yaw_err < -M_PI / 2.0) {
            target_yaw_err += M_PI;
        }

        double yaw_cmd = target_yaw_err * KP_YAW;

        // 각도 오차도 3도(약 0.05 rad) 이내면 회전을 멈춰서 요동침 방지
        if (std::abs(target_yaw_err) < 0.05) yaw_cmd = 0.0;

        vel_cmd.angular.z = std::clamp(yaw_cmd, -MAX_YAW_VEL, MAX_YAW_VEL);

        // =======================================================
        // ⬇️ 4. 오차 반경에 따른 강하 속도 조절
        // =======================================================
        double dist_m = std::hypot(err_x_m, err_y_m);
        double yaw_deg = target_yaw_err * (180.0 / M_PI); // 래핑된 각도 출력

        if (dist_m < 0.5) {
            vel_cmd.linear.z = PRECISION_DESCEND_SPEED;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🎯 [Aligned] 정밀 하강 중 (오차: %.2fm, 각도: %.1f도, 고도: %.1fm)", dist_m, yaw_deg, current_alt_);
        } else if (dist_m < 1.5) {
            vel_cmd.linear.z = PRECISION_DESCEND_SPEED * 0.5;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "↘️ [Approaching] 센터 진입 중... (각도: %.1f도)", yaw_deg);
        } else {
            vel_cmd.linear.z = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🔄 [Aligning] 센터 맞추기 집중... (현재 오차: %.2fm)", dist_m);
        }

        vel_pub_->publish(vel_cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrecisionLander>());
    rclcpp::shutdown();
    return 0;
}
