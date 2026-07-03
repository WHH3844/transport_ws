#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

class DdsOutAndBack : public rclcpp::Node
{
public:
    DdsOutAndBack() : Node("dds_out_and_back_node")
    {
        target_z_ = this->declare_parameter<double>("target_z", -0.30);
        out_x_offset_ = this->declare_parameter<double>("out_x_offset", 0.30);
        out_y_offset_ = this->declare_parameter<double>("out_y_offset", 0.0);
        max_xy_offset_ = this->declare_parameter<double>("max_xy_offset", 0.50);
        max_height_ = this->declare_parameter<double>("max_height", 0.50);
        max_xy_speed_ = this->declare_parameter<double>("max_xy_speed", 0.15);
        max_z_speed_ = this->declare_parameter<double>("max_z_speed", 0.15);
        reach_tolerance_ = this->declare_parameter<double>("reach_tolerance", 0.08);
        hold_time_s_ = this->declare_parameter<double>("hold_time_s", 2.0);
        auto_offboard_ = this->declare_parameter<bool>("auto_offboard", false);
        auto_arm_ = this->declare_parameter<bool>("auto_arm", false);
        require_offboard_ = this->declare_parameter<bool>("require_offboard", true);
        require_armed_ = this->declare_parameter<bool>("require_armed", true);

        sanitizeParameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        offboard_control_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", qos);

        trajectory_setpoint_pub_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", qos);

        vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", qos);

        local_pos_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", qos,
                std::bind(&DdsOutAndBack::localPositionCallback, this, std::placeholders::_1));

        vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status", qos,
                std::bind(&DdsOutAndBack::vehicleStatusCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            50ms, std::bind(&DdsOutAndBack::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "DDS out-and-back node started.");
        RCLCPP_INFO(this->get_logger(),
                    "target_z=%.2f out_offset=(%.2f, %.2f) max_xy_speed=%.2f auto_offboard=%s auto_arm=%s",
                    target_z_, out_x_offset_, out_y_offset_, max_xy_speed_,
                    auto_offboard_ ? "true" : "false",
                    auto_arm_ ? "true" : "false");
    }

private:
    enum class Phase
    {
        WaitingForReady,
        GoOut,
        HoldOut,
        ReturnHome,
        HoldHome,
        Done
    };

    uint64_t nowUs()
    {
        return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
    }

    void sanitizeParameters()
    {
        max_xy_offset_ = std::clamp(max_xy_offset_, 0.05, 0.50);
        max_height_ = std::clamp(max_height_, 0.15, 0.50);
        max_xy_speed_ = std::clamp(max_xy_speed_, 0.05, 0.30);
        max_z_speed_ = std::clamp(max_z_speed_, 0.05, 0.30);
        reach_tolerance_ = std::clamp(reach_tolerance_, 0.03, 0.20);
        hold_time_s_ = std::clamp(hold_time_s_, 0.5, 10.0);

        out_x_offset_ = std::clamp(out_x_offset_, -max_xy_offset_, max_xy_offset_);
        out_y_offset_ = std::clamp(out_y_offset_, -max_xy_offset_, max_xy_offset_);

        const double min_height = 0.15;
        const double requested_height = std::abs(target_z_);
        const double safe_height = std::clamp(requested_height, min_height, max_height_);
        target_z_ = -safe_height;
    }

    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        local_pos_ = *msg;
        has_local_pos_ = true;
    }

    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        vehicle_status_ = *msg;
        has_vehicle_status_ = true;
    }

    bool localPositionHealthy() const
    {
        return has_local_pos_
            && local_pos_.xy_valid
            && local_pos_.z_valid
            && local_pos_.v_xy_valid
            && local_pos_.v_z_valid
            && local_pos_.heading_good_for_control
            && !local_pos_.dead_reckoning
            && std::isfinite(local_pos_.x)
            && std::isfinite(local_pos_.y)
            && std::isfinite(local_pos_.z);
    }

    bool vehicleReadyForMission() const
    {
        if (!has_vehicle_status_) {
            return false;
        }

        if (require_offboard_ &&
            vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            return false;
        }

        if (require_armed_ &&
            vehicle_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            return false;
        }

        return true;
    }

    void initializeHome()
    {
        if (home_initialized_) {
            return;
        }

        home_x_ = local_pos_.x;
        home_y_ = local_pos_.y;
        out_x_ = home_x_ + static_cast<float>(out_x_offset_);
        out_y_ = home_y_ + static_cast<float>(out_y_offset_);

        setpoint_x_ = home_x_;
        setpoint_y_ = home_y_;
        setpoint_z_ = static_cast<float>(target_z_);

        home_initialized_ = true;

        RCLCPP_WARN(this->get_logger(),
                    "Home initialized: home=(%.3f, %.3f, %.3f), out=(%.3f, %.3f, %.3f)",
                    home_x_, home_y_, target_z_, out_x_, out_y_, target_z_);
    }

    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = nowUs();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publishTrajectorySetpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = nowUs();

        msg.position = {
            static_cast<float>(setpoint_x_),
            static_cast<float>(setpoint_y_),
            static_cast<float>(setpoint_z_)
        };

        msg.velocity = {
            std::nanf(""),
            std::nanf(""),
            std::nanf("")
        };

        msg.acceleration = {
            std::nanf(""),
            std::nanf(""),
            std::nanf("")
        };

        msg.yaw = std::nanf("");
        msg.yawspeed = std::nanf("");

        trajectory_setpoint_pub_->publish(msg);
    }

    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = nowUs();

        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_pub_->publish(msg);
    }

    void setOffboardMode()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,
            6.0f);
        RCLCPP_WARN(this->get_logger(), "Sent command: SET_MODE OFFBOARD");
    }

    void arm()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f,
            0.0f);
        RCLCPP_WARN(this->get_logger(), "Sent command: ARM");
    }

    float stepToward(float current, float target, float max_step) const
    {
        const float delta = target - current;

        if (std::fabs(delta) <= max_step) {
            return target;
        }

        return current + std::copysign(max_step, delta);
    }

    void moveSetpointToward(float target_x, float target_y, float target_z)
    {
        const float xy_step = static_cast<float>(max_xy_speed_ * 0.05);
        const float z_step = static_cast<float>(max_z_speed_ * 0.05);

        setpoint_x_ = stepToward(setpoint_x_, target_x, xy_step);
        setpoint_y_ = stepToward(setpoint_y_, target_y, xy_step);
        setpoint_z_ = stepToward(setpoint_z_, target_z, z_step);
    }

    bool reached(float target_x, float target_y, float target_z) const
    {
        const float dx = local_pos_.x - target_x;
        const float dy = local_pos_.y - target_y;
        const float dz = local_pos_.z - target_z;
        const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        return distance <= static_cast<float>(reach_tolerance_);
    }

    void startHold()
    {
        hold_started_at_ = this->get_clock()->now();
        hold_timer_active_ = true;
    }

    bool holdElapsed()
    {
        if (!hold_timer_active_) {
            return false;
        }

        const auto elapsed = this->get_clock()->now() - hold_started_at_;
        return elapsed.seconds() >= hold_time_s_;
    }

    void advanceMission()
    {
        const float target_z = static_cast<float>(target_z_);

        switch (phase_) {
        case Phase::WaitingForReady:
            phase_ = Phase::GoOut;
            RCLCPP_WARN(this->get_logger(), "Mission start: go to out point.");
            break;

        case Phase::GoOut:
            moveSetpointToward(out_x_, out_y_, target_z);
            if (reached(out_x_, out_y_, target_z)) {
                setpoint_x_ = out_x_;
                setpoint_y_ = out_y_;
                setpoint_z_ = target_z;
                startHold();
                phase_ = Phase::HoldOut;
                RCLCPP_WARN(this->get_logger(), "Reached out point, hold.");
            }
            break;

        case Phase::HoldOut:
            setpoint_x_ = out_x_;
            setpoint_y_ = out_y_;
            setpoint_z_ = target_z;
            if (holdElapsed()) {
                hold_timer_active_ = false;
                phase_ = Phase::ReturnHome;
                RCLCPP_WARN(this->get_logger(), "Return to home point.");
            }
            break;

        case Phase::ReturnHome:
            moveSetpointToward(home_x_, home_y_, target_z);
            if (reached(home_x_, home_y_, target_z)) {
                setpoint_x_ = home_x_;
                setpoint_y_ = home_y_;
                setpoint_z_ = target_z;
                startHold();
                phase_ = Phase::HoldHome;
                RCLCPP_WARN(this->get_logger(), "Reached home point, hold.");
            }
            break;

        case Phase::HoldHome:
            setpoint_x_ = home_x_;
            setpoint_y_ = home_y_;
            setpoint_z_ = target_z;
            if (holdElapsed()) {
                hold_timer_active_ = false;
                phase_ = Phase::Done;
                RCLCPP_WARN(this->get_logger(), "Out-and-back mission complete. Holding home point.");
            }
            break;

        case Phase::Done:
            setpoint_x_ = home_x_;
            setpoint_y_ = home_y_;
            setpoint_z_ = target_z;
            break;
        }
    }

    void timerCallback()
    {
        if (!localPositionHealthy()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Waiting for healthy local position: xy=%d z=%d v_xy=%d v_z=%d heading_good=%d dead_reckoning=%d",
                local_pos_.xy_valid, local_pos_.z_valid,
                local_pos_.v_xy_valid, local_pos_.v_z_valid,
                local_pos_.heading_good_for_control, local_pos_.dead_reckoning);
            return;
        }

        initializeHome();

        publishOffboardControlMode();

        offboard_counter_++;

        if (auto_offboard_ && !offboard_sent_ && offboard_counter_ > 40) {
            setOffboardMode();
            offboard_sent_ = true;
        }

        if (auto_arm_ && offboard_sent_ && !arm_sent_ && offboard_counter_ > 80) {
            arm();
            arm_sent_ = true;
        }

        if (vehicleReadyForMission()) {
            advanceMission();
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Publishing home setpoint, waiting for mission ready: offboard=%d armed=%d",
                has_vehicle_status_ &&
                    vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,
                has_vehicle_status_ &&
                    vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        }

        publishTrajectorySetpoint();
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    px4_msgs::msg::VehicleLocalPosition local_pos_{};
    px4_msgs::msg::VehicleStatus vehicle_status_{};

    bool has_local_pos_{false};
    bool has_vehicle_status_{false};
    bool home_initialized_{false};
    bool auto_offboard_{false};
    bool auto_arm_{false};
    bool require_offboard_{true};
    bool require_armed_{true};
    bool offboard_sent_{false};
    bool arm_sent_{false};
    bool hold_timer_active_{false};

    int offboard_counter_{0};

    double target_z_{-0.30};
    double out_x_offset_{0.30};
    double out_y_offset_{0.0};
    double max_xy_offset_{0.50};
    double max_height_{0.50};
    double max_xy_speed_{0.15};
    double max_z_speed_{0.15};
    double reach_tolerance_{0.08};
    double hold_time_s_{2.0};

    float home_x_{0.0f};
    float home_y_{0.0f};
    float out_x_{0.0f};
    float out_y_{0.0f};
    float setpoint_x_{0.0f};
    float setpoint_y_{0.0f};
    float setpoint_z_{-0.30f};

    Phase phase_{Phase::WaitingForReady};
    rclcpp::Time hold_started_at_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DdsOutAndBack>());
    rclcpp::shutdown();
    return 0;
}
