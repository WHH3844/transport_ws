#include <chrono>
#include <cmath>
#include <array>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

class DdsOffboardControl : public rclcpp::Node
{
public:
    DdsOffboardControl() : Node("dds_offboard_control_node")
    {
        target_z_ = this->declare_parameter<double>("target_z", -0.30);
        auto_offboard_ = this->declare_parameter<bool>("auto_offboard", false);
        auto_arm_ = this->declare_parameter<bool>("auto_arm", false);

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
                std::bind(&DdsOffboardControl::localPositionCallback, this, std::placeholders::_1));

        vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status", qos,
                std::bind(&DdsOffboardControl::vehicleStatusCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            50ms, std::bind(&DdsOffboardControl::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "DDS Offboard Control node started.");
        RCLCPP_INFO(this->get_logger(), "target_z = %.2f, auto_offboard = %s, auto_arm = %s",
                    target_z_,
                    auto_offboard_ ? "true" : "false",
                    auto_arm_ ? "true" : "false");
    }

private:
    uint64_t nowUs()
    {
        return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
    }

    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        local_pos_ = *msg;
        has_local_pos_ = true;

        if (!hold_position_initialized_) {
            if (msg->xy_valid && msg->z_valid &&
                std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z)) {

                hold_x_ = msg->x;
                hold_y_ = msg->y;
                hold_yaw_ = std::isfinite(msg->heading) ? msg->heading : 0.0f;
                hold_position_initialized_ = true;

                RCLCPP_INFO(this->get_logger(),
                            "Hold position initialized: x=%.3f y=%.3f z_target=%.3f yaw=%.3f",
                            hold_x_, hold_y_, target_z_, hold_yaw_);
            }
        }
    }

    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        vehicle_status_ = *msg;
        has_vehicle_status_ = true;
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
            static_cast<float>(hold_x_),
            static_cast<float>(hold_y_),
            static_cast<float>(target_z_)
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

        msg.yaw = std::nanf("");  // do not force yaw while heading_good_for_control=false
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

    void timerCallback()
    {
        if (!has_local_pos_ || !hold_position_initialized_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Waiting for valid local position...");
            return;
        }

        if (!local_pos_.xy_valid || !local_pos_.z_valid) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Local position not valid: xy_valid=%d z_valid=%d",
                local_pos_.xy_valid, local_pos_.z_valid);
            return;
        }

        publishOffboardControlMode();
        publishTrajectorySetpoint();

        offboard_counter_++;

        if (auto_offboard_ && !offboard_sent_ && offboard_counter_ > 40) {
            setOffboardMode();
            offboard_sent_ = true;
        }

        if (auto_arm_ && offboard_sent_ && !arm_sent_ && offboard_counter_ > 80) {
            arm();
            arm_sent_ = true;
        }
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
    bool hold_position_initialized_{false};

    bool auto_offboard_{false};
    bool auto_arm_{false};

    bool offboard_sent_{false};
    bool arm_sent_{false};

    int offboard_counter_{0};

    double target_z_{-0.30};

    float hold_x_{0.0f};
    float hold_y_{0.0f};
    float hold_yaw_{0.0f};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DdsOffboardControl>());
    rclcpp::shutdown();
    return 0;
}
