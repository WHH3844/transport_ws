#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

class MocapToPx4Bridge : public rclcpp::Node
{
public:
    MocapToPx4Bridge() : Node("mocap_to_px4_bridge")
    {
        mocap_topic_ = this->declare_parameter<std::string>("mocap_topic", "/Robot_1/pose");
        px4_odom_topic_ = this->declare_parameter<std::string>("px4_odom_topic", "/fmu/in/vehicle_mocap_odometry");

        invert_y_ = this->declare_parameter<bool>("invert_y", true);
        invert_z_ = this->declare_parameter<bool>("invert_z", true);

        auto sensor_qos = rclcpp::SensorDataQoS();
        auto px4_qos = rclcpp::QoS(10).best_effort();

        mocap_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            mocap_topic_,
            sensor_qos,
            std::bind(&MocapToPx4Bridge::mocapCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            px4_odom_topic_,
            px4_qos);

        RCLCPP_INFO(this->get_logger(), "mocap_to_px4_bridge started.");
        RCLCPP_INFO(this->get_logger(), "Subscribe: %s", mocap_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish  : %s", px4_odom_topic_.c_str());
    }

private:
    void mocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        px4_msgs::msg::VehicleOdometry odom{};

        const uint64_t now_us = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
        const float nan = std::numeric_limits<float>::quiet_NaN();

        odom.timestamp = now_us;
        odom.timestamp_sample = now_us;

        // 第一版按 NED 位置输入 PX4：
        // 假设 mocap / ROS 为 x前、y左、z上
        // PX4 NED 为 x前、y右、z下
        odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        const double raw_x = msg->pose.position.x;
	const double raw_y = msg->pose.position.y;
	const double raw_z = msg->pose.position.z;

	if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(raw_z)) {
    	RCLCPP_WARN_THROTTLE(
        	this->get_logger(), *this->get_clock(), 1000,
        	"Invalid mocap data: NaN or inf, skip publish");
    	return;
	}

	if (std::fabs(raw_x) > 10.0 || std::fabs(raw_y) > 10.0 || std::fabs(raw_z) > 5.0) {
    	RCLCPP_WARN_THROTTLE(
        	this->get_logger(), *this->get_clock(), 1000,
        	"Invalid mocap data: out of range x=%.3f y=%.3f z=%.3f, skip publish",
        	raw_x, raw_y, raw_z);
    	return;
	}

	odom.position[0] = static_cast<float>(raw_x);
	odom.position[1] = static_cast<float>(invert_y_ ? -raw_y : raw_y);
	odom.position[2] = static_cast<float>(invert_z_ ? -raw_z : raw_z);

        // 融合 NOKOV 姿态/yaw。
        // /Robot_1/pose 的 orientation 顺序是 x, y, z, w。
        // 当前位置转换为：PX4 x = raw_x, PX4 y = -raw_y, PX4 z = -raw_z。
        // 对应姿态先按 q_px4 = [w, x, -y, -z] 转成 PX4 VehicleOdometry.q。
        float qx = static_cast<float>(msg->pose.orientation.x);
        float qy = static_cast<float>(msg->pose.orientation.y);
        float qz = static_cast<float>(msg->pose.orientation.z);
        float qw = static_cast<float>(msg->pose.orientation.w);

        const float q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

        if (std::isfinite(q_norm) && q_norm > 1e-6f) {
            qw /= q_norm;
            qx /= q_norm;
            qy /= q_norm;
            qz /= q_norm;

            odom.q[0] = qw;
            odom.q[1] = qx;
            odom.q[2] = invert_y_ ? -qy : qy;
            odom.q[3] = invert_z_ ? -qz : qz;

            // 避免四元数整体符号跳变
            if (odom.q[0] < 0.0f) {
                odom.q[0] = -odom.q[0];
                odom.q[1] = -odom.q[1];
                odom.q[2] = -odom.q[2];
                odom.q[3] = -odom.q[3];
            }
        } else {
            odom.q[0] = nan;
            odom.q[1] = nan;
            odom.q[2] = nan;
            odom.q[3] = nan;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Invalid mocap orientation quaternion: x=%.6f y=%.6f z=%.6f w=%.6f norm=%.6f, publish NaN orientation",
                qx, qy, qz, qw, q_norm);
        }

        odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        odom.velocity[0] = nan;
        odom.velocity[1] = nan;
        odom.velocity[2] = nan;

        odom.angular_velocity[0] = nan;
        odom.angular_velocity[1] = nan;
        odom.angular_velocity[2] = nan;

        // 先给一个保守方差，后面根据 NOKOV 实际精度再调。
        odom.position_variance[0] = 0.01f;
        odom.position_variance[1] = 0.01f;
        odom.position_variance[2] = 0.01f;

        if (std::isfinite(odom.q[0])) {
            odom.orientation_variance[0] = 0.05f;
            odom.orientation_variance[1] = 0.05f;
            odom.orientation_variance[2] = 0.05f;
        } else {
            odom.orientation_variance[0] = nan;
            odom.orientation_variance[1] = nan;
            odom.orientation_variance[2] = nan;
        }

        odom.velocity_variance[0] = nan;
        odom.velocity_variance[1] = nan;
        odom.velocity_variance[2] = nan;

        odom.reset_counter = 0;
        odom.quality = 100;

        odom_pub_->publish(odom);
    }

private:
    std::string mocap_topic_;
    std::string px4_odom_topic_;
    bool invert_y_;
    bool invert_z_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapToPx4Bridge>());
    rclcpp::shutdown();
    return 0;
}
