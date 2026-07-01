#include <vision_to_mavros/vision_to_mavros.hpp>

/**
 * @brief Constructor for the VisionToMavros class.
 */
VisionToMavros::VisionToMavros() : Node("vision_to_mavros_node") {
    buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    buffer->setCreateTimerInterface(timer_interface);
    
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

    this->navigationParameters();
    this->precisionLandParameters();
}

void VisionToMavros::navigationParameters(void) {
    camera_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose", 10);
    body_path_publisher = this->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);
    
    this->declare_parameter<std::string>("target_frame_id", "/camera_odom_frame");
    this->get_parameter("target_frame_id", target_frame_id);

    this->declare_parameter<std::string>("source_frame_id", "/camera_link");
    this->get_parameter("source_frame_id", source_frame_id);

    this->declare_parameter<double>("output_rate", 20.0);
    this->get_parameter("output_rate", output_rate);

    this->declare_parameter<double>("gamma_world", -1.5707963);
    this->get_parameter("gamma_world", gamma_world);

    this->declare_parameter<double>("roll_cam", 0.0);
    this->get_parameter("roll_cam", roll_cam);
    
    this->declare_parameter<double>("pitch_cam", 0.0);
    this->get_parameter("pitch_cam", pitch_cam);

    this->declare_parameter<double>("yaw_cam", 1.5707963);
    this->get_parameter("yaw_cam", yaw_cam);

    RCLCPP_INFO(this->get_logger(), "Get target_frame_id: %s", target_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Get source_frame_id: %s", source_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Get output_rate: %f", output_rate);
    RCLCPP_INFO(this->get_logger(), "Get gamma_world: %f", gamma_world);
    RCLCPP_INFO(this->get_logger(), "Get roll_cam: %f", roll_cam);
    RCLCPP_INFO(this->get_logger(), "Get pitch_cam: %f", pitch_cam);
    RCLCPP_INFO(this->get_logger(), "Get yaw_cam: %f", yaw_cam);
}

void VisionToMavros::precisionLandParameters(void) {
    this->declare_parameter<bool>("enable_precland", false);
    this->get_parameter("enable_precland", enable_precland);

    RCLCPP_INFO(this->get_logger(), "Precision landing: %s", enable_precland ? "enabled" : "disabled");

    if (enable_precland) {
        this->declare_parameter<std::string>("precland_target_frame_id", "/landing_target");
        this->get_parameter("precland_target_frame_id", precland_target_frame_id);
        this->declare_parameter<std::string>("precland_camera_frame_id", "/camera_fisheye2_optical_frame");
        this->get_parameter("precland_camera_frame_id", precland_camera_frame_id);

        RCLCPP_INFO(this->get_logger(), "Get precland_target_frame_id: %s", precland_target_frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "Get precland_camera_frame_id: %s", precland_camera_frame_id.c_str());

        precland_msg_publisher = this->create_publisher<mavros_msgs::msg::LandingTarget>("landing_raw", 10);
    }
}

bool VisionToMavros::waitForFirstTransform(double timeout) {
    bool received = false;
    std::string error_msg;
    auto start_time = this->now();

    RCLCPP_INFO(this->get_logger(), "Waiting for transform between %s and %s", target_frame_id.c_str(), source_frame_id.c_str());

    rclcpp::Rate rate(3.0);
    while (rclcpp::ok() && (this->now() - start_time < rclcpp::Duration::from_seconds(timeout))) {
        if (buffer->canTransform(target_frame_id, source_frame_id, this->get_clock()->now(), rclcpp::Duration::from_seconds(3.0), &error_msg)) {
            received = true;
            break;
        } else {
            RCLCPP_WARN(this->get_logger(), "Error message: %s", error_msg.c_str());
            RCLCPP_INFO(this->get_logger(), "Waiting for transform...");
        }
        rate.sleep();
    }

    if (!received) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for transform after %.1f seconds.", timeout);
    }

    return received;
}

void VisionToMavros::run(void) {
    RCLCPP_INFO(this->get_logger(), "Running Vision To Mavros");

    double timeout = 12.0;

    if (!this->waitForFirstTransform(timeout)) return;

    RCLCPP_INFO(this->get_logger(), "First transform is received");

    this->last_tf_time = this->get_clock()->now();

    auto timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / this->output_rate)),
        std::bind(&VisionToMavros::publishVisionPositionEstimate, this)
    );

    rclcpp::spin(this->shared_from_this());
}

void VisionToMavros::publishVisionPositionEstimate() {
    try {
        transform_stamped = buffer->lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero);

        if (last_tf_time < transform_stamped.header.stamp) {
            last_tf_time = transform_stamped.header.stamp;

            // ? 修正这里：构造 tf2::Vector3 以避免 Foxy 中不支持的 fromMsg
            auto t = transform_stamped.transform.translation;
            position_orig = tf2::Vector3(t.x, t.y, t.z);

            position_body.setX(cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY());
            position_body.setY(-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY());
            position_body.setZ(position_orig.getZ());

            auto q = transform_stamped.transform.rotation;
            quat_cam = tf2::Quaternion(q.x, q.y, q.z, q.w);

            quat_cam_to_body_x.setRPY(roll_cam, 0, 0);
            quat_cam_to_body_y.setRPY(0, pitch_cam, 0);
            quat_cam_to_body_z.setRPY(0, 0, yaw_cam);

            quat_rot_z.setRPY(0, 0, -gamma_world);

            quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
            quat_body.normalize();

            msg_body_pose.header.stamp = transform_stamped.header.stamp;
            msg_body_pose.header.frame_id = transform_stamped.header.frame_id;
            msg_body_pose.pose.position.x = position_body.getX();
            msg_body_pose.pose.position.y = position_body.getY();
            msg_body_pose.pose.position.z = position_body.getZ();
            msg_body_pose.pose.orientation.x = quat_body.getX();
            msg_body_pose.pose.orientation.y = quat_body.getY();
            msg_body_pose.pose.orientation.z = quat_body.getZ();
            msg_body_pose.pose.orientation.w = quat_body.getW();

            camera_pose_publisher->publish(msg_body_pose);

            body_path.header.stamp = msg_body_pose.header.stamp;
            body_path.header.frame_id = msg_body_pose.header.frame_id;
            body_path.poses.push_back(msg_body_pose);
            body_path_publisher->publish(body_path);
        }
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionToMavros>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
