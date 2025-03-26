#include "icg_ros/ros_camera.h"
#include "icg_ros/icg_ros_interface.h"

#include "icg_msgs/msg/marker_poses.hpp"


#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ICGTestNode : public rclcpp::Node
{
public:
    ICGTestNode()
            : Node("icg_eyeinhand")
    {
        RCLCPP_INFO(this->get_logger(), "ICG Test Node has been started.");
        // Declare and get parameters
        this->declare_parameter<std::string>("config_dir", "/home/ipu/Documents/ips_icg/src/temp/pick_and_place/icg_ros/icg/config");
        this->declare_parameter<std::string>("camera_frame", "D455_color_optical_frame");

        // Create publisher
        pose_publisher_ = this->create_publisher<icg_msgs::msg::MarkerPoses>("pose", 10);
    }

    void initialize() {
        std::string config_dir = this->get_parameter("config_dir").as_string();
        std::string camera_frame = this->get_parameter("camera_frame").as_string();

        // Configure ICG
        icg_ros::ICG_ROS_Config config;
        config.config_dir = config_dir;
        config.body_names.push_back("cube1");
        // config.body_names.push_back("cube2");
        for (const auto &body_name : config.body_names) {
            RCLCPP_INFO(this->get_logger(), "Body name: %s", body_name.c_str());
        }
        config.tracker_name = "tracker";
        config.renderer_geometry_name = "renderer_geometry";
        config.color_camera_name = "color_interface";
        config.color_camera_topic = "/robot1/D455_1/color/image_raw";
        config.color_camera_info_topic = "/robot1/D455_1/color/camera_info";
        config.depth_camera_name = "depth_interface";
        config.depth_camera_topic = "/robot1/D455_1/depth/image_rect_raw";
        config.depth_camera_info_topic = "/robot1/D455_1/depth/camera_info";
        config.color_depth_renderer_name = "color_depth_renderer";
        config.depth_depth_renderer_name = "depth_depth_renderer";
        config.color_viewer_name = "color_viewer";
        config.depth_viewer_name = "depth_viewer";

        // Now safe to use shared_from_this() since node is fully constructed
        interface_ = std::make_shared<icg_ros::ICG_ROS>(shared_from_this(), config);

        tracker_ptr_ = interface_->GetTrackerPtr();

        RCLCPP_INFO(this->get_logger(), "tracker_ptr_ has been configured.");

        // Create subscription
        tracking_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "start_tracking", 10,
                [this](const std_msgs::msg::Bool::SharedPtr msg)
                {
                    tracker_ptr_->ExecuteDetection(msg->data);
                });

        // Create timer
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(3),
                std::bind(&ICGTestNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        static int iteration = 0;
        interface_->RunTrackerProcessOneFrame(iteration);
        icg::Transform3fA temp_transform = tracker_ptr_->body_ptrs()[0]->body2world_pose();
        Eigen::Quaternionf rotation(temp_transform.matrix().block<3, 3>(0, 0));
        Eigen::Vector3f trans(temp_transform.matrix().block<3, 1>(0, 3));

        auto msg = icg_msgs::msg::MarkerPoses();
        msg.header.frame_id = "D455_1_color_optical_frame";
        msg.header.stamp = this->now();

        msg.marker_ids.push_back(0);

        geometry_msgs::msg::Pose pose;
        pose.position.x = trans.x();
        pose.position.y = trans.y();
        pose.position.z = trans.z();
        pose.orientation.x = rotation.x();
        pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z();
        pose.orientation.w = rotation.w();
        msg.poses.push_back(pose);

        pose_publisher_->publish(msg);
        iteration++;
    }

    std::shared_ptr<icg_ros::ICG_ROS> interface_;
    std::shared_ptr<icg::Tracker> tracker_ptr_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_sub_;
    rclcpp::Publisher<icg_msgs::msg::MarkerPoses>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Create the node first
    auto node = std::make_shared<ICGTestNode>();
    
    // Initialize after node creation
    node->initialize();

    RCLCPP_INFO(node->get_logger(), "ICG Test Node has been initialized.");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}