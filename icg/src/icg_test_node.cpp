#include "icg_ros/ros_camera.h"
#include "icg_ros/icg_ros_interface.h"

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
            : Node("icg_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "ICG Test Node has been started.");
        // Declare and get parameters
        this->declare_parameter<std::string>("config_dir", "/home/ipu/Documents/robot_learning/icg_ros2/config");
        this->declare_parameter<std::string>("camera_frame", "D455_color_optical_frame");

        // Create publisher
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    }

    void initialize() {
        std::string config_dir = this->get_parameter("config_dir").as_string();
        std::string camera_frame = this->get_parameter("camera_frame").as_string();

        // Configure ICG
        icg_ros::ICG_ROS_Config config;
        config.config_dir = config_dir;
        config.body_names.push_back("cube1");
        config.body_names.push_back("cube2");
        for (const auto &body_name : config.body_names) {
            RCLCPP_INFO(this->get_logger(), "Body name: %s", body_name.c_str());
        }
        config.tracker_name = "tracker";
        config.renderer_geometry_name = "renderer_geometry";
        config.color_camera_name = "color_interface";
        config.color_camera_topic = "/camera/D455/color/image_raw";
        config.color_camera_info_topic = "/camera/D455/color/camera_info";
        config.depth_camera_name = "depth_interface";
        config.depth_camera_topic = "/camera/D455/depth/image_rect_raw";
        config.depth_camera_info_topic = "/camera/D455/depth/camera_info";
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

        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.frame_id = "D455_color_optical_frame";
        pose.header.stamp = this->now();
        pose.pose.orientation.w = rotation.w();
        pose.pose.orientation.x = rotation.x();
        pose.pose.orientation.y = rotation.y();
        pose.pose.orientation.z = rotation.z();
        pose.pose.position.x = trans.x();
        pose.pose.position.y = trans.y();
        pose.pose.position.z = trans.z();

        // RCLCPP_INFO(this->get_logger(), "Pose: [Position: (%f, %f, %f), Orientation: (%f, %f, %f, %f)]",
        //         pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        //         pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

        
        pose_publisher_->publish(pose);
        iteration++;
    }

    std::shared_ptr<icg_ros::ICG_ROS> interface_;
    std::shared_ptr<icg::Tracker> tracker_ptr_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
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