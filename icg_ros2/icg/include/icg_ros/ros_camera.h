//
// Created on 25-01-27.
//

#ifndef ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ROS_CAMERA_H_
#define ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ROS_CAMERA_H_

// Standard library headers for memory management, strings, and file paths
#include <memory>
#include <string>
#include <icg/common.h>
#include <icg/camera.h>

// ROS 2 core headers for node management and message handling
#include <rclcpp/rclcpp.hpp>

// ROS 2 message types for camera information and images
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace icg_ros
{

/**
 * brief: Class for handling a ROS 2 color camera, subscribing to image topics and processing data.
 */
class RosColorCamera : public icg::ColorCamera
{
 public:
  /**
   * brief: Constructor to initialize the color camera.
   * param name The name of the camera.
   * param color_image_topic The ROS 2 topic name for the color image.
   * param camera_info_topic The ROS 2 topic name for camera info.
   * param node A shared pointer to the ROS 2 node.
   */
  RosColorCamera(const std::string &name,
                 const std::string &color_image_topic,
                 const std::string &camera_info_topic,
                 std::shared_ptr<rclcpp::Node> node);

  /**
   * brief: Set up the camera before use.
   * return True if successful, false otherwise.
   */
  bool SetUp() override;

  /**
   * brief: Update the image data by subscribing to the topic.
   * param synchronized If true, updates are synchronized.
   * return True if successful, false otherwise.
   */
  bool UpdateImage(bool synchronized) override;

 private:
  /**
   * brief: Callback function for processing color images.
   * param color The received image message.
   */
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr color);

  /**
   * brief: Convert ROS 2 CameraInfo message to icg::Intrinsics format.
   * param camera_info The camera info message.
   * param intrinsics The resulting camera intrinsics.
   */
  void RosCamera2Intrinsics(const sensor_msgs::msg::CameraInfo &camera_info, icg::Intrinsics &intrinsics);

  std::shared_ptr<rclcpp::Node> node_;  ///< Pointer to the ROS 2 node.
  std::string color_image_topic_;       ///< Topic name for color images.
  std::string camera_info_topic_;       ///< Topic name for camera info.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_; ///< Subscription for color images.
};

/**
 * brief: Class for handling a ROS 2 depth camera, subscribing to depth image topics and processing data.
 */
class RosDepthCamera : public icg::DepthCamera
{
 public:
  /**
   * brief: Constructor to initialize the depth camera.
   * param name The name of the camera.
   * param depth_image_topic The ROS 2 topic name for the depth image.
   * param camera_info_topic The ROS 2 topic name for camera info.
   * param node A shared pointer to the ROS 2 node.
   */
  RosDepthCamera(const std::string &name,
                 const std::string &depth_image_topic,
                 const std::string &camera_info_topic,
                 std::shared_ptr<rclcpp::Node> node);

  /**
   * brief: Set up the camera before use.
   * return True if successful, false otherwise.
   */
  bool SetUp() override;

  /**
   * brief: Update the depth image data by subscribing to the topic.
   * param synchronized If true, updates are synchronized.
   * return True if successful, false otherwise.
   */
  bool UpdateImage(bool synchronized) override;

 private:
  /**
   * brief: Callback function for processing depth images.
   * param depth The received depth image message.
   */
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr depth);

  /**
   * brief: Convert ROS 2 CameraInfo message to icg::Intrinsics format.
   * param camera_info The camera info message.
   * param intrinsics The resulting camera intrinsics.
   */
  void RosCamera2Intrinsics(const sensor_msgs::msg::CameraInfo &camera_info, icg::Intrinsics &intrinsics);

  std::shared_ptr<rclcpp::Node> node_;  ///< Pointer to the ROS 2 node.
  std::string depth_image_topic_;       ///< Topic name for depth images.
  std::string camera_info_topic_;       ///< Topic name for camera info.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_; ///< Subscription for depth images.
};

}  // namespace icg_ros

#endif //ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ROS_CAMERA_H_