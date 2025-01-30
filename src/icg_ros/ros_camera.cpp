//
// Created on 25-01-27.
//

#include <icg_ros/ros_camera.h>
#include <cv_bridge/cv_bridge.h>

namespace icg_ros
{

RosColorCamera::RosColorCamera(const std::string &name,
                               const std::string &color_image_topic,
                               const std::string &camera_info_topic,
                               std::shared_ptr<rclcpp::Node> node)
    : ColorCamera{name},
      color_image_topic_{color_image_topic},
      camera_info_topic_{camera_info_topic},
      node_{node} {
  // Create a subscription for color images
  color_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      color_image_topic_, 10, std::bind(&RosColorCamera::ImageCallback, this, std::placeholders::_1));
}

void RosColorCamera::RosCamera2Intrinsics(const sensor_msgs::msg::CameraInfo &camera_info, icg::Intrinsics &intrinsics) {
  intrinsics.fu = camera_info.k[0]; // Note: Lowercase 'k' in ROS 2
  intrinsics.fv = camera_info.k[4];
  intrinsics.ppu = camera_info.k[2];
  intrinsics.ppv = camera_info.k[5];
  intrinsics.width = camera_info.width;
  intrinsics.height = camera_info.height;
}

void RosColorCamera::ImageCallback(const sensor_msgs::msg::Image::SharedPtr color) {
  try {
    image_ = cv_bridge::toCvCopy(color, "bgr8")->image;
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

bool RosColorCamera::SetUp() {
  set_up_ = false;

  // 创建订阅器来获取 CameraInfo
  auto camera_info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        RosCamera2Intrinsics(*msg, intrinsics_);
        set_up_ = true; // 设置标志位为 true
      });

  // 等待标志位更新
  rclcpp::Rate rate(10); // 10 Hz
  while (rclcpp::ok() && !set_up_) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  return UpdateImage(true);
}

bool RosColorCamera::UpdateImage(bool synchronized) {
  if (!set_up_ || !rclcpp::ok()) {
    RCLCPP_ERROR(node_->get_logger(), "Set up ros color camera %s first", name_.c_str());
    return false;
  }

  // 创建订阅器来获取图像
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      color_image_topic_, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
          image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception &e) {
          RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        }
      });

  // 等待图像数据准备好
  rclcpp::Rate rate(10); // 10 Hz
  while (rclcpp::ok() && image_.empty()) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  if (image_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not update image from topic %s", color_image_topic_.c_str());
    return false;
  }

  return true;
}

RosDepthCamera::RosDepthCamera(const std::string &name,
                               const std::string &depth_image_topic,
                               const std::string &camera_info_topic,
                               std::shared_ptr<rclcpp::Node> node)
    : DepthCamera{name},
      depth_image_topic_{depth_image_topic},
      camera_info_topic_{camera_info_topic},
      node_{node} {
  // Create a subscription for depth images
  depth_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, 10, std::bind(&RosDepthCamera::ImageCallback, this, std::placeholders::_1));
}

void RosDepthCamera::RosCamera2Intrinsics(const sensor_msgs::msg::CameraInfo &camera_info, icg::Intrinsics &intrinsics) {
  intrinsics.fu = camera_info.k[0];
  intrinsics.fv = camera_info.k[4];
  intrinsics.ppu = camera_info.k[2];
  intrinsics.ppv = camera_info.k[5];
  intrinsics.width = camera_info.width;
  intrinsics.height = camera_info.height;
}


void RosDepthCamera::ImageCallback(const sensor_msgs::msg::Image::SharedPtr depth) {
  try {
    image_ = cv_bridge::toCvCopy(depth, "16UC1")->image;
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

bool RosDepthCamera::SetUp() {
  set_up_ = false;

  // 创建订阅器来监听 CameraInfo 消息
  auto camera_info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        RosCamera2Intrinsics(*msg, intrinsics_);
        set_up_ = true; // 设置标志位
      });

  // 等待标志位更新
  rclcpp::Rate rate(10); // 10 Hz
  while (rclcpp::ok() && !set_up_) {
    rclcpp::spin_some(node_); // 处理订阅回调
    rate.sleep();
  }

  return UpdateImage(true);
}

bool RosDepthCamera::UpdateImage(bool synchronized) {
  if (!set_up_ || !rclcpp::ok()) {
    RCLCPP_ERROR(node_->get_logger(), "Set up ros depth camera %s first", name_.c_str());
    return false;
  }

  // 创建订阅器来监听图像消息
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
          image_ = cv_bridge::toCvCopy(msg, "16UC1")->image;
        } catch (const cv_bridge::Exception &e) {
          RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        }
      });

  // 等待图像更新
  rclcpp::Rate rate(10); // 10 Hz
  while (rclcpp::ok() && image_.empty()) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  if (image_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not update image from topic %s", depth_image_topic_.c_str());
    return false;
  }

  return true;
}

}  // namespace icg_ros