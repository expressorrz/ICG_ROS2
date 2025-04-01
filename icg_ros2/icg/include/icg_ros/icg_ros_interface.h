//
// Created on 22-9-20.
//

#ifndef ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_
#define ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_

// Include necessary standard libraries for general-purpose functionality
#include <iostream>       // For standard I/O operations
#include <utility>        // For utility functions like std::move
#include <vector>         // For managing dynamic arrays
#include <string>         // For string handling
#include <memory>         // For modern C++ memory management (std::shared_ptr)
#include <filesystem>     // For managing file and directory paths

// Include ROS 2 core libraries and Eigen for mathematical operations
#include <rclcpp/rclcpp.hpp>    // ROS 2 core library for node and logging
#include <Eigen/Geometry>       // For geometric computations like transformations

// Include ICG (Iterative Closest Geometry) specific headers
#include <icg/basic_depth_renderer.h>  // For depth rendering
#include <icg/body.h>                  // For handling object bodies
#include <icg/common.h>                // For common utilities
#include <icg/depth_modality.h>        // For depth-related modalities
#include <icg/depth_model.h>           // For depth models
#include <icg/normal_viewer.h>         // For normal visualization
#include <icg/region_modality.h>       // For region-related modalities
#include <icg/region_model.h>          // For region models
#include <icg/renderer_geometry.h>     // For geometry rendering
#include <icg/static_detector.h>       // For static object detection
#include <icg/tracker.h>               // For object tracking

// Include custom ROS 2 camera interface header
#include <icg_ros/ros_camera.h>        // Custom header for ROS camera integration

namespace icg_ros
{

/**
 * brief: Configuration structure for the ICG_ROS class.
 * 
 * This structure holds various parameters and settings required for the ICG_ROS class.
 */
struct ICG_ROS_Config
{
  std::filesystem::path config_dir;          ///< Path to the configuration directory
  std::vector<std::string> body_names;      ///< List of body names to track
  std::string tracker_name;                 ///< Name of the tracker
  std::string renderer_geometry_name;       ///< Name of the geometry renderer
  std::string color_camera_name;            ///< Name of the color camera
  std::string color_camera_topic;           ///< Topic name for the color camera image
  std::string color_camera_info_topic;      ///< Topic name for the color camera info
  std::string depth_camera_name;            ///< Name of the depth camera
  std::string depth_camera_topic;           ///< Topic name for the depth camera image
  std::string depth_camera_info_topic;      ///< Topic name for the depth camera info
  std::string color_viewer_name;            ///< Name of the color viewer
  std::string depth_viewer_name;            ///< Name of the depth viewer
  std::string color_depth_renderer_name;    ///< Name of the color depth renderer
  std::string depth_depth_renderer_name;    ///< Name of the depth renderer
  std::string detector_suffix = "detector"; ///< Suffix for detectors
  std::string region_model_suffix = "region_model"; ///< Suffix for region models
  std::string depth_model_suffix = "depth_model";   ///< Suffix for depth models
  std::string region_modality_suffix = "region_modality"; ///< Suffix for region modalities
  std::string depth_modality_suffix = "depth_modality";   ///< Suffix for depth modalities
  std::string optimizer_suffix = "optimizer"; ///< Suffix for optimizers
};

/**
 * brief: Main class for integrating ICG functionality with ROS 2.
 * 
 * This class manages the configuration, setup, and execution of the ICG tracker in a ROS 2 environment.
 */
class ICG_ROS
{
 public:
  /**
   * brief: Constructor for the ICG_ROS class.
   * 
   * param node A shared pointer to the ROS 2 node.
   * param config A reference to the ICG_ROS_Config structure.
   */
  ICG_ROS(std::shared_ptr<rclcpp::Node> node,
          const ICG_ROS_Config &config);

  /**
   * brief: Retrieve a pointer to the tracker object.
   * 
   * return A shared pointer to the tracker object.
   */
  inline std::shared_ptr<icg::Tracker> &GetTrackerPtr() { return tracker_ptr_; }

  /**
   * brief: Run the tracker for a single frame.
   * 
   * param iteration The current iteration number for the tracking process.
   * return True if the tracker successfully processes the frame, false otherwise.
   */
  bool RunTrackerProcessOneFrame(int iteration);

 private:
  std::shared_ptr<rclcpp::Node> node_;  ///< Shared pointer to the ROS 2 node
  ICG_ROS_Config config_;               ///< Configuration object for the tracker
  std::shared_ptr<icg::Tracker> tracker_ptr_; ///< Pointer to the ICG tracker object
};

}  // namespace icg_ros

#endif //ICG_ROS_ICG_ROS_INCLUDE_ICG_ROS_ICG_ROS_INTERFACE_H_