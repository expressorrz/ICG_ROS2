//
// Created by baozhe on 22-9-20.
//

#include "icg_ros/icg_ros_interface.h"
#include <rclcpp/rclcpp.hpp> // ROS 2 core functionality for logging and nodes

namespace icg_ros
{

ICG_ROS::ICG_ROS(std::shared_ptr<rclcpp::Node> node, const icg_ros::ICG_ROS_Config &config)
    : node_{node}, config_{config} {
  constexpr bool kUseDepthViewer = true;
  constexpr bool kMeasureOcclusions = true;
  constexpr bool kModelOcclusions = false;
  constexpr bool kVisualizePoseResult = false;
  constexpr bool kSaveImages = false;

  // Renderer and camera setup
  auto renderer_geometry_ptr_ = std::make_shared<icg::RendererGeometry>(config_.renderer_geometry_name);
  auto color_camera_ptr_ = std::make_shared<RosColorCamera>(config_.color_camera_name,
                                                            config_.color_camera_topic,
                                                            config_.color_camera_info_topic,
                                                            node_);
  auto depth_camera_ptr_ = std::make_shared<RosDepthCamera>(config_.depth_camera_name,
                                                            config_.depth_camera_topic,
                                                            config_.depth_camera_info_topic,
                                                            node_);

  tracker_ptr_ = std::make_shared<icg::Tracker>(config_.tracker_name);
  tracker_ptr_->AddViewer(std::make_shared<icg::NormalColorViewer>(config_.color_viewer_name, color_camera_ptr_, renderer_geometry_ptr_));

  if (kUseDepthViewer) {
    auto depth_viewer_ptr_ = std::make_shared<icg::NormalDepthViewer>(config_.depth_viewer_name,
                                                                      depth_camera_ptr_,
                                                                      renderer_geometry_ptr_,
                                                                      0.3f, 2.0f);
    tracker_ptr_->AddViewer(depth_viewer_ptr_);
  }

  // Depth renderer setup
  auto color_depth_renderer_ptr_ = std::make_shared<icg::FocusedBasicDepthRenderer>(config_.color_depth_renderer_name,
                                                                                    renderer_geometry_ptr_,
                                                                                    color_camera_ptr_);
  auto depth_depth_renderer_ptr_ = std::make_shared<icg::FocusedBasicDepthRenderer>(config_.depth_depth_renderer_name,
                                                                                    renderer_geometry_ptr_,
                                                                                    depth_camera_ptr_);

  // Body and modality setup
  for (const auto &body_name : config_.body_names) {
    auto body_ptr = std::make_shared<icg::Body>(body_name, config_.config_dir / (body_name + ".yaml"));
    renderer_geometry_ptr_->AddBody(body_ptr);
    color_depth_renderer_ptr_->AddReferencedBody(body_ptr);
    depth_depth_renderer_ptr_->AddReferencedBody(body_ptr);

    // Detector setup
    auto detector_ptr = std::make_shared<icg::StaticDetector>(body_name + "_detector",
                                                              config_.config_dir / (body_name + "_detector.yaml"),
                                                              body_ptr);
    tracker_ptr_->AddDetector(detector_ptr);

    // Models and modalities
    auto region_model_ptr = std::make_shared<icg::RegionModel>(body_name + "_region_model",
                                                               body_ptr,
                                                               config_.config_dir / (body_name + "_region_model.bin"));
    auto depth_model_ptr = std::make_shared<icg::DepthModel>(body_name + "_depth_model",
                                                             body_ptr,
                                                             config_.config_dir / (body_name + "_depth_model.bin"));

    auto region_modality_ptr = std::make_shared<icg::RegionModality>(body_name + "_region_modality",
                                                                     body_ptr,
                                                                     color_camera_ptr_,
                                                                     region_model_ptr);
    auto depth_modality_ptr = std::make_shared<icg::DepthModality>(body_name + "_depth_modality",
                                                                   body_ptr,
                                                                   depth_camera_ptr_,
                                                                   depth_model_ptr);

    if (kVisualizePoseResult) {
      region_modality_ptr->set_visualize_pose_result(true);
    }

    if (kMeasureOcclusions) {
      region_modality_ptr->MeasureOcclusions(depth_camera_ptr_);
      depth_modality_ptr->MeasureOcclusions();
    }

    if (kModelOcclusions) {
      region_modality_ptr->ModelOcclusions(color_depth_renderer_ptr_);
      depth_modality_ptr->ModelOcclusions(depth_depth_renderer_ptr_);
    }

    auto optimizer_ptr = std::make_shared<icg::Optimizer>(body_name + "_optimizer");
    optimizer_ptr->AddModality(region_modality_ptr);
    optimizer_ptr->AddModality(depth_modality_ptr);
    tracker_ptr_->AddOptimizer(optimizer_ptr);
  }

  if (!tracker_ptr_->SetUp()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot set up tracker. Exiting...");
    rclcpp::shutdown();
    exit(1);
  }
}

bool ICG_ROS::RunTrackerProcessOneFrame(int iteration) {
  if (!tracker_ptr_->set_up()) {
    RCLCPP_ERROR(node_->get_logger(), "Set up tracker %s first", tracker_ptr_->name().c_str());
    return false;
  }

  auto begin = std::chrono::high_resolution_clock::now();
  if (!tracker_ptr_->UpdateCameras(tracker_ptr_->execute_detection_)) return false;
  if (tracker_ptr_->execute_detection_) {
    if (!tracker_ptr_->ExecuteDetectionCycle(iteration)) return false;
    tracker_ptr_->tracking_started_ = false;
    tracker_ptr_->execute_detection_ = false;
  }
  if (tracker_ptr_->start_tracking_) {
    if (!tracker_ptr_->StartModalities(iteration)) return false;
    tracker_ptr_->tracking_started_ = true;
    tracker_ptr_->start_tracking_ = false;
  }
  if (tracker_ptr_->tracking_started_) {
    if (!tracker_ptr_->ExecuteTrackingCycle(iteration)) return false;
  }
  if (!tracker_ptr_->UpdateViewers(iteration)) return false;
  if (tracker_ptr_->quit_tracker_process_) return true;
  if (!tracker_ptr_->synchronize_cameras_) tracker_ptr_->WaitUntilCycleEnds(begin);

  return true;
}

}  // namespace icg_ros