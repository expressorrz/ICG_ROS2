// SPDX-License-Identifier: MIT
// Copyright (c) 2022 Manuel Stoiber, German Aerospace Center (DLR)

#include <icg/manual_detector.h>

#include <opencv2/core/eigen.hpp>

namespace icg {

PointDetector::PointDetector(const cv::Mat& image,
                             const std::string& window_name,
                             const std::filesystem::path& detector_image_path)
    : image_{image},
      window_name_{window_name},
      detector_image_path_{detector_image_path} {}

void PointDetector::set_image(const cv::Mat& image) { image_ = image; }

void PointDetector::set_window_name(const std::string& window_name) {
  window_name_ = window_name;
}

void PointDetector::set_detector_image_path(
    const std::filesystem::path& detector_image_path) {
  detector_image_path_ = detector_image_path;
}

void PointDetector::MouseClickCallback(int event, int x, int y, int flags,
                                       void* param) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    auto point_detector{static_cast<PointDetector*>(param)};
    if (point_detector->detected_points_.size() >= 4) return;

    point_detector->detected_points_.push_back(cv::Point2f{float(x), float(y)});
    cv::circle(point_detector->viewer_image_, cv::Point2i{x, y}, 3,
               cv::Vec3b{255, 0, 255}, cv::FILLED, cv::LINE_8);
    cv::imshow(point_detector->window_name_, point_detector->viewer_image_);
  }
}

bool PointDetector::DetectPoints() {
  // Load detector image if it exists
  cv::Mat unmodified_viewer_image{image_};
  auto detector_image{cv::imread(detector_image_path_.string())};
  if (!detector_image.empty()) {
    int scale_factor = 5;
    cv::resize(
        detector_image, detector_image,
        cv::Size{
            image_.rows * image_.cols / (scale_factor * detector_image.cols),
            image_.rows * image_.cols / (scale_factor * detector_image.rows)});
    cv::Rect roi{cv::Point{0, 0}, detector_image.size()};
    detector_image.copyTo(unmodified_viewer_image(roi));
  }

  // Create window and callback
  unmodified_viewer_image.copyTo(viewer_image_);
  cv::namedWindow(window_name_, 1);
  cv::setMouseCallback(window_name_, PointDetector::MouseClickCallback, this);

  // Iterate until 4 points are detected or quit
  while (true) {
    imshow(window_name_, viewer_image_);
    char key = cv::waitKey(1000);
    if (key == 'c' || key == 8) {
      detected_points_.clear();
      unmodified_viewer_image.copyTo(viewer_image_);
    } else if (key == 'q' || key == 13) {
      cv::destroyWindow(window_name_);
      return detected_points_.size() >= 4;
    }
  }
}

const cv::Mat& PointDetector::image() const { return image_; }

const std::string& PointDetector::window_name() const { return window_name_; }

const std::filesystem::path& PointDetector::detector_image_path() const {
  return detector_image_path_;
}

const std::vector<cv::Point2f>& PointDetector::detected_points() const {
  return detected_points_;
}

ManualDetector::ManualDetector(
    const std::string& name, const std::shared_ptr<icg::Body>& body_ptr,
    const std::shared_ptr<icg::ColorCamera>& color_camera_ptr,
    const std::vector<cv::Point3f>& reference_points,
    const std::filesystem::path& detector_image_path)
    : Detector{name},
      body_ptr_{body_ptr},
      color_camera_ptr_{color_camera_ptr},
      reference_points_{reference_points},
      detector_image_path_{detector_image_path} {}

ManualDetector::ManualDetector(
    const std::string& name, const std::filesystem::path& metafile_path,
    const std::shared_ptr<icg::Body>& body_ptr,
    const std::shared_ptr<icg::ColorCamera>& color_camera_ptr)
    : Detector{name, metafile_path},
      body_ptr_{body_ptr},
      color_camera_ptr_{color_camera_ptr} {}

bool ManualDetector::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!body_ptr_->set_up()) {
    std::cerr << "Body " << body_ptr_->name() << " was not set up" << std::endl;
    return false;
  }
  if (!color_camera_ptr_->set_up()) {
    std::cerr << "Color camera " << color_camera_ptr_->name()
              << " was not set up" << std::endl;
    return false;
  }

  set_up_ = true;
  return true;
}

void ManualDetector::set_body_ptr(const std::shared_ptr<Body>& body_ptr) {
  body_ptr_ = body_ptr;
  set_up_ = false;
}

void ManualDetector::set_color_camera_ptr(
    const std::shared_ptr<icg::ColorCamera>& color_camera_ptr) {
  color_camera_ptr_ = color_camera_ptr;
  set_up_ = false;
}

void ManualDetector::set_reference_points(
    const std::vector<cv::Point3f>& reference_points) {
  reference_points_ = reference_points;
}

void ManualDetector::set_detector_image_path(
    const std::filesystem::path& detector_image_path) {
  detector_image_path_ = detector_image_path;
}

bool ManualDetector::DetectBody() {
  if (!set_up_) {
    std::cerr << "Set up manual detector " << name_ << " first" << std::endl;
    return false;
  }

  // Detect points and calculate rotation and translation
  PointDetector point_detector{color_camera_ptr_->image(),
                               "Detect " + body_ptr_->name(),
                               detector_image_path_};
  if (!point_detector.DetectPoints()) return false;
  cv::Mat rotation, translation;
  cv::solvePnP(reference_points_, point_detector.detected_points(),
               GetCameraMatrixFromIntrinsics(), {}, rotation, translation,
               false, cv::SOLVEPNP_ITERATIVE);

  // Transform translation to Eigen
  icg::Transform3fA body2world_pose;
  Eigen::Vector3f eigen_translation;
  cv::cv2eigen(translation, eigen_translation);
  body2world_pose.translation() = eigen_translation;

  // Transform rotation to Eigen
  cv::Mat rotation_matrix;
  Eigen::Matrix3f eigen_rotation;
  cv::Rodrigues(rotation, rotation_matrix);
  cv::cv2eigen(rotation_matrix, eigen_rotation);
  body2world_pose.linear() = eigen_rotation;

  body_ptr_->set_body2world_pose(body2world_pose);
  return true;
}

const std::shared_ptr<Body> &ManualDetector::body_ptr() const {
  return body_ptr_;
}

const std::shared_ptr<icg::ColorCamera>& ManualDetector::color_camera_ptr()
    const {
  return color_camera_ptr_;
}

const std::vector<cv::Point3f>& ManualDetector::reference_points() const {
  return reference_points_;
}

const std::filesystem::path& ManualDetector::detector_image_path() const {
  return detector_image_path_;
}

std::vector<std::shared_ptr<Body>> ManualDetector::body_ptrs() const {
  return {body_ptr_};
}

std::shared_ptr<icg::Camera> ManualDetector::camera_ptr() const {
  return color_camera_ptr_;
}

cv::Mat ManualDetector::GetCameraMatrixFromIntrinsics() const {
  return cv::Mat_<float>(3, 3) << color_camera_ptr_->intrinsics().fu, 0,
         color_camera_ptr_->intrinsics().ppu, 0,
         color_camera_ptr_->intrinsics().fv,
         color_camera_ptr_->intrinsics().ppv, 0, 0, 1.;
}

bool ManualDetector::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!ReadRequiredValueFromYaml(fs, "reference_points", &reference_points_)) {
    std::cerr << "Could not read all required body parameters from " << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "detector_image_path", &detector_image_path_);
  fs.release();

  // Process parameters
  if (detector_image_path_.is_relative())
    detector_image_path_ = metafile_path_.parent_path() / detector_image_path_;
  return true;
}

}  // namespace icg
