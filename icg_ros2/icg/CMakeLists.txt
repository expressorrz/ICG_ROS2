cmake_minimum_required(VERSION 3.8)
project(icg_ros)


set(CMAKE_CXX_STANDARD 17)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)



# ===================== Dependencies =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package(OpenMP REQUIRED)

# ===================== Include Directories =====================
include_directories(
    include
    third_party
    ${EIGEN3_INCLUDE_DIR}
    ${ros2_aruco_interfaces_INCLUDE_DIRS}
)

# ===================== Library =====================
set(SOURCES
    src/icg/common.cpp
    src/icg/body.cpp
    src/icg/renderer_geometry.cpp
    src/icg/renderer.cpp
    src/icg/normal_renderer.cpp
    src/icg/basic_depth_renderer.cpp
    src/icg/model.cpp
    src/icg/region_model.cpp
    src/icg/depth_model.cpp
    src/icg/camera.cpp
    src/icg/loader_camera.cpp
    src/icg/viewer.cpp
    src/icg/image_viewer.cpp
    src/icg/normal_viewer.cpp
    src/icg/modality.cpp
    src/icg/region_modality.cpp
    src/icg/depth_modality.cpp
    src/icg/optimizer.cpp
    src/icg/detector.cpp
    src/icg/static_detector.cpp
    src/icg/manual_detector.cpp
    src/icg/refiner.cpp
    src/icg/publisher.cpp
    src/icg/tracker.cpp
    src/icg/generator.cpp
)


add_library(icg STATIC ${SOURCES})

ament_target_dependencies(
    icg
    rclcpp
    std_msgs
    ros2_aruco_interfaces
    sensor_msgs
    OpenCV
    Eigen3
)

# ===================== Executables =====================
add_executable(icg_eyeinhand
    src/icg_eyeinhand.cpp
    src/icg_ros/icg_ros_interface.cpp
    src/icg_ros/ros_camera.cpp
)

ament_target_dependencies(
    icg_eyeinhand
    rclcpp
    std_msgs
    ros2_aruco_interfaces
    sensor_msgs
    geometry_msgs
    cv_bridge
    OpenCV
    Eigen3
)


target_link_libraries(
    icg_eyeinhand
    icg
    GLEW::GLEW
    glfw
    ${OPENGL_gl_LIBRARY}
    OpenGL::GL
    gflags
    OpenMP::OpenMP_CXX
)


add_executable(icg_eyetohand
    src/icg_eyetohand.cpp
    src/icg_ros/icg_ros_interface.cpp
    src/icg_ros/ros_camera.cpp
)

ament_target_dependencies(
    icg_eyetohand
    rclcpp
    std_msgs
    ros2_aruco_interfaces
    sensor_msgs
    geometry_msgs
    cv_bridge
    OpenCV
    Eigen3
)


target_link_libraries(
    icg_eyetohand
    icg
    GLEW::GLEW
    glfw
    ${OPENGL_gl_LIBRARY}
    OpenGL::GL
    gflags
    OpenMP::OpenMP_CXX
)

# ===================== Install =====================
install(
    TARGETS icg icg_eyeinhand
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS icg icg_eyetohand
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.h"
)

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()