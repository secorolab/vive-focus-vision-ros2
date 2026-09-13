cmake_minimum_required(VERSION 3.16)
project(vr_mujoco CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Same user cache mj_kdl_wrapper fetches into, so both link one MuJoCo. No system paths are
# searched: a stray /opt/mujoco-3.8.0 would silently mismatch the wrapper's ABI.
set(VR_MUJOCO_DIR "$ENV{HOME}/.cache/mj_kdl_wrapper/mujoco-3.9.0"
    CACHE PATH "MuJoCo install root (include/ and lib/)")
find_library(MUJOCO_LIBRARY mujoco PATHS "${VR_MUJOCO_DIR}/lib" NO_DEFAULT_PATH REQUIRED)
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS "${VR_MUJOCO_DIR}/include" NO_DEFAULT_PATH
          REQUIRED)
message(STATUS "MuJoCo: ${MUJOCO_LIBRARY}")

add_executable(scene_export
  src/scene_export.cpp
  src/gltf_writer.cpp
  src/geom_mesh.cpp
)
target_include_directories(scene_export PRIVATE include "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(scene_export PRIVATE "${MUJOCO_LIBRARY}")

add_executable(stream_node src/stream_node.cpp)
target_include_directories(stream_node PRIVATE include "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(stream_node PRIVATE
  "${MUJOCO_LIBRARY}"
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2_ros::tf2_ros
)

# MuJoCo ships as a plain .so outside the ament prefix; without this the installed binaries only
# run from a shell that exports LD_LIBRARY_PATH.
set_target_properties(scene_export stream_node PROPERTIES
  BUILD_RPATH "${VR_MUJOCO_DIR}/lib"
  INSTALL_RPATH "${VR_MUJOCO_DIR}/lib"
)

install(TARGETS scene_export stream_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
