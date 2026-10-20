cmake_minimum_required(VERSION 3.15)
project(multisense_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(multisense_lib REQUIRED)
find_package(MultiSense REQUIRED)
find_package(multisense_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# libjpeg-turbo, the libjpeg OpenCV loads: the process's global jpeg_* names stay one ABI.
find_package(JPEG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_environment REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(GST IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
endif()

include_directories(include)

generate_parameter_library(multisense_parameters src/multisense_parameters.yaml)
generate_parameter_library(multisense_init_parameters src/init_parameters.yaml)
generate_parameter_library(thermal_parameters src/thermal_parameters.yaml)

add_library(${PROJECT_NAME}
            src/multisense.cpp
            src/h264_decoder.cpp
            src/aux_jpeg.cpp
            src/ycbcr.cpp
            src/publisher_utilities.cpp
            src/thermal_publisher.cpp)

# Optional so hosts without GStreamer still build; h264_images then refuses to start.
if(GST_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE PkgConfig::GST)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MULTISENSE_ROS_HAVE_GSTREAMER)
else()
  message(WARNING "GStreamer not found: multisense_ros builds without h264_images support")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE JPEG::JPEG)

# NVJPG for aux/image_rect_color/compressed where the Jetson Multimedia API is installed;
# libjpeg-turbo alone elsewhere, and as the fallback where NVJPG fails.
set(MULTISENSE_MMAPI_DIR /usr/src/jetson_multimedia_api)
# The headers must be the ones libnvmm_jpeg was built with: refuse to guess between several.
file(GLOB MULTISENSE_MMAPI_LIBJPEG_DIR LIST_DIRECTORIES true ${MULTISENSE_MMAPI_DIR}/include/libjpeg-*)
list(LENGTH MULTISENSE_MMAPI_LIBJPEG_DIR MULTISENSE_MMAPI_LIBJPEG_DIRS)
find_library(MULTISENSE_NVBUFSURFACE_LIBRARY nvbufsurface
             PATHS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra NO_DEFAULT_PATH)
if(MULTISENSE_MMAPI_LIBJPEG_DIRS GREATER 1)
  message(WARNING "multisense_ros: several Multimedia API libjpeg headers (${MULTISENSE_MMAPI_LIBJPEG_DIR}): "
                  "aux JPEG on libjpeg-turbo only")
elseif(EXISTS ${MULTISENSE_MMAPI_DIR}/include/nvbufsurface.h AND MULTISENSE_MMAPI_LIBJPEG_DIRS EQUAL 1
       AND MULTISENSE_NVBUFSURFACE_LIBRARY)
  target_sources(${PROJECT_NAME} PRIVATE src/nv_jpeg.cpp)
  # The Multimedia API's libjpeg headers for this file alone: every other file sees libjpeg-turbo's.
  set_source_files_properties(src/nv_jpeg.cpp PROPERTIES
      INCLUDE_DIRECTORIES "${MULTISENSE_MMAPI_DIR}/include;${MULTISENSE_MMAPI_LIBJPEG_DIR}")
  target_compile_definitions(${PROJECT_NAME} PRIVATE MULTISENSE_ROS_HAVE_NVJPG)
  # libnvmm_jpeg itself is opened at run time; nv_jpeg.cpp says why.
  target_link_libraries(${PROJECT_NAME} PRIVATE ${MULTISENSE_NVBUFSURFACE_LIBRARY} ${CMAKE_DL_LIBS})
  message(STATUS "multisense_ros: aux JPEG on NVJPG, libjpeg-turbo where it fails")
else()
  message(STATUS "multisense_ros: no Jetson Multimedia API: aux JPEG on libjpeg-turbo")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
                      multisense_init_parameters
                      multisense_parameters
                      thermal_parameters
                      MultiSense)

target_include_directories(${PROJECT_NAME}
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PUBLIC $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
                         angles
                         Eigen3
                         geometry_msgs
                         generate_parameter_library
                         image_geometry
                         image_transport
                         MultiSense
                         multisense_lib
                         multisense_msgs
                         OpenCV
                         rclcpp
                         sensor_msgs
                         stereo_msgs
                         tf2
                         tf2_ros
                         tf2_geometry_msgs)

add_executable(ros_driver src/ros_driver.cpp)
target_link_libraries(ros_driver PUBLIC ${PROJECT_NAME})

ament_target_dependencies(ros_driver PUBLIC
                          rclcpp)

add_executable(aux_compressed_probe tools/aux_compressed_probe.cpp)
target_link_libraries(aux_compressed_probe PRIVATE MultiSense)

install(
  TARGETS
    ${PROJECT_NAME}
    multisense_init_parameters
    multisense_parameters
    thermal_parameters
    ros_driver
  EXPORT multisense_ros-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ros_driver aux_compressed_probe
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(
  DIRECTORY launch materials meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT multisense_ros-targets
  NAMESPACE multisense_ros::
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_h264_packet test/test_h264_packet.cpp)
  target_include_directories(test_h264_packet PRIVATE include)
  ament_target_dependencies(test_h264_packet rclcpp sensor_msgs)
  ament_add_gtest(test_ycbcr test/test_ycbcr.cpp)
  target_link_libraries(test_ycbcr ${PROJECT_NAME})
  ament_add_gtest(test_aux_jpeg test/test_aux_jpeg.cpp)
  target_link_libraries(test_aux_jpeg ${PROJECT_NAME})
  ament_add_gtest(test_aux_jpeg_worker test/test_aux_jpeg_worker.cpp)
  target_link_libraries(test_aux_jpeg_worker ${PROJECT_NAME})
  # Starts nodes: kept off the robot's DDS domains.
  ament_add_gtest(test_aux_jpeg_topic test/test_aux_jpeg_topic.cpp TIMEOUT 60
      ENV ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS= ROS_LOCALHOST_ONLY=
          CYCLONEDDS_URI= DDS_LOCAL_IP=)
  target_link_libraries(test_aux_jpeg_topic ${PROJECT_NAME})
endif()

ament_export_targets(multisense_ros-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_export_dependencies(
  angles
  Eigen3
  geometry_msgs
  generate_parameter_library
  image_geometry
  image_transport
  JPEG
  MultiSense
  multisense_lib
  multisense_msgs
  OpenCV
  rclcpp
  sensor_msgs
  stereo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
