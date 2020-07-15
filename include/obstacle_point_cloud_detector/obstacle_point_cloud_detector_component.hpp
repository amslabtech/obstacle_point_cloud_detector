// Copyright 2020 amsl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBSTACLE_POINT_CLOUD_DETECTOR__OBSTACLE_POINT_CLOUD_DETECTOR_COMPONENT_HPP_
#define OBSTACLE_POINT_CLOUD_DETECTOR__OBSTACLE_POINT_CLOUD_DETECTOR_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_EXPORT __attribute__ (( \
      dllexport))
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_IMPORT __attribute__ (( \
      dllimport))
  #else
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_EXPORT __declspec(dllexport)
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_BUILDING_DLL
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC \
  OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_EXPORT
  #else
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC \
  OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_IMPORT
  #endif
  #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC_TYPE \
  OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC
  #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_LOCAL
#else
  #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_EXPORT __attribute__ (( \
      visibility("default")))
  #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC __attribute__ (( \
      visibility("default")))
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_LOCAL  __attribute__ (( \
      visibility("hidden")))
  #else
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC
    #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_LOCAL
  #endif
  #define OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>


namespace obstacle_point_cloud_detector
{
class ObstaclePointCloudDetectorComponent : public rclcpp::Node
{
public:
  typedef pcl::PointXYZI PointXYZI;
  typedef pcl::PointCloud<PointXYZI> CloudXYZI;
  typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
  typedef pcl::PointXYZINormal PointXYZIN;
  typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
  typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;

  OBSTACLE_POINT_CLOUD_DETECTOR_OBSTACLE_POINT_CLOUD_DETECTOR_PUBLIC
  explicit ObstaclePointCloudDetectorComponent(const rclcpp::NodeOptions & options);

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void detect_obstacle_cloud(
    const CloudXYZINPtr input, const CloudXYZINPtr obstacle_cloud,
    const CloudXYZINPtr ground_cloud);

private:
  std::string input_topic_name_;
  std::string obstacle_cloud_topic_name_;
  std::string ground_cloud_topic_name_;
  // consider within a square whose length on one side is (cell_size_ * grid_name_) [m]
  double cell_size_;
  int grid_num_;
  double height_difference_threshold_;


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
};
}  //  namespace obstacle_point_cloud_detector

#endif  // OBSTACLE_POINT_CLOUD_DETECTOR__OBSTACLE_POINT_CLOUD_DETECTOR_COMPONENT_HPP_
