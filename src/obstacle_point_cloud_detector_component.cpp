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

#include "obstacle_point_cloud_detector/obstacle_point_cloud_detector_component.hpp"

#include <algorithm>
#include <vector>

namespace obstacle_point_cloud_detector
{
ObstaclePointCloudDetectorComponent::ObstaclePointCloudDetectorComponent(
  const rclcpp::NodeOptions & options)
: Node("obstacle_point_cloud_detector", options)
{
  declare_parameter("input_topic_name", "velodyne_points");
  get_parameter("input_topic_name", input_topic_name_);
  declare_parameter("obstacle_cloud_topic_name", input_topic_name_ + "/obstacle");
  get_parameter("obstacle_cloud_topic_name", obstacle_cloud_topic_name_);
  declare_parameter("ground_cloud_topic_name", input_topic_name_ + "/ground");
  get_parameter("ground_cloud_topic_name", ground_cloud_topic_name_);
  declare_parameter("cell_size", 0.25);
  get_parameter("cell_size", cell_size_);
  declare_parameter("grid_num", 100);
  get_parameter("grid_num", grid_num_);
  declare_parameter("height_difference_threshold", 0.1);
  get_parameter("height_difference_threshold", height_difference_threshold_);

  cloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_name_, 1,
    std::bind(&ObstaclePointCloudDetectorComponent::cloud_callback, this, std::placeholders::_1));

  obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    obstacle_cloud_topic_name_,
    1);
  ground_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_cloud_topic_name_, 1);
}

void ObstaclePointCloudDetectorComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  CloudXYZINPtr cloud_ptr(new CloudXYZIN);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  CloudXYZINPtr obstacle_cloud_ptr(new CloudXYZIN);
  CloudXYZINPtr ground_cloud_ptr(new CloudXYZIN);

  detect_obstacle_cloud(cloud_ptr, obstacle_cloud_ptr, ground_cloud_ptr);

  obstacle_cloud_ptr->header = cloud_ptr->header;
  obstacle_cloud_ptr->width = obstacle_cloud_ptr->points.size();
  obstacle_cloud_ptr->height = 1;
  ground_cloud_ptr->header = cloud_ptr->header;
  ground_cloud_ptr->width = ground_cloud_ptr->points.size();
  ground_cloud_ptr->height = 1;

  // publish point clouds
  sensor_msgs::msg::PointCloud2 obstacle_cloud_msg;
  pcl::toROSMsg(*obstacle_cloud_ptr, obstacle_cloud_msg);
  obstacle_cloud_pub_->publish(obstacle_cloud_msg);

  sensor_msgs::msg::PointCloud2 ground_cloud_msg;
  pcl::toROSMsg(*ground_cloud_ptr, ground_cloud_msg);
  ground_cloud_pub_->publish(ground_cloud_msg);
}

void ObstaclePointCloudDetectorComponent::detect_obstacle_cloud(
  const CloudXYZINPtr input,
  const CloudXYZINPtr obstacle_cloud,
  const CloudXYZINPtr ground_cloud)
{
  const unsigned int cloud_size = input->points.size();
  if (cloud_size == 0) {
    RCLCPP_ERROR(get_logger(), "input cloud is empty");
    return;
  }

  obstacle_cloud->points.reserve(cloud_size);
  ground_cloud->points.reserve(cloud_size);

  std::vector<std::vector<float>> min_height(grid_num_, std::vector<float>(
      grid_num_,
      input->points[0].z));
  std::vector<std::vector<float>> max_height(grid_num_, std::vector<float>(
      grid_num_,
      input->points[0].z));
  std::vector<std::vector<bool>> points_exist_flag(grid_num_, std::vector<bool>(grid_num_, false));

  const double cell_size_reciprocal = 1.0 / cell_size_;

  for (unsigned int i = 0; i < cloud_size; i++) {
    const int x = (grid_num_ * 0.5) + input->points[i].x * cell_size_reciprocal;
    const int y = (grid_num_ * 0.5) + input->points[i].y * cell_size_reciprocal;
    if (0 <= x && x < grid_num_ && 0 <= y && y <= grid_num_) {
      if (!points_exist_flag[x][y]) {
        min_height[x][y] = input->points[i].z;
        max_height[x][y] = input->points[i].z;
      } else {
        min_height[x][y] = std::min(min_height[x][y], input->points[i].z);
        max_height[x][y] = std::max(max_height[x][y], input->points[i].z);
      }
    }
  }

  for (unsigned int i = 0; i < cloud_size; i++) {
    const int x = (grid_num_ * 0.5) + input->points[i].x * cell_size_reciprocal;
    const int y = (grid_num_ * 0.5) + input->points[i].y * cell_size_reciprocal;
    if (0 <= x && x < grid_num_ && 0 <= y && y <= grid_num_ && points_exist_flag[x][y]) {
      if (max_height[x][y] - min_height[x][y] > height_difference_threshold_) {
        obstacle_cloud->points.emplace_back(input->points[i]);
      } else {
        ground_cloud->points.emplace_back(input->points[i]);
      }
    }
  }
}

}  // namespace obstacle_point_cloud_detector
