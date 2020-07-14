# obstalce_point_cloud_detector
Obstacle and ground point cloud detection for ROS2

# Requirement

- ROS2 (tested on foxy) 

# Install and build
```
cd your_ros2_ws/src
git clone https://github.com/amslabtech/obstacle_point_cloud_detector.git
vcs install < obstacle_point_cloud_detector/.rosinstall 
cd your_ros2_ws
rosdep install -i -r -y --from-paths src
colcon build 
```

---
### *Note* 
This repo was implemented with reference to the algorithm in [velodyne_heigthmap](https://github.com/jack-oquin/velodyne_height_map).
