#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <queue>
#include <vector>

#include "voxel_grid.h"

class BlockMap {
public:
  BlockMap();
  ~BlockMap();
  void regPub(ros::NodeHandle &n);
  void initBlockMap();
  void addPoints(const nav_msgs::Odometry::ConstPtr &pose_msg, const sensor_msgs::PointCloudConstPtr &points_msg);
  void publishMap();
private:
  std::vector<VoxelGrid> block_map;
  ros::Publisher pub_block_map;
  const int edge_dim = 40; // This should be in meters
  const std_msgs::Float32 voxel_size = 0.2; // This should be in meters
}
