#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <unordered_map>
#include <vector>
#include <cmath>

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
  std::unordered_map<const geometry_msgs::Point32, VoxelGrid> block_map;
  ros::Publisher pub_block_map;
  std_msgs::uint32 block_map_pub_seq; 
  const int edge_dim = 40; // (meters)
  const std_msgs::Float32 voxel_size = 0.2; // (meters)
  const int max_range_to_include_point 80; // This is also in meters and is the
                                            // range from current pose to the
                                            // farthest point that will be counted.
}
