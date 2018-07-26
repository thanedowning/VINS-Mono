#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <nav_msgs/Odometry.h>
#include "voxel_grid.h"
#include "int_3d_point.h"

class BlockMap {
public:
  BlockMap();
  ~BlockMap();
  void regPub(ros::NodeHandle &n);
  void initBlockMap();
  void addPoints(const sensor_msgs::PointCloudConstPtr &points_msg, const nav_msgs::Odometry::ConstPtr &pose_msg);
  int const getNumVoxelGrids();
  int const getPointsAdded();
  void publishMap();
  void publish_echo(const sensor_msgs::PointCloudConstPtr &points_msg, const nav_msgs::Odometry::ConstPtr &pose_msg); // For Debugging
private:
  std::vector<VoxelGrid> block_map;
  ros::Publisher pub_block_map;
  ros::Publisher pub_point_cloud_echo;
  unsigned int block_map_pub_seq;
  unsigned int points_added;
  const int edge_dim = 10; // (meters)
  const float voxel_res = 0.5; // (meters)
  const int num_edge_voxels = (int)(edge_dim/voxel_res);
  const int max_range = 20;
  int alt_signum(int in);
  float logistic(float in);
  float logistic_param_x0 = 0.5;
  float logistic_param_L = 1.0;
  float logistic_param_k = 5.0;
  float visible_threshold = 0.23;
  float saturation_max = 1.0;
  float add_constant = 0.1;
  float decay_constant = 0.015;
  float decay_numerator = .0005;
};
