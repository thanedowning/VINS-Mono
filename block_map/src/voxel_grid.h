#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <vector>

class VoxelGrid {
public:
  VoxelGrid();
  VoxelGrid(geometry_msgs::Point32 center_in, int x_dim_in, int y_dim_in, int z_dim_in, std_msgs::Float32 voxel_size_in);
  ~VoxelGrid();
  void initVoxelGrid();
  void setCenterPoint(geometry_msgs::Point32 center_in);
  geometry_msgs::Point32 const getCenterPoint();
  void setXDimention(int x_dim_in);
  void setYDimention(int y_dim_in);
  void setZDimention(int z_dim_in);
  int const getXDimention();
  int const getYDimention();
  int const getZDimention();
  void setVoxelSize(std_msgs::Float32 voxel_size_in);
  std_msgs::Float32 const getVoxelSize();
  void addPoints(const sensor_msgs::PointCloudConstPtr &points_msg);
  sensor_msgs::PointCloudConstPtr getPointCloud(const std_msgs::Header &header);

private:
  std::map<int,std::map<int,std::map<int,std_msgs::Float32>>> voxel_grid;
  geometry_msgs::Point32 center_point;
  int x_dim;
  int y_dim;
  int z_dim;
  std_msgs::Float32 voxel_size;
  int num_voxel_x;
  int num_voxel_y;
  int num_voxel_z;
  sensor_msgs::PointCloud saved_cloud;
  bool cloud_needs_update;
}
