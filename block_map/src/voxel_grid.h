#pragma once

#include <stdio.h>
#include <queue>
#include <map>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include "int_3d_point.h"

class VoxelGrid {
public:
  VoxelGrid();
  VoxelGrid(Int3DPoint start_point_in, int x_edge_dim_in, int y_edge_dim_in, int z_edge_dim_in, float sat_val_in);
  VoxelGrid(Int3DPoint start_point_in, int edge_dim_in, float sat_val_in);
  ~VoxelGrid();
  void initVoxelGrid();
  void setStartPoint(Int3DPoint start_point_in);
  Int3DPoint const getStartPoint();
  void setXDimention(int x_edge_dim_in);
  void setYDimention(int y_edge_dim_in);
  void setZDimention(int z_edge_dim_in);
  void setEdgeDimention (int edge_dim_in);
  void setSatVal(float sat_val_in);
  int const getXDimention();
  int const getYDimention();
  int const getZDimention();
  int const getEdgeDimention();
  float const getSatVal();
  void addPoint(Int3DPoint point_in, float add_val);
  std::map<int,std::map<int,std::map<int,float>>>* const getVoxelGrid();

private:
  std::map<int,std::map<int,std::map<int,float>>> voxel_grid;
  int x_edge_dim;
  int y_edge_dim;
  int z_edge_dim;
  Int3DPoint start_point;
  float sat_val;
};
