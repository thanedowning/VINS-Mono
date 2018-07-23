#include "voxel_grid.h"

VoxelGrid::VoxelGrid() {
  Int3DPoint zero_start_point;
  zero_start_point.x = 0;
  zero_start_point.y = 0;
  zero_start_point.z = 0;
  setStartPoint(zero_start_point);
  setEdgeDimention(0);
  setSatVal(0.0);
}

VoxelGrid::VoxelGrid(Int3DPoint start_point_in, int x_edge_dim_in, int y_edge_dim_in, int z_edge_dim_in, float sat_val_in) {
  setStartPoint(start_point_in);
  setXDimention(x_edge_dim_in);
  setYDimention(y_edge_dim_in);
  setZDimention(z_edge_dim_in);
  setSatVal(sat_val_in);
  initVoxelGrid();
}

VoxelGrid::VoxelGrid(Int3DPoint start_point_in, int edge_dim_in, float sat_val_in) {
  setStartPoint(start_point_in);
  setEdgeDimention(edge_dim_in);
  setSatVal(sat_val_in);
  initVoxelGrid();
}

VoxelGrid::~VoxelGrid() {

}

void VoxelGrid::initVoxelGrid(){
  std::map<int,float> inner_z;
  std::map<int,std::map<int,float>> inner_y;
  for (int i = 0; i < abs(x_edge_dim); i++) {
    for (int j = 0; j < abs(y_edge_dim); j++) {
      for (int k = 0; k < abs(z_edge_dim); k++) {
        inner_z.insert({k, 0.0});
      }
      inner_y.insert({j,inner_z});
    }
    voxel_grid.insert({i,inner_y});
  }
  return;
}

void VoxelGrid::setStartPoint(Int3DPoint start_point_in) {
  start_point = start_point_in;
  return;
}

Int3DPoint const VoxelGrid::getStartPoint() {
 return start_point;
}

void VoxelGrid::setXDimention(int x_edge_dim_in) {
  x_edge_dim = x_edge_dim_in;
  return;
}

void VoxelGrid::setYDimention(int y_edge_dim_in) {
  y_edge_dim = y_edge_dim_in;
  return;
}

void VoxelGrid::setZDimention(int z_edge_dim_in) {
  z_edge_dim = z_edge_dim_in;
  return;
}

void VoxelGrid::setEdgeDimention (int edge_dim_in) {
  x_edge_dim = edge_dim_in;
  y_edge_dim = edge_dim_in;
  z_edge_dim = edge_dim_in;
  return;
}

void VoxelGrid::setSatVal(float sat_val_in) {
  sat_val = sat_val_in;
  return;
}

int const VoxelGrid::getXDimention() {
  return x_edge_dim;
}

int const VoxelGrid::getYDimention() {
  return y_edge_dim;
}

int const VoxelGrid::getZDimention() {
  return z_edge_dim;
}

int const VoxelGrid::getEdgeDimention() {
  if (x_edge_dim == y_edge_dim == z_edge_dim) {
    return x_edge_dim;
  }
  else {
    return 0;
  }
}

float const VoxelGrid::getSatVal() {
  return sat_val;
}

void VoxelGrid::addPoint(Int3DPoint point_in, float add_val) {
  voxel_grid[abs(point_in.x)][abs(point_in.y)][abs(point_in.z)] += add_val;   // This is intended as the input to a logistic function for growth.
  if (voxel_grid[abs(point_in.x)][abs(point_in.y)][abs(point_in.z)] > sat_val) {
    voxel_grid[abs(point_in.x)][abs(point_in.y)][abs(point_in.z)] = sat_val;
  }
  return;
}

std::map<int,std::map<int,std::map<int,float>>>* const VoxelGrid::getVoxelGrid() {
  return &voxel_grid;
}
