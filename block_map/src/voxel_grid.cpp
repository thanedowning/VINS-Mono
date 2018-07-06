#include "voxel_grid.h"

VoxelGrid::VoxelGrid() {
  geometry_msgs::Point32 zero_center;
  zero_center.x = 0.0;
  zero_center.y = 0.0;
  zero_center.z = 0.0;
  setCenterPoint(zero_center);
  setXDimention(0);
  setYDimention(0);
  setZDimention(0);
  setVoxelSize(0.0);
  cloud_needs_update = true;
}

VoxelGrid::VoxelGrid(geometry_msgs::Point32 center_in, int x_dim_in, int y_dim_in, int z_dim_in, std_msgs::Float32 voxel_size_in) {
  setCenterPoint(center_in);
  setXDimention(x_dim_in);
  setYDimention(y_dim_in);
  setZDimention(z_dim_in);
  setVoxelSize(voxel_size_in);
  initVoxelGrid();
  cloud_needs_update = true;
}

VoxelGrid::~VoxelGrid() {

}

void VoxelGrid::initVoxelGrid(){
  if (x_dim <= 0) {
    printf("Please set a valid X dimention (must be a positive integer)\n");
    ROS_BREAK();
  }
  else if (y_dim <= 0) {
    printf("Please set a valid Y dimention (must be a positive integer)\n");
    ROS_BREAK();
  }
  else if (z_dim <= 0) {
    printf("Please set a valid Z dimention (must be a positive integer)\n");
    ROS_BREAK();
  }
  else if (voxel_size <= 0.0) {
    printf("Please set a valid voxel size (must be a positive floating point number)\n");
    ROS_BREAK();
  }
  num_voxel_x = int(x_dim/voxel_size);
  num_voxel_y = int(y_dim/voxel_size);
  num_voxel_z = int(z_dim/voxel_size);
  if ((num_voxel_x%2)||(num_voxel_y%2)||(num_voxel_z%2)) {
    printf("Please set X, Y, and Z dimentions and voxel size such \n
            that (dimention)/(voxel size) is even.\n");
    ROS_BREAK();
  }   // This keeps us from running into integer math errors when we oraganize points into voxel bins.
  std::map<int,std_msgs::Float32> inner_z;
  std::map<int,std::map<int,std_msgs::Float32>> inner_y;
  for (int i = 0; i < num_voxel_x; i++) {
    for (int j = 0; j < num_voxel_y; j++) {
      for (int k = 0; k < num_voxel_z; k++) {
        inner_z.insert({k, 0.0});
      }
      inner_y.insert({j,inner_z});
    }
    voxel_grid.insert({i,inner_y});
  }
  return;
}

void VoxelGrid::setCenterPoint(geometry_msgs::Point32 center_in) {
  center_point = center_in;
  return;
}

geometry_msgs::Point32 VoxelGrid::getCenterPoint() {
 return center_point;
}

void VoxelGrid::setXDimention(int x_dim_in) {
  x_dim = x_dim_in;
  return;
}

void VoxelGrid::setYDimention(int y_dim_in) {
  y_dim = y_dim_in;
  return;
}

void VoxelGrid::setZDimention(int z_dim_in) {
  z_dim = z_dim_in;
  return;
}

int VoxelGrid::getXDimention() {
  return x_dim;
}

int VoxelGrid::getYDimention() {
  return y_dim;
}

int VoxelGrid::getZDimention() {
  return z_dim;
}

void VoxelGrid::setVoxelSize(std_msgs::Float32 voxel_size_in) {
  voxel_size = voxel_size_in;
  return;
}

std_msgs::Float32 VoxelGrid::getVoxelSize() {
  return voxel_size;
}

void VoxelGrid::addPoints(const std::vector<geometry_msgs::Point32> &points_vec) {
  for (int iterator = 0; iterator < point_vec.size(); iterator++) {
    geometry_msgs::Point32 it_point = point_vec[iterator];
    int x_coord = int((it_point.x-center_point.x)/voxel_size) + (num_voxel_x/2); // These lines may introduce some weird
    int y_coord = int((it_point.y-center_point.y)/voxel_size) + (num_voxel_y/2); // integer math inconsistencies if
    int z_coord = int((it_point.z-center_point.z)/voxel_size) + (num_voxel_z/2); // num_voxel _x, _y, or _z are odd.
    if (voxel_grid[x_coord][y_coord][z_coord] < 1.0) {
      voxel_grid[x_coord][y_coord][z_coord] += 0.05   // This just implements a linear probability growth;
                                                      // replace with a more sophisticated function in the future.
    }
  }
  cloud_needs_update = true;
  return;
}

void VoxelGrid::addAPoint(const geometry_msgs::Point32 &a_point) {
  int x_coord = int((a_point.x-center_point.x)/voxel_size) + (num_voxel_x/2); // These lines may introduce some weird
  int y_coord = int((a_point.y-center_point.y)/voxel_size) + (num_voxel_y/2); // integer math inconsistencies if
  int z_coord = int((a_point.z-center_point.z)/voxel_size) + (num_voxel_z/2); // num_voxel _x, _y, or _z are odd.
  if (voxel_grid[x_coord][y_coord][z_coord] < 1.0) {
    voxel_grid[x_coord][y_coord][z_coord] += 0.05   // This just implements a linear probability growth; replace with a
                                                    // more sophisticated function in the future.  1.0 modulo this constant
                                                    // must equal 0 for the probability stuff to work properly.
  }
  cloud_needs_update = true;
  return;
}

sensor_msgs::PointCloudConstPtr VoxelGrid::getPointCloud(const std_msgs::Header &header) {
  if (cloud_needs_update) {
    sensor_msgs::PointCloud out_cloud;
    out_cloud.header = header;
    sensor_msgs::ChannelFloat32 prob_channel;
    prob_channel.name = "Occupancy Probability";
    for (int i = 0; i < num_voxel_x; i++) {
      for (int j = 0; j < num_voxel_y; j++) {
        for (int k = 0; k < num_voxel_z; k++) {
          if (voxel_grid[i,j,k] > 0.0) {
            geometry_msgs::Point32 p;
            p.x = center_point.x - (num_voxel_x/2)*voxel_size + i*voxel_size; // These lines may introduce some weird
            p.y = center_point.y - (num_voxel_y/2)*voxel_size + j*voxel_size; // integer math inconsistencies if
            p.z = center_point.z - (num_voxel_z/2)*voxel_size + k*voxel_size; // num_voxel _x, _y, or _z are odd.
            out_cloud.points.push_back(p);
            prob_channel.values.push_back(voxel_grid[i,j,k]);
          }
        }
      }
    }
    out_cloud.channels.push_back(prob_channel);
    saved_cloud = out_cloud;
    cloud_needs_update = false;
    return &saved_cloud;
  }
  else {
    return &saved_cloud;
  }
}
