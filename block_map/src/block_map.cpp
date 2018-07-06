#include "block_map.h"

BlockMap::BlockMap() {
  initBlockMap();
}

BlockMap::~BlockMap() {

}

void BlockMap::regPub(ros::NodeHandle &n) {
  pub_block_map = n.advertise<sensor_msgs::PointCloud>("block_map", 1000);
}

void BlockMap::initBlockMap() {
  geometry_msgs::Point32 init_center;
  init_center.x = 0.0;
  init_center.y = 0.0;
  init_center.z = 0.0;
  VoxelGrid init_grid = VoxelGrid(init_center, edge_dim, edge_dim, edge_dim, voxel_size);
  block_map.push_back(init_grid);
}

void BlockMap::addPoints(const nav_msgs::Odometry::ConstPtr &pose_msg, const sensor_msgs::PointCloudConstPtr &points_msg) {
  for (int first_it = 0; first_it < *points_msg.points.size(); first_it++) {
    bool proper_grid_found = false;
    for (int second_it = 0; second_it < block_map.size(); second_it++) {
      if () {
        
      }
    }
    if (!proper_grid_found) {

    }
  }
}

void BlockMap::publish() {
  sensor_msgs::PointCloud out_cloud;

}
