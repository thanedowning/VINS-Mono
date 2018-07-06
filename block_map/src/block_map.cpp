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
  block_map.insert({init_center, init_grid});
  block_map_pub_seq = 0;
}

void BlockMap::addPoints(const nav_msgs::Odometry::ConstPtr &pose_msg, const sensor_msgs::PointCloudConstPtr &points_msg) {
  for (int iterator = 0; iterator < points_msg->points.size(); iterator++) {
    geometry_msgs::Point32 curr_point = points_msg->points[iterator];
    if (sqrt(pow((curr_point.x),2)+pow((curr_point.y),2)+pow((curr_point.z),2)) > max_range_to_include_point) {
      continue;
    }
    geometry_msgs::Point32 grid_center;
    curr_point.x += pose_msg->pose.pose.position.x;
    curr_point.y += pose_msg->pose.pose.position.y;
    curr_point.z += pose_msg->pose.pose.position.z;
    grid_center.x = ((curr_point.x > 0) - (curr_point.x < 0)) * (round(fabs(curr_point.x)+edge_dim/2)/edge_dim) * edge_dim;
    grid_center.y = ((curr_point.y > 0) - (curr_point.y < 0)) * (round(fabs(curr_point.y)+edge_dim/2)/edge_dim) * edge_dim;
    grid_center.z = ((curr_point.z > 0) - (curr_point.z < 0)) * (round(fabs(curr_point.z)+edge_dim/2)/edge_dim) * edge_dim;
    if (block_map.count(grid_center)) {
      block_map[block_map.find(grid_center)].addAPoint(curr_point);
    }
    else {
      VoxelGrid new_grid = VoxelGrid(grid_center, edge_dim, edge_dim, edge_dim, voxel_size);
      block_map.insert({grid_center, new_grid});
      block_map[block_map.find(grid_center)].addAPoint(curr_point);
    }
  }
}

void BlockMap::publish() {
  sensor_msgs::PointCloud out_cloud;
  out_cloud.header.frame_id = "world";
  out_cloud.header.stamp = ros::Time::now();
  out_cloud.header.seq = block_map_pub_seq;
  block_map_pub_seq += 1;
  for (auto it1 : block_map) {
    it_cloud_ptr = it.second.getPointCloud(out_cloud.header);
    for (int it2 = 0; it2 < it_cloud_ptr->points.size(); it2++) {
      out_cloud.points.push_back(it_cloud_ptr->points[it2]);
    }
    for (int it3 = 0; it3 < it_cloud_ptr->channels.size(); it3++) {
      out_cloud.channels[it3].name = it_cloud_ptr->channels[it3].name;
      for (int it4 = 0; it4 < it_cloud_ptr->channels[it3].values.size(); it4++) {
        out_cloud.channels[it3].values.push_back(it_cloud_ptr->channels[it3].values[it4]);
      }
    }
  }
  pub_block_map.publish(out_cloud);
}
