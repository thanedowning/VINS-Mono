#include "block_map.h"

BlockMap::BlockMap() {
  initBlockMap();
}

BlockMap::~BlockMap() {

}

void BlockMap::regPub(ros::NodeHandle &n) {
  pub_block_map = n.advertise<sensor_msgs::PointCloud>("block_map", 100);
  pub_point_cloud_echo = n.advertise<sensor_msgs::PointCloud>("point_cloud_echo", 100); // This is for debugging purposes.
}

void BlockMap::initBlockMap() {
  Int3DPoint init_center;
  init_center.x = 0;
  init_center.y = 0;
  init_center.z = 0;
  VoxelGrid init_grid0 = VoxelGrid(init_center, num_edge_voxels, saturation_max);
  block_map.push_back(init_grid0);
  block_map_pub_seq = 0;
  points_added = 0;
}

void BlockMap::addPoints(const sensor_msgs::PointCloudConstPtr &points_msg, const nav_msgs::Odometry::ConstPtr &pose_msg) {
  geometry_msgs::Point32 curr_point;
  Int3DPoint point_it;
  Int3DPoint start_point_it;
  Int3DPoint add_point;
  for (int it1 = 0; it1 < points_msg->points.size(); it1++) {
    bool added_point = false;
    curr_point = points_msg->points[it1];
    if (sqrt(pow((curr_point.x - pose_msg->pose.pose.position.x),2)
        +pow((curr_point.y - pose_msg->pose.pose.position.y),2)
        +pow((curr_point.z - pose_msg->pose.pose.position.z),2)) > max_range) {
      continue;
    }
    point_it.x = int{trunc(curr_point.x/voxel_res)};
    point_it.y = int{trunc(curr_point.y/voxel_res)};
    point_it.z = int{trunc(curr_point.z/voxel_res)};
    if (point_it.x == 0 && point_it.y == 0 && point_it.z == 0) {
      block_map[0].addPoint(point_it, add_constant);
      points_added += 1;
      continue;
    }
    start_point_it.x = div(point_it.x,num_edge_voxels).quot*num_edge_voxels;
    start_point_it.y = div(point_it.y,num_edge_voxels).quot*num_edge_voxels;
    start_point_it.z = div(point_it.z,num_edge_voxels).quot*num_edge_voxels;
    int x_edge_dim_it = alt_signum(point_it.x)*num_edge_voxels;
    int y_edge_dim_it = alt_signum(point_it.y)*num_edge_voxels;
    int z_edge_dim_it = alt_signum(point_it.z)*num_edge_voxels;
    for (int it2 = 0; it2 < block_map.size(); it2++) {
      if (start_point_it == block_map[it2].getStartPoint() &&
          x_edge_dim_it == block_map[it2].getXDimention() &&
          y_edge_dim_it == block_map[it2].getYDimention() &&
          z_edge_dim_it == block_map[it2].getZDimention()) {
        add_point.x = point_it.x - start_point_it.x;
        add_point.y = point_it.y - start_point_it.y;
        add_point.z = point_it.z - start_point_it.z;
        block_map[it2].addPoint(add_point, add_constant);
        added_point = true;
        points_added += 1;
        break;
      }
    }
    if (!added_point) {
      VoxelGrid new_voxel_grid = VoxelGrid(start_point_it, x_edge_dim_it, y_edge_dim_it, z_edge_dim_it, saturation_max);
      block_map.push_back(new_voxel_grid);
      add_point.x = point_it.x - start_point_it.x;
      add_point.y = point_it.y - start_point_it.y;
      add_point.z = point_it.z - start_point_it.z;
      block_map.back().addPoint(add_point, add_constant);
      points_added += 1;
      printf("Added new VoxelGrid with start point at x = %i, y = %i, z = %i\n"
             "Xdim = %i, Ydim = %i, Zdim = %i\n"
             "When fed a point interpreted as  x = %i, y = %i, z = %i\n\n",
             start_point_it.x, start_point_it.y, start_point_it.z,
             block_map.back().getXDimention(),block_map.back().getYDimention(),block_map.back().getZDimention(),
             point_it.x, point_it.y, point_it.z);
    }
  }
}

int const BlockMap::getNumVoxelGrids() {
  return block_map.size();
}

int const BlockMap::getPointsAdded() {
  return points_added;
}

void BlockMap::publish_echo(const sensor_msgs::PointCloudConstPtr &points_msg, const nav_msgs::Odometry::ConstPtr &pose_msg) {
  geometry_msgs::Point32 curr_point;
  sensor_msgs::PointCloud out_cloud;
  out_cloud.header = points_msg->header;
  for (int it1 = 0; it1 < points_msg->points.size(); it1++) {
    curr_point = points_msg->points[it1];
    out_cloud.points.push_back(curr_point);
  }
  pub_point_cloud_echo.publish(out_cloud);
}

void BlockMap::publishMap() {
  sensor_msgs::PointCloud out_cloud;
  out_cloud.header.frame_id = "world";
  out_cloud.header.stamp = ros::Time::now();
  out_cloud.header.seq = block_map_pub_seq;
  block_map_pub_seq += 1;
  sensor_msgs::ChannelFloat32 new_channel;
  new_channel.name = "Occupancy Prob";
  geometry_msgs::Point32 new_point;
  Int3DPoint start_point_it;
  std::map<int,std::map<int,std::map<int,float>>>* voxel_grid_it;
  float logistic_out;
  for (int it1 = 0; it1 < block_map.size(); it1++) {
    start_point_it = block_map[it1].getStartPoint();
    voxel_grid_it = block_map[it1].getVoxelGrid();
    for (int i = 0; i < num_edge_voxels; i++) {
      for (int j = 0; j < num_edge_voxels; j++) {
        for (int k = 0; k < num_edge_voxels; k++) {
          if ((*voxel_grid_it)[i][j][k] > 0.0) {
            logistic_out = logistic((*voxel_grid_it)[i][j][k]);
            if (logistic_out > visible_threshold) {
              new_point.x = voxel_res*(alt_signum(block_map[it1].getXDimention())*i+start_point_it.x);
              new_point.y = voxel_res*(alt_signum(block_map[it1].getYDimention())*j+start_point_it.y);
              new_point.z = voxel_res*(alt_signum(block_map[it1].getZDimention())*k+start_point_it.z);
              out_cloud.points.push_back(new_point);
              new_channel.values.push_back(logistic(logistic_out));
            }
            (*voxel_grid_it)[i][j][k] -= decay_numerator/((*voxel_grid_it)[i][j][k]);
            if ((*voxel_grid_it)[i][j][k] < 0.0) {
              (*voxel_grid_it)[i][j][k] = 0.0;
            }
          }
        }
      }
    }
  }
  out_cloud.channels.push_back(new_channel);
  pub_block_map.publish(out_cloud);
}

int BlockMap::alt_signum(int in) {
  return (((in>0)-(in<0))!=0)?((in>0)-(in<0)):1; // Returns 1 if zero or positive, -1 if negative.
}

float BlockMap::logistic(float in) {
  return (logistic_param_L/(1.0 + exp(-logistic_param_k*(in-logistic_param_x0))));
}
