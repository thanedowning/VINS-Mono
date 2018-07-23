#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include "voxel_grid.h"
#include "int_3d_point.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "voxel_grid_test");
  ros::NodeHandle n("~");

  ros::Publisher pub_voxel_grid_test = n.advertise<sensor_msgs::PointCloud>("voxel_grid_test", 10);

  int edge_dim = 10;
  Int3DPoint start_point_in;
  start_point_in.x = 0;
  start_point_in.y = 0;
  start_point_in.z = 0;
  VoxelGrid voxelGrid = VoxelGrid(start_point_in, edge_dim, edge_dim, edge_dim);

  sensor_msgs::PointCloud out_cloud;
  out_cloud.header.frame_id = "world";
  unsigned int out_cloud_seq = 0;
  int x_it_1 = 0;
  int y_it_1 = 0;
  int z_it_1 = 0;

  ros::Rate rate(10);
  while(ros::ok()) {
    out_cloud.header.stamp = ros::Time::now();
    out_cloud.header.seq = out_cloud_seq;
    out_cloud_seq += 1;
    geometry_msgs::Point32 out_point;
    out_point.x = (float)x_it_1;
    out_point.y = (float)y_it_1;
    out_point.z = (float)z_it_1;
    out_cloud.points.push_back(out_point);

    x_it_1 += 1;
    if (x_it_1 >= edge_dim) {
      x_it_1 = 0;
      y_it_1 += 1;
    }
    if (y_it_1 >= edge_dim) {
      y_it_1 = 0;
      z_it_1 += 1;
    }
    if (z_it_1 >= edge_dim) {
      z_it_1 = 0;
    }

    printf("Publishing a point at x=%i, y=%i, z=%i\n", x_it_1, y_it_1, z_it_1);
    pub_voxel_grid_test.publish(out_cloud);
    while(!out_cloud.points.empty()) {
      out_cloud.points.pop_back();
    }
    rate.sleep();
  }
  return 0;
}
