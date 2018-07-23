#include <stdio.h>
#include <queue>
#include <map>
#include <mutex>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include "block_map.h"

BlockMap blockMap;

void cloud_callback(const sensor_msgs::PointCloudConstPtr &points_msg, const nav_msgs::Odometry::ConstPtr &pose_msg) {
  blockMap.addPoints(points_msg, pose_msg);
  printf("Points Added: %i\n", blockMap.getPointsAdded());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "block_map");
  ros::NodeHandle n("~");
  blockMap.regPub(n);

  message_filters::Subscriber<sensor_msgs::PointCloud> pointCloudSub(n, "/vins_estimator/point_cloud", 1);
  message_filters::Subscriber<nav_msgs::Odometry> realPositionSub(n, "/vins_estimator/odometry", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, nav_msgs::Odometry> approx_time;
  message_filters::Synchronizer<approx_time> sync(approx_time(10), pointCloudSub, realPositionSub);
  sync.registerCallback(boost::bind(&cloud_callback, _1, _2));

  ros::Rate rate(2);
  while(ros::ok()) {
    ros::spinOnce();
    blockMap.publishMap();
    printf("Number of grids in the map: %i\n", blockMap.getNumVoxelGrids());
    rate.sleep();
  }
  return 0;
}
