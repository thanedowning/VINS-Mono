#include <stdio.h>
#include <queue>
#include <map>
#include <mutex>
#include <chrono>
#include <thread>
#include <ros/ros.h>

#include "block_map.h"

using namespace std;

std::mutex buffer;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;

BlockMap blockmap;

void keyframe_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
  buffer.lock();
  pose_buf.push(pose_msg);
  buffer.unlock();
}

void point_cloud_callback(const sensor_msgs::PointCloudConstPtr &points_msg) {
  buffer.lock();
  point_buf.push(point_msg);
  buffer.unlock();
}

void refresh_loop() {
  while(true) {

    std::chrono::milliseconds sleep_length(500);
    std::this_thread::sleep_for(sleep_length);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "block_map");
  ros::NodeHandle n("~");
  blockmap.regPub(n);

  ros::Subscriber pointCloudSub = n.subscribe("/vins_estimator/point_cloud", 1000, point_cloud_callback);
  ros::Subscriber poseSub = n.subscribe("/vins_estimator/keyframe_pose", 1000, keyframe_pose_callback);

  std::thread block_map_refresh_loop;

  block_map_refresh_loop = std::thread(refresh_loop);

  ros::spin();
  return 0;
}
