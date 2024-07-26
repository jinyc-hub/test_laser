#include <cmath>
#include <iostream>
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <utility>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle nh;

  ros::Rate ros_loop(20);
  while (ros::ok()) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser",
                                          "base_link"));

    ros::spinOnce();
    ros_loop.sleep();
  }
  return 0;
}
