#include "nav_msgs/Odometry.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odometry_demo_pub =
      n.advertise<nav_msgs::Odometry>("/odometry/filtered_odom", 10);

  int rate{};
  n.param<int>("/rate", rate, 1);

  ros::Rate loop_rate(rate);

  int count = 0;
  while (ros::ok()) {
    nav_msgs::Odometry msg{};
    ros::Time now = ros::Time::now();

    msg.header.stamp = now;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = count % 255;
    msg.pose.pose.position.y = -(count++ % 255);
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0;
    msg.pose.pose.orientation.w = 1l;

    odometry_demo_pub.publish(msg);
    //ROS_INFO("[%f, %f, %f]", msg.pose.pose.position.x, msg.pose.pose.position.y,
    //         msg.pose.pose.orientation.x);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}