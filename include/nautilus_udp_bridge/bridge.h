#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <tf/tf.h>

#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"

#include <iostream>
#include <memory>
#include <thread>

struct Bridge {
  Bridge(const int port);
  void spin();

private:
  double x{}, y{}, theta{};
  int rate{}, port{};
  std::unique_ptr<Poco::Net::DatagramSocket> server;
  
  ros::NodeHandle n;
  ros::Subscriber odometry_sub;
  void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);

  void receive(Poco::Net::SocketAddress &sender);
  std::atomic<bool> transmit{false};
};