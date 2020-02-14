#include "bridge.h"

using namespace Poco::Net;

Bridge::Bridge(const int port) : port{port} {

  SocketAddress sa(IPAddress(), port);

  server = std::make_unique<DatagramSocket>(sa);

  odometry_sub = n.subscribe("/odometry/filtered_odom", 10,
                             &Bridge::odometry_callback, this);

  n.param<int>("/rate", rate, 1);
  ros::MultiThreadedSpinner spinner(2);
}

void Bridge::receive(SocketAddress &sender) {
  char buffer[2];
  ros::Rate spin_rate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    spin_rate.sleep();

    try {
      int n = server->receiveFrom(buffer, sizeof(buffer) - 1, sender);
    } catch (Poco::Exception &exc) {
      std::cerr << "UDP Server - cannot receive. " << exc.displayText()
                << std::endl;
    }

    if (buffer[0] == '0') {
      transmit = false;
      std::cout << "off" << std::endl;
    } else if (buffer[0] == '1') {
      transmit = true;
      std::cout << "on" << std::endl;
    }
  }
}

void Bridge::spin() {
  SocketAddress sender;
  std::thread receive_thread(&Bridge::receive, this, std::ref(sender));
  ros::Rate spin_rate(rate);
  std::stringstream ss;
  std::string s;
  while (ros::ok()) {

    if (transmit) {
      try {
        ss << "{\"timestamp\": " << ros::Time::now() << ", \"x\": " << x
           << ",\"y\": " << y << ", \"theta\": " << theta << "}";
        s = ss.str();
        ss.str("");
        char cstr[s.size() + 1];
        strcpy(cstr, s.c_str());

        server->sendTo(cstr, sizeof(cstr), sender);
        // std::cout << cstr << std::endl;

      } catch (Poco::Exception &exc) {
        std::cerr << "UDP Server - cannot send " << exc.displayText()
                  << std::endl;
      }
    }
    ros::spinOnce();
    spin_rate.sleep();
  }

  receive_thread.detach();
}

void Bridge::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg) {

  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;

  tf::Quaternion q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};

  tf::Matrix3x3 m{q};
  double temp;
  m.getRPY(temp, temp, theta);

  // ROS_INFO("listening odometry: (%f, %f, %f)", x, y, theta);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "bridge_node");

  Bridge my_server{1234};
  my_server.spin();

  return 0;
}
