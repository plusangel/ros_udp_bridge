# ROS UDP bridge

This repository engages a Poco net library with ROS to communicate with a non-ROS UDP client.

## bring-up the server
To start the server, we need to use the following roslaung file:
```
roslaunch nautilus_udp_bridge bridge.launch
```
In case that we need a dummy odometry stub that is similar to the output of the **robot_localization** package (message type *nav_msgs::Odometry*), we can execute:
 ```
roslaunch nautilus_udp_bridge odometry_stub.launch
```

## functionality
The bridge listens continiously the odometry topic. The client send 0 or 1 to enable transmition of UDP data or not.


## testing
Use the standard linux tool nc like
```
nc -u locahost 1234
```

## compatibility
- Ubuntu Linux 18 (Bionic)
- ROS melodic
- Poco library 1.8.0.1

## maintener
angelos plastropoulos
