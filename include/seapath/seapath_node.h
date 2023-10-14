#ifndef SEAPATHNODE_H
#define SEAPATHNODE_H

#include "defs.h"
#include "types.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

NS_HEAD
#define BUFSIZE 44

class SeapathNode
{
public:
  SeapathNode();

  void publishNavsat(Seapath23_raw_t* raw);
  void publishOdom(Seapath23_raw_t* raw);

  void spinOnce();
  void spin();

private:
  size_t recvlen;
  uint16_t crc_calc;
  int fd;
  struct sockaddr_in remaddr;
  ros::NodeHandlePtr nh_;
  socklen_t addrlen;
  uint8_t buf[BUFSIZE];
  struct{
    ros::Publisher navsat;
    ros::Publisher odom;
  } pub_;


};

NS_FOOT

#endif // SEAPATHNODE_H
