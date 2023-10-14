
#include "seapath_node.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "seapath_node");
  seapath::SeapathNode node;
  node.spin();
}




///*
//The MIT License (MIT)

//Copyright (c) 2014 J. Ian S. Vaughn

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

//-------------------------------------------------------------------------------
//A special warning is noted as follows:

//THIS SYSTEM IS NOT IMO or IHO COMPLIANT, AND SHALL NOT BE USED FOR NAVIGATION,
//HYDROGRAPHY, OR ANY CRITICAL APPLICATIONS THAT PLACE PEOPLE OR EQUIPMENT
//AT SIGNIFICANT RISK.  IT IS USEFUL FOR SCIENCE / PLANNING USE ONLY!

//This is included in the warning about the "AS IS" nature of this software,
//but re-iterated because it could in theory be used for navigation

//*/

//#include <stdio.h>
//#include <unistd.h>
//#include <stdlib.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netinet/ip.h>
//#include <netdb.h>
//#include <arpa/inet.h>
//#include <stdint.h>
//#include <string.h>
////#include <glib.h>
//#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/NavSatFix.h>
//#include <sensor_msgs/Imu.h>
//#include <nav_msgs/Odometry.h>
//#include <tf/tf.h>


//#define BUFSIZE 44
//#define POLY 0x8408

//// 2^30
//#define LAT_SCALE  ((double)1073741824)

//// 2^30
//#define LON_SCALE  ((double)1073741824)

//// 2^14
//#define ANG_SCALE  ((double)16384)

//uint16_t seapath_crc16(uint8_t* buf, size_t len) {
//  // based on the code listing on page 144 of the seapath manual.
//  // Appears to be a standard CCITT CRC16
//  uint8_t i;
//  uint16_t data;
//  uint16_t crc = 0xffff;

//  if (len == 0) {
//    return ~crc;
//  }

//  do {
//    data = (uint16_t)(0xff & *buf++);
    
//    for (i=0; i<8; i++) {

//      if ((crc & 0x0001) ^ (data & 0x0001)) {
//        crc = (crc >> 1) ^ POLY;
//      } else {
//        crc >>= 1;
//      }
//      data >>= 1;
//    }
//  } while (--len);

//  crc = ~crc;
//  data = crc;

//  // flips byte order.  From the manual
//  crc = (crc << 8) | ((data >> 8) & 0xff);

//  return crc;
// }

//#pragma pack(1)
//typedef struct Seapath23_raw {

//  uint16_t hdr; // MAGIC == 0xAA51
//  int32_t time;
//  uint16_t time_frac;

//  int32_t latitude;
//  int32_t longitude;
//  int32_t height;

//  int16_t heave;
//  int16_t vel_north;
//  int16_t vel_east;
//  int16_t vel_down;

//  int16_t roll;
//  int16_t pitch;
//  uint16_t heading;

//  int16_t rate_roll;
//  int16_t rate_pitch;
//  int16_t rate_heading;

//  int16_t status;

//  int16_t checksum;

//} Seapath23_raw_t;
//#pragma pack()

//// my own version, signed
//int32_t s_ntohl(int32_t val) {
//  return (int32_t)ntohl((uint32_t)val);
//}

//int16_t s_ntohs(int16_t val) {
//  return (int16_t)ntohs((uint16_t)val);
//}

//void publishNavsat(ros::NodeHandle &n , ros::Publisher &pub ,Seapath23_raw_t* raw){
//  //std::cout << "lat: " << (double)s_ntohl(raw->latitude) << std::endl;
//  sensor_msgs::NavSatFix msg;
//  msg.latitude = ((double)s_ntohl(raw->latitude)) / ((double)(LAT_SCALE)) * 90;
//  msg.longitude = ((double)s_ntohl(raw->longitude)) / ((double)(LON_SCALE)) * 90;
//  msg.altitude = ((double)s_ntohs(raw->heave)) / 100.0;
//  msg.header.stamp.fromSec((double)s_ntohl(raw->time) + (double)ntohs(raw->time_frac)/10000.0);
//  msg.header.frame_id = "mru";
//  pub.publish(msg);
//}

//void publishImu(ros::NodeHandle &n , ros::Publisher &pub ,Seapath23_raw_t* raw){
//  nav_msgs::Odometry msg;
//  tf2::Quaternion quat;
//  double pi = 3.14159;
//  double roll =((double)s_ntohs(raw->roll)) / ANG_SCALE * 90;
//  roll = roll * pi/180;
//  double pitch = ((double)s_ntohs(raw->pitch)) / ANG_SCALE * 90;
//  pitch = pitch * pi/180;
//  double hdg = ((double)ntohs(raw->heading)) / ANG_SCALE * 90;
//  std::cout << hdg << std::endl;
//  hdg = hdg * pi/180;
//  quat.setRPY(roll,
//              pitch,
//              hdg);

//  msg.pose.pose.orientation.x = quat.x();
//  msg.pose.pose.orientation.y = quat.y();
//  msg.pose.pose.orientation.z = quat.z();
//  msg.pose.pose.orientation.w = quat.w();

//  double rollRate = ((double)s_ntohs(raw->rate_roll)) / ANG_SCALE * 90;
//  rollRate = rollRate * pi/180;
//  double pitchRate = ((double)s_ntohs(raw->rate_pitch)) / ANG_SCALE * 90;
//  pitchRate = pitchRate * pi/180;
//  double yawRate = ((double)s_ntohs(raw->rate_heading)) / ANG_SCALE * 90;
//  yawRate = yawRate * pi/180;

//  msg.twist.twist.angular.x = rollRate;
//  msg.twist.twist.angular.y = pitchRate;
//  msg.twist.twist.angular.z = yawRate;

//  msg.twist.twist.linear.x = ((double)s_ntohs(raw->vel_north)) / 100.0;
//  msg.twist.twist.linear.y = ((double)s_ntohs(raw->vel_east)) / 100.0;
//  msg.twist.twist.linear.z = ((double)s_ntohs(raw->vel_down)) / 100.0;

//  msg.header.stamp.fromSec((double)s_ntohl(raw->time) + (double)ntohs(raw->time_frac)/10000.0);
//  msg.header.frame_id = "sea_surface";
//  msg.child_frame_id = "mru";

//  pub.publish(msg);
//}

//int main(int argc, char** argv) {
//  ros::init(argc, argv, "seapath_broadcaster_node");
//  ros::NodeHandle n;
//  ros::Publisher navsatPub = n.advertise<sensor_msgs::NavSatFix>("nautilus/seapath/fix", 1000);
//  ros::Publisher imuPub = n.advertise<nav_msgs::Odometry>("nautilus/seapath/odom", 1000);


//  int fd;
//  struct sockaddr_in addr;
//  size_t OUTPUTBUF_LEN = 2048;
//  char outputBuffer[OUTPUTBUF_LEN];

//  struct sockaddr_in remaddr;
//  socklen_t addrlen = sizeof(remaddr);
//  struct addrinfo hints;
//  size_t recvlen;
//  uint16_t crc_calc;
//  size_t toSend = 0;

//  uint8_t buf[BUFSIZE];

//  int recv_port;

//  recv_port = 14150;

//  printf("STARTING SEAPATH_CAP...\n");
//  printf(" (c) 2015 Ian Vaughn\n");
//  printf(" modified  2018 for ROS compatibility: Kris Krasnosky\n");
//  printf(" for the Ocean Exploration Trust Inc\n");
//  printf("\n");
//  printf("NOT FOR CRITICAL NAVIGATION, NOT IMO/IHO COMPLIANT!\n");
//  printf("\n");
//  printf("Listening on port %d\n", recv_port);

//  printf("\n\n\n\n");


//  // create and initialize the UDP socket
//  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
//    perror("Cannot create socket");
//    return -1;
//  }

//  memset(&addr, 0, sizeof(addr));
//  addr.sin_family = AF_INET;
//  addr.sin_addr.s_addr = htonl(INADDR_ANY);
//  addr.sin_port = htons(recv_port);

//  // bind to the input socket
//  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
//    perror("bind failed");
//    return -1;
//  }

//  // resolve the name of the output socket
//  memset(&hints, 0, sizeof(hints));
//  hints.ai_family = AF_INET; // IPv4
//  hints.ai_socktype = SOCK_DGRAM; // UDP
//  hints.ai_flags = AI_PASSIVE; // give me an IP


//  // listen for stuff
//  while (ros::ok()) {
//    recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr*)&remaddr, &addrlen);
//    crc_calc = seapath_crc16(buf+2, recvlen-4);
//    if (recvlen == BUFSIZE && crc_calc != ntohs(((uint16_t*)buf)[21])) {
//      printf("CRC_CALC: %04x, CRC_RECV: %04x\n",
//            crc_calc, ntohs(((uint16_t*)buf)[21]));
//    } else {
//      //toSend = seapathToJson(outputBuffer, OUTPUTBUF_LEN-2, (Seapath23_raw_t*)buf, crc_calc);
//      publishNavsat(n,navsatPub,(Seapath23_raw_t*)buf);
//      publishImu(n,imuPub,(Seapath23_raw_t*)buf);

//    }
//  }
//}
