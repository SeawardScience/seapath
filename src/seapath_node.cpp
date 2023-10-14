#include "seapath_node.h"


NS_HEAD

// my own version, signed
int32_t s_ntohl(int32_t val) {
  return (int32_t)ntohl((uint32_t)val);
}

int16_t s_ntohs(int16_t val) {
  return (int16_t)ntohs((uint16_t)val);
}

uint16_t seapath_crc16(uint8_t* buf, size_t len) {
  // based on the code listing on page 144 of the seapath manual.
  // Appears to be a standard CCITT CRC16
  uint8_t i;
  uint16_t data;
  uint16_t crc = 0xffff;

  if (len == 0) {
    return ~crc;
  }

  do {
    data = (uint16_t)(0xff & *buf++);

    for (i=0; i<8; i++) {

      if ((crc & 0x0001) ^ (data & 0x0001)) {
        crc = (crc >> 1) ^ POLY;
      } else {
        crc >>= 1;
      }
      data >>= 1;
    }
  } while (--len);

  crc = ~crc;
  data = crc;

  // flips byte order.  From the manual
  crc = (crc << 8) | ((data >> 8) & 0xff);

  return crc;
 }

SeapathNode::SeapathNode()
{
  nh_.reset(new ros::NodeHandle("~"));

  int recv_port;
  nh_->param<int>("port",    recv_port, 14150);

  pub_.navsat = nh_->advertise<sensor_msgs::NavSatFix>("/nautilus/seapath/fix", 1000);
  pub_.odom = nh_->advertise<nav_msgs::Odometry>("/nautilus/seapath/odom", 1000);


  struct sockaddr_in addr;
  size_t OUTPUTBUF_LEN = 2048;
  char outputBuffer[OUTPUTBUF_LEN];


  addrlen = sizeof(remaddr);
  struct addrinfo hints;
  size_t recvlen;
  uint16_t crc_calc;
  size_t toSend = 0;





//  recv_port = 14150;

  // create and initialize the UDP socket
  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("Cannot create socket");
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(recv_port);

  // bind to the input socket
  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind failed");
  }

  // resolve the name of the output socket
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET; // IPv4
  hints.ai_socktype = SOCK_DGRAM; // UDP
  hints.ai_flags = AI_PASSIVE; // give me an IP

}

void SeapathNode::publishNavsat(Seapath23_raw_t *raw){
  sensor_msgs::NavSatFix msg;
  msg.latitude = ((double)s_ntohl(raw->latitude)) / ((double)(LAT_SCALE)) * 90;
  msg.longitude = ((double)s_ntohl(raw->longitude)) / ((double)(LON_SCALE)) * 90;
  msg.altitude = ((double)s_ntohs(raw->heave)) / 100.0;
  msg.header.stamp.fromSec((double)s_ntohl(raw->time) + (double)ntohs(raw->time_frac)/10000.0);
  msg.header.frame_id = "mru";
  pub_.navsat.publish(msg);
}

void SeapathNode::publishOdom(Seapath23_raw_t *raw){
  nav_msgs::Odometry msg;
  tf2::Quaternion quat;
  double pi = PI;
  double roll =((double)s_ntohs(raw->roll)) / ANG_SCALE * 90;
  roll = roll * pi/180;
  double pitch = ((double)s_ntohs(raw->pitch)) / ANG_SCALE * 90;
  pitch = pitch * pi/180;
  double hdg = ((double)ntohs(raw->heading)) / ANG_SCALE * 90;
  //std::cout << hdg << std::endl;
  hdg = hdg * pi/180;
  quat.setRPY(roll,
              pitch,
              hdg);

  msg.pose.pose.orientation.x = quat.x();
  msg.pose.pose.orientation.y = quat.y();
  msg.pose.pose.orientation.z = quat.z();
  msg.pose.pose.orientation.w = quat.w();

  double rollRate = ((double)s_ntohs(raw->rate_roll)) / ANG_SCALE * 90;
  rollRate = rollRate * pi/180;
  double pitchRate = ((double)s_ntohs(raw->rate_pitch)) / ANG_SCALE * 90;
  pitchRate = pitchRate * pi/180;
  double yawRate = ((double)s_ntohs(raw->rate_heading)) / ANG_SCALE * 90;
  yawRate = yawRate * pi/180;

  msg.twist.twist.angular.x = rollRate;
  msg.twist.twist.angular.y = pitchRate;
  msg.twist.twist.angular.z = yawRate;

  msg.twist.twist.linear.x = ((double)s_ntohs(raw->vel_north)) / 100.0;
  msg.twist.twist.linear.y = ((double)s_ntohs(raw->vel_east)) / 100.0;
  msg.twist.twist.linear.z = ((double)s_ntohs(raw->vel_down)) / 100.0;

  msg.header.stamp.fromSec((double)s_ntohl(raw->time) + (double)ntohs(raw->time_frac)/10000.0);
  msg.header.frame_id = "sea_surface";
  msg.child_frame_id = "mru";

  pub_.odom.publish(msg);
}


void SeapathNode::spinOnce(){
  recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr*)&remaddr, &addrlen);
  crc_calc = seapath_crc16(buf+2, recvlen-4);
  if (recvlen == BUFSIZE && crc_calc != ntohs(((uint16_t*)buf)[21])) {
    printf("CRC_CALC: %04x, CRC_RECV: %04x\n",
          crc_calc, ntohs(((uint16_t*)buf)[21]));
  } else {
    publishNavsat((Seapath23_raw_t*)buf);
    publishOdom((Seapath23_raw_t*)buf);

  }
}

void SeapathNode::spin(){
  while (ros::ok()) {
    spinOnce();
  }
}

NS_FOOT
