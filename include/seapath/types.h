#ifndef TYPES_H
#define TYPES_H

#include "defs.h"

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

NS_HEAD



#pragma pack(1)
typedef struct Seapath23_raw {

  uint16_t hdr; // MAGIC == 0xAA51
  int32_t time;
  uint16_t time_frac;

  int32_t latitude;
  int32_t longitude;
  int32_t height;

  int16_t heave;
  int16_t vel_north;
  int16_t vel_east;
  int16_t vel_down;

  int16_t roll;
  int16_t pitch;
  uint16_t heading;

  int16_t rate_roll;
  int16_t rate_pitch;
  int16_t rate_heading;

  int16_t status;

  int16_t checksum;

} Seapath23_raw_t;
#pragma pack()



NS_FOOT

#endif // TYPES_H
