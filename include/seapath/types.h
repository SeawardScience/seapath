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

/// Seapath Binary format 23
/// This binary format consists of a fixed-length message using 1, 2 and 4–byte signal and
/// unsigned integers. The signed integers are represented as two-complement numbers.
/// For the multi-byte elements, the most significant byte is transmitted first. The total
/// number of bytes is 44.

#pragma pack(1)
typedef struct Seapath23_raw {

  uint16_t hdr; // MAGIC == 0xAA51
  /// Seconds since 1970
  int32_t time;
  /// Fraction of a second. 0.0001 second steps.
  uint16_t time_frac;

  /// Latitude is positive north of the Equator. 2^30 = 90 degrees
  int32_t latitude;
  /// Longitude is positive east of Greenwich. 2^30 = 90 degrees
  int32_t longitude;

  /// Height is above the ellipsoid (centimeters)
  int32_t height;

  /// Heave is positive down (centimeters). 
  int16_t heave;

  /// cm/s
  int16_t vel_north;
  /// cm/s
  int16_t vel_east;
  /// cm/s
  int16_t vel_down;

  /// Roll is positive with port side up (2^14 = 90 degrees).
  int16_t roll;
  /// Pitch is positive with bow up (2^14 = 90 degrees).
  int16_t pitch;
  /// (2^14 = 90 degrees)
  uint16_t heading;

  /// 2^14 = 90 degrees/sec
  int16_t rate_roll;
  /// 2^14 = 90 degrees/sec
  int16_t rate_pitch;
  /// 2^14 = 90 degrees/sec
  int16_t rate_heading;

  /// The status word consists of 16 single bit flags numbered from 0 to 15, where 0 is the
  /// least significant bit.
  /// Bit no. Interpretation
  /// 0 Reduced horizontal position and velocity performance.
  /// 1 Invalid horizontal position and velocity data.
  /// 2 Reduced heave and vertical velocity performance.
  /// 3 Invalid heave and vertical velocity data.
  /// 4 Reduced roll and pitch performance.
  /// 5 Invalid roll and pitch data.
  /// 6 Reduced heading performance.
  /// 7 Invalid heading data
  int16_t status;

  int16_t checksum;

} Seapath23_raw_t;
#pragma pack()



NS_FOOT

#endif // TYPES_H
