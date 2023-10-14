#ifndef HERC_NAV_DEFS_H
#define HERC_NAV_DEFS_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

//#define BUFSIZE 44
#define POLY 0x8408

///
///  Namespace stuff
///
#define NS_HEAD namespace seapath {

#define NS_FOOT }


// 2^30
#define LAT_SCALE  ((double)1073741824)

// 2^30
#define LON_SCALE  ((double)1073741824)

// 2^14
#define ANG_SCALE  ((double)16384)

#define PI                                                                 \
  3.1415926535897932384626433832795028841971693993751058209749445923078164062





#endif // HERC_NAV_DEFS_H
