#ifndef __PANDORA_TYPES_H
#define __PANDORA_TYPES_H

#include <sys/time.h>
#define ETHERNET_MTU (1500)

typedef struct PandarPacket_s
{
  double stamp;
  uint8_t data[ETHERNET_MTU];
  uint32_t size;
}PandarPacket;



typedef struct GPS_STRUCT_{
    int used;
    time_t gps;
    int usedHour;
    struct tm t;
}GPS_STRUCT_T;

enum DeviceType
{
  deviceTypeSingleReturn,
  deviceTypeDualReturn
};

#define PANDORA_GPS_PACKET_SIZE 512

#endif