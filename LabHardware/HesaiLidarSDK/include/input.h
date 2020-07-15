#ifndef __PANDAR_INPUT_H
#define __PANDAR_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include "pandora_types.h"
#include "pcap.h"

namespace pandar_pointcloud
{

class Input
{
public:
  Input(uint16_t port, uint16_t gpsPort);
  ~Input();
  Input(std::string filePath, int type);
  int getPacket(PandarPacket *pkt);
  int getPacketFromPcap(PandarPacket *pkt);
private:
  int socketForLidar;
  int socketForGPS;
  int socketNumber;
  pcap_t *pcap_;
  std::string pcapFilePath;
  char pcapErrorBuffer[PCAP_ERRBUF_SIZE];
  int realLidarPacketSize;
  struct timeval getPacketStartTime, getPacketStopTime;
};

} // pandar_driver namespace

#endif // __PANDAR_INPUT_H
