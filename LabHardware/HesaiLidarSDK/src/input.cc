#include <sstream>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <string.h>
#include <iostream>
#include "input.h"

namespace pandar_pointcloud
{
const int lidarPacketSize = ETHERNET_MTU;


Input::Input(std::string filePath, int type)
{
  if(type == 0)
    realLidarPacketSize = 1240;
  else if(type == 1)
    realLidarPacketSize = 1256;
  pcapFilePath = filePath;
  if ((pcap_ = pcap_open_offline(pcapFilePath.c_str(), pcapErrorBuffer) ) == NULL)
  {
    printf("read pcap file wrong\n");
  }
}


Input::Input(uint16_t port, uint16_t gpsPort)
{
  socketForLidar = -1;
  socketForLidar = socket(PF_INET, SOCK_DGRAM, 0);
  if (socketForLidar == -1)
  {
    perror("socket"); // TODO: perror errno
    return;
  }

  sockaddr_in myAddress;                    // my address information
  memset(&myAddress, 0, sizeof(myAddress)); // initialize to zeros
  myAddress.sin_family = AF_INET;           // host byte order
  myAddress.sin_port = htons(port);         // port in network byte order
  myAddress.sin_addr.s_addr = INADDR_ANY;   // automatically fill in my IP

  if (bind(socketForLidar, (sockaddr *)&myAddress, sizeof(sockaddr)) == -1)
  {
    perror("bind"); // TODO: perror errno
    return;
  }

  if (fcntl(socketForLidar, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }

  if (port == gpsPort)
  {
    socketNumber = 1;
    return;
  }
  //gps socket
  socketForGPS = -1;
  socketForGPS = socket(PF_INET, SOCK_DGRAM, 0);
  if (socketForGPS == -1)
  {
    perror("socket"); // TODO: perror errno
    return;
  }

  sockaddr_in myAddressGPS;                       // my address information
  memset(&myAddressGPS, 0, sizeof(myAddressGPS)); // initialize to zeros
  myAddressGPS.sin_family = AF_INET;              // host byte order
  myAddressGPS.sin_port = htons(gpsPort);         // port in network byte order
  myAddressGPS.sin_addr.s_addr = INADDR_ANY;      // automatically fill in my IP

  if (bind(socketForGPS, (sockaddr *)&myAddressGPS, sizeof(sockaddr)) == -1)
  {
    perror("bind"); // TODO: perror errno
    return;
  }

  if (fcntl(socketForGPS, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }
  socketNumber = 2;
}

Input::~Input(void)
{
  if (socketForGPS > 0)
    close(socketForGPS);
  if (socketForLidar > 0)
    (void)close(socketForLidar);
}

// return : 0 - lidar
//          1 - gps
//          -1 - error
int Input::getPacket(PandarPacket *pkt)
{
  gettimeofday(&getPacketStartTime, NULL);
  struct pollfd fds[socketNumber];
  if (socketNumber == 2)
  {
    fds[0].fd = socketForGPS;
    fds[0].events = POLLIN;

    fds[1].fd = socketForLidar;
    fds[1].events = POLLIN;
  }
  else if (socketNumber == 1)
  {
    fds[0].fd = socketForLidar;
    fds[0].events = POLLIN;
  }
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

  sockaddr_in senderAddress;
  socklen_t senderAddressLen = sizeof(senderAddress);
  int retval = poll(fds, socketNumber, POLL_TIMEOUT);
  if (retval < 0) // poll() error?
  {
    if (errno != EINTR)
      printf("poll() error: %s", strerror(errno));
    return -1;
  }
  if (retval == 0) // poll() timeout?
  {
    return -1;
  }
  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
  {
    perror("poll() reports Pandar error");
    return -1;
  }

  senderAddressLen = sizeof(senderAddress);
  ssize_t nbytes;
  for (int i = 0; i != socketNumber; ++i)
  {
    if (fds[i].revents & POLLIN)
    {
      nbytes = recvfrom(fds[i].fd, &pkt->data[0],
                        lidarPacketSize, 0,
                        (sockaddr *)&senderAddress,
                        &senderAddressLen);
      break;
    }
  }

  if (nbytes < 0)
  {
    if (errno != EWOULDBLOCK)
    {
      perror("recvfail");
      return -1;
    }
  }
  pkt->size = nbytes;

  gettimeofday(&getPacketStopTime, NULL);
  pkt->stamp = static_cast<double>(getPacketStartTime.tv_sec + static_cast<double>(getPacketStartTime.tv_usec) / 1000000 + getPacketStopTime.tv_sec + static_cast<double>(getPacketStopTime.tv_usec) / 1000000) / 2;
  
  if ((size_t)nbytes == PANDORA_GPS_PACKET_SIZE) // gps
    return 1;
  
  return 0;
}

int Input::getPacketFromPcap(PandarPacket *pkt)
{
  struct pcap_pkthdr *header;
  const u_char *pcapData;

  int ret = pcap_next_ex(pcap_, &header, &pcapData);

  if ( ret == 1)
  {
    if (header->caplen == (PANDORA_GPS_PACKET_SIZE + 42)) // gps packet
    {
      memcpy(&pkt->data[0], pcapData + 42, PANDORA_GPS_PACKET_SIZE);
      return 1;
    }

    else if (header->caplen == (realLidarPacketSize + 42))
    {
      memcpy(&pkt->data[0], pcapData + 42, realLidarPacketSize);
      gettimeofday(&getPacketStartTime, NULL);
      pkt->stamp = getPacketStartTime.tv_sec + static_cast<double>(getPacketStartTime.tv_usec) / 1000000;
      pkt->size = realLidarPacketSize;
      return 0; // success
    }
    return -1;
  }

  else if (ret == -2) // there are no more packets to read from the savefile, close and reopen file
  {
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(pcapFilePath.c_str(), pcapErrorBuffer);
  }

  else if (ret == -1) //  an error occurred while reading the packet
  {
    perror("read pcap file wrong: ");
  }
}

}
