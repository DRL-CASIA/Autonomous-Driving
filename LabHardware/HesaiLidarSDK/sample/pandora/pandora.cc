#include "hesaiLidarSDK.h"


void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{
  printf("cameraid: %d, timestamp: %lf\n", pic_id, timestamp);
}

void gpsCallback(int timestamp)
{
  printf("gps: %d\n", timestamp);
}



void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
   printf("lidar: %lf\n", timestamp);
}

int main(int argc, char **argv)
{
  HesaiLidarSDK psdk(
    std::string("192.168.20.51"), 
    9870, 
    std::string("calibration.yml"), 
    cameraCallback,
    2368, 10110, 0,
    std::string("correction.csv"),
    lidarCallback, gpsCallback,
    HESAI_LIDAR_RAW_DATA_STRCUT_DUAL_RETURN, 40, HESAI_LIDAR_PCL_DATA_TYPE_REDUCED);

  psdk.start();
  while(true)
  {
    sleep(100);
  }
}