#include "hesaiLidarSDK.h"

int imageNo = 0;
unsigned long imageNoForSave = 0;
unsigned int modnum = 0;
int lidarNo = 0;
unsigned long lidarNoForSave = 0;
FILE* cameraTimestampFile = fopen("camera-timestamp.txt", "w");
FILE* lidarTimestampFile = fopen("lidar-timestamp.txt", "w");
FILE* gpsTimestampFile = fopen("gps-timestamp.txt", "w");

double pandoraToSysTimeGap = 0;
int gpsTimestamp = 0;
#ifdef HESAI_WITH_CAMERA
void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{
  // Mat myMat = imread("t.png");
  // imshow("camera", myMat);
  // std::cout<<myMat<<std::endl;
  // if(++imageNo > 50  && modnum < 4)
  // { 
  //   ++modnum;
  //   cv::imwrite(boost::to_string(++imageNoForSave) + "-" + boost::to_string(pic_id) + ".jpg", *matp);
  //   if (modnum == 4)
  //   {
  //     imageNo = 0;
  //     modnum = 0;
  //   }
  // }
  // if(pic_id == 0)
  // {
  //   cv::imwrite(boost::to_string(++imageNoForSave) + "-" + boost::to_string(pic_id) + ".bmp", *matp);
  // }
  struct timeval ts;
  gettimeofday(&ts, NULL);
  // fprintf(cameraTimestampFile, "%d,%f\n", pic_id, ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - timestamp);
  fprintf(cameraTimestampFile, "%d,%lf,%lf\n", pic_id, timestamp, (double)ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - timestamp);

  printf("cameraid: %d, timestamp: %lf\n", pic_id, timestamp);
}

void cameraCallbackForDelay(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{
  struct timeval ts;
  gettimeofday(&ts, NULL);
  fprintf(cameraTimestampFile, "%d,%f\n", pic_id, ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - timestamp);
}
#endif


void gpsCallback(int timestamp)
{
  struct timeval ts;
  gettimeofday(&ts, NULL);
  gpsTimestamp = timestamp;
  pandoraToSysTimeGap = (double)ts.tv_sec + ( (double)ts.tv_usec / 1000000.0  ) - (double)timestamp;
  printf("gps: %d, gap: %f\n", timestamp, pandoraToSysTimeGap);
  fprintf(gpsTimestampFile, "%d, %f, %f\n", timestamp, ts.tv_sec + ts.tv_usec / 1000000.0, pandoraToSysTimeGap);
}



void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
  struct timeval ts;
  gettimeofday(&ts, NULL);
  fprintf(lidarTimestampFile, "%d, %f,%f\n", gpsTimestamp, timestamp, ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - timestamp);
  printf("lidar: %lf\n", timestamp);
}

int main(int argc, char **argv)
{
#ifdef HESAI_WITH_CAMERA
  HesaiLidarSDK psdk(
    std::string("192.168.20.51"), 
    9870, 
    std::string("calibration.yml"), 
    cameraCallback,
    2368, 10110, 0,
    std::string("correction.csv"),
    lidarCallback, gpsCallback,
    HESAI_LIDAR_RAW_DATA_STRCUT_DUAL_RETURN, 40, HESAI_LIDAR_PCL_DATA_TYPE_REDUCED);
#else
  HesaiLidarSDK psdk(2368, 10110, std::string("correction.csv"), lidarCallback, gpsCallback, 
      HESAI_LIDAR_RAW_DATA_STRCUT_DUAL_RETURN, 40, HESAI_LIDAR_PCL_DATA_TYPE_REDUCED);
#endif
  psdk.start();
  while(true)
  {
    sleep(100);
  }
}