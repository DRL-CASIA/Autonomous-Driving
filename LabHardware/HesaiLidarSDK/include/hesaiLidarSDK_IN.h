#ifndef __PANDORA_SDK_IN_H
#define __PANDORA_SDK_IN_H

#include <pthread.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <semaphore.h>

#ifdef HESAI_WITH_CAMERA  
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
using namespace cv;


#define HesaiLidarSDK_CAMERA_NUM 5
#define HesaiLidarSDK_IMAGE_WIDTH 1280
#define HesaiLidarSDK_IMAGE_HEIGHT 720

#include "pandora_client.h"
#endif

#include "input.h"
#include "rawdata.h"

#define HesaiLidarSDK_DEFAULT_START_ANGLE 0.0
typedef struct HS_LIDAR_L40_GPS_PACKET_s
{
	unsigned short flag;
	unsigned short year;
	unsigned short month;
	unsigned short day;
	unsigned short second;
	unsigned short minute;
	unsigned short hour;
	unsigned int fineTime;
} HS_LIDAR_L40_GPS_Packet;

#define HS_LIDAR_L40_GPS_PACKET_SIZE (512)
#define HS_LIDAR_L40_GPS_PACKET_FLAG_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_YEAR_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_MONTH_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_DAY_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_HOUR_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_MINUTE_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_SECOND_SIZE (2)
#define HS_LIDAR_L40_GPS_ITEM_NUM (7)

#define HesaiLidarSDK_DEFAULT_LIDAR_RECV_PORT 8080
#define HesaiLidarSDK_DEFAULT_GPS_RECV_PORT 10110




// 10000 us  = 100 ms = 0.1s per packet
#define HesaiLidarSDK_PCAP_TIME_INTERVAL (100000 / 3000)

enum SDKUseMode
{
	PandoraUseModeOnlyLidar,			// 仅使用lidar
	PandoraUseModeReadPcap,			// 仅读取lidar的pcap
	PandoraUseModeLidarAndCamera //使用lidar和camera
};

class HesaiLidarSDK_internal
{
public:
	HesaiLidarSDK_internal(
#ifdef HESAI_WITH_CAMERA  
			const std::string pandoraIP,
			const unsigned short pandoraCameraPort,
			const std::string intrinsicFile,
			boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
#endif
			const unsigned short lidarRecvPort,
			const unsigned short gpsRecvPort,
			const double startAngle,			
			const std::string lidarCorrectionFile,
			boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
			boost::function<void(unsigned int timestamp)> gpsCallback,
			const unsigned int laserReturnType,
			const unsigned int laserCount,
			const unsigned int pclDataType);

	HesaiLidarSDK_internal(
#ifdef HESAI_WITH_CAMERA  
			const std::string pandoraIP,
			const unsigned short pandoraCameraPort,
			boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
#endif
			boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback);
	

	HesaiLidarSDK_internal(
			const std::string pcapPath,
			const std::string lidarCorrectionFile,
			const unsigned int laserReturnType,
			const unsigned int laserCount,
			const unsigned int pclDataType,
			boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);

	~HesaiLidarSDK_internal();
	int start();
	void stop();
#ifdef HESAI_WITH_CAMERA  
	void pushPicture(PandoraPic *pic);
#endif

private:
	void init(
#ifdef HESAI_WITH_CAMERA   
			const std::string pandoraIP,
			const unsigned short pandoraCameraPort,
			const std::string intrinsicFile,
			boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
#endif
			const unsigned short lidarRecvPort,
			const unsigned short gpsRecvPort,
			const double startAngle,			
			const std::string lidarCorrectionFile,
			boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
			boost::function<void(unsigned int timestamp)> gpsCallback,
			const unsigned int laserReturnType,
			const unsigned int laserCount,
			const unsigned int pclDataType);
	void init(
			const std::string pcapPath,
			const std::string lidarCorrectionFile,
			const unsigned int laserReturnType,
			const unsigned int laserCount,
			const unsigned int pclDataType,
			boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);
#ifdef HESAI_WITH_CAMERA   
	void setupCameraClient();
	void processPic();
	bool loadIntrinsics(const std::string &intrinsicFile);

	
	pthread_mutex_t picLock;
	sem_t picSem;
	boost::thread *processPicThread;
	bool continueProcessPic;
	bool needRemapPicMat;
	std::string ip;
	unsigned short cport;
	void *pandoraCameraClient;
	std::list<PandoraPic *> pictureList;
	std::vector<Mat> mapxList;
	std::vector<Mat> mapyList;
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> userCameraCallback;
	unsigned int cameraLastGPSSecond;
#endif
	void setupLidarClient();
	void setupReadPcap();
	void lidarRecvTask();
	void readPcapFile();
	void processGps(HS_LIDAR_L40_GPS_Packet &gpsMsg);
	void processLiarPacket();
	void pushLiDARData(PandarPacket packet);
	void internalFuncForGPS(const unsigned int &gpsstamp);

	
	pthread_mutex_t lidarLock, lidarGpsLock;
	sem_t lidarSem;
	boost::thread *lidarRecvThread;
	boost::thread *lidarProcessThread;
	boost::thread *readPacpThread;
	bool continueLidarRecvThread;
	bool continueProcessLidarPacket;
	bool continueReadPcap;
	unsigned short lport;
	int lidarRotationStartAngle;
	HS_LIDAR_L40_GPS_Packet hesaiGps;

	std::list<PandarPacket> lidarPacketList;
	
	boost::shared_ptr<pandar_pointcloud::Input> input;
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> userLidarCallback;
	boost::function<void(double timestamp)> userGpsCallback;
	boost::shared_ptr<pandar_rawdata::RawData> data_;
	time_t gps1;
	GPS_STRUCT_T gps2;
	unsigned int lidarLastGPSSecond;
	unsigned int lidarLastGPSHourSecond;

	SDKUseMode useMode;
};

#endif