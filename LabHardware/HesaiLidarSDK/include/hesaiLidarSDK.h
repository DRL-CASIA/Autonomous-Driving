#ifndef __PANDORA_SDK_H
#define __PANDORA_SDK_H

#include <string>
#include <boost/function.hpp>
#ifdef HESAI_WITH_CAMERA  
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#endif
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "point_types.h"

typedef enum{
	HESAI_LIDAR_PCL_DATA_TYPE_REDUCED = 0, // Don't include the useless points such as xyz(0,0,0) or xyz(na/na/na)
	HESAI_LIDAR_PCL_DATA_TYPE_ALIGNMENT = 1, // Align the points , such as 1800x40 (1800 indicates the frequency of each laser one circle)
}HesaiLidarPCLDataType;

typedef enum{
	HESAI_LIDAR_RAW_DATA_STRCUT_SINGLE_RETURN = 0,  // Pandar40
	HESAI_LIDAR_RAW_DATA_STRCUT_DUAL_RETURN = 1,    // Pandar40-AC , Pandar40P
}HesaiLidarRawDataSturct;

class HesaiLidarSDK_internal;

class HesaiLidarSDK
{
public:
#ifdef HESAI_WITH_CAMERA  
	HesaiLidarSDK(
		const std::string pandoraIP, 														// pandora的ip
		const unsigned short pandoraCameraPort,		   						// pandora的port
		const std::string intrinsicFile, 												// 摄像头的内参文件路径，为空时，输出的图像是未经过矫正的
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int picid)> cameraCallback, 	// 摄像头的callback函数

		const unsigned short lidarRecvPort,     								// lidar的数据接收端口， 默认为8080
		const unsigned short gpsRecvPort,												// gps数据接收端口， 默认为10110
		const double startAngle,		  													// lidar的旋转起始角度，默认为0,单位是度，例如135度
		const std::string lidarCorrectionFile, 									// lidar的标定文件路径，为空时，将使用默认参数
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback, 				// lidar的callback函数
		boost::function<void(unsigned int timestamp)> gpsCallback, 		// gps数据的callback函数，可为NULL.timestamp为当前gps的时间戳
		const HesaiLidarRawDataSturct dataStruct,											// 回波类型,0为单回波,1为双回波.
		const unsigned int laserCount,													// lidar的线数量
		const HesaiLidarPCLDataType pclDataType);												// 返回给lidarcallback的pcl数据的格式类型.0是以block遍历生成的数据,1是以laserId遍历生成的数据
#endif

	HesaiLidarSDK(
#ifdef HESAI_WITH_CAMERA  
		const std::string pandoraIP, //此时,lidar的数据接收端口默认为8080, gps数据接收端口默认为10110, lidar的旋转起始角度默认为0, 输出的图像是未经过矫正的, lidar的标定参数使用默认值
		const unsigned short pandoraCameraPort,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int picid)> cameraCallback,
#endif
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);

	HesaiLidarSDK(
		const unsigned short lidarRecvPort,
		const unsigned short gpsRecvPort,
		const std::string lidarCorrectionFile,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
		boost::function<void(unsigned int timestamp)> gpsCallback,
		const HesaiLidarRawDataSturct laserReturnType,
		const unsigned int laserCount,
		const HesaiLidarPCLDataType pclDataType);


	HesaiLidarSDK(
		const std::string pcapPath,
		const std::string lidarCorrectionFile,
		const HesaiLidarRawDataSturct laserReturnType,
		const unsigned int laserCount,
		const HesaiLidarPCLDataType pclDataType,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);

	HesaiLidarSDK(){};

	~HesaiLidarSDK();
	int start(); //开始数据接收和传输任务,成功时返回0,否则返回-1
	void stop(); //停止数据接收和传输任务
private:
	HesaiLidarSDK_internal *psi;
};

#endif
