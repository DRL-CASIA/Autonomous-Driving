#include "hesaiLidarSDK.h"
#include "hesaiLidarSDK_IN.h"

#ifdef HESAI_WITH_CAMERA 
HesaiLidarSDK::HesaiLidarSDK(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	const std::string intrinsicFile,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	const unsigned short lidarRecvPort,
	const unsigned short gpsRecvPort,
	const double startAngle,
	const std::string lidarCorrectionFile,
	
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
	boost::function<void(unsigned int timestamp)> gpsCallback,
	const HesaiLidarRawDataSturct laserReturnType,
	const unsigned int laserCount,
	const HesaiLidarPCLDataType pclDataType)
{
	psi = new HesaiLidarSDK_internal(
		pandoraIP,
		pandoraCameraPort,
		intrinsicFile,
		cameraCallback,
		lidarRecvPort,
		gpsRecvPort,
		startAngle,
		lidarCorrectionFile,
		lidarCallback,
		gpsCallback,
		(unsigned int)laserReturnType, laserCount, (unsigned int)pclDataType);
}

#endif

HesaiLidarSDK::HesaiLidarSDK(
#ifdef HESAI_WITH_CAMERA 
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
#endif
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	psi = new HesaiLidarSDK_internal(
#ifdef HESAI_WITH_CAMERA 
		pandoraIP,
		pandoraCameraPort,
		cameraCallback,
#endif
		lidarCallback);
}

HesaiLidarSDK::HesaiLidarSDK(
	const unsigned short lidarRecvPort,
	const unsigned short gpsRecvPort,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
	boost::function<void(unsigned int timestamp)> gpsCallback,
	const HesaiLidarRawDataSturct laserReturnType,
	const unsigned int laserCount,
	const HesaiLidarPCLDataType pclDataType)
{
	psi = new HesaiLidarSDK_internal(
#ifdef HESAI_WITH_CAMERA 
		std::string(""),
		9870,
		std::string(""),
		NULL,
#endif
		lidarRecvPort,
		gpsRecvPort,
		0,
		lidarCorrectionFile,
		lidarCallback,
		gpsCallback,
		(unsigned int)laserReturnType, laserCount, (unsigned int)pclDataType);
}

HesaiLidarSDK::HesaiLidarSDK(
		const std::string pcapPath,
		const std::string lidarCorrectionFile,
		const HesaiLidarRawDataSturct laserReturnType,
		const unsigned int laserCount,
		const HesaiLidarPCLDataType pclDataType,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback)
{
	psi = new HesaiLidarSDK_internal(
		pcapPath,
		lidarCorrectionFile,
		(unsigned int)laserReturnType, laserCount, (unsigned int)pclDataType,
		lidarCallback);
}

HesaiLidarSDK::~HesaiLidarSDK()
{
	delete psi;
}

int HesaiLidarSDK::start()
{
	psi->start();
}
void HesaiLidarSDK::stop()
{
	psi->stop();
}
