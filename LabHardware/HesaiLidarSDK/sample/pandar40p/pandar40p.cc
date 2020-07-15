#include "hesaiLidarSDK.h"

void gpsCallback(int timestamp)
{
    printf("gps: %d" , timestamp);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
    printf("lidar: time %lf , points %d\n", timestamp , cld->points.size());
}

int main(int argc, char **argv)
{
    HesaiLidarSDK psdk(
	2368				/* lidar data port */, 
	10110				/* gps data port */, 
	std::string("correction.csv")	/* calibration file of lidar */, 
	lidarCallback 			/* point cloud data call back */, 
	gpsCallback 			/* gps data callback */, 
	HESAI_LIDAR_RAW_DATA_STRCUT_DUAL_RETURN/* Return Mode: Dual Return data structure */, 
	40				/* laser counter */, 
	HESAI_LIDAR_PCL_DATA_TYPE_REDUCED/* pcl data alignment */
	);
    psdk.start();
    while(true)
    {
        sleep(100);
    }
}