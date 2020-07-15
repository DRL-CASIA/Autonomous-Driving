
#ifndef __PANDAR_RAWDATA_H
#define __PANDAR_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <pcl/point_cloud.h>
#include "point_types.h"
#include "calibration.h"
#include "pandora_types.h"  

namespace pandar_rawdata
{

#define SOB_ANGLE_SIZE 4
#define RAW_MEASURE_SIZE 5
#define LASER_COUNT 40
#define BLOCKS_PER_PACKET 6
#define BLOCK_SIZE (RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE)
#define TIMESTAMP_SIZE 4
#define FACTORY_ID_SIZE 2
#define RESERVE_SIZE 8
#define REVOLUTION_SIZE 2
#define INFO_SIZE (TIMESTAMP_SIZE + FACTORY_ID_SIZE + RESERVE_SIZE + REVOLUTION_SIZE)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE)
#define ROTATION_MAX_UNITS 36001
#define ROTATION_RESOLUTION 0.01

#define PandoraSDK_MIN_RANGE 0.5
#define PandoraSDK_MAX_RANGE 250.0

#define LASER_RETURN_TO_DISTANCE_RATE 0.002

#define MAX_LASER_COUNT (128)
#define MAX_BLOCK_SIZE (20)
// DUAL-VERSION
#define DUAL_VERSION_SOB_ANGLE_SIZE 4
#define DUAL_VERSION_RAW_MEASURE_SIZE 3
#define DUAL_VERSION_LASER_COUNT 40
#define DUAL_VERSION_BLOCKS_PER_PACKET 10
#define DUAL_VERSION_BLOCK_SIZE (DUAL_VERSION_RAW_MEASURE_SIZE * DUAL_VERSION_LASER_COUNT + DUAL_VERSION_SOB_ANGLE_SIZE)
#define DUAL_VERSION_TIMESTAMP_SIZE 4
#define DUAL_VERSION_FACTORY_INFO_SIZE 1
#define DUAL_VERSION_ECHO_SIZE 1
#define DUAL_VERSION_RESERVE_SIZE 8
#define DUAL_VERSION_REVOLUTION_SIZE 2
#define DUAL_VERSION_INFO_SIZE (DUAL_VERSION_TIMESTAMP_SIZE + DUAL_VERSION_FACTORY_INFO_SIZE + DUAL_VERSION_ECHO_SIZE + DUAL_VERSION_RESERVE_SIZE + DUAL_VERSION_REVOLUTION_SIZE)
#define DUAL_VERSION_PACKET_SIZE (DUAL_VERSION_BLOCK_SIZE * DUAL_VERSION_BLOCKS_PER_PACKET + DUAL_VERSION_INFO_SIZE)
#define DUAL_VERSION_LASER_RETURN_TO_DISTANCE_RATE 0.004

// typedef struct RAW_MEASURE_{
//     uint32_t range;
//     uint16_t reflectivity;
// } RAW_MEASURE_T;

// typedef struct RAW_BLOCK
// {
//     uint16_t sob;
//     uint16_t azimuth;
//     RAW_MEASURE_T measures[LASER_COUNT];
// } RAW_BLOCK_T;

// typedef struct RAW_PACKET
// {
//     RAW_BLOCK_T blocks[BLOCKS_PER_PACKET];
//     uint8_t reserved[RESERVE_SIZE];
//     uint16_t revolution;
//     uint32_t timestamp;
//     uint8_t factory[2];
//     double recv_time;
// } RAW_PACKET_T;

typedef struct RAW_MEASURE_{
    double range;
    uint16_t reflectivity;
} RAW_MEASURE_T;

typedef struct RAW_BLOCK
{
    uint16_t sob;
    uint16_t azimuth;
    RAW_MEASURE_T measures[MAX_LASER_COUNT];
    // int laserCount;
} RAW_BLOCK_T;

typedef struct RAW_PACKET
{
    RAW_BLOCK_T blocks[MAX_BLOCK_SIZE];
    int blockAmount;
    uint32_t timestamp;
    unsigned int echo;
} RAW_PACKET_T;

// dual-return version
typedef struct RAW_MEASURE_DUAL_{
    unsigned int range;
    unsigned short reflectivity;
} RAW_MEASURE_DUAL_T;

typedef struct RAW_BLOCK_DUAL
{
    uint16_t sob;
    uint16_t azimuth;
    RAW_MEASURE_DUAL_T measures[DUAL_VERSION_LASER_COUNT];
} RAW_BLOCK_DUAL_T;

typedef struct RAW_PACKET_DUAL
{
    RAW_BLOCK_DUAL_T blocks[DUAL_VERSION_BLOCKS_PER_PACKET];
    uint8_t reserved[DUAL_VERSION_RESERVE_SIZE];
    uint16_t revolution;
    uint32_t timestamp;
    unsigned int echo;
    unsigned char factory;
} RAW_PACKET_DUAL_T;

class RawData
{
public:

    RawData(const std::string& correctionFile,
        const unsigned int laserReturnType,
        const unsigned int laserNumber,
        const unsigned int pclType);
    ~RawData() {}
    int setup();
    int unpack(
        PandarPacket &packet,
        PPointCloud &pc,
        time_t& gps1 , 
        GPS_STRUCT_T &gps2,
        int& lidarRotationStartAngle);
    // int unpackSingleReturn(
    //     PandarPacket &packet,
    //     PPointCloud &pc,
    //     time_t& gps1 , 
    //     GPS_STRUCT_T &gps2,
    //     double& firstStamp,
    //     int& lidarRotationStartAngle);
    // int unpackDualReturn(
    //     PandarPacket &packet,
    //     PPointCloud &pc,
    //     time_t& gps1 , 
    //     GPS_STRUCT_T &gps2,
    //     double& firstStamp,
    //     int& lidarRotationStartAngle);

private:

    typedef struct {
        std::string calibrationFile;     ///< calibration file name
        double maxRange;                ///< maximum range to publish
        double minRange;                ///< minimum range to publish
        int minAngle;                   ///< minimum angle to publish
        int maxAngle;                   ///< maximum angle to publish
    } CONFIG_T;
    CONFIG_T config_;

    pandar_pointcloud::Calibration calibration_;
    float sin_lookup_table_[ROTATION_MAX_UNITS];
    float cos_lookup_table_[ROTATION_MAX_UNITS];
    RAW_PACKET_T *bufferPacket;

	int parseRawData(RAW_PACKET_T* packet, const uint8_t* buf, const int len);
    void toPointClouds (RAW_PACKET_T* packet,int laser,  PPointCloud& pc, double stamp);

	// int parseRawData(RAW_PACKET_DUAL_T* packet, const uint8_t* buf, const int len);
    // void toPointClouds (RAW_PACKET_DUAL_T* packet,int laser,  PPointCloud& pc, double stamp, double& firstStamp);

    void toPointClouds(RAW_PACKET_T* packet,int laser, int block,PPointCloud &pc,double blockstamp);
    // void toPointClouds(RAW_PACKET_DUAL_T* packet,int laser, int block,PPointCloud &pc,double blockstamp);

	void computeXYZIR(
        PPoint& point,
        int azimuth,
		const RAW_MEASURE_T& laserReturn,
		const pandar_pointcloud::PandarLaserCorrection& correction);
    // void computeXYZIR(
    //     PPoint& point,
    //     int azimuth,
	// 	const RAW_MEASURE_DUAL_T& laserReturn,
	// 	const pandar_pointcloud::PandarLaserCorrection& correction);

    int lastBlockEnd;
    int bufferPacketSize;
    int currentPacketStart;
    unsigned int lastTimestamp;
    int lastAzumith;
    DeviceType deviceType;
    int laserCountType;
    unsigned int pclDataType;
    int pandarEnableList[MAX_LASER_COUNT];
};

} // namespace pandar_rawdata

#endif // __PANDAR_RAWDATA_H
