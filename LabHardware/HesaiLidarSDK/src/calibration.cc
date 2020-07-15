#include <iostream>
#include <fstream>
#include <limits>
#include <boost/filesystem.hpp>
#include <cmath>
#include "calibration.h"

namespace angles
{
double degreeToRadian(double degree)
{
    return degree * M_PI / 180;
}
}

namespace pandar_pointcloud
{

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
static const float hesai_elev_angle_map[] = {
    6.96, 5.976, 4.988, 3.996,
    2.999, 2.001, 1.667, 1.333,
    1.001, 0.667, 0.333, 0,
    -0.334, -0.667, -1.001, -1.334,
    -1.667, -2.001, -2.331, -2.667,
    -3, -3.327, -3.663, -3.996,
    -4.321, -4.657, -4.986, -5.311,
    -5.647, -5.974, -6.957, -7.934,
    -8.908, -9.871, -10.826, -11.772,
    -12.705, -13.63, -14.543, -15.444};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
static const float hesai_horizatal_azimuth_offset_map[] = {
    0.005, 0.006, 0.006, 0.006,
    -2.479, -2.479, 2.491, -4.953,
    -2.479, 2.492, -4.953, -2.479,
    2.492, -4.953, 0.007, 2.491,
    -4.953, 0.006, 4.961, -2.479,
    0.006, 4.96, -2.478, 0.006,
    4.958, -2.478, 2.488, 4.956,
    -2.477, 2.487, 2.485, 2.483,
    0.004, 0.004, 0.003, 0.003,
    -2.466, -2.463, -2.46, -2.457};

void Calibration::setDefaultCorrections()
{
    for (int i = 0; i < laserNumber; i++)
    {
        laserCorrections[i].azimuthCorrection = hesai_horizatal_azimuth_offset_map[i];
        laserCorrections[i].distanceCorrection = 0.0;
        laserCorrections[i].horizontalOffsetCorrection = 0.0;
        laserCorrections[i].verticalOffsetCorrection = 0.0;
        laserCorrections[i].verticalCorrection = hesai_elev_angle_map[i];
        laserCorrections[i].sinVertCorrection = std::sin(hesai_elev_angle_map[i] / 180.0 * M_PI);
        laserCorrections[i].cosVertCorrection = std::cos(hesai_elev_angle_map[i] / 180.0 * M_PI);
    }
}

void Calibration::read(const std::string &calibrationFile)
{
    initialized = true;
    setDefaultCorrections();
    if (calibrationFile.empty())
        return;

    boost::filesystem::path filePath(calibrationFile);
    if (!boost::filesystem::is_regular(filePath))
    {   printf("invalid lidar correction file, use default values\n");
        return;
    }

    std::ifstream ifs(filePath.string());
    if (!ifs.is_open())
        return;

    std::string line;
    if (std::getline(ifs, line))
    { // first line "Laser id,Elevation,Azimuth"
        std::cout << "parsing correction file." << std::endl;
    }

    int lineCounter = 0;
    while (std::getline(ifs, line))
    {
        if (lineCounter++ >= 40)
            break;

        int lineId = 0;
        double elev, azimuth;

        std::stringstream ss(line);
        std::string subline;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> lineId;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> elev;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> azimuth;
        lineId--;
        laserCorrections[lineId].azimuthCorrection = azimuth;
        laserCorrections[lineId].distanceCorrection = 0.0;
        laserCorrections[lineId].horizontalOffsetCorrection = 0.0;
        laserCorrections[lineId].verticalOffsetCorrection = 0.0;
        laserCorrections[lineId].verticalCorrection = elev;
        laserCorrections[lineId].sinVertCorrection = std::sin(angles::degreeToRadian(elev));
        laserCorrections[lineId].cosVertCorrection = std::cos(angles::degreeToRadian(elev));
    }
}

} /* pandar_pointcloud */
