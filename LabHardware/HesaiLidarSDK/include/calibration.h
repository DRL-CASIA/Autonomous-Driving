#ifndef __PANDAR_CALIBRATION_H
#define __PANDAR_CALIBRATION_H

#include <map>
#include <string>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace angles
{
  double degreeToRadian(double degrees);
}

namespace pandar_pointcloud {

struct PandarLaserCorrection
{
    double azimuthCorrection;
    double verticalCorrection;
    double distanceCorrection;
    double verticalOffsetCorrection;
    double horizontalOffsetCorrection;
    double sinVertCorrection;
    double cosVertCorrection;
    double sinVertOffsetCorrection;
    double cosVertOffsetCorrection;
};

class Calibration {

public:
    Calibration(): initialized(false)
	{}
    Calibration(const std::string& calibrationFile)
    {
        read(calibrationFile);
    }

    void read(const std::string& calibrationFile);
    static const int laserNumber = 40;
	PandarLaserCorrection laserCorrections[laserNumber];
    bool initialized;

private:
    void setDefaultCorrections ();
};

} /* pandar_pointcloud */

#endif /* end of include guard: __PANDAR_CALIBRATION_H */
