/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "plane_corresp.h"

using namespace std;

PlaneCorresp::PlaneCorresp(size_t n_sensors) :
    n_sensors_(n_sensors)
{
    std::cout << "PlaneCorresp... n_sensors_ " << n_sensors_ << std::endl;
    for(unsigned sensor_id1=0; sensor_id1 < n_sensors_; sensor_id1++)
    {
        mm_corresp[sensor_id1] = std::map<unsigned, mrpt::math::CMatrixDouble>();
        for(unsigned sensor_id2=sensor_id1+1; sensor_id2 < n_sensors_; sensor_id2++)
            mm_corresp[sensor_id1][sensor_id2] = mrpt::math::CMatrixDouble(0, 10);
    }
}

/*! Get the rate of inliers near the border of the sensor (the border nearer to the next lower index Asus sensor) */
float PlaneCorresp::inliersUpperFringe(mrpt::pbmap::Plane &plane, float fringeWidth) // This only works for QVGA resolution
{
    size_t count = 0;
    size_t im_size = 320 * 240;
    int limit = fringeWidth * im_size;
    for(size_t i=0; i < plane.inliers.size(); ++i)
        if(plane.inliers[i] < limit)
            ++count;

    return float(count) / (im_size*fringeWidth);
}

/*! Get the rate of inliers near the border of the sensor (the border nearer to the next upper index Asus sensor) */
float PlaneCorresp::inliersLowerFringe(mrpt::pbmap::Plane &plane, float fringeWidth)
{
    size_t count = 0;
    size_t im_size = 320 * 240;
    int limit = (1 - fringeWidth) * im_size;
    for(unsigned i=0; i < plane.inliers.size(); ++i)
        if(plane.inliers[i] > limit)
            ++count;

    return float(count) / (im_size*fringeWidth);
}