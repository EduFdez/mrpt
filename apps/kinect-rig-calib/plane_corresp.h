/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef PLANE_CORRESP_H
#define PLANE_CORRESP_H

#include "feat_corresp.h"

/*! This class is used to gather 3D plane observations from set of sensors to perform extrinsic calibration.
 *  These are analogous to the control points used to create panoramic images with a regular camera) from a sequence of RGBD360 observations.
 *  These permit to find the extrinsic calibration between RGB-D sensors like Asus XPL.
 *
 *  Each plane correspondence is stored as a new row in mm_corresp, with 8 columns (4 paramenters for each plane {n,d} in that order).
 *
 *  \ingroup calib_group
 */
class PlaneCorresp : public FeatCorresp
{
  public:

    /*! Constructor */
    PlaneCorresp(size_t n_sensors = 2);

    /*! Destructor */
    virtual ~PlaneCorresp(){};

    /*! Get the rate of inliers near the border of the sensor (the border nearer to the next lower index Asus sensor).
     *  This only works for QVGA resolution
     */
    float inliersUpperFringe(mrpt::pbmap::Plane &plane, float fringeWidth);

    ///*! Get the rate of inliers near the border of the sensor (the border nearer to the next upper index Asus sensor) */
    float inliersLowerFringe(mrpt::pbmap::Plane &plane, float fringeWidth);
};

#endif // PLANE_CORRESP_H
