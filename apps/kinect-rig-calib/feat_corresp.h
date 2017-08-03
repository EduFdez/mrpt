/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef FEAT_CORRESP_H
#define FEAT_CORRESP_H

#include <mrpt/math/CMatrixTemplateNumeric.h>  // For mrpt::math::CMatrixDouble
#include <map>

/*! This base class stores geometric correspondences from RGB-D observations from set of sensors to perform extrinsic calibration.
 *  These are analogous to the control points used to create panoramic images with a regular camera) from a sequence of RGBD360 observations.
 *
 *  \ingroup calib_group
 */
template<typename T>
class FeatCorresp
{
  public:

    /*! The number of sensors to calibrate */
    size_t n_sensors;

    /*! The plane correspondences between the different sensors */
    std::map<size_t, std::map<size_t, mrpt::math::CMatrixDouble> > mm_corresp;

    /*! Constructor */
    FeatCorresp(size_t sensors = 2);
    /*! Destructor */
    virtual ~FeatCorresp(){};

    /*! Load the plane correspondences between the different Asus sensors from file */
    void saveCorrespondences(const std::string & dir);

    /*! Load the plane correspondences between the different Asus sensors from file */
    void loadCorrespondences(const std::string dir);

    /*! Load the plane correspondences between a pair of Asus sensors from file */
    mrpt::math::CMatrixDouble loadCorrespondencesPair(const std::string file);
};

#endif // FEAT_CORRESP_H
