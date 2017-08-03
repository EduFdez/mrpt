/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef LINE_CORRESP_H
#define LINE_CORRESP_H

#include "feat_corresp.h"

/*! This class is used to gather 3D plane observations from set of sensors to perform extrinsic calibration.
 *  These are analogous to the control points used to create panoramic images with a regular camera) from a sequence of RGBD360 observations.
 *  These permit to find the extrinsic calibration between RGB-D sensors like Asus XPL.
 *
 *  \ingroup calib_group
 */
template<typename T>
class LineCorresp : public FeatCorresp
{
  public:

    /*! Constructor */
    LineCorresp(size_t sensors = 2);

    /*! Destructor */
    virtual ~LineCorresp(){};

};

#endif // LINE_CORRESP_H
