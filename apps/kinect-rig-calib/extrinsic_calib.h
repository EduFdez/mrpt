/*
 *  Copyright (c) 2013, Universidad de Málaga  - Grupo MAPIR
 *                      INRIA Sophia Antipolis - LAGADIC Team
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: Eduardo Fernandez-Moral
 */

#pragma once

#include <mrpt/math/CMatrixTemplateNumeric.h>  // For mrpt::math::CMatrixDouble
#include <mrpt/pbmap/PbMap.h>
#include <Eigen/Geometry>
#include <map>

/*! This generic class serves as base class for extrinsic calibration. It stores the number of sensors to calibrate and their poses.
 */
template<typename T>
class ExtrinsicCalib
{
  public:

    /*! Number of sensors to calibrate */
    size_t num_sensors;

    /*! The extrinsic matrix estimated by this calibration method */
    std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > Rt_estimated;
//    std::vector< Eigen::Transform<T,3,Eigen::Affine>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > Rt_estimated;

    /*! Conditioning numbers used to indicate if there is enough reliable information to calculate the extrinsic calibration */
    std::map<size_t, std::map<size_t, double> > conditioning;

    /*! Rotation covariance matrices from adjacent sensors */
    std::vector<Eigen::Matrix<T,3,3>, Eigen::aligned_allocator<Eigen::Matrix<T,3,3> > > covariances;

    /*! This meassure represents the confidence region for the initial poses for the translation (e.g. like the largest eigenvalue of the covariance matrix) */
    std::vector<double> v_approx_trans;

    /*! This meassure represents the confidence region for the initial poses for the rotation (e.g. like the largest eigenvalue of the covariance matrix) */
    std::vector<double> v_approx_rot;

    /*! Planes extracted from the current observation */
    std::vector<mrpt::pbmap::PbMap> v_pbmap;

    /*! Constructor */
    ExtrinsicCalib(const size_t n_sensors = 2): num_sensors(n_sensors)
    {
        if(!std::is_floating_point<T>::value)
            throw std::runtime_error("\nExtrinsicCalibPlanes template type is not valid, not floating point.");

//        Rt_estimated = std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > >(num_sensors, Eigen::Matrix<T,4,4>::Identity());
    }

    /*! Print the number of correspondences and the conditioning number to the standard output */
    virtual void printConditioning() = 0;

    /*! Calculate adjacent conditioning (information between a pair of adjacent sensors) */
    virtual void calcAdjacentConditioning(unsigned couple_id) = 0;
};
