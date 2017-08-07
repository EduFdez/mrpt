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
  protected:
    /*! Indices of the pair of sensors being evaluated */
//    size_t sensor1, sensor2;
    std::array<size_t,2> sensor_pair;

    /*! Calibration parameter to consider if the problem is ill-posed */
    const double threshold_conditioning = 1e-3;

  public:

    /*! Number of sensors to calibrate */
    size_t num_sensors;

    /*! The extrinsic matrix estimated by this calibration method */
    std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > Rt_estimated;
//    std::vector< Eigen::Transform<T,3,Eigen::Affine>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > Rt_estimated;

    /*! Conditioning numbers used to indicate if there is enough reliable information to calculate the extrinsic calibration.
        It is computed as the quotient between the smallest and largest eigenvalues of the covarianve matrix (inverse Hessian). */
    std::map<size_t, std::map<size_t, double> > mm_conditioning;

    /*! Rotation covariance matrices from adjacent sensors */
    std::map<size_t, std::map<size_t, Eigen::Matrix<T,3,3> > > mm_covariance;
//    std::vector<Eigen::Matrix<T,3,3>, Eigen::aligned_allocator<Eigen::Matrix<T,3,3> > > covariances;

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

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline void setInitRt(const std::string Rt_file, const size_t sensor_id = 1)
    {
        if( !mrpt::system::fileExists(Rt_file) || sensor_id >= Rt_estimated.size() )
            throw std::runtime_error("\nERROR...");

        Rt_estimated[sensor_id].loadFromTextFile(Rt_file);
    }

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline void setInitRt(Eigen::Matrix<T,4,4> initRt, const size_t sensor_id = 1)
    {
        if( sensor_id >= Rt_estimated.size() || sensor_id >= Rt_estimated.size() )
            throw std::runtime_error("\nERROR...");

        Rt_estimated[sensor_id] = initRt;
    }

    /*! Print the number of correspondences and the conditioning number to the standard output */
    void printConditioning()
    {
        std::cout << "Conditioning\n";
        for(std::map<size_t, std::map<size_t, double> >::iterator it1=mm_conditioning.begin(); it1 != mm_conditioning.end(); it1++)
            for(std::map<size_t, std::map<size_t, double> >::iterator it2=it1->second.begin(); it2 != it1->second.end(); it2++)
                std::cout << "    sensors " << it1->first << "-" << it2->first << " : " << mm_conditioning[sensor_id][sensor_id+1] << "\n";
        std::cout << std::endl;
    }

    /*! Calculate adjacent conditioning (information between a pair of adjacent sensors) */
    void calcConditioningPair(const size_t sensor1, const size_t sensor2)
    {
        if( sensor1 > sensor2 )
            throw std::runtime_error("\nERROR: ExtrinsicCalib::calcConditioningPair sensor1 > sensor2!");

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(mm_covariance[sensor1][sensor2], Eigen::ComputeFullU | Eigen::ComputeFullV);
        mm_conditioning[sensor1][sensor2] = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();
        if( std::isnan(mm_conditioning[sensor1][sensor2]) )
            mm_conditioning[sensor1][sensor2] = 0.0;
    }

};
