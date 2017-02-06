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

#ifndef CALIB_FROM_PLANES_3D_H
#define CALIB_FROM_PLANES_3D_H

#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/Miscellaneous.h>


/*! This class is used to gather 3D plane observations from set of sensors to perform extrinsic calibration.
 *  These are analogous to the control points used to create panoramic images with a regular camera) from a sequence of RGBD360 observations.
 *  These permit to find the extrinsic calibration between RGB-D sensors like Asus XPL.
 *
 *  \ingroup calib_group
 */
class PlaneCorresp
{
  public:

    /*! Thenumber of sensors to calibrate */
    size_t n_sensors_;

    /*! The plane correspondences between the different Asus sensors */
    std::map<unsigned, std::map<unsigned, mrpt::math::CMatrixDouble> > mm_corresp_;

    /*! Conditioning numbers used to indicate if there is enough reliable information to calculate the extrinsic calibration */
    std::vector<float> conditioning_;

    /*! Rotation covariance matrices from adjacent sensors */
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances_;

    /*! Constructor */
    PlaneCorresp(size_t n_sensors = 2);

    /*! Destructor */
    virtual ~PlaneCorresp(){};

    /*! Load the plane correspondences between the different Asus sensors from file */
    void saveCorrespondences(const std::string &planeCorrespDirectory);

    /*! Load the plane correspondences between a pair of Asus sensors from file */
    mrpt::math::CMatrixDouble getPlaneCorrespondences(const std::string matchedPlanesFile);

    /*! Load the plane correspondences between the different Asus sensors from file */
    void loadPlaneCorrespondences(const std::string planeCorrespDirectory);

    /*! Get the rate of inliers near the border of the sensor (the border nearer to the next lower index Asus sensor).
     *  This only works for QVGA resolution
     */
    float inliersUpperFringe(mrpt::pbmap::Plane &plane, float fringeWidth);

    ///*! Get the rate of inliers near the border of the sensor (the border nearer to the next upper index Asus sensor) */
    float inliersLowerFringe(mrpt::pbmap::Plane &plane, float fringeWidth);

    /*! Print the number of correspondences and the conditioning number to the standard output */
    void printConditioning();

    //    /*! Update adjacent conditioning (information between a pair of adjacent sensors) */
    //    void updateAdjacentConditioning(unsigned couple_id, pair< Eigen::Vector4f, Eigen::Vector4f> &match)
    //    {
    //      ++conditioning_measCount[couple_id];
    //      covariances_[couple_id] += match.second.head(3) * match.first.head(3).transpose();
    ////      Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariances_[couple_id], Eigen::ComputeFullU | Eigen::ComputeFullV);
    ////      conditioning[couple_id] = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    //    }

    /*! Calculate adjacent conditioning (information between a pair of adjacent sensors) */
    void calcAdjacentConditioning(unsigned couple_id);

};

#endif // CALIB_FROM_PLANES_3D_H
