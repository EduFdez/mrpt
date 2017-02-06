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

#include <mrpt/system/filesystem.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>  // For mrpt::math::CMatrixDouble
#include <Eigen/Dense>

//#include "calib_from_planes3D.h"

template<typename Scalar> inline Eigen::Matrix<Scalar,3,3> skew(const Eigen::Matrix<Scalar,3,1> &vec)
{
  Eigen::Matrix<Scalar,3,3> skew_matrix = Eigen::Matrix<Scalar,3,3>::Zero();
  skew_matrix(0,1) = -vec(2);
  skew_matrix(1,0) = vec(2);
  skew_matrix(0,2) = vec(1);
  skew_matrix(2,0) = -vec(1);
  skew_matrix(1,2) = -vec(0);
  skew_matrix(2,1) = vec(0);
  return skew_matrix;
}

/*! This class contains the functionality to calibrate the extrinsic parameters of a pair of non-overlapping depth cameras.
 *  This extrinsic calibration is obtained by matching planes that are observed by both sensors at the same time instants.
 *
 *  \ingroup calib_group
 */
class CalibratePairRange
{
public:

    /*! 3D plane correspondences */
    //PlaneCorresp corresp_;

    /*! The extrinsic matrix estimated by this calibration method */
    Eigen::Matrix4f Rt_estimated;

    Eigen::Matrix3f rotation;

    Eigen::Vector3f translation;

    /*! The plane correspondences between the pair of Asus sensors */
    mrpt::math::CMatrixDouble correspondences;
    mrpt::math::CMatrixDouble correspondences_cov;

    /*! Constructor */
    CalibratePairRange() //: corresp_(PlaneCorresp(2))
    {
//            std::map<unsigned, std::map<unsigned, mrpt::math::CMatrixDouble> > mm_corresp_;

//            /*! Conditioning numbers used to indicate if there is enough reliable information to calculate the extrinsic calibration */
//            std::vector<float> conditioning;

//            /*! Rotation covariance matrices from adjacent sensors */
//            std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
    }

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline void setInitRt(const std::string Rt_file)
    {
        if( !mrpt::system::fileExists(Rt_file) )
            throw std::runtime_error("\nERROR...");

        Rt_estimated.loadFromTextFile(Rt_file);
    }

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline void setInitRt(Eigen::Matrix4f initRt)
    {
        Rt_estimated = initRt;
        //      Rt_estimated = Eigen::Matrix4f::Identity();
        //      Rt_estimated(1,1) = Rt_estimated(2,2) = cos(45*PI/180);
        //      Rt_estimated(1,2) = -sin(45*PI/180);
        //      Rt_estimated(2,1) = -Rt_estimated(1,2);
    }

    /*! Calculate the angular error of the plane correspondences.*/
    float calcCorrespRotError(Eigen::Matrix3f &Rot_);

    /*! \overload Calculate the angular error of the plane correspondences.*/
    inline float calcCorrespRotError(Eigen::Matrix4f &Rt_)
    {
        Eigen::Matrix3f R = Rt_.block(0,0,3,3);
        return calcCorrespRotError(R);
    }

    /*! \overload Calculate the angular error of the plane correspondences.*/
    inline float calcCorrespRotError()
    {
        Eigen::Matrix3f R = Rt_estimated.block(0,0,3,3);
        return calcCorrespRotError(R);
    }

    //    float calcCorrespTransError(Eigen::Matrix3f &Rot_)

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline Eigen::Vector3f calcScoreRotation(Eigen::Vector3f &n1, Eigen::Vector3f &n2)
    {
        Eigen::Vector3f n2_ref1 = Rt_estimated.block(0,0,3,3)*n2;
        Eigen::Vector3f score = - skew(n2_ref1) * n1;

        return score;
    }

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline Eigen::Vector3f calcScoreTranslation(Eigen::Vector3f &n1, float &d1, float &d2)
    {
        Eigen::Vector3f score = (d1 - d2) * n1;
        return score;
    }

    /*! Calculate the Fisher Information Matrix (FIM) of the rotation estimate. */
    Eigen::Matrix3f calcFIMRotation();

    /*! Calculate the Fisher Information Matrix (FIM) of the translation estimate. */
    Eigen::Matrix3f calcFIMTranslation();

    //    Eigen::Matrix3f calcFisherInfMat(const int weightedLS = 0)

    /*! Calibrate the relative rotation of the pair. */
    Eigen::Matrix3f CalibrateRotation(int weightedLS = 0);

    /*! \overload Calibrate the relative rotation of the pair (double precision). */
    Eigen::Matrix3f CalibrateRotationD(int weightedLS = 0);

    /*! Calibrate the relative rotation of the pair iteratively on a manifold formulation. */
    Eigen::Matrix3f CalibrateRotationManifold(int weightedLS = 0);

    /*! Calibrate the relative translation of the pair. */
    Eigen::Vector3f CalibrateTranslation(int weightedLS = 0);

    /*! Calibrate the relative rigid transformation (Rt) of the pair. */
    void CalibratePair();

};
