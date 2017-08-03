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

#include "extrinsic_calib.h"
#include "plane_corresp.h"
//#include <mrpt/system/filesystem.h>
#include <mrpt/pbmap/PbMap.h>

/*! This class contains the functionality to calibrate the extrinsic parameters of a pair of non-overlapping depth cameras.
 *  This extrinsic calibration is obtained by matching planes that are observed by both sensors at the same time instants.
 */
template<typename T>
class ExtrinsicCalibPlanes : public virtual ExtrinsicCalib<T>
{    
  private:
    using ExtrinsicCalib<T>::num_sensors;
    using ExtrinsicCalib<T>::Rt_estimated;

    /*! Indices of the candidate correspondences */
    size_t plane_candidate[2];
    size_t plane_candidate_all[2];
    map<unsigned, unsigned> plane_corresp;

  public:

//    /*! The current extrinsic calibration parameters */
//    ExtrinsicCalib<T> * calib;

    /*! The plane correspondences between the different sensors */
    PlaneCorresp<T> planes;

    /*! The plane correspondences between the different sensors */
    std::map<size_t, std::map<size_t, mrpt::math::CMatrixDouble> > mm_corresp;

    /*! Whether to visually check or not every proposed correspondence */
    bool b_confirm_visually;

    /*! Return value of visual confirmation */
    int confirmed_corresp; // {0: waiting for confirmation, 1: confirmation true, -1: confirmation false (do not use this corresp)}

    /*! Number of plane correspondences */
    size_t n_plane_corresp;

    /*! Constructor */
    ExtrinsicCalibPlanes() : b_confirm_visually(false)
    {
    }
//    ExtrinsicCalibPlanes(const ExtrinsicCalib<T> * cal): calib(cal)
//    {
//        v_pbmap.resize(calib->n_sensors);
//    }

    /*! Extract plane correspondences between the different sensors.*/
    void getCorrespondences(const std::vector<pcl::PointCloud<PointT>::Ptr> & cloud);

    /*! Calculate the angular error of the plane correspondences.*/
    double calcCorrespRotError(const std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > & Rt);

    /*! \overload Calculate the angular error of the plane correspondences.*/
    inline double calcCorrespRotError() { return calcCorrespRotError(Rt_estimated); }

    //    float calcCorrespTransError(Eigen::Matrix<T,3,3> &Rot_)

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline Eigen::Matrix<T,3,1> calcScoreRotation(Eigen::Matrix<T,3,1> &n1, Eigen::Matrix<T,3,1> &n2)
    {
        Eigen::Matrix<T,3,1> n2_ref1 = Rt_estimated.block(0,0,3,3)*n2;
        Eigen::Matrix<T,3,1> score = - skew(n2_ref1) * n1;

        return score;
    }

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline Eigen::Matrix<T,3,1> calcScoreTranslation(Eigen::Matrix<T,3,1> &n1, float &d1, float &d2)
    {
        Eigen::Matrix<T,3,1> score = (d1 - d2) * n1;
        return score;
    }

    /*! Calculate the Fisher Information Matrix (FIM) of the rotation estimate. */
    Eigen::Matrix<T,3,3> calcFIMRotation(const mrpt::math::CMatrixDouble & correspondences);

    /*! Calculate the Fisher Information Matrix (FIM) of the translation estimate. */
    Eigen::Matrix<T,3,3> calcFIMTranslation();

    //    Eigen::Matrix<T,3,3> calcFisherInfMat(const int weightedLS = 0)

    /*! Calibrate the relative rotation of the pair. */
    Eigen::Matrix<T,3,3> CalibrateRotation(int weightedLS = 0);

    /*! \overload Calibrate the relative rotation of the pair (double precision). */
    Eigen::Matrix<T,3,3> CalibrateRotationD(int weightedLS = 0);

    /*! Calibrate the relative rotation of the pair iteratively on a manifold formulation. */
    Eigen::Matrix<T,3,3> CalibrateRotationManifold(int weightedLS = 0);

    /*! Calibrate the relative translation of the pair. */
    Eigen::Matrix<T,3,1> CalibrateTranslation(int weightedLS = 0);

    /*! Calibrate the relative rigid transformation (Rt) of the pair. */
    void Calibrate();

    /*! Print the number of correspondences and the conditioning number to the standard output */
    void printConditioning();

    /*! Calculate adjacent conditioning (information between a pair of adjacent sensors) */
    void calcAdjacentConditioning(unsigned couple_id);
};
