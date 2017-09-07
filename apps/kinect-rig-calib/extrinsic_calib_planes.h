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
#include "feat_corresp.h"
//#include "plane_corresp.h"
//#include <mrpt/system/filesystem.h>
#include <mrpt/pbmap/PbMap.h>

typedef double T;

/*! This class contains the functionality to calibrate the extrinsic parameters of a pair of non-overlapping depth cameras.
 *  This extrinsic calibration is obtained by matching planes that are observed by both sensors at the same time instants.
 */
//template<typename T>
class ExtrinsicCalibPlanes : public virtual ExtrinsicCalib//<T>
{    
  public:

//    /*! The current extrinsic calibration parameters */
//    ExtrinsicCalib<T> * calib;

    /*! Constructor */
    ExtrinsicCalibPlanes() : b_wait_plane_confirm(false)
    {
    }
//    ExtrinsicCalibPlanes(const ExtrinsicCalib<T> * cal): calib(cal)
//    {
//        v_pbmap.resize(calib->n_sensors);
//    }

    /*! Extract plane correspondences between the different sensors.*/
    void getCorrespondences(const std::vector<pcl::PointCloud<PointT>::Ptr> & cloud);

    /*! Calculate the angular error of the plane correspondences.*/
    double calcRotationErrorPair(const mrpt::math::CMatrixDouble & correspondences, const Eigen::Matrix<T,3,3> & Rot, bool in_deg = false);

    /*! Calculate the angular error of the plane correspondences.*/
    inline double calcRotationErrorPair(const size_t sensor1, const size_t sensor2, bool in_deg = false)
    {
        return calcRotationErrorPair( planes.mm_corresp[sensor1][sensor2], Rt_estimated[sensor1].block<3,3>(0,0).transpose()*Rt_estimated[sensor2].block<3,3>(0,0) );
    }

    /*! Calculate the angular error of the plane correspondences.*/
    double calcRotationError(const std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > & Rt, bool in_deg = false);

//    /*! \overload Calculate the angular error of the plane correspondences.*/
//    inline double calcRotationError() { return calcRotationError(Rt_estimated); }

    double calcTranslationErrorPair(const mrpt::math::CMatrixDouble & correspondences, const Eigen::Matrix<T,4,4> & Rt1, const Eigen::Matrix<T,4,4> & Rt2, bool in_meters = false);

    double calcTranslationError(const std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > & Rt, bool in_meters = false);

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline Eigen::Matrix<T,3,1> calcScoreTranslation(Eigen::Matrix<T,3,1> &n1, float &d1, float &d2)
    {
        Eigen::Matrix<T,3,1> score = (d1 - d2) * n1;
        return score;
    }

    /*! Calculate the Fisher Information Matrix (FIM) of the rotation estimate. */
    Eigen::Matrix<T,3,3> calcRotationFIM(const mrpt::math::CMatrixDouble & correspondences);

    /*! Calculate the Fisher Information Matrix (FIM) of the translation estimate. */
    Eigen::Matrix<T,3,3> calcTranslationFIM();

    //    Eigen::Matrix<T,3,3> calcFisherInfMat(const int weightedLS = 0)

    /*! Calibrate the relative rotation between the pair of sensors. Closed form solution. */
    Eigen::Matrix<T,3,3> CalibrateRotationPair(const size_t sensor1 = 0, const size_t sensor2 = 1);//, const bool weight_uncertainty = false);

    /*! Calibrate the relative translation between the pair of sensors. Closed form solution (linear LS). */
    Eigen::Matrix<T,3,1> CalibrateTranslationPair(const size_t sensor1 = 0, const size_t sensor2 = 1);//, const bool weight_uncertainty = false);

    /*! Get the rotation of each sensor in a multisensor setup. The first sensor (sensor_id=0) is taken as reference. */
    void CalibrateRotationManifold(const bool weight_uncertainty = false);

    /*! Get the rotation of each sensor in a multisensor setup. The first sensor (sensor_id=0) is taken as reference. */
    void CalibrateTranslation(const bool weight_uncertainty = false);

    /*! Calibrate the relative rigid transformation (Rt) of the pair. */
    void Calibrate();

  protected:

    /*! The plane correspondences between the different sensors */
    //    PlaneCorresp planes;
    FeatCorresp planes;

    /*! A PbMap containing all the planes of the current observation (approx. synchronization) */
    mrpt::pbmap::PbMap all_planes;

    /*! Indices of the candidate correspondences (for visualization) */
    std::array<size_t,2> plane_candidate_all;

    /*! Indices of the matched correspondences from the current observation (for visualization) */
    std::map< size_t, std::map< size_t, std::map<size_t, size_t> > > mmm_plane_matches_all;

    /*! True if waiting for visual confirmation (for visualization) */
    bool b_wait_plane_confirm;

    /*! Parameters for plane segmentation */
    float th_dist_plane;
    float th_angle_plane;
    const size_t min_inliers = 2e4;
};
