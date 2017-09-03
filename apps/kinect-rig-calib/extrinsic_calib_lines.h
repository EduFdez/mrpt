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
//#include "line_corresp.h"
#include <mrpt/system/filesystem.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>  // For mrpt::math::CMatrixDouble
//#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/utils/TCamera.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

typedef double T;

/*! This class contains the functionality to calibrate the extrinsic parameters of a pair of non-overlapping depth cameras.
 *  This extrinsic calibration is obtained by matching lines that are observed by both sensors at the same time instants.
 *
 *  \ingroup calib_group
 */
//template<typename T>
class ExtrinsicCalibLines : public virtual ExtrinsicCalib//<T>
{
protected:

    size_t min_pixels_line;
    std::vector< std::vector<cv::Vec4i> > vv_segments2D;
    cv::Vec4i line_match1;
    cv::Vec4i line_match2;
    std::vector< std::vector<Eigen::Matrix<T,3,1> > > vv_segment_n; // The normal vector to the plane containing the 2D line segment and the optical center
    std::vector< std::vector<mrpt::math::TLine3D> > vv_lines3D;
    std::vector< std::vector<Eigen::Matrix<T,6,1> > > vv_segments3D;
    std::vector< std::vector<bool> > vv_line_has3D;

    /*! Indices of the candidate correspondences */
    std::array<size_t,2> line_candidate;
//    std::map<unsigned, unsigned> line_corresp;

//    /*! True if waiting for visual confirmation */
//    bool b_wait_line_confirm;

    /*! The plane correspondences between the different sensors */
//    LineCorresp<T> lines;
    FeatCorresp lines;

    /*! The coordinates of the optical center of the rgb cameras */
    std::vector<float> cx, cy;

public:

//    /*! The current extrinsic calibration parameters */
//    mrpt::math::TLine3D makeTLine3D(const Eigen::Vector3f & p1, const Eigen::Vector3f & p2)
//    {
//        mrpt::math::TLine3D line;
//    }

    /*! The current extrinsic calibration parameters */
//    ExtrinsicCalib<T> * calib;

    /*! Constructor */
    ExtrinsicCalibLines() : min_pixels_line(150)//, b_wait_line_confirm(false) //calib(cal),
    {
//            std::map<unsigned, std::map<unsigned, mrpt::math::CMatrixDouble> > mm_corresp_;

//            /*! Conditioning numbers used to indicate if there is enough reliable information to calculate the extrinsic calibration */
//            std::vector<float> conditioning;

//            /*! Rotation covariance matrices from adjacent sensors */
//            std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
    }

    /*! Extract 3D lines from 2D segments in a RGB-D image.*/
    static void getSegments3D(const mrpt::utils::TCamera & cam, const pcl::PointCloud<PointT>::Ptr & cloud, const mrpt::pbmap::PbMap & pbmap, const std::vector<cv::Vec4i> & segments2D,
                              std::vector<Eigen::Matrix<T,3,1> > &segments_n, std::vector<Eigen::Matrix<T,6,1> > & segments3D, std::vector<bool> & line_has3D);

    /*! Extract line correspondences between the different sensors.*/
    void getCorrespondences(const std::vector<cv::Mat> & rgb, const std::vector<pcl::PointCloud<PointT>::Ptr> & cloud);

    /*! Extract plane correspondences between the different sensors.*/
    void getCorrespondences(const std::vector<pcl::PointCloud<PointT>::Ptr> & cloud);

    /*! Calculate the angular error of the plane correspondences.*/
    double calcRotationErrorPair(const mrpt::math::CMatrixDouble & correspondences, const Eigen::Matrix<T,3,3> & Rot1, const Eigen::Matrix<T,3,3> & Rot2, bool in_deg = false);

    /*! Calculate the angular error of the plane correspondences.*/
    inline double calcRotationErrorPair(const size_t sensor1, const size_t sensor2, bool in_deg = false)
    {
        return calcRotationErrorPair( lines.mm_corresp[sensor1][sensor2], Rt_estimated[sensor1].block<3,3>(0,0), Rt_estimated[sensor2].block<3,3>(0,0) );
    }

    /*! Calculate the angular error of the plane correspondences.*/
    double calcRotationError(const std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > & Rt, bool in_deg = false);

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

    /*! Calibrate the relative rotation between the pair of sensors. Closed form solution. */
    Eigen::Matrix<T,3,3> ApproximateRotationZeroTrans(const size_t sensor1 = 0, const size_t sensor2 = 1, const bool weight_uncertainty = false);

    /*! Calibrate the relative translation between the pair of sensors. Closed form solution (linear LS). */
    Eigen::Matrix<T,3,1> CalibrateTranslationPair(const size_t sensor1 = 0, const size_t sensor2 = 1, const bool weight_uncertainty = false);

    /*! Get the rotation of each sensor in a multisensor setup. The first sensor (sensor_id=0) is taken as reference. */
    void CalibrateRotationManifold(const bool weight_uncertainty = false);

    /*! Get the rotation of each sensor in a multisensor setup. The first sensor (sensor_id=0) is taken as reference. */
    void CalibrateTranslation(const bool weight_uncertainty = false);

    /*! Calibrate the relative rigid transformation (Rt) of the pair. */
    void Calibrate();
};
