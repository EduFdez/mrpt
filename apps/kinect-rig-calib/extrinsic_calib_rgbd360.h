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

#include "calib_from_planes3D.h"
#include <rv/sensor/rgbd360_calib.h>

#define NUM_ASUS_SENSORS 4

namespace rv
{
  namespace calib
  {
    /*! This class contains the functionality to calibrate the extrinsic parameters of the omnidirectional RGB-D device (RGBD360).
     *  This extrinsic calibration is obtained by matching planes that are observed by several Asus XPL sensors at the same time.
     *
     *  \ingroup calib_group
     */
    class CalibrateRgbd360
    {
      private:

//        /*! The number of sensors composing the rig */
//        size_t n_sensors_;

        /*! Conditioning of the system of equations used to indicate if there is enough reliable information to calculate the extrinsic calibration */
        float conditioning_;

        /*! Hessian of the of the least-squares problem. This container is used indifferently for both rotation and translation as both systems are decoupled and have the same dimensions */
        Eigen::MatrixXf hessian_;

        /*! Gradient of the of the least-squares problem. This container is used indifferently for both rotation and translation as both systems are decoupled and have the same dimensions */
        Eigen::VectorXf gradient_;

        /*! Calculate system's conditioning */
        inline float calcConditioning()
        {
            const int n_DoF = 3 * (NUM_ASUS_SENSORS - 1);
            Eigen::JacobiSVD<Eigen::Matrix<float,n_DoF,n_DoF> > svd(hessian_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            conditioning_ = svd.singularValues().maxCoeff() / svd.singularValues().minCoeff();

            return conditioning_;
        }

      public:

//        /*! Estimated calibration */
//        Rgbd360Calib *_calib_estim_;

        /*! The plane correspondences between the different Asus sensors */
        //    std::map<unsigned, std::map<unsigned, mrpt::math::CMatrixDouble> > mm_corresp_;
        PlaneCorresp matchedPlanes_;

        /*! This threshold limits the unevenness of information in the different degrees of freedom (DoF) */
        float threshold_conditioning_;

        /*! The extrinsic parameters given by the construction specifications of the omnidirectional sensor */
        Eigen::Matrix4f Rt_specs_[NUM_ASUS_SENSORS];

        /*! The extrinsic parameter matrices estimated by this calibration method */
        Eigen::Matrix4f Rt_estimated_[NUM_ASUS_SENSORS];

        /*! Constructor */
        CalibrateRgbd360() : // const size_t n_sensors
//            _calib_estim_()
//            matchedPlanes_(NUM_ASUS_SENSORS),
            matchedPlanes_(4),
            threshold_conditioning_(10000)
        {
            //      string mouseMsg2D ("Mouse coordinates in image viewer");
            //      string keyMsg2D ("Key event for image viewer");
            //      viewer.registerKeyboardCallback(&RGBD360_Visualizer::keyboard_callback, *this, static_cast<void*> (&keyMsg2D));
        }

        /*! Load the extrinsic parameters given by the construction specifications of the omnidirectional sensor */
        void loadConstructionSpecs();

        /*! Get the sum of squared rotational errors for the input extrinsic matrices. TODO: the input argument of this function is unsafe -> fix it */
        float calcCorrespRotError(Eigen::Matrix4f *Rt_);

        /*! Get the sum of weighted squared rotational errors for the input extrinsic matrices. TODO: the input argument of this function is unsafe -> fix it */
        float calcCorrespRotErrorWeight(Eigen::Matrix4f *Rt_);

        /*! Get the sum of squared translational errors for the input extrinsic matrices. TODO: the input argument of this function is unsafe -> fix it */
        float calcCorrespTransError(Eigen::Matrix4f *Rt_);

        /*! Get the rotation of each sensor in the multisensor RGBD360 setup */
        void CalibrateRotation(int weightedLS = 0);

        /*! Get the translation of each sensor in the multisensor RGBD360 setup. Warning: this method has being implemented to be applied always after rotation calibration */
        void CalibrateTranslation(int weightedLS = 0);

        /*! Get the Rt of each sensor in the multisensor RGBD360 setup */
        inline void Calibrate()
        {
            CalibrateRotation();
            CalibrateTranslation();
        }

        /*! \brief Save the estimated calibration (asking the user for agreement).
         * \param[in] path to save the calibration matrices
         * \param[in] filename root name of the calibration matrices, e.g. "Rt".
         */
        void saveCalibration_askUser(const std::string & path, const std::string & filename);
    };
  }
}
