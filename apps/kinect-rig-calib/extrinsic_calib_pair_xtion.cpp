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

#include "extrinsic_calib_pair_xtion.h"
#include <iostream>
#include <mrpt/poses/CPose3D.h>

#define VISUALIZE_SENSOR_DATA 0
#define SHOW_IMAGES 0

using namespace std;

float CalibratePairRange::calcCorrespRotError(Eigen::Matrix3f &Rot_)
{
    float accum_error2 = 0.0;
    //      float accum_error_deg = 0.0;
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //        float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
        float weight = 1.0;
        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        Eigen::Vector3f n_ii = Rot_ * n_obs_ii;
        Eigen::Vector3f rot_error = (n_obs_i - n_ii);
        accum_error2 += weight * fabs(rot_error.dot(rot_error));
        //        accum_error_deg += acos(fabs(rot_error.dot(rot_error)));
    }

    //      std::cout << "AvError deg " << accum_error_deg/correspondences.rows() << std::endl;
    return accum_error2/correspondences.rows();
}

//    float calcCorrespTransError(Eigen::Matrix3f &Rot_)
//    {
//      float accum_error2 = 0.0;
//      float accum_error_m = 0.0;
//      for(unsigned i=0; i < correspondences.rows(); i++)
//      {
////        float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
//        float weight = 1.0;
//        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        float trans_error = (correspondences(i,3) - correspondences(i,7) + n_obs_i.dot());
//        accum_error2 += weight * fabs(rot_error.dot(rot_error));
//        accum_error_deg += acos(fabs(rot_error.dot(rot_error)));
//      }
//
//      std::cout << "AvError deg " << accum_error_deg/correspondences.rows() << std::endl;
//      return accum_error2/correspondences.rows();
//    }

Eigen::Matrix3f CalibratePairRange::calcFIMRotation()
{
    Eigen::Matrix3f FIM = Eigen::Matrix3f::Zero();

    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);

        Eigen::Vector3f score = calcScoreRotation(n_obs_i, n_obs_ii);

        FIM += score * score.transpose();
    }

    return FIM;
}

Eigen::Matrix3f CalibratePairRange::calcFIMTranslation()
{
    Eigen::Matrix3f FIM = Eigen::Matrix3f::Zero();

    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        //        Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        float d_obs_i = correspondences(i,3);
        float d_obs_ii = correspondences(i,7);

        Eigen::Vector3f score = calcScoreTranslation(n_obs_i, d_obs_i, d_obs_ii);

        FIM += score * score.transpose();
    }

    return FIM;
}


//    Eigen::Matrix3f calcFisherInfMat(const int weightedLS)
//    {
//      // Calibration system
//      Eigen::Matrix3f rotationCov = Eigen::Matrix3f::Zero();
//
//      Eigen::Matrix3f FIM_rot = Eigen::Matrix3f::Zero();
//      Eigen::Matrix3f FIM_trans = Eigen::Matrix3f::Zero();
//      Eigen::Vector3f score;
//
//      float accum_error2 = 0;
////      rotationCov += v3normal2 * v3normal1.transpose();
//      for(unsigned i=0; i < correspondences.rows(); i++)
//      {
////          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
//        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
//        Eigen::Vector3f n_i = n_obs_i;
//        Eigen::Vector3f n_ii = Rt_estimated.block(0,0,3,3) * n_obs_ii;
//        Eigen::Vector3f rot_error = (n_i - n_ii);
//        accum_error2 += fabs(rot_error.dot(rot_error));
//
//        if(weightedLS == 1 && correspondences.cols() == 10)
//        {
//          float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
//          rotationCov += weight * n_obs_ii * n_obs_i.transpose();
//        }
//        else
//          rotationCov += n_obs_ii * n_obs_i.transpose();
//
//        float d_obs_i = correspondences(i,3);
//        float d_obs_ii = correspondences(i,7);
//        score = calcScoreRotation(n_obs_i, n_obs_ii);
//        FIM_rot += score * score.transpose();
//        score = calcScoreTranslation(n_obs_i, d_obs_i, d_obs_ii);
//        FIM_trans += score * score.transpose();
////      std::cout << "\nFIM_rot \n" << FIM_rot << "\nrotationCov \n" << rotationCov << "\nFIM_trans \n" << FIM_trans << "\n det " << FIM_rot.determinant() << "\n det2 " << FIM_trans.determinant() << std::endl;
//      }
//      Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotationCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
//      float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//
//      float minFIM_rot = std::min(FIM_rot(0,0), std::min(FIM_rot(1,1), FIM_rot(2,2)));
//      float minFIM_trans = std::min(FIM_trans(0,0), std::min(FIM_trans(1,1), FIM_trans(2,2)));
////      std::cout << "minFIM_rot " << minFIM_rot << " " << minFIM_trans << " conditioning " << conditioning << " numCorresp " << correspondences.rows() << std::endl;
//      std::cout << "\nFIM_rot \n" << FIM_rot << std::endl;
//      std::cout << "\nFIM_trans \n" << FIM_trans << std::endl;
//    }

Eigen::Matrix3f CalibratePairRange::CalibrateRotation(int weightedLS)
{
    // Calibration system
    Eigen::Matrix3f rotationCov = Eigen::Matrix3f::Zero();

    //      Eigen::Matrix3f FIM_rot = Eigen::Matrix3f::Zero();
    //      Eigen::Matrix3f FIM_trans = Eigen::Matrix3f::Zero();
    //      Eigen::Vector3f score;

    float accum_error2 = 0;
    //      rotationCov += v3normal2 * v3normal1.transpose();
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        Eigen::Vector3f n_i = n_obs_i;
        Eigen::Vector3f n_ii = Rt_estimated.block(0,0,3,3) * n_obs_ii;
        Eigen::Vector3f rot_error = (n_i - n_ii);
        accum_error2 += fabs(rot_error.dot(rot_error));

        if(weightedLS == 1 && correspondences.cols() == 10)
        {
            float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
            rotationCov += weight * n_obs_ii * n_obs_i.transpose();
        }
        else
            rotationCov += n_obs_ii * n_obs_i.transpose();

        //        float d_obs_i = correspondences(i,3);
        //        float d_obs_ii = correspondences(i,7);
        //        score = calcScoreRotation(n_obs_i, n_obs_ii);
        //        FIM_rot += score * score.transpose();
        //        score = calcScoreTranslation(n_obs_i, d_obs_i, d_obs_ii);
        //        FIM_trans += score * score.transpose();
        ////      std::cout << "\nFIM_rot \n" << FIM_rot << "\nrotationCov \n" << rotationCov << "\nFIM_trans \n" << FIM_trans << "\n det " << FIM_rot.determinant() << "\n det2 " << FIM_trans.determinant() << std::endl;
    }
    //      float minFIM_rot = std::min(FIM_rot(0,0), std::min(FIM_rot(1,1), FIM_rot(2,2)));
    //      std::cout << "minFIM_rot " << minFIM_rot << std::endl;// << " " << calcCorrespRotError(Rt_estimated) << std::endl;
    //      std::cout << "accum_rot_error2 av_deg " << acos(accum_error2/correspondences.rows()) << std::endl;// << " " << calcCorrespRotError(Rt_estimated) << std::endl;
    //      std::cout << "Rt_estimated\n" << Rt_estimated << std::endl;

    // Calculate calibration Rt
    //      cout << "Solve system\n";
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotationCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    std::cout << "conditioning " << conditioning << std::endl;
    //      if(conditioning > 20000)
    if(conditioning > 100)
    {
        std::cout << "Bad conditioning " << conditioning << std::endl;
        return Eigen::Matrix3f::Identity();
    }

    rotation = svd.matrixV() * svd.matrixU().transpose();
    double det = rotation.determinant();
    if(det != 1)
    {
        Eigen::Matrix3f aux;
        aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
        rotation = svd.matrixV() * aux * svd.matrixU().transpose();
    }
    std::cout << "accum_rot_error2 av_deg" << acos(calcCorrespRotError(rotation)) << std::endl;

    return rotation;
}


Eigen::Matrix3f CalibratePairRange::CalibrateRotationD(int weightedLS)
{
    // Calibration system
    Eigen::Matrix3d rotationCov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rotation_estim = Rt_estimated.block(0,0,3,3).cast<double>();

    double accum_error2 = 0;
    //      rotationCov += v3normal2 * v3normal1.transpose();
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //          double weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Eigen::Vector3d n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Vector3d n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        Eigen::Vector3d n_i = n_obs_i;
        Eigen::Vector3d n_ii = rotation_estim * n_obs_ii;
        Eigen::Vector3d rot_error = (n_i - n_ii);
        accum_error2 += fabs(rot_error.dot(rot_error));

        if(weightedLS == 1 && correspondences.cols() == 10)
        {
            double weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
            rotationCov += weight * n_obs_ii * n_obs_i.transpose();
        }
        else
            rotationCov += n_obs_ii * n_obs_i.transpose();
    }
    std::cout << "accum_rot_error2 av deg" << acos(accum_error2/correspondences.rows()) << std::endl;// << " " << calcCorrespRotError(Rt_estimated) << std::endl;
    std::cout << "Rt_estimated\n" << Rt_estimated << std::endl;

    // Calculate calibration Rt
    cout << "Solve system\n";
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotationCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    std::cout << "conditioning " << conditioning << std::endl;
    //      if(conditioning > 20000)
    if(conditioning > 100)
    {
        std::cout << "Bad conditioning " << conditioning << std::endl;
        return Eigen::Matrix3f::Identity();
    }

    Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
    double det = rotation.determinant();
    if(det != 1)
    {
        Eigen::Matrix3d aux;
        aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
        rotation = svd.matrixV() * aux * svd.matrixU().transpose();
    }
    std::cout << "accum_rot_error2 optim av deg" << acos(calcCorrespRotError(Rt_estimated)) << std::endl;
    cout << "Rotation (double)\n" << rotation << endl;

    return rotation.cast<float>();
    //      return Eigen::Matrix3f::Identity();
}

/*! Get the rotation of each sensor in the multisensor RGBD360 setup */
Eigen::Matrix3f CalibratePairRange::CalibrateRotationManifold(int weightedLS)
{
    cout << "CalibrateRotationManifold...\n";
    Eigen::Matrix<float,3,3> hessian;
    Eigen::Matrix<float,3,1> gradient;
    Eigen::Matrix<float,3,1> update_vector;
    Eigen::Matrix3f jacobian_rot_i, jacobian_rot_ii; // Jacobians of the rotation
    float accum_error2;
    float av_angle_error;
    unsigned numPlaneCorresp;

    // Load the extrinsic calibration from the device' specifications
    //        Rt_estimated[sensor_id] = Rt_specs[sensor_id];

    Eigen::Matrix4f Rt_estimatedTemp;
    Rt_estimatedTemp = Rt_estimated;

    // Parameters of the Least-Squares optimization
    int _max_iterations = 10;
    float _epsilon_transf = 0.00001;
    float _convergence_error = 0.000001;

    float increment = 1000, diff_error = 1000;
    int it = 0;
    while(it < _max_iterations && increment > _epsilon_transf && diff_error > _convergence_error)
    {
        // Calculate the hessian and the gradient
        hessian = Eigen::Matrix<float,3,3>::Zero(); // Hessian of the rotation of the decoupled system
        gradient = Eigen::Matrix<float,3,1>::Zero(); // Gradient of the rotation of the decoupled system
        accum_error2 = 0.0;
        av_angle_error = 0.0;
        numPlaneCorresp = 0;

        //for(int sensor_id = 0; sensor_id < NUM_ASUS_SENSORS-1; sensor_id++)
        {
            for(unsigned i=0; i < correspondences.rows(); i++)
            {
                //          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
                Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
                Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
                //            Eigen::Vector3f n_i = n_obs_i;
                Eigen::Vector3f n_ii = Rt_estimated.block(0,0,3,3) * n_obs_ii;
                //            jacobian_rot_i = rv::math::skew(-n_i);
                jacobian_rot_ii = skew(n_ii);
                Eigen::Vector3f rot_error = (n_obs_i - n_ii);
                accum_error2 += fabs(rot_error.dot(rot_error));
                av_angle_error += acos(n_obs_i.dot(n_ii));
                numPlaneCorresp++;
                //          cout << "rotation error_i " << rot_error.transpose() << endl;
                if(weightedLS == 1 && correspondences.cols() == 10)
                {
                    // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
                    //              float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
                    //              hessian += weight * (jacobian_rot_ii.transpose() * jacobian_rot_ii);
                    //              gradient += weight * (jacobian_rot_ii.transpose() * rot_error);
                    Eigen::Matrix3f information;
                    information << correspondences(i,8), correspondences(i,9), correspondences(i,10), correspondences(i,11),
                            correspondences(i,9), correspondences(i,12), correspondences(i,13), correspondences(i,14),
                            correspondences(i,10), correspondences(i,13), correspondences(i,15), correspondences(i,16),
                            correspondences(i,11), correspondences(i,14), correspondences(i,16), correspondences(i,17);
                    hessian += jacobian_rot_ii.transpose() * information.block(0,0,3,3) * jacobian_rot_ii;
                    gradient += jacobian_rot_ii.transpose() * information.block(0,0,3,3) * rot_error;
                }
                else
                {
                    hessian += jacobian_rot_ii.transpose() * jacobian_rot_ii;
                    gradient += jacobian_rot_ii.transpose() * rot_error;
                }

                //            Eigen::JacobiSVD<Eigen::Matrix3f> svd(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
                //            Eigen::Matrix3f cov = hessian.inverse();
                //            Eigen::JacobiSVD<Eigen::Matrix3f> svd2(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

                //            float minFIM_rot = std::min(hessian(0,0), std::min(hessian(1,1), hessian(2,2)));
                //            float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
                //            std::cout << " det " << hessian.determinant() << " minFIM_rot " << minFIM_rot << " conditioningX " << conditioning << std::endl;
                ////            std::cout << hessian(0,0) << " " << hessian(1,1) << " " << hessian(2,2) << endl;
                ////            std::cout << "COV " << svd2.singularValues().transpose() << endl;
                //            std::cout << "FIM rotation " << svd.singularValues().transpose() << endl;
            }
            accum_error2 /= numPlaneCorresp;
            av_angle_error /= numPlaneCorresp;

        }

        //        Eigen::JacobiSVD<Eigen::Matrix3f> svd(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        //        float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
        //        Eigen::Matrix3f cov;
        //        svd.pinv(cov);
        //        std::cout << "hessian \n" << hessian << "inv\n" << hessian.inverse() << "\ncov \n" << cov << std::endl;

        //        std::cout << "conditioning " << conditioning << std::endl;
        //        if(conditioning > 100)
        //          return Eigen::Matrix3f::Identity();

        // Solve the rotation
        update_vector = -hessian.inverse() * gradient;
        //      cout << "update_vector " << update_vector.transpose() << endl;

        // Update rotation of the poses
        //        for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
        {
            mrpt::poses::CPose3D pose;
            mrpt::math::CArrayNumeric< double, 3 > rot_manifold;
            rot_manifold[0] = update_vector(0,0);
            rot_manifold[1] = update_vector(1,0);
            rot_manifold[2] = update_vector(2,0);
            //          rot_manifold[2] = update_vector(3*sensor_id-3,0) / 4; // Limit the turn around the Z (depth) axis
            //          rot_manifold[2] = 0; // Limit the turn around the Z (depth) axis
            mrpt::math::CMatrixDouble33 update_rot = pose.exp_rotation(rot_manifold);
            //      cout << "update_rot\n" << update_rot << endl;
            Eigen::Matrix3f update_rot_eig;
            update_rot_eig << update_rot(0,0), update_rot(0,1), update_rot(0,2),
                    update_rot(1,0), update_rot(1,1), update_rot(1,2),
                    update_rot(2,0), update_rot(2,1), update_rot(2,2);
            Rt_estimatedTemp = Rt_estimated;
            Rt_estimatedTemp.block(0,0,3,3) = update_rot_eig * Rt_estimated.block(0,0,3,3);
            //      cout << "old rotation" << sensor_id << "\n" << Rt_estimated.block(0,0,3,3) << endl;
            //      cout << "new rotation\n" << Rt_estimatedTemp.block(0,0,3,3) << endl;
        }

        accum_error2 = calcCorrespRotError(Rt_estimated);
        //        float new_accum_error2 = calcCorrespRotErrorWeight(Rt_estimatedTemp);
        float new_accum_error2 = calcCorrespRotError(Rt_estimatedTemp);

        //        cout << "New rotation error " << new_accum_error2 << " previous " << accum_error2 << endl;
        //    cout << "Closing loop? \n" << Rt_estimated[0].inverse() * Rt_estimated[7] * Rt_78;

        // Assign new rotations
        if(new_accum_error2 < accum_error2)
            //          for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
            Rt_estimated = Rt_estimatedTemp;
        //            Rt_estimated.block(0,0,3,3) = Rt_estimatedTemp.block(0,0,3,3);

        increment = update_vector .dot (update_vector);
        diff_error = accum_error2 - new_accum_error2;
        ++it;
        //      cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
    }

    std::cout << "ErrorCalibRotation " << accum_error2 << " " << av_angle_error << std::endl;
    std::cout << "Rotation \n"<< Rt_estimated.block(0,0,3,3) << std::endl;

    rotation = Rt_estimated.block(0,0,3,3);
    return rotation;
}

Eigen::Vector3f CalibratePairRange::CalibrateTranslation(int weightedLS)
{
    // Calibration system
    Eigen::Matrix3f translationHessian = Eigen::Matrix3f::Zero();
    Eigen::Vector3f translationGradient = Eigen::Vector3f::Zero();

    //      Eigen::Vector3f translation2 = Eigen::Vector3f::Zero();
    //      Eigen::Vector3f sumNormals = Eigen::Vector3f::Zero();

    //              translationHessian += v3normal1 * v3normal1.transpose();
    //  //            double error = d2 - d1;
    //              translationGradient += v3normal1 * (d2 - d1);
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        //        Eigen::Vector3f n_i = Rt_estimated[sensor_id].block(0,0,3,3) * n_obs_i;
        //        Eigen::Vector3f n_ii = Rt_estimated.block(0,0,3,3) * n_obs_ii;
        float trans_error = (correspondences(i,7) - correspondences(i,3));
        //        accum_error2 += trans_error * trans_error;

        //        translation2[0] += n_obs_i[0] * trans_error;
        //        translation2[1] += n_obs_i[1] * trans_error;
        //        translation2[2] += n_obs_i[2] * trans_error;
        //        sumNormals += n_obs_i;

        if(weightedLS == 1 && correspondences.cols() == 18)
        {
            // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
            //          float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
            float weight = correspondences(i,17);
            translationHessian += weight * (n_obs_i * n_obs_i.transpose() );
            translationGradient += weight * (n_obs_i * trans_error);
        }
        else
        {
            translationHessian += (n_obs_i * n_obs_i.transpose() );
            translationGradient += (n_obs_i * trans_error);
        }
    }

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(translationHessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "FIM translation " << svd.singularValues().transpose() << endl;

    //      cout << "translationHessian \n" << translationHessian << "\n HessianInv \n" << translationHessian.inverse() << endl;
    //      calcFisherInfMat();

    translation = translationHessian.inverse() * translationGradient;

    //      translation2[0] /= sumNormals[0];
    //      translation2[1] /= sumNormals[1];
    //      translation2[2] /= sumNormals[2];
    //      std::cout << "translation " << translation.transpose() << " translation2 " << translation2.transpose() << std::endl;

    return translation;
}

void CalibratePairRange::CalibratePair()
{
    //      calibrated_Rt = Eigen::Matrix4f::Identity();
    Rt_estimated.block(0,0,3,3) = CalibrateRotation();
    Rt_estimated.block(0,3,3,1) = CalibrateTranslation();
    //      std::cout << "Rt_estimated\n" << Rt_estimated << std::endl;

    // Calculate average error
    float av_rot_error = 0;
    float av_trans_error = 0;
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Vector3f n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        av_rot_error += fabs(acos(n_obs_i .dot( rotation * n_obs_ii ) ));
        av_trans_error += fabs(correspondences(i,3) - correspondences(i,7) - n_obs_i .dot(translation));
        //        params_error += plane_correspondences1[i] .dot( plane_correspondences2[i] );
    }
    av_rot_error /= correspondences.rows();
    av_trans_error /= correspondences.rows();
    std::cout << "Errors n.n " << calcCorrespRotError(Rt_estimated) << " av deg " << av_rot_error*180/3.14159f << " av trans " << av_trans_error << std::endl;
}
