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

#include <rv/math/poses.h>
#include <rv/calib/extrinsic_calib_rgbd360.h>
#include <pcl/console/parse.h>

#define VISUALIZE_SENSOR_DATA 0
#define SHOW_IMAGES 0

using namespace std;
using namespace rv::math;
using namespace rv::calib;


/*! Load the extrinsic parameters given by the construction specifications of the omnidirectional sensor */
void CalibrateRgbd360::loadConstructionSpecs()
{
    Rt_specs_[0] = Eigen::Matrix4f::Identity();
    Rt_specs_[0](2,3) = 0.055; // This is the theoretical distance from the first Asus sensor to the center

    // The pose of each sensor is given by a turn of 45deg around the vertical axis which passes through the device' center
    Eigen::Matrix4f turn45deg = Eigen::Matrix4f::Identity();
    turn45deg(1,1) = turn45deg(2,2) = cos(45*PI/180);
    turn45deg(1,2) = -sin(45*PI/180);
    turn45deg(2,1) = -turn45deg(1,2);
    for(unsigned sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
        Rt_specs_[sensor_id] = turn45deg * Rt_specs_[sensor_id-1];
}

/*! Get the sum of squared rotational errors for the input extrinsic matrices. TODO: the input argument of this function is unsafe -> fix it */
float CalibrateRgbd360::calcCorrespRotError(Eigen::Matrix4f *Rt_)
{
    float accum_error2 = 0; // Accumulated square errors
    for(unsigned sensor_id = 0; sensor_id < NUM_ASUS_SENSORS-1; sensor_id++)
    {
        for(std::map<unsigned, mrpt::math::CMatrixDouble>::iterator it_pair=matchedPlanes_.mm_corresp_[sensor_id].begin();
            it_pair != matchedPlanes_.mm_corresp_[sensor_id].end(); it_pair++)
        {
            for(unsigned i=0; i < it_pair->second.rows(); i++)
            {
                //          float weight = (inliers / it_pair->second(i,3)) / it_pair->second.rows()
                Eigen::Vector3f n_obs_i; n_obs_i << it_pair->second(i,0), it_pair->second(i,1), it_pair->second(i,2);
                Eigen::Vector3f n_obs_ii; n_obs_ii << it_pair->second(i,4), it_pair->second(i,5), it_pair->second(i,6);
                Eigen::Vector3f n_i = Rt_[sensor_id].block(0,0,3,3) * n_obs_i;
                Eigen::Vector3f n_ii = Rt_[it_pair->first].block(0,0,3,3) * n_obs_ii;
                Eigen::Vector3f rot_error = (n_i - n_ii);
                accum_error2 += rot_error.dot(rot_error);
            }
        }
    }

    return accum_error2;
}

/*! Get the sum of weighted squared rotational errors for the input extrinsic matrices. TODO: the input argument of this function is unsafe -> fix it */
float CalibrateRgbd360::calcCorrespRotErrorWeight(Eigen::Matrix4f *Rt_)
{
    float accum_error2 = 0; // Accumulated square errors
    for(unsigned sensor_id = 0; sensor_id < NUM_ASUS_SENSORS-1; sensor_id++)
    {
        for(std::map<unsigned, mrpt::math::CMatrixDouble>::iterator it_pair=matchedPlanes_.mm_corresp_[sensor_id].begin();
            it_pair != matchedPlanes_.mm_corresp_[sensor_id].end(); it_pair++)
        {
            for(unsigned i=0; i < it_pair->second.rows(); i++)
            {
                float weight = (it_pair->second(i,8) / (it_pair->second(i,3) * it_pair->second(i,9)));
                Eigen::Vector3f n_obs_i; n_obs_i << it_pair->second(i,0), it_pair->second(i,1), it_pair->second(i,2);
                Eigen::Vector3f n_obs_ii; n_obs_ii << it_pair->second(i,4), it_pair->second(i,5), it_pair->second(i,6);
                Eigen::Vector3f n_i = Rt_[sensor_id].block(0,0,3,3) * n_obs_i;
                Eigen::Vector3f n_ii = Rt_[it_pair->first].block(0,0,3,3) * n_obs_ii;
                Eigen::Vector3f rot_error = (n_i - n_ii);
                accum_error2 += weight * rot_error.dot(rot_error);
            }
        }
    }

    return accum_error2;
}

/*! Get the sum of squared translational errors for the input extrinsic matrices. TODO: the input argument of this function is unsafe -> fix it */
float CalibrateRgbd360::calcCorrespTransError(Eigen::Matrix4f *Rt_)
{
    float accum_error2 = 0; // Accumulated square errors
    //      for(unsigned sensor_id = 0; sensor_id < NUM_ASUS_SENSORS-1; sensor_id++)
    //      {
    //        for(std::map<unsigned, mrpt::math::CMatrixDouble>::iterator it_pair=matchedPlanes_.mm_corresp_[sensor_id].begin();
    //            it_pair != matchedPlanes_.mm_corresp_[sensor_id].end(); it_pair++)
    //        {
    //          for(unsigned i=0; i < it_pair->second.rows(); i++)
    //          {
    ////          float weight = (inliers / it_pair->second(i,3)) / it_pair->second.rows()
    //            Eigen::Vector3f n_obs_i; n_obs_i << it_pair->second(i,0), it_pair->second(i,1), it_pair->second(i,2);
    //            Eigen::Vector3f n_obs_ii; n_obs_ii << it_pair->second(i,4), it_pair->second(i,5), it_pair->second(i,6);
    //            Eigen::Vector3f n_i = Rt_[sensor_id].block(0,0,3,3) * n_obs_i;
    //            Eigen::Vector3f n_ii = Rt_[it_pair->first].block(0,0,3,3) * n_obs_ii;
    //            rot_error = (n_i - n_ii);
    //            accum_error2 += rot_error.dot(rot_error);
    //          }
    //        }
    //      }

    //      // Calculate new error
    //      double new_accum_error2 = 0;
    //      for(sensor_id = 0; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
    //      {
    //        for(unsigned i=0; i < it_pair->second.rows(); i++)
    //        {
    //          Eigen::Vector3f n_obs_i; n_obs_i << it_pair->second(i,0), it_pair->second(i,1), it_pair->second(i,2);
    //          Eigen::Vector3f n_obs_ii; n_obs_ii << it_pair->second(i,4), it_pair->second(i,5), it_pair->second(i,6);
    //          Eigen::Vector3f n_i = Rt_estimatedTemp[sensor_id].block(0,0,3,3) * n_obs_i;
    //          Eigen::Vector3f n_ii = Rt_estimatedTemp[(sensor_id+1)%8].block(0,0,3,3) * n_obs_ii;
    ////            trans_error = it_pair->second(i,3) + n_i.transpose()*Rt_estimatedTemp[sensor_id].block(0,3,3,1) - (it_pair->second(i,7) + n_ii.transpose()*Rt_estimatedTemp[sensor_id+1].block(0,3,3,1));
    //          trans_error = it_pair->second(i,3) - (n_i(0)*Rt_estimatedTemp[sensor_id](0,3) + n_i(1)*Rt_estimatedTemp[sensor_id](1,3) + n_i(2)*Rt_estimatedTemp[sensor_id](2,3))
    //                      - (it_pair->second(i,7) - (n_ii(0)*Rt_estimatedTemp[(sensor_id+1)%8](0,3) + n_ii(1)*Rt_estimatedTemp[(sensor_id+1)%8](1,3) + n_ii(2)*Rt_estimatedTemp[(sensor_id+1)%8](2,3)));
    ////      if(sensor_id==7)
    ////        cout << "translation error_i LC " << trans_error << endl;
    //          new_accum_error2 += trans_error * trans_error;
    //        }
    //      }
    return accum_error2;
}

/*! Get the rotation of each sensor in the multisensor RGBD360 setup */
void CalibrateRgbd360::CalibrateRotation(int weightedLS)
{
    cout << "CalibrateRgbd360::CalibrateRotation...\n";
    const int n_DoF = 3 * (NUM_ASUS_SENSORS - 1);
    Eigen::VectorXf update_vector(n_DoF);
    Eigen::Matrix3f jacobian_rot_i, jacobian_rot_ii; // Jacobians of the rotation
    float accum_error2;
    float av_angle_error;
    unsigned numPlaneCorresp;

    // Load the extrinsic calibration from the device' specifications
    for(unsigned sensor_id = 0; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
        Rt_estimated_[sensor_id] = Rt_specs_[sensor_id];

    Eigen::Matrix4f Rt_estimatedTemp[NUM_ASUS_SENSORS];
    Rt_estimatedTemp[0] = Rt_estimated_[0];

    // Parameters of the Least-Squares optimization
    int _max_iterations = 10;
    float _epsilon_transf = 0.00001;
    float _convergence_error = 0.000001;

    float increment = 1000, diff_error = 1000;
    int it = 0;
    while(it < _max_iterations && increment > _epsilon_transf && diff_error > _convergence_error)
    {
        // Calculate the hessian and the gradient
        hessian_ = Eigen::Matrix<float,n_DoF,n_DoF>::Zero(); // Hessian of the rotation of the decoupled system
        gradient_ = Eigen::Matrix<float,n_DoF,1>::Zero(); // Gradient of the rotation of the decoupled system
        accum_error2 = 0.0;
        av_angle_error = 0.0;
        numPlaneCorresp = 0;

        for(size_t sensor_id = 0; sensor_id < matchedPlanes_.mm_corresp_.size(); sensor_id++)
        {
            if( matchedPlanes_.mm_corresp_[sensor_id].count(sensor_id+1) == 0 || matchedPlanes_.mm_corresp_[sensor_id][sensor_id+1].rows() < 3 )
                throw std::runtime_error("\nERROR... CalibrateRgbd360::CalibrateRotation data is NOT aligned \n\n");

            //        cout << "sensor_id " << sensor_id << endl;
            //        cout << "Rt+1 " << sensor_id << endl;
            //        cout << it_pair->second << endl;
            for(std::map<unsigned, mrpt::math::CMatrixDouble>::iterator it_pair=matchedPlanes_.mm_corresp_[sensor_id].begin();
                it_pair != matchedPlanes_.mm_corresp_[sensor_id].end(); it_pair++)
                //            if( it_pair->first - sensor_id == 1 || it_pair->first - sensor_id == 7)
            {
                std::cout << "Add pair " << sensor_id << " " << it_pair->first << std::endl;
                int id_corresp1 = 3*(sensor_id-1);
                int id_corresp2 = 3*(it_pair->first - 1);

                for(unsigned i=0; i < it_pair->second.rows(); i++)
                {
                    //          float weight = (inliers / it_pair->second(i,3)) / it_pair->second.rows()
                    Eigen::Vector3f n_obs_i; n_obs_i << it_pair->second(i,0), it_pair->second(i,1), it_pair->second(i,2);
                    Eigen::Vector3f n_obs_ii; n_obs_ii << it_pair->second(i,4), it_pair->second(i,5), it_pair->second(i,6);
                    Eigen::Vector3f n_i = Rt_estimated_[sensor_id].block(0,0,3,3) * n_obs_i;
                    Eigen::Vector3f n_ii = Rt_estimated_[it_pair->first].block(0,0,3,3) * n_obs_ii;
                    jacobian_rot_i = -skew(n_i);
                    jacobian_rot_ii = skew(n_ii);
                    Eigen::Vector3f rot_error = (n_i - n_ii);
                    accum_error2 += rot_error.dot(rot_error);
                    av_angle_error += acos(n_i.dot(n_ii));
                    numPlaneCorresp++;
                    //cout << i << " rotation error_i " << rot_error.transpose() << endl;
                    if(weightedLS == 1 && it_pair->second.cols() == 18)
                    {
                        // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
                        float weight = (it_pair->second(i,8) / (it_pair->second(i,3) * it_pair->second(i,9)));// / it_pair->second.rows();
                        if(sensor_id != 0) // The pose of the first camera is fixed
                        {
                            hessian_.block(id_corresp1, id_corresp1, 3, 3) += weight * (jacobian_rot_i.transpose() * jacobian_rot_i);
                            gradient_.block(id_corresp1,0,3,1) += weight * (jacobian_rot_i.transpose() * rot_error);

                            // Cross term
                            hessian_.block(id_corresp1, id_corresp2, 3, 3) += weight * (jacobian_rot_i.transpose() * jacobian_rot_ii);
                        }

                        hessian_.block(id_corresp2, id_corresp2, 3, 3) += weight * (jacobian_rot_ii.transpose() * jacobian_rot_ii);
                        gradient_.block(id_corresp2,0,3,1) += weight * (jacobian_rot_ii.transpose() * rot_error);
                    }
                    else
                    {
                        if(sensor_id != 0) // The pose of the first camera is fixed
                        {
                            hessian_.block(id_corresp1, id_corresp1, 3, 3) += jacobian_rot_i.transpose() * jacobian_rot_i;
                            gradient_.block(id_corresp1,0,3,1) += jacobian_rot_i.transpose() * rot_error;

                            // Cross term
                            hessian_.block(id_corresp1, id_corresp2, 3, 3) += jacobian_rot_i.transpose() * jacobian_rot_ii;
                        }
                        hessian_.block(id_corresp2, id_corresp2, 3, 3) += jacobian_rot_ii.transpose() * jacobian_rot_ii;
                        gradient_.block(id_corresp2,0,3,1) += jacobian_rot_ii.transpose() * rot_error;
                    }
                }

                if(sensor_id != 0) // Fill the lower left triangle with the corresponding cross terms
                    hessian_.block(id_corresp2, id_corresp1, 3, 3) = hessian_.block(id_corresp1, id_corresp2, 3, 3).transpose();
            }
        }

        //cout << "hessian_ \n" << hessian_ << endl;
        //cout << "gradient_ \n" << gradient_.transpose() << endl;
        cout << "Error accumulated " << accum_error2 << endl;

        if( calcConditioning() > threshold_conditioning_ )
        {
            cout << "\tRotation system is bad conditioned " << conditioning_ << " threshold " << threshold_conditioning_ << "\n";
            break;
        }

        // Solve the rotation
        update_vector = -hessian_.inverse() * gradient_;
        cout << "update_vector " << update_vector.transpose() << endl;

        // Update rotation of the poses
        for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
        {
            mrpt::poses::CPose3D pose;
            mrpt::math::CArrayNumeric< double, 3 > rot_manifold;
            rot_manifold[0] = update_vector(3*sensor_id-3,0);
            rot_manifold[1] = update_vector(3*sensor_id-2,0);
            rot_manifold[2] = update_vector(3*sensor_id-1,0);
            //          rot_manifold[2] = update_vector(3*sensor_id-3,0) / 4; // Limit the turn around the Z (depth) axis
            //          rot_manifold[2] = 0; // Limit the turn around the Z (depth) axis
            mrpt::math::CMatrixDouble33 update_rot = pose.exp_rotation(rot_manifold);
            //      cout << "update_rot\n" << update_rot << endl;
            Eigen::Matrix3f update_rot_eig;
            update_rot_eig << update_rot(0,0), update_rot(0,1), update_rot(0,2),
                    update_rot(1,0), update_rot(1,1), update_rot(1,2),
                    update_rot(2,0), update_rot(2,1), update_rot(2,2);
            Rt_estimatedTemp[sensor_id] = Rt_estimated_[sensor_id];
            Rt_estimatedTemp[sensor_id].block(0,0,3,3) = update_rot_eig * Rt_estimated_[sensor_id].block(0,0,3,3);
            //      cout << "old rotation" << sensor_id << "\n" << Rt_estimated_[sensor_id].block(0,0,3,3) << endl;
            //      cout << "new rotation\n" << Rt_estimatedTemp[sensor_id].block(0,0,3,3) << endl;
        }

        accum_error2 = calcCorrespRotErrorWeight(Rt_estimated_);
        float new_accum_error2 = calcCorrespRotErrorWeight(Rt_estimatedTemp);
        //        float new_accum_error2 = calcCorrespRotError(Rt_estimatedTemp);

        //        cout << "New rotation error " << new_accum_error2 << endl;
        //    cout << "Closing loop? \n" << Rt_estimated_[0].inverse() * Rt_estimated_[7] * Rt_78;

        // Assign new rotations
        if(new_accum_error2 < accum_error2)
            for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
                Rt_estimated_[sensor_id] = Rt_estimatedTemp[sensor_id];
        //            Rt_estimated_[sensor_id].block(0,0,3,3) = Rt_estimatedTemp[sensor_id].block(0,0,3,3);

        increment = update_vector .dot (update_vector);
        diff_error = accum_error2 - new_accum_error2;
        ++it;
        cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
    }

    // Make the X axis of the device (the average X axis of all Asus' rotation matrices) coincide with the vertical axis
    Eigen::Matrix3f hessian_rot = Eigen::Matrix3f::Zero();
    Eigen::Vector3f gradient_rot = Eigen::Vector3f::Zero();
    Eigen::Vector3f X_axis; X_axis << 1, 0, 0;
    float rotation_error2 = 0;
    for(unsigned i=0; i < NUM_ASUS_SENSORS; i++)
    {
        Eigen::Vector3f X_pose = Rt_estimated_[i].block(0,0,3,1);
        Eigen::Vector3f error = X_axis .cross(X_pose);
        Eigen::Matrix3f jacobian = -skew(X_axis) * skew(X_pose);
        hessian_rot += jacobian.transpose() * jacobian;
        gradient_rot += jacobian.transpose() * error;
        rotation_error2 += error .dot(error);
    }
    Eigen::Vector3f rotation_manifold = - hessian_rot.inverse() * gradient_rot;
    mrpt::poses::CPose3D pose;
    mrpt::math::CArrayNumeric< double, 3 > rot_manifold;
    rot_manifold[0] = 0;
    //      rot_manifold[0] = rotation_manifold(0);
    rot_manifold[1] = rotation_manifold(1);
    rot_manifold[2] = rotation_manifold(2);
    mrpt::math::CMatrixDouble33 update_rot = pose.exp_rotation(rot_manifold);
    cout << "manifold update " << rot_manifold.transpose() << "\nupdate_rot\n" << update_rot << endl;
    Eigen::Matrix3f rotation;
    rotation << update_rot(0,0), update_rot(0,1), update_rot(0,2),
            update_rot(1,0), update_rot(1,1), update_rot(1,2),
            update_rot(2,0), update_rot(2,1), update_rot(2,2);

    //      float new_rotation_error2 = 0;
    //      for(unsigned i=0; i<8; i++)
    //      {
    //        Eigen::Vector3f X_pose = rotation * Rt_estimated_[i].block(0,0,3,1);
    //        Eigen::Vector3f error = X_axis .cross(X_pose);
    //        new_rotation_error2 += error .dot(error);
    //      }
    //    cout << "Previous error " << rotation_error2 << " new error " << new_rotation_error2 << endl;

    // Rotate the camera system to make the vertical direction correspond to the X axis
    for(unsigned sensor_id=0; sensor_id<NUM_ASUS_SENSORS; sensor_id++)
        Rt_estimated_[sensor_id].block(0,0,3,3) = rotation * Rt_estimated_[sensor_id].block(0,0,3,3);

    std::cout << "ErrorCalibRotation " << accum_error2/numPlaneCorresp << " " << av_angle_error/numPlaneCorresp << std::endl;
}


/*! Get the translation of each sensor in the multisensor RGBD360 setup. Warning: this method has being implemented to be applied always after rotation calibration */
void CalibrateRgbd360::CalibrateTranslation(int weightedLS)
{
    cout << "CalibrateRgbd360::CalibrateTranslation... \n";
    const int n_DoF = 3 * (NUM_ASUS_SENSORS - 1);
    hessian_ = Eigen::Matrix<float,n_DoF,n_DoF>::Zero(); // Hessian of the translation of the decoupled system
    gradient_ = Eigen::Matrix<float,n_DoF,1>::Zero(); // Gradient of the translation of the decoupled system
    Eigen::Matrix<float,n_DoF,1> update_vector;
    Eigen::Matrix<float,n_DoF,1> update_translation;
    Eigen::Matrix<float,1,3> jacobian_trans_i, jacobian_trans_ii; // Jacobians of the translation
    float trans_error;
    float accum_error2 = 0.0;
    unsigned numPlaneCorresp = 0;

    Eigen::Matrix4f Rt_estimatedTemp[NUM_ASUS_SENSORS];
    Rt_estimatedTemp[0] = Rt_estimated_[0];

    for(int sensor_id = 0; sensor_id < NUM_ASUS_SENSORS-1; sensor_id++)
    {
        if( matchedPlanes_.mm_corresp_[sensor_id].count(sensor_id+1) == 0 || matchedPlanes_.mm_corresp_[sensor_id][sensor_id+1].rows() < 3 )
            throw std::runtime_error("\nERROR... CalibrateRgbd360::CalibrateTranslation data is NOT aligned \n\n");

        //        cout << "sensor_id " << sensor_id << endl;
        //        cout << "Rt_estimated_ \n" << Rt_estimated_[sensor_id] << endl;
        //        cout << "Rt+1 " << sensor_id << endl;
        //        cout << it_pair->second << endl;
        for(std::map<unsigned, mrpt::math::CMatrixDouble>::iterator it_pair=matchedPlanes_.mm_corresp_[sensor_id].begin();
            it_pair != matchedPlanes_.mm_corresp_[sensor_id].end(); it_pair++)
            //          if( it_pair->first - sensor_id == 1 || it_pair->first - sensor_id == 7)
        {
            int id_corresp1 = 3*(sensor_id-1);
            int id_corresp2 = 3*(it_pair->first - 1);
            //        cout << "id_corresp1 " << id_corresp1 << "id_corresp2 " << id_corresp2 << endl;

            for(unsigned i=0; i < it_pair->second.rows(); i++)
            {
                //          float weight = (inliers / it_pair->second(i,3)) / it_pair->second.rows()
                Eigen::Vector3f n_obs_i; n_obs_i << it_pair->second(i,0), it_pair->second(i,1), it_pair->second(i,2);
                Eigen::Vector3f n_obs_ii; n_obs_ii << it_pair->second(i,4), it_pair->second(i,5), it_pair->second(i,6);
                Eigen::Vector3f n_i = Rt_estimated_[sensor_id].block(0,0,3,3) * n_obs_i;
                Eigen::Vector3f n_ii = Rt_estimated_[it_pair->first].block(0,0,3,3) * n_obs_ii;
                trans_error = (it_pair->second(i,3) - it_pair->second(i,7));
                accum_error2 += trans_error * trans_error;
                numPlaneCorresp++;
                //          cout << "Rt_estimated_ \n" << Rt_estimated_[sensor_id] << " n_i " << n_i.transpose() << " n_ii " << n_ii.transpose() << endl;
                if(weightedLS == 1 && it_pair->second.cols() == 18)
                {
                    // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
                    float weight = (it_pair->second(i,8) / (it_pair->second(i,3) * it_pair->second(i,9)));// / it_pair->second.rows();

                    if(sensor_id != 0) // The pose of the first camera is fixed
                    {
                        hessian_.block(id_corresp1, id_corresp1, 3, 3) += weight * (n_i * n_i.transpose() );
                        gradient_.block(id_corresp1,0,3,1) += weight * (-n_i * trans_error);

                        // Cross term
                        hessian_.block(id_corresp1, id_corresp2, 3, 3) += weight * (-n_i * n_ii.transpose() );
                    }

                    hessian_.block(id_corresp2, id_corresp2, 3, 3) += weight * (n_ii * n_ii.transpose() );
                    gradient_.block(id_corresp2,0,3,1) += weight * (n_ii * trans_error);
                }
                else
                {
                    if(sensor_id != 0) // The pose of the first camera is fixed
                    {
                        hessian_.block(id_corresp1, id_corresp1, 3, 3) += n_i * n_i.transpose();
                        gradient_.block(id_corresp1,0,3,1) += -n_i * trans_error;

                        // Cross term
                        hessian_.block(id_corresp1, id_corresp2, 3, 3) += -n_i * n_ii.transpose();
                    }

                    hessian_.block(id_corresp2, id_corresp2, 3, 3) += n_ii * n_ii.transpose();
                    gradient_.block(id_corresp2,0,3,1) += n_ii * trans_error;
                }
            }

            // Fill the lower left triangle with the corresponding cross terms
            if(sensor_id != 0)
                hessian_.block(id_corresp2, id_corresp1, 3, 3) = hessian_.block(id_corresp1, id_corresp2, 3, 3).transpose();
        }
    }

    //      cout << "hessian\n" << hessian_ << endl;
    //      cout << "calcConditioning " << calcConditioning() << endl;

    if( calcConditioning() < threshold_conditioning_ )
    {
        // Solve translation
        update_vector = -hessian_.inverse() * gradient_;
        cout << "update_vector translations " << update_vector.transpose() << endl;

        // Get center of the multicamera system
        Eigen::Vector3f centerDevice = Eigen::Vector3f::Zero();
        for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
            centerDevice += update_vector.block(3*sensor_id-3,0,3,1);
        centerDevice = centerDevice / NUM_ASUS_SENSORS;

        // Update translation of the poses
        Rt_estimatedTemp[0].block(0,3,3,1) = -centerDevice;
        for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
            Rt_estimatedTemp[sensor_id].block(0,3,3,1) = update_vector.block(3*sensor_id-3,0,3,1) - centerDevice;

        //      // Assign new translations
        //      float new_accum_error2 = calcCorrespTransError(Rt_estimatedTemp);
        //      if(new_accum_error2 < accum_error2)
        for(int sensor_id = 0; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
            Rt_estimated_[sensor_id].block(0,3,3,1) = Rt_estimatedTemp[sensor_id].block(0,3,3,1);
        //          Rt_estimated_[sensor_id].block(0,3,3,1) = Eigen::Matrix<float,3,1>::Zero();

        std::cout << "ErrorCalibTranslation " << accum_error2/numPlaneCorresp << std::endl;
        //      cout << "\tCalibration finished\n";
    }
    else
        cout << "\tTranslation system is bad conditioned " << conditioning_ << " threshold " << threshold_conditioning_ << "\n";
}

void CalibrateRgbd360::saveCalibration_askUser(const string & path, const string & filename)
{
    string input;
    cout << "Do you want to save the calibrated extrinsic matrices? (y/n)" << endl;
    getline(cin, input);
    if(input == "y" || input == "Y")
    {
        ofstream calib_file;
        for(unsigned sensor_id=0; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
        {
            string calibFileName = mrpt::format("%s/%s_0%u.txt", path.c_str(), filename.c_str(), sensor_id+1);
            calib_file.open(calibFileName);
            if( calib_file.is_open() )
            {
                calib_file << Rt_estimated_[sensor_id];
                calib_file.close();
            }
            else
                cout << "Unable to open file " << calibFileName << endl;
        }
    }
}
