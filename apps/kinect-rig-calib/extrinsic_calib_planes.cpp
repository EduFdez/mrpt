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

#include "extrinsic_calib_planes.h"
#include "kinect-rig-calib_misc.h"
#include <iostream>
#include <mrpt/poses/CPose3D.h>
//#include <mrpt/math/ransac_applications.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/thread.hpp>

#define VISUALIZE_SENSOR_DATA 0
#define SHOW_IMAGES 0

using namespace std;
using namespace Eigen;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::pbmap;

// Obtain the rigid transformation from 3 matched planes
CMatrixDouble registerMatchedPlanes( const CMatrixDouble &matched_planes )
{
    ASSERT_(size(matched_planes,1) == 8 && size(matched_planes,2) == 3);

    //Calculate rotation
    Matrix3f normalCovariances = Matrix3f::Zero();
    normalCovariances(0,0) = 1;
    for(unsigned i=0; i<3; i++)
    {
        Vector3f n_i = Vector3f(matched_planes(0,i), matched_planes(1,i), matched_planes(2,i));
        Vector3f n_ii = Vector3f(matched_planes(4,i), matched_planes(5,i), matched_planes(6,i));
        normalCovariances += n_i * n_ii.transpose();
        //    normalCovariances += matched_planes.block(i,0,1,3) * matched_planes.block(i,4,1,3).transpose();
    }
    Matrix3f Rotation;
    float cond = ExtrinsicCalib::rotationFromNormals(normalCovariances, Rotation);
//    if(cond < 0.01f){ cout << "ExtrinsicCalibLines::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
//        Rotation = Matrix3f::Identity();
//    }

    // Calculate translation
    Vector3f translation;
    Matrix3f hessian = Matrix3f::Zero();
    Vector3f gradient = Vector3f::Zero();
    hessian(0,0) = 1;
    for(unsigned i=0; i<3; i++)
    {
        float trans_error = (matched_planes(3,i) - matched_planes(7,i)); //+n*t
        //    hessian += matched_planes.block(i,0,1,3) * matched_planes.block(i,0,1,3).transpose();
        //    gradient += matched_planes.block(i,0,1,3) * trans_error;
        Vector3f n_i = Vector3f(matched_planes(0,i), matched_planes(1,i), matched_planes(2,i));
        hessian += n_i * n_i.transpose();
        gradient += n_i * trans_error;
    }
    translation = -hessian.inverse() * gradient;
    //cout << "Previous average translation error " << sumError / matched_planes.size() << endl;

    //  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
    //  Eigen::Matrix4f rigidTransf;
    CMatrixDouble44 rigidTransf;
    rigidTransf.block(0,0,3,3) = Rotation;
    rigidTransf.block(0,3,3,1) = translation;
    rigidTransf.row(3) << 0,0,0,1;

    return rigidTransf;
}

// Ransac functions to detect outliers in the plane matching
void ransacPlaneAlignment_fit ( const CMatrixDouble &planeCorresp,
                                const mrpt::vector_size_t  &useIndices,
                                vector< CMatrixDouble > &fitModels )
                        //        vector< Matrix<T,4,4> > &fitModels )
{
    ASSERT_(useIndices.size()==3);

    try
    {
        CMatrixDouble corresp(8,3);

        //  cout << "Size planeCorresp: " << endl;
        //  cout << "useIndices " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << endl;
        for(unsigned i=0; i<3; i++)
            corresp.col(i) = planeCorresp.col(useIndices[i]);

        fitModels.resize(1);
        //    Matrix<T,4,4> &M = fitModels[0];
        CMatrixDouble &M = fitModels[0];
        M = registerMatchedPlanes(corresp);
    }
    catch(exception &)
    {
        fitModels.clear();
        return;
    }
}

void ransac3Dplane_distance(const CMatrixDouble &planeCorresp,
                            const vector< CMatrixDouble > & testModels,
                            const double distanceThreshold,
                            unsigned int & out_bestModelIndex,
                            mrpt::vector_size_t & out_inlierIndices )
{
    ASSERT_( testModels.size()==1 )
            out_bestModelIndex = 0;
    const CMatrixDouble &M = testModels[0];

    Matrix<T,3,3> Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
    Matrix<T,3,1> translation; translation << M(0,3), M(1,3), M(2,3);

    ASSERT_( size(M,1)==4 && size(M,2)==4 )

            const size_t N = size(planeCorresp,2);
    out_inlierIndices.clear();
    out_inlierIndices.reserve(100);
    for (size_t i=0;i<N;i++)
    {
        const Matrix<T,3,1> n_1(planeCorresp(0,i), planeCorresp(1,i), planeCorresp(2,i));
        const Matrix<T,3,1> n_2 = Rotation * Matrix<T,3,1> (planeCorresp(4,i), planeCorresp(5,i), planeCorresp(6,i));
        const float d_error = fabs((planeCorresp(7,i) - translation.dot(n_1)) - planeCorresp(3,i));
        const float angle_error = (n_1 .cross (n_2 )).norm();

        if (d_error < distanceThreshold)
            if (angle_error < distanceThreshold) // Warning: this threshold has a different dimension
                out_inlierIndices.push_back(i);
    }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
bool ransac3Dplane_degenerate(const CMatrixDouble &planeCorresp, const mrpt::vector_size_t &useIndices )
{
    ASSERT_( useIndices.size()==3 )

    const Matrix<T,3,1> n_1(planeCorresp(0,useIndices[0]), planeCorresp(1,useIndices[0]), planeCorresp(2,useIndices[0]));
    const Matrix<T,3,1> n_2(planeCorresp(0,useIndices[1]), planeCorresp(1,useIndices[1]), planeCorresp(2,useIndices[1]));
    const Matrix<T,3,1> n_3(planeCorresp(0,useIndices[2]), planeCorresp(1,useIndices[2]), planeCorresp(2,useIndices[2]));
    //cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

    if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
        return true;

    return false;
}

void ExtrinsicCalibPlanes::getCorrespondences(const vector<pcl::PointCloud<PointT>::Ptr> & cloud)
{
    cout << "ExtrinsicCalibPlanes::getCorrespondences... \n";
    vector<size_t> planesSourceIdx(num_sensors+1, 0);
    all_planes = mrpt::pbmap::PbMap();

    //						Segment local planes
    //==================================================================
    // #pragma omp parallel num_threads(num_sensors)
    for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
    {
        // sensor_id = omp_get_thread_num();
//        cout << sensor_id << " cloud " << cloud[sensor_id]->height << "x" << cloud[sensor_id]->width << endl;
//        PbMap::pbMapFromPCloud(cloud[sensor_id], v_pbmap[sensor_id]);
        all_planes.MergeWith(v_pbmap[sensor_id], Rt_estimated[sensor_id].cast<float>());
        planesSourceIdx[sensor_id+1] = planesSourceIdx[sensor_id] + v_pbmap[sensor_id].vPlanes.size();
        //cout << planesSourceIdx[sensor_id+1] << " ";
    }

    //==============================================================================
    //								Data association
    //==============================================================================
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
    {
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            sensor_pair = {sensor1, sensor2};
            double thres_trans = max(v_approx_trans[sensor1], v_approx_trans[sensor2]);
            double thres_rot_cos = 1 - pow(sin(mrpt::utils::DEG2RAD(max(v_approx_rot[sensor1], v_approx_rot[sensor2]))),2);
            for(size_t i=0; i < v_pbmap[sensor1].vPlanes.size(); i++) // Find plane correspondences
            {
                size_t planesIdx_i = planesSourceIdx[sensor1];
                for(size_t j=0; j < v_pbmap[sensor2].vPlanes.size(); j++)
                {
                    size_t planesIdx_j = planesSourceIdx[sensor2];
                    //cout << "  Check planes " << planesIdx_i+i << " and " << planesIdx_j+j << endl;

                    if( all_planes.vPlanes[planesIdx_i+i].inliers.size() > min_inliers && all_planes.vPlanes[planesIdx_j+j].inliers.size() > min_inliers &&
                        all_planes.vPlanes[planesIdx_i+i].elongation < 5 && all_planes.vPlanes[planesIdx_j+j].elongation < 5 &&
                        all_planes.vPlanes[planesIdx_i+i].v3normal .dot (all_planes.vPlanes[planesIdx_j+j].v3normal) > thres_rot_cos &&
                        fabs(all_planes.vPlanes[planesIdx_i+i].d - all_planes.vPlanes[planesIdx_j+j].d) < thres_trans )//&&
                        // v_pbmap[0].vPlanes[i].hasSimilarDominantColor(v_pbmap[1].vPlanes[j],0.06) &&
                        // v_pbmap[0].vPlanes[planes_counter_i+i].isPlaneNearby(v_pbmap[1].vPlanes[planes_counter_j+j], 0.5)
                    {
                        cout << "  b_confirm_visually " << b_confirm_visually << "\n";
                        if(b_confirm_visually)
                        {
                            plane_candidate_all[0] = planesIdx_i; plane_candidate_all[1] = planesIdx_j;
                            b_wait_plane_confirm = true;
                            confirm_corresp = 0;
                            while(confirm_corresp == 0)
                                boost::this_thread::sleep (boost::posix_time::milliseconds (50));
//                                mrpt::system::sleep(50);
                            b_wait_plane_confirm = false;
                        }
                        if(confirm_corresp == -1)
                            continue;

                        cout << "\tAssociate planes " << endl;
                        // cout << "Corresp " << v_pbmap[sensor1].vPlanes[i].v3normal.transpose() << " vs " << v_pbmap[sensor2].vPlanes[j].v3normal.transpose() << " = " << v_pbmap[1].vPlanes[j].v3normal.transpose() << endl;
                        mmm_plane_matches_all[sensor1][sensor2][planesIdx_i] = planesIdx_j;

                        // Calculate conditioning
                        mm_covariance[sensor1][sensor2] += (v_pbmap[sensor2].vPlanes[j].v3normal * v_pbmap[sensor1].vPlanes[i].v3normal.transpose()).cast<T>();
                        calcConditioningPair(sensor1, sensor2);
                        cout << "    conditioning " << sensor1 << "-" << sensor2 << " : " << mm_conditioning[sensor1][sensor2] << "\n";

                        // Store the parameters of the matched planes
                        size_t prevSize = planes.mm_corresp[sensor1][sensor2].getRowCount();
                        planes.mm_corresp[sensor1][sensor2].setSize(prevSize+1, planes.mm_corresp[sensor1][sensor2].getColCount());
                        planes.mm_corresp[sensor1][sensor2](prevSize, 0) = v_pbmap[sensor1].vPlanes[i].v3normal[0];
                        planes.mm_corresp[sensor1][sensor2](prevSize, 1) = v_pbmap[sensor1].vPlanes[i].v3normal[1];
                        planes.mm_corresp[sensor1][sensor2](prevSize, 2) = v_pbmap[sensor1].vPlanes[i].v3normal[2];
                        planes.mm_corresp[sensor1][sensor2](prevSize, 3) = v_pbmap[sensor1].vPlanes[i].d;
                        planes.mm_corresp[sensor1][sensor2](prevSize, 4) = v_pbmap[sensor2].vPlanes[j].v3normal[0];
                        planes.mm_corresp[sensor1][sensor2](prevSize, 5) = v_pbmap[sensor2].vPlanes[j].v3normal[1];
                        planes.mm_corresp[sensor1][sensor2](prevSize, 6) = v_pbmap[sensor2].vPlanes[j].v3normal[2];
                        planes.mm_corresp[sensor1][sensor2](prevSize, 7) = v_pbmap[sensor2].vPlanes[j].d;

                        // Store the uncertainty information about the matched planes
//                        Matrix<T,4,4> informationFusion;
//                        Matrix<T,4,4> tf = Matrix<T,4,4>::Identity();
//                        tf.block(0,0,3,3) = calib_planes.Rt_estimated.block(0,0,3,3);
//                        tf.block(3,0,1,3) = -calib_planes.Rt_estimated.block(0,3,3,1).transpose();
//                        informationFusion = v_pbmap[0].vPlanes[i].information;
//                        informationFusion += tf * v_pbmap[1].vPlanes[j].information * tf.inverse();
//                        JacobiSVD<Matrix<T,4,4> > svd_cov(informationFusion, ComputeFullU | ComputeFullV);
//                        Vector4f minEigenVector = svd_cov.matrixU().block(0,3,4,1);
//                        cout << "minEigenVector " << minEigenVector.transpose() << endl;
//                        informationFusion -= svd.singularValues().minCoeff() * minEigenVector * minEigenVector.transpose();
//                        cout << "informationFusion \n" << informationFusion << "\n minSV " << svd.singularValues().minCoeff() << endl;

//                        planes.mm_corresp[sensor1][sensor2](prevSize, 8) = informationFusion(0,0);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 9) = informationFusion(0,1);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 10) = informationFusion(0,2);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 11) = informationFusion(0,3);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 12) = informationFusion(1,1);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 13) = informationFusion(1,2);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 14) = informationFusion(1,3);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 15) = informationFusion(2,2);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 16) = informationFusion(2,3);
//                        planes.mm_corresp[sensor1][sensor2](prevSize, 17) = informationFusion(3,3);


//                        FIMrot += -skew(v_pbmap[1].vPlanes[j].v3normal) * informationFusion.block(0,0,3,3) * skew(v_pbmap[1].vPlanes[j].v3normal);
//                        FIMtrans += v_pbmap[0].vPlanes[i].v3normal * v_pbmap[0].vPlanes[i].v3normal.transpose() * informationFusion(3,3);

//                        JacobiSVD<Matrix<T,3,3> > svd_rot(FIMrot, ComputeFullU | ComputeFullV);
//                        JacobiSVD<Matrix<T,3,3> > svd_trans(FIMtrans, ComputeFullU | ComputeFullV);
//                        // T conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//                        CMatrixDouble conditioningFIM;
//                        conditioningFIM.setSize(prevSize+1, conditioningFIM.getColCount());
//                        conditioningFIM(prevSize, 0) = svd_rot.singularValues()[0];
//                        conditioningFIM(prevSize, 1) = svd_rot.singularValues()[1];
//                        conditioningFIM(prevSize, 2) = svd_rot.singularValues()[2];
//                        conditioningFIM(prevSize, 3) = svd_trans.singularValues()[0];
//                        conditioningFIM(prevSize, 4) = svd_trans.singularValues()[1];
//                        conditioningFIM(prevSize, 5) = svd_trans.singularValues()[2];

////                            matches.covariances[sensor1] += all_planes.vPlanes[planesIdx_i+i].v3normal * all_planes.vPlanes[planesIdx_j+j].v3normal.transpose();
////                            matches.calcConditioningPair(sensor1);

                    }
                }
            }
        }
    }
}

double ExtrinsicCalibPlanes::calcRotationErrorPair(const CMatrixDouble & correspondences, const Matrix<T,3,3> & Rot, bool in_deg)
{
    double accum_error2 = 0.0;
    double accum_error_deg = 0.0;
    for(int i=0; i < correspondences.rows(); i++)
    {
        // T weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
        T weight = 1.0;
        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        Matrix<T,3,1> rot_error = (n1 - Rot*n2);
        accum_error2 += weight * rot_error.dot(rot_error);
        accum_error_deg += mrpt::utils::RAD2DEG(acos((n1).dot(Rot*n2)));
    }

    if(in_deg)
        return accum_error_deg;
    return accum_error2;
}

double ExtrinsicCalibPlanes::calcRotationError(const vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > > & Rt, bool in_deg)
{
    if( Rt.size() != num_sensors )
        throw runtime_error("ERROR ExtrinsicCalibPlanes::calcRotationError -> Rt.size() != num_sensors \n\n");

    double accum_error2 = 0.0;
//    double accum_error_deg = 0.0;
    size_t num_corresp = 0;
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            accum_error2 += calcRotationErrorPair(planes.mm_corresp[sensor1][sensor2], Rt[sensor1].block<3,3>(0,0).transpose()*Rt[sensor2].block<3,3>(0,0), in_deg);
            num_corresp += planes.mm_corresp[sensor1][sensor2].rows();
        }

    //cout << "AvError deg " << accum_error2 / num_corresp << endl;
    //return accum_error2 / num_corresp;
    return accum_error2;
}

double ExtrinsicCalibPlanes::calcTranslationErrorPair(const mrpt::math::CMatrixDouble & correspondences, const Eigen::Matrix<T,4,4> & Rt1, const Eigen::Matrix<T,4,4> & Rt2, bool in_meters)
{
    T accum_error2 = 0.0;
    T accum_error_m = 0.0;
    Matrix<T,3,1> translation = Rt1.block(0,0,3,3).transpose() * (Rt2.block(0,3,3,1) - Rt1.block(0,3,3,1));
    for(int i=0; i < correspondences.rows(); i++)
    {
        //        T weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        T trans_error = (correspondences(i,3) - correspondences(i,7) + n1.dot(translation));
        accum_error2 += trans_error*trans_error; // weight *
        accum_error_m += fabs(trans_error);
    }
    cout << "calcTranslationErrorPair " << accum_error2 << " AvError deg " << accum_error_m / correspondences.rows() << endl;

    if(in_meters)
        return accum_error_m;
    return accum_error2;
}

double ExtrinsicCalibPlanes::calcTranslationError(const vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > > & Rt, bool in_meters)
{
    if( Rt.size() != num_sensors )
        throw runtime_error("ERROR ExtrinsicCalibPlanes::calcTranslationError -> Rt.size() != num_sensors \n\n");

    double error = 0.0;
    size_t num_corresp = 0;
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            error += calcTranslationErrorPair(planes.mm_corresp[sensor1][sensor2], Rt[sensor1], Rt[sensor2], in_meters);
            num_corresp += planes.mm_corresp[sensor1][sensor2].rows();
        }

    //cout << "AvError deg " << accum_error2 / num_corresp << endl;
    if(in_meters)
        return error / num_corresp;
    return error;
}

//Matrix<T,3,3> ExtrinsicCalibPlanes::calcRotationFIM(const CMatrixDouble & correspondences)
//{
//    Matrix<T,3,3> FIM = Matrix<T,3,3>::Zero();

//    for(unsigned i=0; i < correspondences.rows(); i++)
//    {
//        //          T weight = (inliers / correspondences(i,3)) / correspondences.rows()
//        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);

//        Matrix<T,3,1> n1_x_n2 = (Rt_estimated[sensor1]<3,3>(0,0)*n1) .cross (Rt_estimated[sensor2]<3,3>(0,0)*n2);

//        FIM += n1_x_n2 * n1_x_n2.transpose();
//    }

//    return FIM;
//}

//Matrix<T,3,3> ExtrinsicCalibPlanes::calcTranslationFIM()
//{
//    Matrix<T,3,3> FIM = Matrix<T,3,3>::Zero();

//    for(unsigned i=0; i < correspondences.rows(); i++)
//    {
//        //          T weight = (inliers / correspondences(i,3)) / correspondences.rows()
//        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        //        Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);
//        T d_obs_i = correspondences(i,3);
//        T d_obs_2 = correspondences(i,7);

//        Matrix<T,3,1> score = calcScoreTranslation(n1, d_obs_i, d_obs_2);

//        FIM += score * score.transpose();
//    }

//    return FIM;
//}


//    Matrix<T,3,3> calcFisherInfMat(const int weightedLS)
//    {
//      // Calibration system
//      Matrix<T,3,3> cov = Matrix<T,3,3>::Zero();
//
//      Matrix<T,3,3> FIM_rot = Matrix<T,3,3>::Zero();
//      Matrix<T,3,3> FIM_trans = Matrix<T,3,3>::Zero();
//      Matrix<T,3,1> score;
//
//      T accum_error2 = 0;
////      cov += v3normal2 * v3normal1.transpose();
//      for(unsigned i=0; i < correspondences.rows(); i++)
//      {
////          T weight = (inliers / correspondences(i,3)) / correspondences.rows()
//        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);
//        Matrix<T,3,1> n_1 = n1;
//        Matrix<T,3,1> n_2 = Rt_estimated.block(0,0,3,3) * n2;
//        Matrix<T,3,1> rot_error = (n_1 - n_2);
//        accum_error2 += fabs(rot_error.dot(rot_error));
//
//        if(weightedLS == 1 && correspondences.cols() == 10)
//        {
//          T weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
//          cov += weight * n2 * n1.transpose();
//        }
//        else
//          cov += n2 * n1.transpose();
//
//        T d_obs_i = correspondences(i,3);
//        T d_obs_2 = correspondences(i,7);
//        score = calcRotationErrorCross(n1, n2);
//        FIM_rot += score * score.transpose();
//        score = calcScoreTranslation(n1, d_obs_i, d_obs_2);
//        FIM_trans += score * score.transpose();
////      cout << "\nFIM_rot \n" << FIM_rot << "\ncov \n" << cov << "\nFIM_trans \n" << FIM_trans << "\n det " << FIM_rot.determinant() << "\n det2 " << FIM_trans.determinant() << endl;
//      }
//      JacobiSVD<Matrix<T,3,3> > svd(cov, ComputeFullU | ComputeFullV);
//      T conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//
//      T minFIM_rot = min(FIM_rot(0,0), min(FIM_rot(1,1), FIM_rot(2,2)));
//      T minFIM_trans = min(FIM_trans(0,0), min(FIM_trans(1,1), FIM_trans(2,2)));
////      cout << "minFIM_rot " << minFIM_rot << " " << minFIM_trans << " conditioning " << conditioning << " numCorresp " << correspondences.rows() << endl;
//      cout << "\nFIM_rot \n" << FIM_rot << endl;
//      cout << "\nFIM_trans \n" << FIM_trans << endl;
//    }

Matrix<T,3,3> ExtrinsicCalibPlanes::CalibrateRotationPair(const size_t sensor1, const size_t sensor2)//, const bool weight_uncertainty)
{
    cout << "ExtrinsicCalibPlanes::CalibrateRotationPair... " << planes.mm_corresp[sensor1][sensor2].rows() << " correspondences\n";
    CTicTac clock; clock.Tic(); //Clock to measure the runtime

    mm_covariance[sensor1][sensor2] = Matrix<T,3,3>::Zero();
//    Matrix<T,3,3> cov = Matrix<T,3,3>::Zero();
    // Matrix<T,3,3> FIM_rot = Matrix<T,3,3>::Zero();
    T accum_error2 = 0;
    CMatrixDouble & correspondences = planes.mm_corresp[sensor1][sensor2];
    for(int i=0; i < correspondences.rows(); i++)
    {
        //T weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Matrix<T,3,1> n1(correspondences(i,0), correspondences(i,1), correspondences(i,2));
        Matrix<T,3,1> n2(correspondences(i,4), correspondences(i,5), correspondences(i,6));
        Matrix<T,3,1> n_1 = Rt_estimated[sensor1].block(0,0,3,3) * n1;
        Matrix<T,3,1> n_2 = Rt_estimated[sensor2].block(0,0,3,3) * n2;
        Matrix<T,3,1> rot_error = (n_1 - n_2);
        accum_error2 += rot_error.dot(rot_error);
        mm_covariance[sensor1][sensor2] += n2 * n1.transpose();

        // Add perpendicular vectors for each pair of correspondences
        for(int j=i+1; j < correspondences.rows(); j++)
        {
            Matrix<T,3,1> n1b(correspondences(j,0), correspondences(j,1), correspondences(j,2));
            Matrix<T,3,1> n2b(correspondences(j,4), correspondences(j,5), correspondences(j,6));
//            Matrix<T,3,1> n_1b = Rt_estimated[sensor1].block(0,0,3,3) * n1b;
//            Matrix<T,3,1> n_2b = Rt_estimated[sensor2].block(0,0,3,3) * n2b;
            mm_covariance[sensor1][sensor2] += (n2.cross(n2b)) * (n1.cross(n1b)).transpose();
        }
    }

    // Calculate Rotation
    // cout << "Solve rotation";
    Matrix<T,3,3> rotation;
    T conditioning = rotationFromNormals(mm_covariance[sensor1][sensor2], rotation);
    cout << "conditioning " << conditioning << "accum_rot_error2 " << accum_error2 << " " << calcRotationErrorPair(planes.mm_corresp[sensor1][sensor2], rotation) << endl;
    cout << "average error: "
              << calcRotationErrorPair(planes.mm_corresp[sensor1][sensor2], Rt_estimated[sensor1].block<3,3>(0,0).transpose()*Rt_estimated[sensor2].block<3,3>(0,0), true) << " vs "
              << calcRotationErrorPair(planes.mm_corresp[sensor1][sensor2], rotation, true) << " degrees\n";
    cout << "  Estimation took " << 1000*clock.Tac() << " ms.\n";

    return rotation;
}


Eigen::Matrix<T,3,1> ExtrinsicCalibPlanes::CalibrateTranslationPair(const size_t sensor1, const size_t sensor2)//, const bool weight_uncertainty)
{
    // Calibration system
    Matrix<T,3,3> Hessian = Matrix<T,3,3>::Zero();
    Matrix<T,3,1> gradient = Matrix<T,3,1>::Zero();
    Matrix<T,3,1> translation = Rt_estimated[sensor1].block(0,0,3,3).transpose() * (Rt_estimated[sensor2].block(0,3,3,1) - Rt_estimated[sensor1].block(0,3,3,1));

    T accum_error2 = 0;
    CMatrixDouble & correspondences = planes.mm_corresp[sensor1][sensor2];
    for(int i=0; i < correspondences.rows(); i++)
    {
        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        T trans_error = (correspondences(i,7) - correspondences(i,3));
//        T trans_error = correspondences(i,3) - correspondences(i,7) + n1.dot(translation);
        accum_error2 += trans_error * trans_error;

//        if(weightedLS == 1 && correspondences.cols() == 18)
//        {
//            // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
//            //          T weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
//            T weight = correspondences(i,17);
//            Hessian += weight * (n1 * n1.transpose() );
//            gradient += weight * (n1 * trans_error);
//        }
//        else
        {
            Hessian += (n1 * n1.transpose() );
            gradient += (n1 * trans_error);
        }
    }

    //      cout << "Hessian \n" << Hessian << "\n HessianInv \n" << Hessian.inverse() << endl;
    //      calcFisherInfMat();
    JacobiSVD<Matrix<T,3,3> > svd(Hessian, ComputeFullU | ComputeFullV);
    T conditioning = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();
    cout << "conditioning " << conditioning << " FIM translation " << svd.singularValues().transpose() << endl;
    if(conditioning < threshold_conditioning)
        return translation;

    translation = Hessian.inverse() * gradient;

    return translation;
}

/*! Get the rotation of each sensor in a multisensor setup. The first sensor (sensor_id=0) is taken as reference. */
//vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > >
void ExtrinsicCalibPlanes::CalibrateRotationManifold(const bool weight_uncertainty)
{
    cout << "ExtrinsicCalibPlanes::CalibrateRotationManifold...\n";
    const size_t n_DoF = 3*(num_sensors-1);
    T accum_error2;
    T av_angle_error;

    // Parameters of the Least-Squares optimization
    int _max_iterations = 10;
    T _epsilon_transf = 0.00001;
    T _convergence_error = 0.000001;

    T increment = 1000, diff_error = 1000;
    int it = 0;
    while(it < _max_iterations && increment > _epsilon_transf && diff_error > _convergence_error)
    {
        // Calculate the Hessian and the gradient
        Matrix<T,Dynamic,Dynamic> Hessian(n_DoF,n_DoF); //= Matrix<T,Dynamic,Dynamic>::Zero(n_DoF,n_DoF); // Hessian of the rotation of the decoupled system
        Matrix<T,Dynamic,1> gradient(n_DoF,1); //= Matrix<T,Dynamic,Dynamic>::Zero(n_DoF,1); // Gradient of the rotation of the decoupled system
        Matrix<T,Dynamic,1> update_vector(n_DoF,1);
        cout << it << " Hessian \n" << Hessian << "gradient \n" << gradient.transpose() << endl;
        accum_error2 = 0.0;
        av_angle_error = 0.0;
        size_t numPlaneCorresp = 0;

        for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
        {
            for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
            {
                CMatrixDouble & correspondences = planes.mm_corresp[sensor1][sensor2];
                numPlaneCorresp += correspondences.rows();
                for(int i=0; i < correspondences.rows(); i++)
                {
                    //          T weight = (inliers / correspondences(i,3)) / correspondences.rows()
                    Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
                    Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);
                    Matrix<T,3,1> n_1 = Rt_estimated[sensor1].block(0,0,3,3) * n1;
                    Matrix<T,3,1> n_2 = Rt_estimated[sensor2].block(0,0,3,3) * n2;
                    Matrix<T,3,3> jacobian_rot_1 = skew<T>(-n_1);
                    Matrix<T,3,3> jacobian_rot_2 = skew<T>(n_2);
                    Matrix<T,3,1> rot_error = (n1 - n_2);
                    accum_error2 += rot_error.dot(rot_error);
                    av_angle_error += acos(n_1.dot(n_2));
                    //          cout << "rotation error_i " << rot_error.transpose() << endl;
//                    if(weight_uncertainty && correspondences.cols() == 10)
//                    {
//                        // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
//                        //              T weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
//                        //              Hessian += weight * (jacobian_rot_2.transpose() * jacobian_rot_2);
//                        //              gradient += weight * (jacobian_rot_2.transpose() * rot_error);
//                        Matrix<T,3,3> information;
//                        information << correspondences(i,8), correspondences(i,9), correspondences(i,10), correspondences(i,11),
//                                correspondences(i,9), correspondences(i,12), correspondences(i,13), correspondences(i,14),
//                                correspondences(i,10), correspondences(i,13), correspondences(i,15), correspondences(i,16),
//                                correspondences(i,11), correspondences(i,14), correspondences(i,16), correspondences(i,17);
//                        Hessian += jacobian_rot_2.transpose() * information.block(0,0,3,3) * jacobian_rot_2;
//                        gradient += jacobian_rot_2.transpose() * information.block(0,0,3,3) * rot_error;
//                    }
//                    else
                    {
                        if(sensor1 != 0) // The pose of the first camera is fixed
                        {
                            Hessian.block(3*(sensor1-1), 3*(sensor1-1), 3, 3) += jacobian_rot_1.transpose() * jacobian_rot_1;
                            gradient.block(3*(sensor1-1),0,3,1) += jacobian_rot_1.transpose() * rot_error;
                            // Cross term
                            Hessian.block(3*(sensor1-1), 3*(sensor2-1), 3, 3) += jacobian_rot_1.transpose() * jacobian_rot_2;
                        }
                        Hessian.block(3*(sensor2-1), 3*(sensor2-1), 3, 3) += jacobian_rot_2.transpose() * jacobian_rot_2;
                        gradient.block(3*(sensor2-1),0,3,1) += jacobian_rot_2.transpose() * rot_error;
                    }
                }
                if(sensor1 != 0) // Fill the lower left triangle with the corresponding cross terms
                    Hessian.block(3*(sensor2-1), 3*(sensor1-1), 3, 3) = Hessian.block(3*(sensor1-1), (sensor2-1), 3, 3).transpose();
            }
        }
        av_angle_error /= numPlaneCorresp;

        Eigen::JacobiSVD<Matrix<T,Dynamic,Dynamic> > svd(Hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        T conditioning = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();
        cout << "conditioning " << conditioning << endl;
        if(conditioning < threshold_conditioning)
            return; // Rt_estimated;

        // Solve the rotation
        update_vector = -Hessian.inverse() * gradient;
        cout << "update_vector " << update_vector.transpose() << endl;

        // Update rotation of the poses
        vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > > Rt_estim = Rt_estimated; // Load the initial extrinsic calibration from the device'
        for(size_t sensor_id = 1; sensor_id < num_sensors; sensor_id++)
        {
            mrpt::poses::CPose3D pose;
            CArrayNumeric< double, 3 > rot_manifold;
            rot_manifold[0] = update_vector(3*sensor_id-3,0);
            rot_manifold[1] = update_vector(3*sensor_id-2,0);
            rot_manifold[2] = update_vector(3*sensor_id-1,0);
            CMatrixDouble33 update_rot = pose.exp_rotation(rot_manifold);
            //cout << "update_rot\n" << update_rot << endl;
            Matrix<T,3,3> update_rot_eig;
            update_rot_eig <<   update_rot(0,0), update_rot(0,1), update_rot(0,2),
                                update_rot(1,0), update_rot(1,1), update_rot(1,2),
                                update_rot(2,0), update_rot(2,1), update_rot(2,2);
            Rt_estim[sensor_id].block(0,0,3,3) = update_rot_eig * Rt_estimated[sensor_id].block(0,0,3,3);
            //      cout << "old rotation" << sensor_id << "\n" << Rt_estimated[sensor_id].block(0,0,3,3) << endl;
            //      cout << "new rotation\n" << Rt_estim[sensor_id].block(0,0,3,3) << endl;
        }

        cout << " accum_error2 " << accum_error2 << endl;
        accum_error2 = calcRotationError(Rt_estimated);
        T new_accum_error2 = calcRotationError(Rt_estim);
        cout << "New rotation error " << new_accum_error2 << " previous " << accum_error2 << endl;

        // Assign new rotations
        if(new_accum_error2 < accum_error2)
        {
//            Rt_estimated = Rt_estim;
            for(size_t sensor_id = 1; sensor_id < num_sensors; sensor_id++)
                Rt_estimated[sensor_id].block(0,0,3,3) = Rt_estim[sensor_id].block(0,0,3,3);
        }

        increment = update_vector .dot (update_vector);
        diff_error = accum_error2 - new_accum_error2;
        ++it;
        //      cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
    }

    cout << "ErrorCalibRotation " << accum_error2 << " " << av_angle_error << endl;

//    return Rt_estim;
}

void ExtrinsicCalibPlanes::CalibrateTranslation(const bool weight_uncertainty)
{
    cout << "ExtrinsicCalibPlanes::CalibrateTranslation...\n";
    const size_t n_DoF = 3*(num_sensors-1);
    T accum_error2;
    size_t numPlaneCorresp = 0;

    Matrix<T,Dynamic,Dynamic> Hessian(n_DoF,n_DoF); //= Matrix<T,Dynamic,Dynamic>::Zero(n_DoF,n_DoF); // Hessian of the rotation of the decoupled system
    Matrix<T,Dynamic,1> gradient(n_DoF,1); //= Matrix<T,Dynamic,Dynamic>::Zero(n_DoF,1); // Gradient of the rotation of the decoupled system
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
    {
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            CMatrixDouble & correspondences = planes.mm_corresp[sensor1][sensor2];
            numPlaneCorresp += correspondences.rows();
            for(int i=0; i < correspondences.rows(); i++)
            {
                //          float weight = (inliers / it_pair->second(i,3)) / it_pair->second.rows()
                Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
                Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);
                Matrix<T,3,1> n_1 = Rt_estimated[sensor1].block(0,0,3,3) * n1;
                Matrix<T,3,1> n_2 = Rt_estimated[sensor2].block(0,0,3,3) * n2;
                T trans_error = (correspondences(i,3) - correspondences(i,7));
                accum_error2 += trans_error * trans_error;

                //          cout << "Rt_estimated_ \n" << Rt_estimated_[sensor_id] << " n_1 " << n_1.transpose() << " n_2 " << n_2.transpose() << endl;
//                if(weightedLS == 1 && it_pair->second.cols() == 18)
//                {
//                    // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
//                    float weight = (it_pair->second(i,8) / (it_pair->second(i,3) * it_pair->second(i,9)));// / it_pair->second.rows();

//                    if(sensor1 != 0) // The pose of the first camera is fixed
//                    {
//                        Hessian.block(3*(sensor1-1), 3*(sensor1-1), 3, 3) += weight * (n_1 * n_1.transpose() );
//                        gradient.block(3*(sensor1-1),0,3,1) += weight * (-n_1 * trans_error);

//                        // Cross term
//                        Hessian.block(3*(sensor1-1), 3*(sensor2-1), 3, 3) += weight * (-n_1 * n_2.transpose() );
//                    }

//                    Hessian.block(3*(sensor2-1), 3*(sensor2-1), 3, 3) += weight * (n_2 * n_2.transpose() );
//                    gradient.block(3*(sensor2-1),0,3,1) += weight * (n_2 * trans_error);
//                }
//                else
                {
                    if(sensor1 != 0) // The pose of the first camera is fixed
                    {
                        Hessian.block(3*(sensor1-1), 3*(sensor1-1), 3, 3) += n_1 * n_1.transpose();
                        gradient.block(3*(sensor1-1),0,3,1) += -n_1 * trans_error;

                        // Cross term
                        Hessian.block(3*(sensor1-1), 3*(sensor2-1), 3, 3) += -n_1 * n_2.transpose();
                    }

                    Hessian.block(3*(sensor2-1), 3*(sensor2-1), 3, 3) += n_2 * n_2.transpose();
                    gradient.block(3*(sensor2-1),0,3,1) += n_2 * trans_error;
                }
            }
            // Fill the lower left triangle with the corresponding cross terms
            if(sensor1 != 0)
                Hessian.block(3*(sensor2-1), 3*(sensor1-1), 3, 3) = Hessian.block(3*(sensor1-1), 3*(sensor2-1), 3, 3).transpose();
        }
    }
//    av_error /= numPlaneCorresp;

    Eigen::JacobiSVD<Eigen::Matrix<T,Dynamic,Dynamic> > svd(Hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    T conditioning = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();
    cout << "conditioning " << conditioning << endl;
    if(conditioning < threshold_conditioning)
        return; // Rt_estimated;

    // Solve the rotation
    Matrix<T,Dynamic,1> update_vector = -Hessian.inverse() * gradient;
    cout << "update_vector " << update_vector.transpose() << endl;

    // Update translation of the poses
//    Matrix<T,3,1> translation0 = Rt_estimated[0].block(0,3,3,1);
//    Rt_estimated[0].block(0,3,3,1) = -centerDevice;
    for(size_t sensor_id = 1; sensor_id < num_sensors; sensor_id++)
        Rt_estimated[sensor_id].block(0,3,3,1) = update_vector.block(3*sensor_id-3,0,3,1);// + translation0;
}

void ExtrinsicCalibPlanes::Calibrate()
{
    if(num_sensors == 2)
         CalibrateRotationPair();
    else
        CalibrateRotationManifold();

    CalibrateTranslation();
    //      cout << "Rt_estimated\n" << Rt_estimated << endl;

    cout << "Errors " << calcRotationError(Rt_estimated) << " av deg " << calcRotationError(Rt_estimated,true) << " av trans " << calcTranslationError(Rt_estimated,true) << endl;
}
