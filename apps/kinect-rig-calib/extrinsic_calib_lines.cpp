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

#include "extrinsic_calib_lines.h"
#include <mrpt/poses/CPose3D.h>
#include <mrpt/vision/CFeatureLines.h>
#include <mrpt/pbmap/PbMap.h>
#include <iostream>

#define VISUALIZE_SENSOR_DATA 0
#define SHOW_IMAGES 0

using namespace std;
using namespace Eigen;
using namespace mrpt::vision;
using namespace mrpt::math;

//// Ransac functions to detect outliers in the plane matching
//void ransacLineAlignment_fit(
//        const CMatrixDouble &planeCorresp,
//        const mrpt::vector_size_t  &useIndices,
//        vector< CMatrixDouble > &fitModels )
////        vector< Eigen::Matrix4f > &fitModels )
//{
//  ASSERT_(useIndices.size()==3);

//  try
//  {
//    CMatrixDouble corresp(8,3);

////  cout << "Size planeCorresp: " << endl;
////  cout << "useIndices " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << endl;
//    for(unsigned i=0; i<3; i++)
//      corresp.col(i) = planeCorresp.col(useIndices[i]);

//    fitModels.resize(1);
////    Eigen::Matrix4f &M = fitModels[0];
//    CMatrixDouble &M = fitModels[0];
//    M = getAlignment(corresp);
//  }
//  catch(exception &)
//  {
//    fitModels.clear();
//    return;
//  }
//}

//void ransacLine_distance(
//        const CMatrixDouble &planeCorresp,
//        const vector< CMatrixDouble > & testModels,
//        const double distanceThreshold,
//        unsigned int & out_bestModelIndex,
//        mrpt::vector_size_t & out_inlierIndices )
//{
//  ASSERT_( testModels.size()==1 )
//  out_bestModelIndex = 0;
//  const CMatrixDouble &M = testModels[0];

//  Eigen::Matrix3f Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
//  Eigen::Vector3f translation; translation << M(0,3), M(1,3), M(2,3);

//    ASSERT_( size(M,1)==4 && size(M,2)==4 )

//  const size_t N = size(planeCorresp,2);
//  out_inlierIndices.clear();
//  out_inlierIndices.reserve(100);
//  for (size_t i=0;i<N;i++)
//  {
//    const Eigen::Vector3f n_i = Eigen::Vector3f(planeCorresp(0,i), planeCorresp(1,i), planeCorresp(2,i));
//    const Eigen::Vector3f n_ii = Rotation * Eigen::Vector3f(planeCorresp(4,i), planeCorresp(5,i), planeCorresp(6,i));
//    const float d_error = fabs((planeCorresp(7,i) - translation.dot(n_i)) - planeCorresp(3,i));
//    const float angle_error = (n_i .cross (n_ii )).norm();

//    if (d_error < distanceThreshold)
//     if (angle_error < distanceThreshold) // Warning: this threshold has a different dimension
//      out_inlierIndices.push_back(i);
//  }
//}

///** Return "true" if the selected points are a degenerate (invalid) case.
//  */
//bool ransacLine_degenerate(
//        const CMatrixDouble &planeCorresp,
//        const mrpt::vector_size_t &useIndices )
//{
//  ASSERT_( useIndices.size()==3 )

//  const Eigen::Vector3f n_1 = Eigen::Vector3f(planeCorresp(0,useIndices[0]), planeCorresp(1,useIndices[0]), planeCorresp(2,useIndices[0]));
//  const Eigen::Vector3f n_2 = Eigen::Vector3f(planeCorresp(0,useIndices[1]), planeCorresp(1,useIndices[1]), planeCorresp(2,useIndices[1]));
//  const Eigen::Vector3f n_3 = Eigen::Vector3f(planeCorresp(0,useIndices[2]), planeCorresp(1,useIndices[2]), planeCorresp(2,useIndices[2]));
////cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

//  if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
//    return true;

//  return false;
//}

void ExtrinsicCalibLines::getCorrespondences(const vector<cv::Mat> & rgb)
{
    // Extract line segments
    v_lines3D[sensor_id].clear();
    CFeatureLines::extractLines(rgb[sensor_id], v_lines[sensor_id], min_pixels_line);
    cout << sensor_id << " lines " << v_lines[sensor_id].size() << endl;
    auto line = begin(v_lines[sensor_id]);
    while( line != end(v_lines[sensor_id]) ) // Filter lines with low gradient
    {
        float gradient = float((*line)[3] - (*line)[1]) / ((*line)[2] - (*line)[0]);
        if( fabs(gradient) < 0.8 )
            v_lines[sensor_id].erase(line);
        else
            ++line;
    }
    cout << sensor_id << " filtered lines " << v_lines[sensor_id].size() << endl;

//    if( v_pbmap[0].vPlanes.size() == 1 && v_pbmap[1].vPlanes.size() == 1)
//    {
//        cv::Mat image_lines;
//        //                rgb[sensor_id].convertTo(image_lines, CV_8UC1, 1.0 / 2);
//        for(auto line = begin(v_lines[sensor_id]); line != end(v_lines[sensor_id]); ++line)
//        {
//            double gradient = double((*line)[3] - (*line)[1]) / ((*line)[2] - (*line)[0]);
//            cout << "gradient " << gradient << endl;
//            cout << "segments " << (*line)[2] << " " << (*line)[3] << " " << (*line)[0] << " " << (*line)[1] << endl;
//            //                    float d1 = (v_pbmap[0].vPlanes[0].v3center.dot(v_pbmap[0].vPlanes[0].v3center))/(l.dot(v_pbmap[0].vPlanes[0].v3center));
//            //                    Vector3f l1 = d1*l;
//            rgb[0].convertTo(image_lines, CV_8UC1, 1.0 / 2);
//            cv::line(image_lines, cv::Point((*line)[0], (*line)[1]), cv::Point((*line)[2], (*line)[3]), cv::Scalar(255, 0, 255), 1);
//            cv::imshow("Output image", image_lines);
//            cv::waitKey(0);
//        }
//        //                cv::namedWindow("Output image");
//        //                cv::imshow("Output image", image_lines);
//        //                cv::waitKey(0);
//    }

    //==============================================================================
    //									Data association
    //==============================================================================
    cout << "Data association\n";
    line_corresp.clear();
    for(size_t sensor1=0; sensor1 < calib->num_sensors; sensor1++)
    {
        lines.mm_corresp[sensor1] = std::map<unsigned, mrpt::math::CMatrixDouble>();
        for(size_t sensor2=sensor1+1; sensor2 < calib->num_sensors; sensor2++)
        {
            Eigen::Matrix<T,4,4> pose_rel = calib->Rt_estimated[sensor2].inverse() * calib->Rt_estimated[sensor1];
//            cout << " sensor1 " << sensor1 << " " << v_lines[sensor1].size() << " sensor2 " << sensor2 << " " << v_lines[sensor2].size() << endl;
//            lines.mm_corresp[sensor1][sensor2] = mrpt::math::CMatrixDouble(0, 10);
        }
    }

    // Find line correspondences (in 2D by assuming a known rotation and zero translation)
    Eigen::Vector2f offset2(rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(0,2), rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(1,2));
    //v_lines3D[sensor1].resize(v_lines[sensor1].size());
    for(size_t i=0; i < v_lines[sensor1].size(); i++)
    {
        //cv::Vec4i &l1 = v_lines[sensor1][i];
        line_match1 = v_lines[sensor1][i];
        Vector3f v1( v_lines[sensor1][i][0]-rgbd_intrinsics[sensor1].rightCamera.intrinsicParams(0,2), (v_lines[sensor1][i][1]-rgbd_intrinsics[sensor1].rightCamera.intrinsicParams(1,2))*mxmy[sensor1], rgbd_intrinsics[sensor1].rightCamera.intrinsicParams(0,0));
        Vector3f v2( v_lines[sensor1][i][2]-rgbd_intrinsics[sensor1].rightCamera.intrinsicParams(0,2), (v_lines[sensor1][i][3]-rgbd_intrinsics[sensor1].rightCamera.intrinsicParams(1,2))*mxmy[sensor1], rgbd_intrinsics[sensor1].rightCamera.intrinsicParams(0,0));
        Vector3f n1 = v1.cross(v2); n1.normalize();
        //cout << "v1 " << v1.transpose() << " v2 " << v2.transpose() << endl;
        if( v_pbmap[sensor1].vPlanes.size() == 1 ) // Get the 3D line parameters from the plane intersection
        {
            // Compute the 3D line as the intersection of two planes (http://mathworld.wolfram.com/Plane-PlaneIntersection.html)
            Plane &plane = v_pbmap[sensor1].vPlanes[0];
            Vector3f p(0, 0, 0);
            p(2) = -plane.d / (plane.v3normal(2) - (plane.v3normal(0)*n1(2)/n1(0)));
            p(0) = -n1(2)*p(2)/n1(0);

            MatrixXf m = MatrixXf::Zero(2,3);
            m.row(0) = n1;
            m.row(1) = plane.v3normal;
            FullPivLU<MatrixXf> lu(m);
            MatrixXf m_null_space = lu.kernel(); m_null_space.normalize();
            Map<Vector3f> l(m_null_space.data(),3);
//                    v_lines3D[sensor1][i].block<3,1>(0,0) = p + l/2;
//                    v_lines3D[sensor1][i].block<3,1>(3,0) = p - l/2;
            Vector3f p1 = p + l/2, p2 = p - l/;
            mrpt::math::TLine3D line(mrpt::math::TPoint3D(p1[0],p1[1],p1[2]), mrpt::math::TPoint3D(p2[0],p2[1],p2[2])); //= makeTLine3D(p + l/2, p - l/2);
            v_lines3D[sensor1].push_back(line);
//            line3D_match1 = line;
            cout << "line3D " << v_lines3D[sensor1][i] << endl;
            //cout << "m_null_space \n" << m_null_space << endl;
        }

        for(size_t j=0; j < v_lines[sensor2].size(); j++)
        {
            Vector3f v1( v_lines[sensor2][j][0]-rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(0,2), (v_lines[sensor2][j][1]-rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(1,2))*mxmy[sensor2], rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(0,0));
            Vector3f v2( v_lines[sensor2][j][2]-rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(0,2), (v_lines[sensor2][j][3]-rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(1,2))*mxmy[sensor2], rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(0,0));
            Vector3f n2 = v1.cross(v2); n2.normalize();
            //cout << "v1 " << v1.transpose() << " v2 " << v2.transpose() << endl;
            cout << "n1 " << n1.transpose() << " n2 " << n2.transpose() << " dot " << fabs((calib->Rt_estimated[sensor1].block<3,3>(0,0)*n1) .dot (calib->Rt_estimated[sensor2].block<3,3>(0,0)*n2)) << endl;
            if( fabs((calib->Rt_estimated[sensor1].block<3,3>(0,0)*n1) .dot (calib->Rt_estimated[sensor2].block<3,3>(0,0)*n2)) > 0.99 )
            {
                line_match2 = v_lines[sensor2][j];
                b_freeze = false;
//                    cv::waitKey(0);
                Vector3f n1_2 = pose_rel.block<3,3>(0,0) * n1;
                Vector2f p(0, 0); // The 2D point on the image plane at y =width/2
                p(0) = -n1_2(2)*rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(0,0) / n1_2(0);
                MatrixXd m = MatrixXd::Zero(2,3);
                m.row(0) = n1_2.cast<double>();
                m(1,2) = 1;
                FullPivLU<MatrixXd> lu(m);
                MatrixXd m_null_space = lu.kernel(); m_null_space.normalize();
                //cout << "m_null_space \n" << m_null_space << endl;
                Vector2f p1(0,-width/2), p2(0,width/2); // The 2D points on the image plane at y=0 and y =width
                p1(0) = (p1(1)-p(1))*m_null_space(0)/m_null_space(1) + p(0);
                p2(0) = (p2(1)-p(1))*m_null_space(0)/m_null_space(1) + p(0);
                p += offset2; p1 += offset2; p2 += offset2;
//                        Map<Vector3f> l(m_null_space.data(),3);
//                        Vector3f p3(p(0), p(1), );
//                        line3D_match2 = makeLine3D(p3 + l/2, p3 - l/2);
                cout << "p " << p.transpose() << " p1 " << p1.transpose() << " p2 " << p2.transpose() << endl;
                if(display)
                {
                    cv::Mat rgb_concat(height, 2*width+20, CV_8UC3, cv::Scalar(255,255,255));
                    cv::Mat img_transposed, img_rotated;
                    cv::transpose(rgb[0], img_transposed);
                    cv::flip(img_transposed, img_rotated, 0);
                    cv::Mat tmp = rgb_concat(cv::Rect(0, 0, width, height));
                    img_rotated.copyTo(tmp);
                    cv::transpose(rgb[1], img_transposed);
                    cv::flip(img_transposed, img_rotated, 0);
                    tmp = rgb_concat(cv::Rect(width+20, 0, width, height));
                    img_rotated.copyTo(tmp);
                    cv::imshow("rgb", rgb_concat ); cv::moveWindow("rgb", 20,20);

                    cv::Mat image_lines;
                    rgb_concat.convertTo(image_lines, CV_8UC1, 1.0 / 2);
                    cv::line(image_lines, cv::Point(v_lines[sensor1][i][1], height-v_lines[sensor1][i][0]), cv::Point(v_lines[sensor1][i][3], height-v_lines[sensor1][i][2]), cv::Scalar(255, 0, 255), 1);
                    cv::line(image_lines, cv::Point(width+20+v_lines[sensor2][j][1], height-v_lines[sensor2][j][0]), cv::Point(width+20+v_lines[sensor2][j][3], height-v_lines[sensor2][j][2]), cv::Scalar(255, 0, 255), 1);
                    cv::line(image_lines, cv::Point(width+20+p1(1), height-p1(0)), cv::Point(width+20+p2(1), height-p2(0)), cv::Scalar(0, 150, 0), 1);
                    cv::circle(image_lines, cv::Point(width+20+p(1), height-p(0)), 3, cv::Scalar(0, 0, 200), 3);
                    cv::imshow("Line match", image_lines);
                    cv::waitKey(0);
                }
            }
        }
    }
}

float ExtrinsicCalibLines::calcCorrespRotError(Eigen::Matrix3f &Rot_)
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

Eigen::Matrix3f ExtrinsicCalibLines::calcFIMRotation()
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

Eigen::Matrix3f ExtrinsicCalibLines::calcFIMTranslation()
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
//        Eigen::Vector3f n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
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

Eigen::Matrix3f ExtrinsicCalibLines::CalibrateRotation(int weightedLS)
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
        Eigen::Vector3f n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
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
    //      std::cout << "minFIM_rot " << minFIM_rot << std::endl;// << " " << calcCorrespRotError(calib->Rt_estimated) << std::endl;
    //      std::cout << "accum_rot_error2 av_deg " << acos(accum_error2/correspondences.rows()) << std::endl;// << " " << calcCorrespRotError(calib->Rt_estimated) << std::endl;
    //      std::cout << "calib->Rt_estimated\n" << calib->Rt_estimated << std::endl;

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


Eigen::Matrix3f ExtrinsicCalibLines::CalibrateRotationD(int weightedLS)
{
    // Calibration system
    Eigen::Matrix3d rotationCov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rotation_estim = calib->Rt_estimated.block(0,0,3,3).cast<double>();

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
    std::cout << "accum_rot_error2 av deg" << acos(accum_error2/correspondences.rows()) << std::endl;// << " " << calcCorrespRotError(calib->Rt_estimated) << std::endl;
    std::cout << "calib->Rt_estimated\n" << calib->Rt_estimated << std::endl;

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
    std::cout << "accum_rot_error2 optim av deg" << acos(calcCorrespRotError(calib->Rt_estimated)) << std::endl;
    cout << "Rotation (double)\n" << rotation << endl;

    return rotation.cast<float>();
    //      return Eigen::Matrix3f::Identity();
}

/*! Get the rotation of each sensor in the multisensor RGBD360 setup */
Eigen::Matrix3f ExtrinsicCalibLines::CalibrateRotationManifold(int weightedLS)
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
    //        calib->Rt_estimated[sensor_id] = Rt_specs[sensor_id];

    Eigen::Matrix4f calib->Rt_estimatedTemp;
    calib->Rt_estimatedTemp = calib->Rt_estimated;

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
                Eigen::Vector3f n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
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
            calib->Rt_estimatedTemp = calib->Rt_estimated;
            calib->Rt_estimatedTemp.block(0,0,3,3) = update_rot_eig * calib->Rt_estimated.block(0,0,3,3);
            //      cout << "old rotation" << sensor_id << "\n" << calib->Rt_estimated.block(0,0,3,3) << endl;
            //      cout << "new rotation\n" << calib->Rt_estimatedTemp.block(0,0,3,3) << endl;
        }

        accum_error2 = calcCorrespRotError(calib->Rt_estimated);
        //        float new_accum_error2 = calcCorrespRotErrorWeight(calib->Rt_estimatedTemp);
        float new_accum_error2 = calcCorrespRotError(calib->Rt_estimatedTemp);

        //        cout << "New rotation error " << new_accum_error2 << " previous " << accum_error2 << endl;
        //    cout << "Closing loop? \n" << calib->Rt_estimated[0].inverse() * calib->Rt_estimated[7] * Rt_78;

        // Assign new rotations
        if(new_accum_error2 < accum_error2)
            //          for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
            calib->Rt_estimated = calib->Rt_estimatedTemp;
        //            calib->Rt_estimated.block(0,0,3,3) = calib->Rt_estimatedTemp.block(0,0,3,3);

        increment = update_vector .dot (update_vector);
        diff_error = accum_error2 - new_accum_error2;
        ++it;
        //      cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
    }

    std::cout << "ErrorCalibRotation " << accum_error2 << " " << av_angle_error << std::endl;
    std::cout << "Rotation \n"<< calib->Rt_estimated.block(0,0,3,3) << std::endl;

    rotation = calib->Rt_estimated.block(0,0,3,3);
    return rotation;
}

Eigen::Vector3f ExtrinsicCalibLines::CalibrateTranslation(int weightedLS)
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
        //        Eigen::Vector3f n_i = calib->Rt_estimated[sensor_id].block(0,0,3,3) * n_obs_i;
        //        Eigen::Vector3f n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
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

void ExtrinsicCalibLines::CalibratePair()
{
    //      calibrated_Rt = Eigen::Matrix4f::Identity();
    calib->Rt_estimated.block(0,0,3,3) = CalibrateRotation();
    calib->Rt_estimated.block(0,3,3,1) = CalibrateTranslation();
    //      std::cout << "calib->Rt_estimated\n" << calib->Rt_estimated << std::endl;

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
    std::cout << "Errors n.n " << calcCorrespRotError(calib->Rt_estimated) << " av deg " << av_rot_error*180/3.14159f << " av trans " << av_trans_error << std::endl;
}

/*! Print the number of correspondences and the conditioning number to the standard output */
void ExtrinsicCalibLines::printConditioning()
{
    std::cout << "Conditioning\n";
    for(unsigned sensor_id = 0; sensor_id < n_sensors_-1; sensor_id++)
        std::cout << mm_planes[sensor_id][sensor_id+1].rows() << "\t";
    std::cout << mm_planes[0][n_sensors_-1].rows() << "\t";
    std::cout << endl;
    for(unsigned sensor_id = 0; sensor_id < n_sensors_; sensor_id++)
        std::cout << conditioning_[sensor_id] << "\t";
    std::cout << std::endl;
}

//    /*! Update adjacent conditioning (information between a pair of adjacent sensors) */
//    void updateAdjacentConditioning(unsigned couple_id, pair< Eigen::Vector4f, Eigen::Vector4f> &match)
//    {
//      ++conditioning_measCount[couple_id];
//      covariances_[couple_id] += match.second.head(3) * match.first.head(3).transpose();
////      Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariances_[couple_id], Eigen::ComputeFullU | Eigen::ComputeFullV);
////      conditioning_[couple_id] = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//    }

/*! Calculate adjacent conditioning (information between a pair of adjacent sensors) */
void ExtrinsicCalibLines::calcAdjacentConditioning(unsigned couple_id)
{
    //      if(conditioning_measCount[couple_id] > 3)
    //      {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariances_[couple_id], Eigen::ComputeFullU | Eigen::ComputeFullV);
    conditioning_[couple_id] = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    //      }
}
