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
using namespace mrpt::utils;

//// Ransac functions to detect outliers in the plane matching
//void ransacLineAlignment_fit(
//        const CMatrixDouble &planeCorresp,
//        const mrpt::vector_size_t  &useIndices,
//        vector< CMatrixDouble > &fitModels )
////        vector< Matrix4f > &fitModels )
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
////    Matrix4f &M = fitModels[0];
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

//  Matrix3f Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
//  Vector3f translation; translation << M(0,3), M(1,3), M(2,3);

//    ASSERT_( size(M,1)==4 && size(M,2)==4 )

//  const size_t N = size(planeCorresp,2);
//  out_inlierIndices.clear();
//  out_inlierIndices.reserve(100);
//  for (size_t i=0;i<N;i++)
//  {
//    const Vector3f n_i = Vector3f(planeCorresp(0,i), planeCorresp(1,i), planeCorresp(2,i));
//    const Vector3f n_ii = Rotation * Vector3f(planeCorresp(4,i), planeCorresp(5,i), planeCorresp(6,i));
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

//  const Vector3f n_1 = Vector3f(planeCorresp(0,useIndices[0]), planeCorresp(1,useIndices[0]), planeCorresp(2,useIndices[0]));
//  const Vector3f n_2 = Vector3f(planeCorresp(0,useIndices[1]), planeCorresp(1,useIndices[1]), planeCorresp(2,useIndices[1]));
//  const Vector3f n_3 = Vector3f(planeCorresp(0,useIndices[2]), planeCorresp(1,useIndices[2]), planeCorresp(2,useIndices[2]));
////cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

//  if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
//    return true;

//  return false;
//}

void ExtrinsicCalibLines::getSegments3D(const TCamera & cam, const pcl::PointCloud<PointT>::Ptr & cloud, const mrpt::pbmap::PbMap & pbmap, const vector<cv::Vec4i> & segments2D,
                                        const std::vector<Eigen::Vector3f> & segments_n, vector<Matrix<T,6,1> > & segments3D, vector<bool> & line_has3D)
{
    size_t n_seg = segments2D.size();
    segments_n.resize(n_seg);
    segments3D.resize(n_seg);
    line_has3D.resize(n_seg);
    fill(line_has3D.begin(), line_has3D.end(), false);

    for(size_t i=0; i < n_seg; i++)
    {
        // Compute the normal vector to the plane containing the 2D line segment and the optical center
        Vector3f v1( (x1-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y1-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        Vector3f v2( (x2-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y2-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        segments_n[i] = v1.cross(v2); segments_n[i].normalize();

        // Check the depth of the end points
        int x1 = segments2D[i][0], y1 = segments2D[i][1], x2 = segments2D[i][2], y2 = segments2D[i][3];
        int i1 = x1 + y1*cloud->width, i2 = x1 + y1*cloud->width;
        if(cloud->points[i1].z > 0.3f && cloud->points[i1].z < 10.f && cloud->points[i2].z > 0.3f && cloud->points[i2].z < 10.f)
        {
            line_has3D[i] = true;
            segments3D[i] << cloud->points[i1].x, cloud->points[i1].y, cloud->points[i1].z, cloud->points[i2].x, cloud->points[i2].y, cloud->points[i2].z;
        }

        // Check the depth of the line segment

        // Check if the end points are within a panar segment in v_pbmaps
        for(size_t j=0; j < pbmap.vPlanes.size(); j++)
        {
            mrpt::pbmap::Plane & plane = pbmap.vPlanes[j];
            if( plane.isInHull(i1) && plane.isInHull(i2) )
            {
//                // Compute the 3D line as the intersection of two planes (http://mathworld.wolfram.com/Plane-PlaneIntersection.html). Another option is (http://mathworld.wolfram.com/Line-PlaneIntersection.html)
//                Vector3f p(0, 0, 0);
//                p(2) = -plane.d / (plane.v3normal(2) - (plane.v3normal(0)*n1(2)/n1(0)));
//                p(0) = -n1(2)*p(2)/n1(0);

//                MatrixXf m = MatrixXf::Zero(2,3);
//                m.row(0) = n1;
//                m.row(1) = plane.v3normal;
//                FullPivLU<MatrixXf> lu(m);
//                MatrixXf m_null_space = lu.kernel(); m_null_space.normalize();
//                Map<Vector3f> l(m_null_space.data(),3);
//                  //v_lines3D[sensor1][i].block<3,1>(0,0) = p + l/2;
//                  //v_lines3D[sensor1][i].block<3,1>(3,0) = p - l/2;
//                Vector3f p1 = p + l/2, p2 = p - l/;
//                mrpt::math::TLine3D line(mrpt::math::TPoint3D(p1[0],p1[1],p1[2]), mrpt::math::TPoint3D(p2[0],p2[1],p2[2])); //= makeTLine3D(p + l/2, p - l/2);
//                v_lines3D[sensor1].push_back(line);
//    //            line3D_match1 = line;
//                cout << "line3D " << v_lines3D[sensor1][i] << endl;
//                //cout << "m_null_space \n" << m_null_space << endl;

                // Compute the 3D line as the intersection of a line and a plane. The equations are {n*p+d=0; p1+t(p2-p1)=p}, considering p1=(0,0,0) [the optical center] -> t = -d/n.dot(p2)
                double t1 = - plane.d / plane.v3normal.dot(v1);
                Vector3f p1 = t1*v1;
                double t2 = - plane.d / plane.v3normal.dot(v2);
                Vector3f p2 = t2*v2;
                line_has3D[i] = true;
                segments3D[i] << p1[0], p1[1], p1[2], p2[0], p2[1], p2[2];
//                segments3D[i].block(0,0,3,1) = p1.cast<T>();
//                segments3D[i].block(0,3,3,1) = p2.cast<T>();
//                mrpt::math::TLine3D line(mrpt::math::TPoint3D(p1[0],p1[1],p1[2]), mrpt::math::TPoint3D(p2[0],p2[1],p2[2])); //= makeTLine3D(p + l/2, p - l/2);
//                v_lines3D[sensor1].push_back(line);
                break;
            }
        }
    }
}

void ExtrinsicCalibLines::getCorrespondences(const vector<cv::Mat> & rgb, const vector<pcl::PointCloud::Ptr> & cloud)
{
    //						Extract line segments
    //==================================================================
    // #pragma omp parallel num_threads(num_sensors)
    for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
    {
        // sensor_id = omp_get_thread_num();
        cout << sensor_id << " cloud " << cloud[sensor_id]->height << "x" << cloud[sensor_id]->width << endl;

        CFeatureLines::extractLines(rgb[sensor_id], v_segments2D[sensor_id], min_pixels_line);
        cout << sensor_id << " lines " << v_segments2D[sensor_id].size() << endl;
        getSegments3D(rgb[sensor_id], cloud[sensor_id], rgbd_intrinsics[sensor_id].rightCamera, v_segments2D[sensor_id],
                      v_segment_n[sensor_id], v_segments3D[sensor_id], v_line_has3D[sensor_id]);
//        auto line = begin(v_segments2D[sensor_id]);
//        while( line != end(v_segments2D[sensor_id]) ) // Filter lines with low gradient
//        {
//            float gradient = float((*line)[3] - (*line)[1]) / ((*line)[2] - (*line)[0]);
//            if( fabs(gradient) < 0.8 )
//                v_segments2D[sensor_id].erase(line);
//            else
//                ++line;
//        }
//        cout << sensor_id << " filtered lines " << v_segments2D[sensor_id].size() << endl;

        cv::Mat image_lines;
        rgb[sensor_id].convertTo(image_lines, CV_8UC1, 1.0 / 2);
        for(auto line = begin(v_segments2D[sensor_id]); line != end(v_segments2D[sensor_id]); ++line)
        {
            //rgb[0].convertTo(image_lines, CV_8UC1, 1.0 / 2);
            cv::line(image_lines, cv::Point((*line)[0], (*line)[1]), cv::Point((*line)[2], (*line)[3]), cv::Scalar(255, 0, 255), 1);
            cv::imshow("lines", image_lines);
            //cv::waitKey(0);
        }
        cv::waitKey(0);
    }

    //==============================================================================
    //									Data association
    //==============================================================================
    cout << "Data association\n";
    line_corresp.clear();
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
    {
        lines.mm_corresp[sensor1] = map<unsigned, mrpt::math::CMatrixDouble>();
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            sensor_pair = {sensor1, sensor2};
            Matrix<T,4,4> pose_rel = Rt_estimated[sensor1].inverse() * Rt_estimated[sensor2];
            double thres_rot_cos = 1 - pow(sin(DEG2RAD(max(v_approx_rot[sensor_id1], v_approx_rot[sensor_id2]))),2);

//            cout << " sensor1 " << sensor1 << " " << v_segments2D[sensor1].size() << " sensor2 " << sensor2 << " " << v_segments2D[sensor2].size() << endl;
//            lines.mm_corresp[sensor1][sensor2] = mrpt::math::CMatrixDouble(0, 10);

            // Find line correspondences (in 2D by assuming a known rotation and zero translation)
            Vector2f offset2(rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(0,2), rgbd_intrinsics[sensor2].rightCamera.intrinsicParams(1,2));
            for(size_t i=0; i < v_segments2D[sensor1].size(); i++)
            {
                //cv::Vec4i &l1 = v_segments2D[sensor1][i];
//                line_match1 = v_segments2D[sensor1][i];

                for(size_t j=0; j < v_segments2D[sensor2].size(); j++)
                {
                    cout << "n1 " << n1.transpose() << " n2 " << n2.transpose() << " dot " << fabs((Rt_estimated[sensor1].block<3,3>(0,0)*n1) .dot (Rt_estimated[sensor2].block<3,3>(0,0)*n2)) << endl;
                    if( fabs((Rt_estimated[sensor1].block<3,3>(0,0)*v_segment_n[sensor1][i]) .dot (Rt_estimated[sensor2].block<3,3>(0,0)*v_segment_n[sensor2][j])) > thres_rot_cos )
                    {
                        revisar y anadir vidualizacion 2D
                        if(b_confirm_visually)
                        {
                            line_candidate = {i,j};
//                            plane_candidate_all[0] = planesIdx_i; plane_candidate_all[1] = planesIdx_j;
                            confirmed_corresp = 0;
                            while(confirmed_corresp == 0)
                                mrpt::system::sleep(10);
                        }
                        if(confirmed_corresp == -1)
                            continue;

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
                            cv::line(image_lines, cv::Point(v_segments2D[sensor1][i][1], height-v_segments2D[sensor1][i][0]), cv::Point(v_segments2D[sensor1][i][3], height-v_segments2D[sensor1][i][2]), cv::Scalar(255, 0, 255), 1);
                            cv::line(image_lines, cv::Point(width+20+v_segments2D[sensor2][j][1], height-v_segments2D[sensor2][j][0]), cv::Point(width+20+v_segments2D[sensor2][j][3], height-v_segments2D[sensor2][j][2]), cv::Scalar(255, 0, 255), 1);
                            cv::line(image_lines, cv::Point(width+20+p1(1), height-p1(0)), cv::Point(width+20+p2(1), height-p2(0)), cv::Scalar(0, 150, 0), 1);
                            cv::circle(image_lines, cv::Point(width+20+p(1), height-p(0)), 3, cv::Scalar(0, 0, 200), 3);
                            cv::imshow("Line match", image_lines);
                            cv::waitKey(0);
                        }
                    }
                }
            }
        }
    }

}
