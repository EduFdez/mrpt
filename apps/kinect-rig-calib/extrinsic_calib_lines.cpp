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
#include <mrpt/utils/TCamera.h>
#include <mrpt/vision/CFeatureLines.h>
#include <mrpt/pbmap/PbMap.h>
#include <opencv2/highgui/highgui.hpp>
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
//    M = registerMatchedPlanes(corresp);
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
//  Matrix<T,3,1> translation; translation << M(0,3), M(1,3), M(2,3);

//    ASSERT_( size(M,1)==4 && size(M,2)==4 )

//  const size_t N = size(planeCorresp,2);
//  out_inlierIndices.clear();
//  out_inlierIndices.reserve(100);
//  for (size_t i=0;i<N;i++)
//  {
//    const Matrix<T,3,1> n_i = Matrix<T,3,1>(planeCorresp(0,i), planeCorresp(1,i), planeCorresp(2,i));
//    const Matrix<T,3,1> n_ii = Rotation * Matrix<T,3,1>(planeCorresp(4,i), planeCorresp(5,i), planeCorresp(6,i));
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

//  const Matrix<T,3,1> n_1 = Matrix<T,3,1>(planeCorresp(0,useIndices[0]), planeCorresp(1,useIndices[0]), planeCorresp(2,useIndices[0]));
//  const Matrix<T,3,1> n_2 = Matrix<T,3,1>(planeCorresp(0,useIndices[1]), planeCorresp(1,useIndices[1]), planeCorresp(2,useIndices[1]));
//  const Matrix<T,3,1> n_3 = Matrix<T,3,1>(planeCorresp(0,useIndices[2]), planeCorresp(1,useIndices[2]), planeCorresp(2,useIndices[2]));
////cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

//  if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
//    return true;

//  return false;
//}

void ExtrinsicCalibLines::getSegments3D(const TCamera & cam, const pcl::PointCloud<PointT>::Ptr & cloud, const mrpt::pbmap::PbMap & pbmap, const vector<cv::Vec4i> & segments2D,
                                        vector<Matrix<T,3,1> > & segments_n, vector<Matrix<T,6,1> > & segments3D, vector<bool> & line_has3D)
{
    cout << "ExtrinsicCalibLines::getSegments3D... \n";
    size_t n_seg = segments2D.size();
    segments_n.resize(n_seg);
    segments3D.resize(n_seg);
    line_has3D.resize(n_seg);
    fill(line_has3D.begin(), line_has3D.end(), false);

    for(size_t i=0; i < n_seg; i++)
    {
        // Compute the normal vector to the plane containing the 2D line segment and the optical center
        int x1 = segments2D[i][0], y1 = segments2D[i][1], x2 = segments2D[i][2], y2 = segments2D[i][3];
        Matrix<T,3,1> v1( (x1-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y1-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        Matrix<T,3,1> v2( (x2-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y2-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        segments_n[i] = v1.cross(v2);
        // Force the normal vector to be around the 2nd quadrant of X-Y (i.e. positive X and negative Y)
        if(fabs(segments_n[i][0]) > fabs(segments_n[i][1]) ) // Larger X component
        {
            if(segments_n[i][0] < 0.f)
                segments_n[i] = -segments_n[i];
        }
        else
        {
            if(segments_n[i][1] > 0.f)
                segments_n[i] = -segments_n[i];
        }
        segments_n[i].normalize();

        // Check the depth of the end points
        int i1 = x1 + y1*cloud->width, i2 = x2 + y2*cloud->width;
        if(cloud->points[i1].z > 0.3f && cloud->points[i1].z < 10.f && cloud->points[i2].z > 0.3f && cloud->points[i2].z < 10.f)
        {
            line_has3D[i] = true;
            segments3D[i] << cloud->points[i1].x, cloud->points[i1].y, cloud->points[i1].z, cloud->points[i2].x, cloud->points[i2].y, cloud->points[i2].z;
            //cout << i << " Segment3D (from end-points depth) " << segments3D[i].transpose() << endl;
        }

        // Check the depth of the line segment

        // Check if the end points are within a panar segment in v_pbmaps
        for(size_t j=0; j < pbmap.vPlanes.size(); j++)
        {
            const mrpt::pbmap::Plane & plane = pbmap.vPlanes[j];
//            cout << "pt1 " << x1 << " " << y1 << " pt2 " << x2 << " " << y2 << endl;
//            cout << "End-points inside hull? " << plane.isInHull(i1,cloud->width) << endl;
//            cout << "End-points inside hull? " << plane.isInHull(i2,cloud->width) << endl;
//            mrpt::pbmap::PbMap::displayImagePbMap(cloud, cv::Mat(), pbmap, false, cv::Point(x1, y1));
//            mrpt::pbmap::PbMap::displayImagePbMap(cloud, cv::Mat(), pbmap, false, cv::Point(x2, y2));
//            if( plane.isInHull(i1,cloud->width) && plane.isInHull(i2,cloud->width) )
            if( pbmap.vPlanes.size() == 1 )
            {
//                // Compute the 3D line as the intersection of two lines (http://mathworld.wolfram.com/Plane-PlaneIntersection.html). Another option is (http://mathworld.wolfram.com/Line-PlaneIntersection.html)
//                Matrix<T,3,1> p(0, 0, 0);
//                p(2) = -plane.d / (plane.v3normal(2) - (plane.v3normal(0)*n1(2)/n1(0)));
//                p(0) = -n1(2)*p(2)/n1(0);

//                MatrixXf m = MatrixXf::Zero(2,3);
//                m.row(0) = n1;
//                m.row(1) = plane.v3normal;
//                FullPivLU<MatrixXf> lu(m);
//                MatrixXf m_null_space = lu.kernel(); m_null_space.normalize();
//                Map<Matrix<T,3,1>> l(m_null_space.data(),3);
//                  //vv_lines3D[sensor1][i].block<3,1>(0,0) = p + l/2;
//                  //vv_lines3D[sensor1][i].block<3,1>(3,0) = p - l/2;
//                Matrix<T,3,1> p1 = p + l/2, p2 = p - l/;
//                mrpt::math::TLine3D line(mrpt::math::TPoint3D(p1[0],p1[1],p1[2]), mrpt::math::TPoint3D(p2[0],p2[1],p2[2])); //= makeTLine3D(p + l/2, p - l/2);
//                vv_lines3D[sensor1].push_back(line);
//    //            line3D_match1 = line;
//                cout << "line3D " << vv_lines3D[sensor1][i] << endl;
//                //cout << "m_null_space \n" << m_null_space << endl;

                // Compute the 3D line as the intersection of a line and a plane. The equations are {n*p+d=0; p1+t(p2-p1)=p}, considering p1=(0,0,0) [the optical center] -> t = -d/n.dot(p2)
                double t1 = - plane.d / plane.v3normal.dot(v1.cast<float>());
                Matrix<T,3,1> p1 = t1*v1;
                double t2 = - plane.d / plane.v3normal.dot(v2.cast<float>());
                Matrix<T,3,1> p2 = t2*v2;
                line_has3D[i] = true;
                segments3D[i] << p1[0], p1[1], p1[2], p2[0], p2[1], p2[2];
                //cout << i << " Segment3D (from plane intersection) " << segments3D[i].transpose() << endl;
                break;
            }
        }
    }
}

void ExtrinsicCalibLines::getCorrespondences(const vector<cv::Mat> & rgb, const vector<pcl::PointCloud<PointT>::Ptr> & cloud)
{
    cout << "ExtrinsicCalibLines::getCorrespondences... \n";

    //						Extract line segments
    //==================================================================
    // #pragma omp parallel num_threads(num_sensors)
    for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
    {
        // sensor_id = omp_get_thread_num();
        cout << sensor_id << " cloud " << cloud[sensor_id]->height << "x" << cloud[sensor_id]->width << endl;
        CFeatureLines featLines;
        featLines.extractLines(rgb[sensor_id], vv_segments2D[sensor_id], min_pixels_line); //, true);
        cout << sensor_id << " lines " << vv_segments2D[sensor_id].size() << endl;
        ExtrinsicCalibLines::getSegments3D(intrinsics[sensor_id].rightCamera, cloud[sensor_id], v_pbmap[sensor_id], vv_segments2D[sensor_id], vv_segment_n[sensor_id], vv_segments3D[sensor_id], vv_line_has3D[sensor_id]);
        //ExtrinsicCalibLines::getSegments3D(rgb[sensor_id], cloud[sensor_id], intrinsics[sensor_id].rightCamera, vv_segments2D[sensor_id], vv_segment_n[sensor_id], vv_segments3D[sensor_id], vv_line_has3D[sensor_id]);
//        auto line = begin(vv_segments2D[sensor_id]);
//        while( line != end(vv_segments2D[sensor_id]) ) // Filter lines with low gradient
//        {
//            float gradient = float((*line)[3] - (*line)[1]) / ((*line)[2] - (*line)[0]);
//            if( fabs(gradient) < 0.8 )
//                vv_segments2D[sensor_id].erase(line);
//            else
//                ++line;
//        }
//        cout << sensor_id << " filtered lines " << vv_segments2D[sensor_id].size() << endl;
        cout << "1ExtrinsicCalibLines::getCorrespondences... \n";
    }

    //==============================================================================
    //									Data association
    //==============================================================================
    cout << "Data association\n";
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
    {
//        size_t height1 = rgb[sensor1].rows;
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            sensor_pair = {sensor1, sensor2};

//            size_t height2 = rgb[sensor2].rows;
//            cv::Mat rgb_concat;
//            if(b_confirm_visually) // This visualization is specialized for the rgbd360 sensor rig
//            {
//                cv::Mat rgb_concat(max(), 2*width+20, CV_8UC3, cv::Scalar(255,255,255));
//                cv::Mat img_transposed, img_rotated;
//                cv::transpose(rgb[sensor1], img_transposed);
//                cv::flip(img_transposed, img_rotated, 0);
//                cv::Mat tmp = rgb_concat(cv::Rect(0, 0, width, height));
//                img_rotated.copyTo(tmp);
//                cv::transpose(rgb[sensor2], img_transposed);
//                cv::flip(img_transposed, img_rotated, 0);
//                tmp = rgb_concat(cv::Rect(width+20, 0, width, height));
//                img_rotated.copyTo(tmp);

//                cv::Mat image_lines;
//                rgb_concat.convertTo(image_lines, CV_8UC1, 1.0 / 2);
//                cv::line(image_lines, cv::Point(vv_segments2D[sensor1][i][1], height1-vv_segments2D[sensor1][i][0]), cv::Point(vv_segments2D[sensor1][i][3], height1-vv_segments2D[sensor1][i][2]), cv::Scalar(255, 0, 255), 1);
//                cv::line(image_lines, cv::Point(width+20+vv_segments2D[sensor2][j][1], height2-vv_segments2D[sensor2][j][0]), cv::Point(width+20+vv_segments2D[sensor2][j][3], height2-vv_segments2D[sensor2][j][2]), cv::Scalar(255, 0, 255), 1);
//                cv::line(image_lines, cv::Point(width+20+p1(1), height-p1(0)), cv::Point(width+20+p2(1), height-p2(0)), cv::Scalar(0, 150, 0), 1);
//                cv::circle(image_lines, cv::Point(width+20+p(1), height-p(0)), 3, cv::Scalar(0, 0, 200), 3);
//                cv::imshow("Line match", image_lines);
//                cv::waitKey(0);
//            }

//            Matrix<T,4,4> pose_rel = Rt_estimated[sensor1].inverse() * Rt_estimated[sensor2];
            double thres_rot_cos = 1 - pow(sin(DEG2RAD(max(v_approx_rot[sensor1], v_approx_rot[sensor2]))),2);

//            cout << " sensor1 " << sensor1 << " " << vv_segments2D[sensor1].size() << " sensor2 " << sensor2 << " " << vv_segments2D[sensor2].size() << endl;

            // Find line correspondences (in 2D by assuming a known rotation and zero translation)
//            Vector2f offset2(intrinsics[sensor2].rightCamera.intrinsicParams(0,2), intrinsics[sensor2].rightCamera.intrinsicParams(1,2));
            for(size_t i=0; i < vv_segments2D[sensor1].size(); i++)
            {
                //cv::Vec4i &l1 = vv_segments2D[sensor1][i];
//                line_match1 = vv_segments2D[sensor1][i];
                if(b_confirm_visually)
                {
                    cv::Mat img_line1;
                    rgb[sensor1].copyTo(img_line1);
                    cv::line(img_line1, cv::Point(vv_segments2D[sensor1][i][0], vv_segments2D[sensor1][i][1]), cv::Point(vv_segments2D[sensor1][i][2], vv_segments2D[sensor1][i][3]), cv::Scalar(255, 0, 255), 1);
                    cv::circle(img_line1, cv::Point(vv_segments2D[sensor1][i][0], vv_segments2D[sensor1][i][1]), 3, cv::Scalar(0, 0, 200), 3);
                    cv::circle(img_line1, cv::Point(vv_segments2D[sensor1][i][2], vv_segments2D[sensor1][i][3]), 3, cv::Scalar(0, 0, 200), 3);
                    cv::putText(img_line1, string(to_string(i)+"/"+to_string(vv_segments2D[sensor1].size())), cv::Point(30,60), 0, 1.8, cv::Scalar(200,0,0), 3 );
                    cv::imshow("img_line1", img_line1); cv::moveWindow("img_line1", 20,60);
                }

                for(size_t j=0; j < vv_segments2D[sensor2].size(); j++)
                {
//                    // 3D constraint (when the 3D line parameters are observable)
//                    if( vv_line_has3D[sensor1][i] &&vv_line_has3D[sensor2][j] )
//                    {
//                        Matrix<T,3,1> v1 = vv_segments3D[sensor1][i].block<3,1>(3,0) - vv_segments3D[sensor1][i].block<3,1>(0,0); v1.normalize();
//                        Matrix<T,3,1> v2 = vv_segments3D[sensor2][j].block<3,1>(3,0) - vv_segments3D[sensor2][j].block<3,1>(0,0); v2.normalize();
//                        if( fabs((Rt_estimated[sensor1].block<3,3>(0,0)*v1) .dot (Rt_estimated[sensor2].block<3,3>(0,0)*v2)) > thres_rot_cos )

                    // 2D constraint (under the hypothesis of zero translation, valid when the optical centers are closeby wrt the observed scene)
                    if( fabs((Rt_estimated[sensor1].block<3,3>(0,0)*vv_segment_n[sensor1][i]) .dot (Rt_estimated[sensor2].block<3,3>(0,0)*vv_segment_n[sensor2][j])) > thres_rot_cos )
                    {
                        if(b_confirm_visually) // TODO: Add optional 3D visualization
                        {
//                            // Interactive 3D visualization
//                            b_wait_line_confirm = true;
//                            line_candidate = {i,j};
//                            confirm_corresp = 0;
//                            while(confirm_corresp == 0)
//                                mrpt::system::sleep(10);
                            cv::Mat img_line2;
                            rgb[sensor2].copyTo(img_line2);
                            cv::line(img_line2, cv::Point(vv_segments2D[sensor2][j][0], vv_segments2D[sensor2][j][1]), cv::Point(vv_segments2D[sensor2][j][2], vv_segments2D[sensor2][j][3]), cv::Scalar(255, 0, 255), 1);
                            cv::circle(img_line2, cv::Point(vv_segments2D[sensor2][j][0], vv_segments2D[sensor2][j][1]), 3, cv::Scalar(0, 0, 200), 3);
                            cv::circle(img_line2, cv::Point(vv_segments2D[sensor2][j][2], vv_segments2D[sensor2][j][3]), 3, cv::Scalar(0, 0, 200), 3);
                            cv::putText(img_line2, string(to_string(j)+"/"+to_string(vv_segments2D[sensor2].size())), cv::Point(30,60), 0, 1.8, cv::Scalar(200,0,0), 3 );
                            cv::imshow("img_line2", img_line2); cv::moveWindow("img_line2", 20,100+500);
                            char key = 'a';
                            while( key != 'k' && key != 'K' && key != 'l' && key != 'L' )
                                key = cv::waitKey(0);
                            if( key != 'k' && key != 'K' )
                                continue;

                            mmm_line_matches[sensor1][sensor2][i] = j;

                            // Store the parameters of the matched lines
                            size_t prevSize = lines.mm_corresp[sensor1][sensor2].getRowCount();
                            lines.mm_corresp[sensor1][sensor2].setSize(prevSize+1, lines.mm_corresp[sensor1][sensor2].getColCount());
                            lines.mm_corresp[sensor1][sensor2](prevSize, 0) = vv_segment_n[sensor1][i][0];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 1) = vv_segment_n[sensor1][i][1];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 2) = vv_segment_n[sensor1][i][2];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 3) = vv_segments3D[sensor1][i][0];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 4) = vv_segments3D[sensor1][i][1];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 5) = vv_segments3D[sensor1][i][2];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 6) = vv_segments3D[sensor1][i][3];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 7) = vv_segments3D[sensor1][i][4];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 8) = vv_segments3D[sensor1][i][5];

                            lines.mm_corresp[sensor1][sensor2](prevSize, 9) = vv_segment_n[sensor2][j][0];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 10) = vv_segment_n[sensor2][j][1];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 11) = vv_segment_n[sensor2][j][2];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 12) = vv_segments3D[sensor2][j][0];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 13) = vv_segments3D[sensor2][j][1];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 14) = vv_segments3D[sensor2][j][2];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 15) = vv_segments3D[sensor2][j][3];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 16) = vv_segments3D[sensor2][j][4];
                            lines.mm_corresp[sensor1][sensor2](prevSize, 17) = vv_segments3D[sensor2][j][5];
                        }
                    }
                }
            }
        }
    }
}


double ExtrinsicCalibLines::calcRotationErrorPair(const CMatrixDouble & correspondences, const Matrix<T,3,3> & Rot1, const Matrix<T,3,3> & Rot2, bool in_deg)
{
    double accum_error2 = 0.0;
    double accum_error_deg = 0.0;
    for(int i=0; i < correspondences.rows(); i++)
    {
//        T weight = 1.0;
        {
            Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
            Matrix<T,3,1> v2; v2 << correspondences(i,15)-correspondences(i,12), correspondences(i,16)-correspondences(i,13), correspondences(i,17)-correspondences(i,14);
            v2.normalize();
            double rot_error = (Rot1*n1).dot(Rot2*v2);
            accum_error2 += rot_error*rot_error; // weight *
            accum_error_deg += RAD2DEG(acos(rot_error));
        }
        {
            Matrix<T,3,1> v1; v1 << correspondences(i,8)-correspondences(i,5), correspondences(i,7)-correspondences(i,4), correspondences(i,6)-correspondences(i,3);
            Matrix<T,3,1> n2; n2 << correspondences(i,9), correspondences(i,10), correspondences(i,11);
            v1.normalize();
            double rot_error = (Rot1*v1).dot(Rot2*n2);
            accum_error2 += rot_error*rot_error; // weight *
            accum_error_deg += RAD2DEG(acos(rot_error));
        }
    }

    if(in_deg)
        return accum_error_deg;
    return accum_error2;
}

double ExtrinsicCalibLines::calcRotationError(const vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > > & Rt, bool in_deg)
{
    if( Rt.size() != num_sensors )
        throw runtime_error("ERROR ExtrinsicCalibLines::calcRotationError -> Rt.size() != num_sensors \n\n");

    double accum_error2 = 0.0;
//    double accum_error_deg = 0.0;
    size_t num_corresp = 0;
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            accum_error2 += calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], Rt[sensor1].block<3,3>(0,0), Rt[sensor2].block<3,3>(0,0), in_deg);
            num_corresp += lines.mm_corresp[sensor1][sensor2].rows();
        }

    //cout << "AvError deg " << accum_error2 / num_corresp << endl;
    //return accum_error2 / num_corresp;
    return accum_error2;
}

double ExtrinsicCalibLines::calcTranslationErrorPair(const mrpt::math::CMatrixDouble & correspondences, const Eigen::Matrix<T,4,4> & Rt1, const Eigen::Matrix<T,4,4> & Rt2, bool in_meters)
{
    T accum_error2 = 0.0;
    T accum_error_m = 0.0;
    Eigen::Matrix<T,4,4> Rt_1_2 = Rt1.inverse() * Rt2;
    Eigen::Matrix<T,4,4> Rt_2_1 = Rt_1_2.inverse();
    for(int i=0; i < correspondences.rows(); i++)
    {
        {
            Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
            Matrix<T,3,1> p2; p2 << correspondences(i,15)+correspondences(i,12), correspondences(i,16)+correspondences(i,13), correspondences(i,17)+correspondences(i,14); p2 /= 2;
            Matrix<T,3,1> p2_r1 = Rt_1_2.block<3,3>(0,0)*p2 + Rt_1_2.block<3,1>(0,3);
            p2_r1.normalize();
            double trans_error = n1.dot(p2_r1);
            accum_error2 += trans_error*trans_error; // weight *
            accum_error_m += fabs(trans_error);
        }
        {
            Matrix<T,3,1> p1; p1 << correspondences(i,8)+correspondences(i,5), correspondences(i,7)+correspondences(i,4), correspondences(i,6)+correspondences(i,3); p1 /= 2;
            Matrix<T,3,1> n2; n2 << correspondences(i,9), correspondences(i,10), correspondences(i,11);
            Matrix<T,3,1> p1_r2 = Rt_2_1.block<3,3>(0,0)*p1 + Rt_2_1.block<3,1>(0,3);
            p1_r2.normalize();
            double trans_error = n2.dot(p1_r2);
            accum_error2 += trans_error*trans_error; // weight *
            accum_error_m += fabs(trans_error);
        }
    }
    cout << "calcTranslationErrorPair " << accum_error2 << " AvError deg " << accum_error_m / correspondences.rows() << endl;

    if(in_meters)
        return accum_error_m;
    return accum_error2;
}

double ExtrinsicCalibLines::calcTranslationError(const vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > > & Rt, bool in_meters)
{
    if( Rt.size() != num_sensors )
        throw runtime_error("ERROR ExtrinsicCalibLines::calcTranslationError -> Rt.size() != num_sensors \n\n");

    double error = 0.0;
    size_t num_corresp = 0;
    for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
        for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
        {
            error += calcTranslationErrorPair(lines.mm_corresp[sensor1][sensor2], Rt[sensor1], Rt[sensor2], in_meters);
            num_corresp += lines.mm_corresp[sensor1][sensor2].rows();
        }

    //cout << "AvError deg " << accum_error2 / num_corresp << endl;
    if(in_meters)
        return error / num_corresp;
    return error;
}


Matrix<T,3,3> ExtrinsicCalibLines::ApproximateRotationZeroTrans(const size_t sensor1, const size_t sensor2, const bool weight_uncertainty)
{
    cout << "ExtrinsicCalibLines::ApproximateRotationZeroTrans... " << lines.mm_corresp[sensor1][sensor2].rows() << " correspondences\n";

    mm_covariance[sensor1][sensor2] = Matrix<T,3,3>::Zero();
//    Matrix<T,3,3> cov = Matrix<T,3,3>::Zero();
    // Matrix<T,3,3> FIM_rot = Matrix<T,3,3>::Zero();

    T accum_error2 = 0;
    CMatrixDouble & correspondences = lines.mm_corresp[sensor1][sensor2];
    for(int i=0; i < correspondences.rows(); i++)
    {
        //          T weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Matrix<T,3,1> n1(correspondences(i,0), correspondences(i,1), correspondences(i,2));
        Matrix<T,3,1> n2(correspondences(i,9), correspondences(i,10), correspondences(i,11));
        Matrix<T,3,1> n_1 = Rt_estimated[sensor1].block(0,0,3,3) * n1;
        Matrix<T,3,1> n_2 = Rt_estimated[sensor2].block(0,0,3,3) * n2;
        Matrix<T,3,1> rot_error = (n_1 - n_2);
        accum_error2 += rot_error.dot(rot_error);
        mm_covariance[sensor1][sensor2] += n2 * n1.transpose();

        // Add perpendicular vectors for each pair of correspondences
        for(int j=i+1; j < correspondences.rows(); j++)
        {
            Matrix<T,3,1> n1b(correspondences(j,0), correspondences(j,1), correspondences(j,2));
            Matrix<T,3,1> n2b(correspondences(j,9), correspondences(j,10), correspondences(j,11));
//            Matrix<T,3,1> n_1b = Rt_estimated[sensor1].block(0,0,3,3) * n1b;
//            Matrix<T,3,1> n_2b = Rt_estimated[sensor2].block(0,0,3,3) * n2b;
            mm_covariance[sensor1][sensor2] += (n2.cross(n2b)) * (n1.cross(n1b)).transpose();
        }
    }

    // Calculate Rotation
    // cout << "Solve rotation";
    Matrix<T,3,3> rotation = rotationFromNormals(mm_covariance[sensor1][sensor2], threshold_conditioning);

    cout << "accum_rot_error2 " << accum_error2 << " " << calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], Matrix<T,3,3>::Identity(), rotation) << endl;
    cout << "average error: "
              << calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], Rt_estimated[sensor1].block<3,3>(0,0), Rt_estimated[sensor2].block<3,3>(0,0), true) << " vs "
              << calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], Matrix<T,3,3>::Identity(), rotation, true) << " degrees\n";

    return rotation;
}


//Eigen::Matrix<T,3,1> ExtrinsicCalibLines::CalibrateTranslationPair(const size_t sensor1, const size_t sensor2, const bool weight_uncertainty)
//{
//    // Calibration system
//    Matrix<T,3,3> Hessian = Matrix<T,3,3>::Zero();
//    Matrix<T,3,1> gradient = Matrix<T,3,1>::Zero();
//    Matrix<T,3,1> translation = Rt_estimated[sensor1].block(0,0,3,3).transpose() * (Rt_estimated[sensor2].block(0,3,3,1) - Rt_estimated[sensor1].block(0,3,3,1));

//    T accum_error2 = 0;
//    CMatrixDouble & correspondences = lines.mm_corresp[sensor1][sensor2];
//    for(size_t i=0; i < correspondences.rows(); i++)
//    {
//        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
////        Matrix<T,3,1> n2; n2 << correspondences(i,4), correspondences(i,5), correspondences(i,6);
//        T trans_error = (correspondences(i,7) - correspondences(i,3));
////        T trans_error = correspondences(i,3) - correspondences(i,7) + n1.dot(translation);
//        accum_error2 += trans_error * trans_error;

////        if(weightedLS == 1 && correspondences.cols() == 18)
////        {
////            // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
////            //          T weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.rows();
////            T weight = correspondences(i,17);
////            Hessian += weight * (n1 * n1.transpose() );
////            gradient += weight * (n1 * trans_error);
////        }
////        else
//        {
//            Hessian += (n1 * n1.transpose() );
//            gradient += (n1 * trans_error);
//        }
//    }

//    //      cout << "Hessian \n" << Hessian << "\n HessianInv \n" << Hessian.inverse() << endl;
//    //      calcFisherInfMat();
//    JacobiSVD<Matrix<T,3,3> > svd(Hessian, ComputeFullU | ComputeFullV);
//    T conditioning = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();
//    cout << "conditioning " << conditioning << " FIM translation " << svd.singularValues().transpose() << endl;
//    if(conditioning < threshold_conditioning)
//        return translation;

//    translation = Hessian.inverse() * gradient;

//    return translation;
//}

/*! Get the rotation of each sensor in a multisensor setup. The first sensor (sensor_id=0) is taken as reference. */
//vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > >
void ExtrinsicCalibLines::CalibrateRotationManifold(const bool weight_uncertainty)
{
    cout << "ExtrinsicCalibLines::CalibrateRotationManifold...\n";
    const size_t n_DoF = 3*(num_sensors-1);
    T accum_error2;
    T accum_error_deg;

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
        accum_error_deg = 0.0;
        size_t numLineCorresp = 0;

        for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
        {
            for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
            {
                CMatrixDouble & correspondences = lines.mm_corresp[sensor1][sensor2];
                numLineCorresp += correspondences.rows();
                for(int i=0; i < correspondences.rows(); i++)
                {
                    // T weight = (inliers / correspondences(i,3)) / correspondences.rows()
                    // if(weight_uncertainty && correspondences.cols() == 10)
                    // else
                    {
                        Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
                        Matrix<T,3,1> v2; v2 << correspondences(i,15)-correspondences(i,12), correspondences(i,16)-correspondences(i,13), correspondences(i,17)-correspondences(i,14);
                        v2.normalize();
                        Matrix<T,3,1> n_1 = Rt_estimated[sensor1].block(0,0,3,3) * n1;
                        Matrix<T,3,1> v_2 = Rt_estimated[sensor2].block(0,0,3,3) * v2;
                        Matrix<T,1,3> jacobian_rot_1 = (n_1.cross(v_2)).transpose();
                        Matrix<T,1,3> jacobian_rot_2 = -jacobian_rot_1;
                        double rot_error = (n_1).dot(v_2);
                        accum_error2 += rot_error*rot_error; // weight *
                        accum_error_deg += RAD2DEG(acos(rot_error));

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
                    {
                        Matrix<T,3,1> v1; v1 << correspondences(i,8)-correspondences(i,5), correspondences(i,7)-correspondences(i,4), correspondences(i,6)-correspondences(i,3);
                        Matrix<T,3,1> n2; n2 << correspondences(i,9), correspondences(i,10), correspondences(i,11);
                        v1.normalize();
                        Matrix<T,3,1> v_1 = Rt_estimated[sensor1].block(0,0,3,3) * v1;
                        Matrix<T,3,1> n_2 = Rt_estimated[sensor2].block(0,0,3,3) * n2;
                        Matrix<T,1,3> jacobian_rot_1 = (v_1.cross(n_2)).transpose();
                        Matrix<T,1,3> jacobian_rot_2 = -jacobian_rot_1;
                        double rot_error = (v_1).dot(n_2);
                        accum_error2 += rot_error*rot_error; // weight *
                        accum_error_deg += RAD2DEG(acos(rot_error));

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
        accum_error_deg /= numLineCorresp;

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

    cout << "ErrorCalibRotation " << accum_error2 << " " << accum_error_deg << endl;

//    return Rt_estim;
}

void ExtrinsicCalibLines::CalibrateTranslation(const bool weight_uncertainty)
{
    cout << "ExtrinsicCalibLines::CalibrateTranslation...\n";
    const size_t n_DoF = 3*(num_sensors-1);
    T accum_error2, accum_error_m;
//    size_t numLineCorresp = 0;

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
        size_t numLineCorresp = 0;

        for(size_t sensor1=0; sensor1 < num_sensors; sensor1++)
        {
            for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
            {
                Eigen::Matrix<T,4,4> Rt_1_2 = Rt_estimated[sensor1].inverse() * Rt_estimated[sensor2];
                Eigen::Matrix<T,4,4> Rt_2_1 = Rt_1_2.inverse();
                CMatrixDouble & correspondences = lines.mm_corresp[sensor1][sensor2];
                numLineCorresp += correspondences.rows();
                for(int i=0; i < correspondences.rows(); i++)
                {
                    Matrix<T,3,1> n1; n1 << correspondences(i,0), correspondences(i,1), correspondences(i,2);
                    Matrix<T,3,1> n2; n2 << correspondences(i,9), correspondences(i,10), correspondences(i,11);
                    Matrix<T,3,1> p1; p1 << correspondences(i,8)+correspondences(i,5), correspondences(i,7)+correspondences(i,4), correspondences(i,6)+correspondences(i,3); p1 /= 2;
                    Matrix<T,3,1> p2; p2 << correspondences(i,15)+correspondences(i,12), correspondences(i,16)+correspondences(i,13), correspondences(i,17)+correspondences(i,14); p2 /= 2;
                    Matrix<T,3,1> n_1 = Rt_estimated[sensor1].block(0,0,3,3) * n1;
                    Matrix<T,3,1> n_2 = Rt_estimated[sensor2].block(0,0,3,3) * n2;
                    // T weight = (inliers / correspondences(i,3)) / correspondences.rows()
                    // if(weight_uncertainty && correspondences.cols() == 10)
                    // else
                    {
                        Matrix<T,3,1> p2_r1 = Rt_1_2.block<3,3>(0,0)*p2 + Rt_1_2.block<3,1>(0,3);
                        double p2_r1_norm = p2_r1.norm();
                        p2_r1.normalize();
                        double trans_error = n1.dot(p2_r1);
                        accum_error2 += trans_error*trans_error; // weight *
                        accum_error_m += fabs(trans_error);
                        Matrix<T,1,3> jacobian_trans_1 = n_1.transpose() * (-Rt_estimated[sensor1].block(0,0,3,3).transpose() - p2_r1*p2_r1.transpose()) / p2_r1_norm;
                        Matrix<T,1,3> jacobian_trans_2 = n_2.transpose() * (Rt_estimated[sensor2].block(0,0,3,3).transpose() - p2_r1*p2_r1.transpose()) / p2_r1_norm;

                        if(sensor1 != 0) // The pose of the first camera is fixed
                        {
                            Hessian.block(3*(sensor1-1), 3*(sensor1-1), 3, 3) += jacobian_trans_1.transpose() * jacobian_trans_1;
                            gradient.block(3*(sensor1-1),0,3,1) += jacobian_trans_1.transpose() * trans_error;
                            // Cross term
                            Hessian.block(3*(sensor1-1), 3*(sensor2-1), 3, 3) += jacobian_trans_1.transpose() * jacobian_trans_2;
                        }
                        Hessian.block(3*(sensor2-1), 3*(sensor2-1), 3, 3) += jacobian_trans_2.transpose() * jacobian_trans_2;
                        gradient.block(3*(sensor2-1),0,3,1) += jacobian_trans_2.transpose() * trans_error;
                    }
                    {
                        Matrix<T,3,1> p1_r2 = Rt_2_1.block<3,3>(0,0)*p1 + Rt_2_1.block<3,1>(0,3);
                        double p1_r2_norm = p1_r2.norm();
                        p1_r2.normalize();
                        double trans_error = n2.dot(p1_r2);
                        accum_error2 += trans_error*trans_error; // weight *
                        accum_error_m += fabs(trans_error);
                        Matrix<T,1,3> jacobian_trans_1 = n_2.transpose() * (-Rt_estimated[sensor2].block(0,0,3,3).transpose() - p1_r2*p1_r2.transpose()) / p1_r2_norm;
                        Matrix<T,1,3> jacobian_trans_2 = n_1.transpose() * (Rt_estimated[sensor1].block(0,0,3,3).transpose() - p1_r2*p1_r2.transpose()) / p1_r2_norm;

                        if(sensor1 != 0) // The pose of the first camera is fixed
                        {
                            Hessian.block(3*(sensor1-1), 3*(sensor1-1), 3, 3) += jacobian_trans_1.transpose() * jacobian_trans_1;
                            gradient.block(3*(sensor1-1),0,3,1) += jacobian_trans_1.transpose() * trans_error;
                            // Cross term
                            Hessian.block(3*(sensor1-1), 3*(sensor2-1), 3, 3) += jacobian_trans_1.transpose() * jacobian_trans_2;
                        }
                        Hessian.block(3*(sensor2-1), 3*(sensor2-1), 3, 3) += jacobian_trans_2.transpose() * jacobian_trans_2;
                        gradient.block(3*(sensor2-1),0,3,1) += jacobian_trans_2.transpose() * trans_error;
                    }
                }
                if(sensor1 != 0) // Fill the lower left triangle with the corresponding cross terms
                    Hessian.block(3*(sensor2-1), 3*(sensor1-1), 3, 3) = Hessian.block(3*(sensor1-1), (sensor2-1), 3, 3).transpose();
            }
        }
        //    av_error /= numLineCorresp;

        Eigen::JacobiSVD<Matrix<T,Dynamic,Dynamic> > svd(Hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        T conditioning = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();
        cout << "conditioning " << conditioning << endl;
        if(conditioning < threshold_conditioning)
            return; // Rt_estimated;

        // Solve system
        update_vector = -Hessian.inverse() * gradient;
        cout << "update_vector " << update_vector.transpose() << endl;

        // Update translation of the poses
        vector<Matrix<T,4,4>, aligned_allocator<Matrix<T,4,4> > > Rt_estim = Rt_estimated; // Load the initial extrinsic calibration from the device'
        for(size_t sensor_id = 1; sensor_id < num_sensors; sensor_id++)
        {
            Rt_estim[sensor_id].block(0,3,3,1) += update_vector.block(3*sensor_id-3,0,3,1);
            //      cout << "old translation" << sensor_id << "\n" << Rt_estimated[sensor_id].block(0,3,3,1) << endl;
            //      cout << "new translation" << Rt_estim[sensor_id].block(0,3,3,1) << endl;
        }

        cout << " accum_error2 " << accum_error2 << endl;
        accum_error2 = calcTranslationError(Rt_estimated);
        T new_accum_error2 = calcTranslationError(Rt_estim);
        cout << "New rotation error " << new_accum_error2 << " previous " << accum_error2 << endl;

        // Assign new rotations
        if(new_accum_error2 < accum_error2)
        {
//            Rt_estimated = Rt_estim;
            for(size_t sensor_id = 1; sensor_id < num_sensors; sensor_id++)
                Rt_estimated[sensor_id].block(0,3,3,1) = Rt_estim[sensor_id].block(0,3,3,1);
        }

        increment = update_vector .dot (update_vector);
        diff_error = accum_error2 - new_accum_error2;
        ++it;
        //      cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
    }

    cout << "ErrorTranslation " << accum_error2 << " " << accum_error2 << endl;
}

void ExtrinsicCalibLines::Calibrate()
{
    CalibrateRotationManifold();
    CalibrateTranslation();
    //      cout << "Rt_estimated\n" << Rt_estimated << endl;

    cout << "Errors " << calcRotationError(Rt_estimated) << " av deg " << calcRotationError(Rt_estimated,true) << " av trans " << calcTranslationError(Rt_estimated,true) << endl;
}
