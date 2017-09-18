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
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#define VISUALIZE_SENSOR_DATA 0
#define SHOW_IMAGES 0

using namespace std;
using namespace Eigen;
using namespace mrpt::vision;
using namespace mrpt::math;
using namespace mrpt::utils;

// Obtain the rigid transformation from 3 matched lines
CMatrixDouble registerMatchedLineNormals( const CMatrixDouble & matched_lines )
{
    ASSERT_(size(matched_lines,1) == 18 && size(matched_lines,2) >= 2);

    //Calculate rotation
    Matrix3d normalCovariances = Matrix3d::Zero();
    normalCovariances(0,0) = 1;
    for(unsigned i=0; i<size(matched_lines,2); i++)
    {
        Vector3d n1(matched_lines(0,i), matched_lines(1,i), matched_lines(2,i));
        Vector3d n2(matched_lines(3,i), matched_lines(4,i), matched_lines(5,i));
//        Vector3d n2 = Vector3d(matched_lines(9,i), matched_lines(10,i), matched_lines(11,i));
        normalCovariances += n2 * n1.transpose();
        //    normalCovariances += matched_lines.block(i,0,1,3) * matched_lines.block(i,4,1,3).transpose();
    }
    Matrix3d Rotation;
    double cond = ExtrinsicCalib::rotationFromNormals(normalCovariances, Rotation);
//    if(cond < 0.01f){ //cout << "registerMatchedLineNormals::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
//        Rotation = Matrix3d::Identity();
//    }

    //  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
    //  Eigen::Matrix4f rigidTransf;
    CMatrixDouble33 rigidTransf(Rotation);
//    rigidTransf.block(0,0,3,3) = Rotation;
//    rigidTransf.block(0,3,3,1) = translation;
//    rigidTransf.row(3) << 0,0,0,1;
    //  rigidTransf(0,0) = Rotation(0,0);
    //  rigidTransf(0,1) = Rotation(0,1);
    //  rigidTransf(0,2) = Rotation(0,2);
    //  rigidTransf(1,0) = Rotation(1,0);
    //  rigidTransf(1,1) = Rotation(1,1);
    //  rigidTransf(1,2) = Rotation(1,2);
    //  rigidTransf(2,0) = Rotation(2,0);
    //  rigidTransf(2,1) = Rotation(2,1);
    //  rigidTransf(2,2) = Rotation(2,2);
    //  rigidTransf(0,3) = translation(0);
    //  rigidTransf(1,3) = translation(1);
    //  rigidTransf(2,3) = translation(2);
    //  rigidTransf(3,0) = 0;
    //  rigidTransf(3,1) = 0;
    //  rigidTransf(3,2) = 0;
    //  rigidTransf(3,3) = 1;

    return rigidTransf;
}

// Ransac functions to detect outliers in the line matching
void ransacLineRot_fit( const CMatrixDouble         & lineCorresp,
                        const mrpt::vector_size_t   & useIndices,
                        vector< CMatrixDouble >     & fitModels )
                        //vector< Matrix4f > &fitModels )
{
    ASSERT_(useIndices.size()==2);

    try
    {
        CMatrixDouble corresp(18,2);

        //  cout << "Size lineCorresp: " << endl;
        //  cout << "useIndices " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << endl;
        for(unsigned i=0; i<2; i++)
            corresp.col(i) = lineCorresp.col(useIndices[i]);

        fitModels.resize(1);
        //    Matrix4f &M = fitModels[0];
        CMatrixDouble & M = fitModels[0];
        M = registerMatchedLineNormals(corresp);
    }
    catch(exception &)
    {
        fitModels.clear();
        return;
    }
}

void ransacLineRot_angle(const CMatrixDouble        & lineCorresp,
                        const vector<CMatrixDouble> & testModels,
                        const double                th_min_cos_angle,
                        unsigned int                & out_bestModelIndex,
                        mrpt::vector_size_t         & out_inlierIndices )
{
    ASSERT_( testModels.size()==1 )
    const CMatrixDouble & M = testModels[0];
    ASSERT_( size(M,1)==3 && size(M,2)==3 )

    out_bestModelIndex = 0;
    Matrix3d Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);

    const size_t N = size(lineCorresp,2);
    out_inlierIndices.clear();
    out_inlierIndices.reserve(100);
    for (size_t i=0; i<N; i++)
    {
        const Matrix<T,3,1> n1 = Matrix<T,3,1>(lineCorresp(0,i), lineCorresp(1,i), lineCorresp(2,i));
        const Matrix<T,3,1> R_n2 = Rotation * Matrix<T,3,1>(lineCorresp(3,i), lineCorresp(4,i), lineCorresp(5,i));
        const float cos_angle_error = n1 .dot (R_n2);
        if (cos_angle_error > th_min_cos_angle)
            out_inlierIndices.push_back(i);
    }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
bool ransacLineRot_degenerate ( const CMatrixDouble         & lineCorresp,
                                const mrpt::vector_size_t   & useIndices )
{
    ASSERT_( useIndices.size()==2 )

    const Matrix<T,3,1> n1_i = Matrix<T,3,1>(lineCorresp(0,useIndices[0]), lineCorresp(1,useIndices[0]), lineCorresp(2,useIndices[0]));
    const Matrix<T,3,1> n2_i = Matrix<T,3,1>(lineCorresp(0,useIndices[1]), lineCorresp(1,useIndices[1]), lineCorresp(2,useIndices[1]));
    T cos_i = n1_i.dot(n2_i);
    //cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;
    if( fabs(cos_i) > 0.98 ) // acos(0.98) = 11.48 deg
        return true;

    const Matrix<T,3,1> n1_j = Matrix<T,3,1>(lineCorresp(3,useIndices[0]), lineCorresp(4,useIndices[0]), lineCorresp(5,useIndices[0]));
    const Matrix<T,3,1> n2_j = Matrix<T,3,1>(lineCorresp(3,useIndices[1]), lineCorresp(4,useIndices[1]), lineCorresp(5,useIndices[1]));
    T cos_j = n1_j.dot(n2_j);
    if( fabs(acos(cos_i) - acos(cos_j)) > 0.0174533 ) // Check if the rotation is consistent
        return true;

    return false;
}


//map<size_t,size_t> ExtrinsicCalibLines::matchNormalVectorsRANSAC(const vector<Matrix<T,3,1> > & n_cam1, const vector<Matrix<T,3,1> > & n_cam2, Matrix<T,3,3> & rotation, T & conditioning, const T th_angle)
//{

//    if( n_cam1.size() < 2 || n_cam2.size() < 2 )
//    {
//        cout << "Insuficient lines " << n_cam1.size() << " " << n_cam2.size() << endl;
//        return map<size_t,size_t>();
//    }

//    // Fill matrix with all possible line correspondences
//    CMatrixDouble lineCorresp(6, matched_lines.size());
//    unsigned col = 0;
//    for(map<unsigned, unsigned>::iterator it = matched_lines.begin(); it != matched_lines.end(); it++, col++)
//    {
//        lineCorresp(0,col) = PBMSource.vPlanes[it->first].v3normal(0);
//        lineCorresp(1,col) = PBMSource.vPlanes[it->first].v3normal(1);
//        lineCorresp(2,col) = PBMSource.vPlanes[it->first].v3normal(2);
//        lineCorresp(4,col) = PBMTarget.vPlanes[it->second].v3normal(0);
//        lineCorresp(5,col) = PBMTarget.vPlanes[it->second].v3normal(1);
//        lineCorresp(6,col) = PBMTarget.vPlanes[it->second].v3normal(2);
//    }
//    //  cout << "Size " << matched_lines.size() << " " << size(1) << endl;

//    mrpt::vector_size_t inliers;
//    CMatrixDouble best_model;

//    math::RANSAC ransac_executer;
//    ransac_executer.execute(lineCorresp,
//                            ransacPlaneAlignment_fit,
//                            ransac3Dline_distance,
//                            ransac3Dline_degenerate,
//                            0.2,
//                            3,  // Minimum set of points
//                            inliers,
//                            best_model,
//                            true,   // Verbose
//                            0.99999
//                            );

//    //  cout << "Computation time: " << tictac.Tac()*1000.0/TIMES << " ms" << endl;

//    cout << "Size lineCorresp: " << size(lineCorresp,2) << endl;
//    cout << "RANSAC finished: " << inliers.size() << " inliers: " << inliers << " . \nBest model: \n" << best_model << endl;
//    //        cout << "Best inliers: " << best_inliers << endl;

//    Eigen::Matrix4f rigidTransf; rigidTransf << best_model(0,0), best_model(0,1), best_model(0,2), best_model(0,3), best_model(1,0), best_model(1,1), best_model(1,2), best_model(1,3), best_model(2,0), best_model(2,1), best_model(2,2), best_model(2,3), 0, 0, 0, 1;

//    //  return best_model;
//    return rigidTransf;
//}

map<size_t,size_t> ExtrinsicCalibLines::matchNormalVectors(const vector<Matrix<T,3,1> > & n_cam1, const vector<Matrix<T,3,1> > & n_cam2, Matrix<T,3,3> & rotation, T & conditioning, const T th_angle)
{
    // Find line correspondences (in 2D by assuming a known rotation and zero translation)
    // Select a pair of lines (normal vector of projective line) and verify rotation
    CTicTac clock; clock.Tic(); //Clock to measure the runtime
    map<size_t,size_t> best_matches;
    vector<map<size_t,size_t> > discarded_matches; // with at least 3 matches
    T best_error = 10e6;
    const T min_angle_pair = DEG2RAD(15);
    const T min_angle_diff = DEG2RAD(th_angle);
    const T min_angle_diff_cos = cos(min_angle_diff);
    //const T min_conditioning = 0.01;
    // Exhaustive search
    for(size_t i1=0; i1 < n_cam1.size(); i1++)
    {
        for(size_t i2=i1+1; i2 < n_cam1.size(); i2++)
        {
            T angle_i = acos( n_cam1[i1].dot(n_cam1[i2]) );
            if( angle_i < min_angle_pair )
                continue;
            for(size_t j1=0; j1 < n_cam2.size(); j1++)
            {
                for(size_t j2=0; j2 < n_cam2.size(); j2++)
                {
                    if( j1 == j2 )
                        continue;
                    bool already_checked = false;
                    for(size_t k=0; k < discarded_matches.size(); k++)
                        if(discarded_matches[k].count(i1) && discarded_matches[k][i1] == j1 && discarded_matches[k].count(i2) && discarded_matches[k][i2] == j2)
                        {
                            already_checked = true;
                            break;
                        }
                    if( already_checked )
                        continue;
                    if( fabs( acos( n_cam2[j1].dot(n_cam2[j2]) ) - angle_i ) > min_angle_diff )
                        continue;

                    // Compute rotation
                    Matrix<T,3,3> rot;
                    Matrix<T,3,3> cov = n_cam2[j1]*n_cam1[i1].transpose() + n_cam2[j2]*n_cam1[i2].transpose() + (n_cam2[j1].cross(n_cam2[j2]))*(n_cam1[i1].cross(n_cam1[i2])).transpose();
                    T cond = rotationFromNormals(cov, rot);
                    //if(cond < min_conditioning){ //cout << "ExtrinsicCalibLines::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
                    //    continue; }
                    //cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << n_cam1[i1].dot(rot*n_cam2[j1]) << " min_cos " << min_angle_diff_cos << " conditioning " << cond << endl;
                    //if( n_cam1[i1].dot(rot*n_cam2[j1]) < min_angle_diff_cos ) // Check if the rotation is consistent
                    //    continue;

                    //cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << RAD2DEG(acos(n_cam1[i1].dot(rot*n_cam2[j1]))) << " min_cos " << min_angle_diff_cos << " conditioning " << cond << endl;

                    // Check how many lines are consistent with this rotation
                    map<size_t,size_t> matches;
                    matches[i1] = j1;
                    matches[i2] = j2;
                    set<size_t> matches2; matches2.insert(j1); matches2.insert(j2);
                    for(size_t i=0; i < n_cam1.size(); i++)
                    {
                        if( i==i1 || i==i2 )
                            continue;
                        for(size_t j=0; j < n_cam2.size(); j++)
                        {
                            if( matches2.count(j) )
                                continue;
                            //cout << "error " << i << " vs " << j << " : " << n_cam1[i].dot(rot*n_cam2[j]) << " min " << min_angle_diff << endl;
                            if( n_cam1[i].dot(rot*n_cam2[j]) > min_angle_diff_cos )
                            {
                                cov += n_cam2[j]*n_cam1[i].transpose();
                                for(map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
                                    cov += (n_cam2[j].cross(n_cam2[it->second]))*(n_cam1[i].cross(n_cam1[it->first])).transpose();
                                matches[i] = j;
                                matches2.insert(j);
                            }
                        }
                    }

                    // Compute the rotation from the inliers
                    T error = 0;
                    cond = rotationFromNormals(cov, rot);
                    for(map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++){ //cout << "match " << it->first << " - " << it->second << " error " << RAD2DEG(acos(n_cam1[it->first] .dot (rot*n_cam2[it->second]))) << endl;
                        error += acos(n_cam1[it->first] .dot (rot*n_cam2[it->second]));}

                    //cout << "Num corresp " << matches.size() << " conditioning " << cond << " error " << error << " average error " << RAD2DEG(error/matches.size()) << " deg.\n";

                    if(best_matches.size() < matches.size() && matches.size() > 2)
                    {
                        //if(matches.size() > 2)
                            discarded_matches.push_back(matches);

                        best_matches = matches;
                        best_error = error;
                        rotation = rot;
                        conditioning = cond;
                    }
                }
            }
        }
    }
    cout << " ...matchNormalVectors took " << 1000*clock.Tac() << " ms " << best_matches.size() << " matches, best_error " << RAD2DEG(best_error) << " conditioning " << conditioning << endl << rotation << endl;
//    for(map<size_t,size_t>::iterator it=best_matches.begin(); it != best_matches.end(); it++)
//        cout << "match " << it->first << " - " << it->second << endl;

    return best_matches;
}

void ExtrinsicCalibLines::getProjPlaneNormals(const TCamera & cam, const vector<cv::Vec4f> & segments2D, vector<Matrix<T,3,1> > & segments_n)
{
    cout << "ExtrinsicCalibLines::getSegments3D... \n";
    size_t n_seg = segments2D.size();
    segments_n.resize(n_seg);
    for(size_t i=0; i < n_seg; i++)
    {
        // Compute the normal vector to the line containing the 2D line segment and the optical center (the cross product of the end-point vectors in the 3D image line)
        int x1 = segments2D[i][0], y1 = segments2D[i][1], x2 = segments2D[i][2], y2 = segments2D[i][3];
        Matrix<T,3,1> e1( (x1-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y1-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        Matrix<T,3,1> e2( (x2-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y2-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        segments_n[i] = e1.cross(e2);
        segments_n[i].normalize();
//        // Force the normal vector to be around the 2nd quadrant of X-Y (i.e. positive X and negative Y)
//        if(fabs(segments_n[i][0]) > fabs(segments_n[i][1]) ) // Larger X component
//        {
//            if(segments_n[i][0] < 0.f)
//                segments_n[i] = -segments_n[i];
//        }
//        else
//        {
//            if(segments_n[i][1] > 0.f)
//                segments_n[i] = -segments_n[i];
//        }
    }
}

void ExtrinsicCalibLines::getSegments3D(const TCamera & cam, const pcl::PointCloud<PointT>::Ptr & cloud, const mrpt::pbmap::PbMap & pbmap, const vector<cv::Vec4f> & segments2D,
                                        vector<Matrix<T,3,1> > & segments_n, vector<Matrix<T,6,1> > & segments3D, vector<bool> & line_has3D)
{
    //cout << "ExtrinsicCalibLines::getSegments3D... \n";
    size_t n_seg = segments2D.size();
    segments_n.resize(n_seg);
    segments3D.resize(n_seg);
    line_has3D.resize(n_seg);
    fill(line_has3D.begin(), line_has3D.end(), false);

    for(size_t i=0; i < n_seg; i++)
    {
        // Compute the normal vector to the line containing the 2D line segment and the optical center (the cross product of the end-point vectors in the 3D image line)
        T x1 = segments2D[i][0], y1 = segments2D[i][1], x2 = segments2D[i][2], y2 = segments2D[i][3];
        Matrix<T,3,1> e1( (x1-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y1-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        Matrix<T,3,1> e2( (x2-cam.intrinsicParams(0,2))/cam.intrinsicParams(0,0), (y2-cam.intrinsicParams(1,2))/cam.intrinsicParams(1,1), 1);
        segments_n[i] = e1.cross(e2);
        segments_n[i].normalize();
//        // Force the normal vector to be around the 2nd quadrant of X-Y (i.e. positive X and negative Y)
//        if(fabs(segments_n[i][0]) > fabs(segments_n[i][1]) ) // Larger X component
//        {
//            if(segments_n[i][0] < 0.f)
//                segments_n[i] = -segments_n[i];
//        }
//        else
//        {
//            if(segments_n[i][1] > 0.f)
//                segments_n[i] = -segments_n[i];
//        }

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
            const mrpt::pbmap::Plane & line = pbmap.vPlanes[j];
//            cout << "pt1 " << x1 << " " << y1 << " pt2 " << x2 << " " << y2 << endl;
//            cout << "End-points inside hull? " << line.isInHull(i1,cloud->width) << endl;
//            cout << "End-points inside hull? " << line.isInHull(i2,cloud->width) << endl;
//            mrpt::pbmap::PbMap::displayImagePbMap(cloud, cv::Mat(), pbmap, false, cv::Point(x1, y1));
//            mrpt::pbmap::PbMap::displayImagePbMap(cloud, cv::Mat(), pbmap, false, cv::Point(x2, y2));
//            if( line.isInHull(i1,cloud->width) && line.isInHull(i2,cloud->width) )
            if( pbmap.vPlanes.size() == 1 )
            {
//                // Compute the 3D line as the intersection of two lines (http://mathworld.wolfram.com/Plane-PlaneIntersection.html). Another option is (http://mathworld.wolfram.com/Line-PlaneIntersection.html)
//                Matrix<T,3,1> p(0, 0, 0);
//                p(2) = -line.d / (line.v3normal(2) - (line.v3normal(0)*n1(2)/n1(0)));
//                p(0) = -n1(2)*p(2)/n1(0);

//                MatrixXf m = MatrixXf::Zero(2,3);
//                m.row(0) = n1;
//                m.row(1) = line.v3normal;
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

                // Compute the 3D line as the intersection of a line and a line. The equations are {n*p+d=0; p1+t(p2-p1)=p}, considering p1=(0,0,0) [the optical center] -> t = -d/n.dot(p2)
                double t1 = - line.d / line.v3normal.dot(e1.cast<float>());
                Matrix<T,3,1> p1 = t1*e1;
                double t2 = - line.d / line.v3normal.dot(e2.cast<float>());
                Matrix<T,3,1> p2 = t2*e2;
                line_has3D[i] = true;
                segments3D[i] << p1[0], p1[1], p1[2], p2[0], p2[1], p2[2];
                //cout << i << " Segment3D (from line intersection) " << segments3D[i].transpose() << endl;
                break;
            }
        }
    }
}

//double ExtrinsicCalibLines::calcRotationErrorPair(const CMatrixDouble & correspondences, const Matrix<T,3,3> & Rot, bool in_deg)

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
        featLines.extractLines(rgb[sensor_id], vv_segments2D[sensor_id], line_extraction, min_pixels_line, max_lines); //, true);
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
            mmm_line_matches[sensor1][sensor2] = matchNormalVectors(vv_segment_n[sensor1],vv_segment_n[sensor2],min_angle_diff);
            addMatches(sensor1, sensor2, mmm_line_matches[sensor1][sensor2]);

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


//            //cout << " sensor1 " << sensor1 << " " << vv_segments2D[sensor1].size() << " sensor2 " << sensor2 << " " << vv_segments2D[sensor2].size() << endl;
//            double thres_rot_cos = 1 - pow(sin(DEG2RAD(max(v_approx_rot[sensor1], v_approx_rot[sensor2]))),2); // Threshold to prune line candidates based on an initial rotation
//            // Find line correspondences (in 2D by assuming a known rotation and zero translation)
//            for(size_t i=0; i < vv_segments2D[sensor1].size(); i++)
//            {
//                //cv::Vec4f &l1 = vv_segments2D[sensor1][i];
////                line_match1 = vv_segments2D[sensor1][i];
//                if(b_confirm_visually)
//                {
//                    cv::Mat img_line1;
//                    rgb[sensor1].copyTo(img_line1);
//                    cv::line(img_line1, cv::Point(vv_segments2D[sensor1][i][0], vv_segments2D[sensor1][i][1]), cv::Point(vv_segments2D[sensor1][i][2], vv_segments2D[sensor1][i][3]), cv::Scalar(255, 0, 255), 1);
//                    cv::circle(img_line1, cv::Point(vv_segments2D[sensor1][i][0], vv_segments2D[sensor1][i][1]), 3, cv::Scalar(0, 0, 200), 3);
//                    cv::circle(img_line1, cv::Point(vv_segments2D[sensor1][i][2], vv_segments2D[sensor1][i][3]), 3, cv::Scalar(0, 0, 200), 3);
//                    cv::putText(img_line1, string(to_string(i)+"/"+to_string(vv_segments2D[sensor1].size())), cv::Point(30,60), 0, 1.8, cv::Scalar(200,0,0), 3 );
//                    cv::imshow("img_line1", img_line1); cv::moveWindow("img_line1", 20,60);
//                }

//                for(size_t j=0; j < vv_segments2D[sensor2].size(); j++)
//                {
////                    // 3D constraint (when the 3D line parameters are observable)
////                    if( vv_line_has3D[sensor1][i] &&vv_line_has3D[sensor2][j] )
////                    {
////                        Matrix<T,3,1> v1 = vv_segments3D[sensor1][i].block<3,1>(3,0) - vv_segments3D[sensor1][i].block<3,1>(0,0); v1.normalize();
////                        Matrix<T,3,1> v2 = vv_segments3D[sensor2][j].block<3,1>(3,0) - vv_segments3D[sensor2][j].block<3,1>(0,0); v2.normalize();
////                        if( fabs((Rt_estimated[sensor1].block<3,3>(0,0)*v1) .dot (Rt_estimated[sensor2].block<3,3>(0,0)*v2)) > thres_rot_cos )

//                    // 2D constraint (under the hypothesis of zero translation, valid when the optical centers are closeby wrt the observed scene)
//                    if( fabs((Rt_estimated[sensor1].block<3,3>(0,0)*vv_segment_n[sensor1][i]) .dot (Rt_estimated[sensor2].block<3,3>(0,0)*vv_segment_n[sensor2][j])) > thres_rot_cos )
//                    {
//                        if(b_confirm_visually) // TODO: Add optional 3D visualization
//                        {
////                            // Interactive 3D visualization
////                            b_wait_line_confirm = true;
////                            line_candidate = {i,j};
////                            confirm_corresp = 0;
////                            while(confirm_corresp == 0)
////                                mrpt::system::sleep(10);
//                            cv::Mat img_line2;
//                            rgb[sensor2].copyTo(img_line2);
//                            cv::line(img_line2, cv::Point(vv_segments2D[sensor2][j][0], vv_segments2D[sensor2][j][1]), cv::Point(vv_segments2D[sensor2][j][2], vv_segments2D[sensor2][j][3]), cv::Scalar(255, 0, 255), 1);
//                            cv::circle(img_line2, cv::Point(vv_segments2D[sensor2][j][0], vv_segments2D[sensor2][j][1]), 3, cv::Scalar(0, 0, 200), 3);
//                            cv::circle(img_line2, cv::Point(vv_segments2D[sensor2][j][2], vv_segments2D[sensor2][j][3]), 3, cv::Scalar(0, 0, 200), 3);
//                            cv::putText(img_line2, string(to_string(j)+"/"+to_string(vv_segments2D[sensor2].size())), cv::Point(30,60), 0, 1.8, cv::Scalar(200,0,0), 3 );
//                            cv::imshow("img_line2", img_line2); cv::moveWindow("img_line2", 20,100+500);
//                            char key = 'a';
//                            while( key != 'k' && key != 'K' && key != 'l' && key != 'L' )
//                                key = cv::waitKey(0);
//                            if( key != 'k' && key != 'K' )
//                                continue;

//                            cout << "Add line match " << i << " " << j << endl;
//                            mmm_line_matches[sensor1][sensor2][i] = j;
//                        }
//                    }
//                }
//            }
//            addMatches(sensor1, sensor2, mmm_line_matches[sensor1][sensor2]);
        }
    }
}

void ExtrinsicCalibLines::addMatches(const size_t sensor1, const size_t sensor2, map<size_t, size_t> & matches)
{
    // Store the parameters of the matched lines
    size_t prevSize = lines.mm_corresp[sensor1][sensor2].getRowCount();
    lines.mm_corresp[sensor1][sensor2].setSize(prevSize+matches.size(), lines.mm_corresp[sensor1][sensor2].getColCount());
    for(map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
    {
        lines.mm_corresp[sensor1][sensor2](prevSize, 0) = vv_segment_n[sensor1][it->first][0];
        lines.mm_corresp[sensor1][sensor2](prevSize, 1) = vv_segment_n[sensor1][it->first][1];
        lines.mm_corresp[sensor1][sensor2](prevSize, 2) = vv_segment_n[sensor1][it->first][2];
        lines.mm_corresp[sensor1][sensor2](prevSize, 3) = vv_segments3D[sensor1][it->first][0];
        lines.mm_corresp[sensor1][sensor2](prevSize, 4) = vv_segments3D[sensor1][it->first][1];
        lines.mm_corresp[sensor1][sensor2](prevSize, 5) = vv_segments3D[sensor1][it->first][2];
        lines.mm_corresp[sensor1][sensor2](prevSize, 6) = vv_segments3D[sensor1][it->first][3];
        lines.mm_corresp[sensor1][sensor2](prevSize, 7) = vv_segments3D[sensor1][it->first][4];
        lines.mm_corresp[sensor1][sensor2](prevSize, 8) = vv_segments3D[sensor1][it->first][5];

        lines.mm_corresp[sensor1][sensor2](prevSize, 9) = vv_segment_n[sensor2][it->second][0];
        lines.mm_corresp[sensor1][sensor2](prevSize, 10) = vv_segment_n[sensor2][it->second][1];
        lines.mm_corresp[sensor1][sensor2](prevSize, 11) = vv_segment_n[sensor2][it->second][2];
        lines.mm_corresp[sensor1][sensor2](prevSize, 12) = vv_segments3D[sensor2][it->second][0];
        lines.mm_corresp[sensor1][sensor2](prevSize, 13) = vv_segments3D[sensor2][it->second][1];
        lines.mm_corresp[sensor1][sensor2](prevSize, 14) = vv_segments3D[sensor2][it->second][2];
        lines.mm_corresp[sensor1][sensor2](prevSize, 15) = vv_segments3D[sensor2][it->second][3];
        lines.mm_corresp[sensor1][sensor2](prevSize, 16) = vv_segments3D[sensor2][it->second][4];
        lines.mm_corresp[sensor1][sensor2](prevSize, 17) = vv_segments3D[sensor2][it->second][5];
    }
}

double ExtrinsicCalibLines::calcRotationErrorPair(const CMatrixDouble & correspondences, const Matrix<T,3,3> & Rot, bool in_deg)
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
            double rot_error = (n1).dot(Rot*v2);
            accum_error2 += rot_error*rot_error; // weight *
            accum_error_deg += RAD2DEG(acos(rot_error));
        }
        {
            Matrix<T,3,1> v1; v1 << correspondences(i,8)-correspondences(i,5), correspondences(i,7)-correspondences(i,4), correspondences(i,6)-correspondences(i,3);
            Matrix<T,3,1> n2; n2 << correspondences(i,9), correspondences(i,10), correspondences(i,11);
            v1.normalize();
            double rot_error = (v1).dot(Rot*n2);
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
            accum_error2 += calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], Rt[sensor1].block<3,3>(0,0).transpose()*Rt[sensor2].block<3,3>(0,0), in_deg);
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
    CTicTac clock; clock.Tic(); //Clock to measure the runtime

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
    Matrix<T,3,3> rotation;
    T conditioning = rotationFromNormals(mm_covariance[sensor1][sensor2], rotation);
    cout << "conditioning " << conditioning << "accum_rot_error2 " << accum_error2 << " " << calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], rotation) << endl;
    cout << "average error: "
              << calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], Rt_estimated[sensor1].block<3,3>(0,0).transpose()*Rt_estimated[sensor2].block<3,3>(0,0), true) << " vs "
              << calcRotationErrorPair(lines.mm_corresp[sensor1][sensor2], rotation, true) << " degrees\n";
    cout << "  Estimation took " << 1000*clock.Tac() << " ms.\n";

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
////            // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the line to the sensor
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


//// Ransac functions to detect outliers in the line matching
//void ransacLineAlignment_fit(
//        const CMatrixDouble &lineCorresp,
//        const mrpt::vector_size_t  &useIndices,
//        vector< CMatrixDouble > &fitModels )
////        vector< Matrix4f > &fitModels )
//{
//  ASSERT_(useIndices.size()==3);

//  try
//  {
//    CMatrixDouble corresp(8,3);

////  cout << "Size lineCorresp: " << endl;
////  cout << "useIndices " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << endl;
//    for(unsigned i=0; i<3; i++)
//      corresp.col(i) = lineCorresp.col(useIndices[i]);

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
//        const CMatrixDouble &lineCorresp,
//        const vector< CMatrixDouble > & testModels,
//        const double distanceThreshold,
//        unsigned int & out_bestModelIndex,
//        mrpt::vector_size_t & out_inlierIndices )
//{
//  ASSERT_( testModels.size()==1 )
//  out_bestModelIndex = 0;
//  const CMatrixDouble &M = testModels[0];

//  Matrix3d Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
//  Matrix<T,3,1> translation; translation << M(0,3), M(1,3), M(2,3);

//    ASSERT_( size(M,1)==4 && size(M,2)==4 )

//  const size_t N = size(lineCorresp,2);
//  out_inlierIndices.clear();
//  out_inlierIndices.reserve(100);
//  for (size_t i=0;i<N;i++)
//  {
//    const Matrix<T,3,1> n_i = Matrix<T,3,1>(lineCorresp(0,i), lineCorresp(1,i), lineCorresp(2,i));
//    const Matrix<T,3,1> n_ii = Rotation * Matrix<T,3,1>(lineCorresp(4,i), lineCorresp(5,i), lineCorresp(6,i));
//    const float d_error = fabs((lineCorresp(7,i) - translation.dot(n_i)) - lineCorresp(3,i));
//    const float angle_error = (n_i .cross (n_ii )).norm();

//    if (d_error < distanceThreshold)
//     if (angle_error < distanceThreshold) // Warning: this threshold has a different dimension
//      out_inlierIndices.push_back(i);
//  }
//}

///** Return "true" if the selected points are a degenerate (invalid) case.
//  */
//bool ransacLine_degenerate(
//        const CMatrixDouble &lineCorresp,
//        const mrpt::vector_size_t &useIndices )
//{
//  ASSERT_( useIndices.size()==3 )

//  const Matrix<T,3,1> n_1 = Matrix<T,3,1>(lineCorresp(0,useIndices[0]), lineCorresp(1,useIndices[0]), lineCorresp(2,useIndices[0]));
//  const Matrix<T,3,1> n_2 = Matrix<T,3,1>(lineCorresp(0,useIndices[1]), lineCorresp(1,useIndices[1]), lineCorresp(2,useIndices[1]));
//  const Matrix<T,3,1> n_3 = Matrix<T,3,1>(lineCorresp(0,useIndices[2]), lineCorresp(1,useIndices[2]), lineCorresp(2,useIndices[2]));
////cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

//  if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
//    return true;

//  return false;
//}

