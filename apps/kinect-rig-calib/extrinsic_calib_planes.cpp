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
#include <iostream>
#include <mrpt/poses/CPose3D.h>

#define VISUALIZE_SENSOR_DATA 0
#define SHOW_IMAGES 0

using namespace std;
using namespace mrpt::math;
using namespace mrpt::pbmap::PbMap;

typedef float T;

// Ransac functions to detect outliers in the plane matching
void ransacPlaneAlignment_fit ( const CMatrixDouble &planeCorresp,
                                const mrpt::vector_size_t  &useIndices,
                                vector< CMatrixDouble > &fitModels )
                        //        vector< Eigen::Matrix<T,4,4> > &fitModels )
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
        //    Eigen::Matrix<T,4,4> &M = fitModels[0];
        CMatrixDouble &M = fitModels[0];
        M = getAlignment(corresp);
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

    Eigen::Matrix<T,3,3> Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
    Eigen::Matrix<T,3,1> translation; translation << M(0,3), M(1,3), M(2,3);

    ASSERT_( size(M,1)==4 && size(M,2)==4 )

            const size_t N = size(planeCorresp,2);
    out_inlierIndices.clear();
    out_inlierIndices.reserve(100);
    for (size_t i=0;i<N;i++)
    {
        const Eigen::Matrix<T,3,1> n_i(planeCorresp(0,i), planeCorresp(1,i), planeCorresp(2,i));
        const Eigen::Matrix<T,3,1> n_ii = Rotation * Eigen::Matrix<T,3,1> (planeCorresp(4,i), planeCorresp(5,i), planeCorresp(6,i));
        const float d_error = fabs((planeCorresp(7,i) - translation.dot(n_i)) - planeCorresp(3,i));
        const float angle_error = (n_i .cross (n_ii )).norm();

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

    const Eigen::Matrix<T,3,1> n_1(planeCorresp(0,useIndices[0]), planeCorresp(1,useIndices[0]), planeCorresp(2,useIndices[0]));
    const Eigen::Matrix<T,3,1> n_2(planeCorresp(0,useIndices[1]), planeCorresp(1,useIndices[1]), planeCorresp(2,useIndices[1]));
    const Eigen::Matrix<T,3,1> n_3(planeCorresp(0,useIndices[2]), planeCorresp(1,useIndices[2]), planeCorresp(2,useIndices[2]));
    //cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

    if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
        return true;

    return false;
}

void ExtrinsicCalibPlanes::getCorrespondences(const std::vector<pcl::PointCloud<PointT>::Ptr> & cloud)
{
    mrpt::pbmap::PbMap all_planes;
    vector<size_t> planesSourceIdx(calib->num_sensors+1, 0);

    //						Segment local planes
    //==================================================================
    // #pragma omp parallel num_threads(calib->num_sensors)
    for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
    {
        // sensor_id = omp_get_thread_num();
        cout << sensor_id << " cloud " << cloud[sensor_id]->height << "x" << cloud[sensor_id]->width << endl;

//        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//        viewer.showCloud(cloud[sensor_id]);
//        while (!viewer.wasStopped ())
//            boost::this_thread::sleep (boost::posix_time::milliseconds (10));

        //cout << "PbMapMaker::pbMapFromPCloud\n";
        PbMap::pbMapFromPCloud(cloud[sensor_id], v_pbmap[sensor_id]);
        all_planes.MergeWith(v_pbmap[sensor_id], calib->Rt_estimated[sensor_id]);
        planesSourceIdx[sensor_id+1] = planesSourceIdx[sensor_id] + v_pbmap[sensor_id].vPlanes.size();
        //cout << planesSourceIdx[sensor_id+1] << " ";

//        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
//        std::vector<pcl::ModelCoefficients> model_coefficients;
//        std::vector<pcl::PointIndices> inliers;
//        size_t n_planes = segmentPlanes(cloud[sensor_id], regions, model_coefficients, inliers, dist_threshold, angle_threshold, min_inliers);
//        cout << sensor_id << " number of planes " << n_planes << endl;
    }

    if(display)
    {
        // Show registered depth
        cv::Mat img_transposed, img_rotated;
        cv::transpose(depth[0], img_transposed);
        cv::flip(img_transposed, img_rotated, 0);
        cv::Mat depth_concat(height, 2*width+20, CV_32FC1, cv::Scalar(0.f));
        cv::Mat tmp = depth_concat(cv::Rect(0, 0, width, height));
        img_rotated.copyTo(tmp);
        cv::transpose(depth[1], img_transposed);
        cv::flip(img_transposed, img_rotated, 0);
        tmp = depth_concat(cv::Rect(width+20, 0, width, height));
        img_rotated.copyTo(tmp);
        cv::Mat img_depth_display;
        depth_concat.convertTo(img_depth_display, CV_32FC1, 0.3 );
        cv::Mat depth_display = cv::Mat(depth_concat.rows, depth_concat.cols, depth_concat.type(), cv::Scalar(1.f)) - img_depth_display;
        depth_display.setTo(cv::Scalar(0.f), depth_concat < 0.1f);
        cv::imshow("depth", depth_display ); cv::moveWindow("depth", 20,100+640);
//                cout << "Some depth values: " << depth_concat.at<float>(200, 320) << " " << depth_concat.at<float>(50, 320) << " " //<< depth_concat.at<float>(650, 320) << " "
//                     << depth[0].at<float>(200, 320) << " " << depth[0].at<float>(50, 320) << " " << depth[1].at<float>(430, 320) << " " << depth[1].at<float>(280, 320) << "\n";
        cv::waitKey(0);
    }


    //==============================================================================
    //								Data association
    //==============================================================================    
    for(size_t i=0; i < v_pbmap[sensor1].vPlanes.size(); i++) // Find plane correspondences
    {
      size_t planesIdx_i = planesSourceIdx[sensor1];
      for(size_t j=0; j < v_pbmap[sensor2].vPlanes.size(); j++)
      {
        size_t planesIdx_j = planesSourceIdx[sensor2];

//                if(sensor1 == 0 && sensor2 == 2 && i == 0 && j == 0)
//                {
//                  cout << "Inliers " << all_planes.vPlanes[planesIdx_i+i].inliers.size() << " and " << all_planes.vPlanes[planesIdx_j+j].inliers.size() << endl;
//                  cout << "elongation " << all_planes.vPlanes[planesIdx_i+i].elongation << " and " << all_planes.vPlanes[planesIdx_j+j].elongation << endl;
//                  cout << "normal " << all_planes.vPlanes[planesIdx_i+i].v3normal.transpose() << " and " << all_planes.vPlanes[planesIdx_j+j].v3normal.transpose() << endl;
//                  cout << "d " << all_planes.vPlanes[planesIdx_i+i].d << " and " << all_planes.vPlanes[planesIdx_j+j].d << endl;
//                  cout << "color " << all_planes.vPlanes[planesIdx_i+i].hasSimilarDominantColor(all_planes.vPlanes[planesIdx_j+j],0.06) << endl;
//                  cout << "nearby " << all_planes.vPlanes[planesIdx_i+i].isPlaneNearby(all_planes.vPlanes[planesIdx_j+j], 0.5) << endl;
//                }

//                cout << "  Check planes " << planesIdx_i+i << " and " << planesIdx_j+j << endl;

        if( all_planes.vPlanes[planesIdx_i+i].inliers.size() > 1000 && all_planes.vPlanes[planesIdx_j+j].inliers.size() > min_inliers &&
            all_planes.vPlanes[planesIdx_i+i].elongation < 5 && all_planes.vPlanes[planesIdx_j+j].elongation < 5 &&
            all_planes.vPlanes[planesIdx_i+i].v3normal .dot (all_planes.vPlanes[planesIdx_j+j].v3normal) > 0.99 &&
            fabs(all_planes.vPlanes[planesIdx_i+i].d - all_planes.vPlanes[planesIdx_j+j].d) < 0.1 )//&&
            //                    v_pbmap[0].vPlanes[i].hasSimilarDominantColor(v_pbmap[1].vPlanes[j],0.06) &&
            //                    v_pbmap[0].vPlanes[planes_counter_i+i].isPlaneNearby(v_pbmap[1].vPlanes[planes_counter_j+j], 0.5)
        {

//              mrpt::pbmap::PbMap &planes_i = v_pbmap[0];
//              mrpt::pbmap::PbMap &planes_j = v_pbmap[1];

            if(b_confirm_visually)
                while(confirmed_corresp == 0)
                    mrpt::system::sleep(10);
            if(confirmed_corresp == -1)
                continue;

            cout << "\tAssociate planes " << endl;
            Eigen::Vector4f pl1, pl2;
            pl1.head(3) = v_pbmap[0].vPlanes[i].v3normal; pl1[3] = v_pbmap[0].vPlanes[i].d;
            pl2.head(3) = v_pbmap[1].vPlanes[j].v3normal; pl2[3] = v_pbmap[1].vPlanes[j].d;
            //              cout << "Corresp " << v_pbmap[0].vPlanes[i].v3normal.transpose() << " vs " << v_pbmap[1].vPlanes[j].v3normal.transpose() << " = " << v_pbmap[1].vPlanes[j].v3normal.transpose() << endl;
            ////                        float factorDistInliers = std::min(v_pbmap[0].vPlanes[i].inliers.size(), v_pbmap[1].vPlanes[j].inliers.size()) / std::max(v_pbmap[0].vPlanes[i].v3center.norm(), v_pbmap[1].vPlanes[j].v3center.norm());
            //                        float factorDistInliers = (v_pbmap[0].vPlanes[i].inliers.size() + v_pbmap[1].vPlanes[j].inliers.size()) / (v_pbmap[0].vPlanes[i].v3center.norm() * v_pbmap[1].vPlanes[j].v3center.norm());
            //                        weight_pair[couple_id] += factorDistInliers;
            //                        pl1 *= factorDistInliers;
            //                        pl2 *= factorDistInliers;
            //                ++weight_pair[couple_id];

            //Add constraints
            //                  correspondences.push_back(pair<Eigen::Vector4f, Eigen::Vector4f>(pl1, pl2));
            //                        correspondences[couple_id].push_back(pair<Eigen::Vector4f, Eigen::Vector4f>(pl1/v_pbmap[0].vPlanes[i].v3center.norm(), pl2/v_pbmap[1].vPlanes[j].v3center.norm());

            // Calculate conditioning
            ++n_plane_corresp;
            covariance += pl2.head(3) * pl1.head(3).transpose();
            Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
            conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
            cout << "conditioning " << conditioning << endl;

            size_t prevSize = calib_planes.correspondences.getRowCount();
            calib_planes.correspondences.setSize(prevSize+1, calib_planes.correspondences.getColCount());
            calib_planes.correspondences(prevSize, 0) = pl1[0];
            calib_planes.correspondences(prevSize, 1) = pl1[1];
            calib_planes.correspondences(prevSize, 2) = pl1[2];
            calib_planes.correspondences(prevSize, 3) = pl1[3];
            calib_planes.correspondences(prevSize, 4) = pl2[0];
            calib_planes.correspondences(prevSize, 5) = pl2[1];
            calib_planes.correspondences(prevSize, 6) = pl2[2];
            calib_planes.correspondences(prevSize, 7) = pl2[3];

            Eigen::Matrix<T,4,4> informationFusion;
            Eigen::Matrix<T,4,4> tf = Eigen::Matrix<T,4,4>::Identity();
            tf.block(0,0,3,3) = calib_planes.calib->Rt_estimated.block(0,0,3,3);
            tf.block(3,0,1,3) = -calib_planes.calib->Rt_estimated.block(0,3,3,1).transpose();
            informationFusion = v_pbmap[0].vPlanes[i].information;
            informationFusion += tf * v_pbmap[1].vPlanes[j].information * tf.inverse();
            Eigen::JacobiSVD<Eigen::Matrix<T,4,4> > svd_cov(informationFusion, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector4f minEigenVector = svd_cov.matrixU().block(0,3,4,1);
            cout << "minEigenVector " << minEigenVector.transpose() << endl;
            informationFusion -= svd.singularValues().minCoeff() * minEigenVector * minEigenVector.transpose();
            cout << "informationFusion \n" << informationFusion << "\n minSV " << svd.singularValues().minCoeff() << endl;

            calib_planes.correspondences(prevSize, 8) = informationFusion(0,0);
            calib_planes.correspondences(prevSize, 9) = informationFusion(0,1);
            calib_planes.correspondences(prevSize, 10) = informationFusion(0,2);
            calib_planes.correspondences(prevSize, 11) = informationFusion(0,3);
            calib_planes.correspondences(prevSize, 12) = informationFusion(1,1);
            calib_planes.correspondences(prevSize, 13) = informationFusion(1,2);
            calib_planes.correspondences(prevSize, 14) = informationFusion(1,3);
            calib_planes.correspondences(prevSize, 15) = informationFusion(2,2);
            calib_planes.correspondences(prevSize, 16) = informationFusion(2,3);
            calib_planes.correspondences(prevSize, 17) = informationFusion(3,3);


            FIMrot += -skew(v_pbmap[1].vPlanes[j].v3normal) * informationFusion.block(0,0,3,3) * skew(v_pbmap[1].vPlanes[j].v3normal);
            FIMtrans += v_pbmap[0].vPlanes[i].v3normal * v_pbmap[0].vPlanes[i].v3normal.transpose() * informationFusion(3,3);

            Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd_rot(FIMrot, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd_trans(FIMtrans, Eigen::ComputeFullU | Eigen::ComputeFullV);
            //              float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
            conditioningFIM.setSize(prevSize+1, conditioningFIM.getColCount());
            conditioningFIM(prevSize, 0) = svd_rot.singularValues()[0];
            conditioningFIM(prevSize, 1) = svd_rot.singularValues()[1];
            conditioningFIM(prevSize, 2) = svd_rot.singularValues()[2];
            conditioningFIM(prevSize, 3) = svd_trans.singularValues()[0];
            conditioningFIM(prevSize, 4) = svd_trans.singularValues()[1];
            conditioningFIM(prevSize, 5) = svd_trans.singularValues()[2];

//              cout << "normalCovariance " << minEigenVector.transpose() << " covM \n" << svd_cov.matrixU() << endl;

//                calib_planes.correspondences(prevSize, 8) = std::min(v_pbmap[0].vPlanes[i].inliers.size(), v_pbmap[1].vPlanes[j].inliers.size());
//
//                float dist_center1 = 0, dist_center2 = 0;
//                for(size_t k=0; k < v_pbmap[0].vPlanes[i].inliers.size(); k++)
//                  dist_center1 += v_pbmap[0].vPlanes[i].inliers[k] / frameRGBD_[0].getPointCloud()->width + v_pbmap[0].vPlanes[i].inliers[k] % frameRGBD_[0].getPointCloud()->width;
////                      dist_center1 += (v_pbmap[0].vPlanes[i].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[0].vPlanes[i].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[0].vPlanes[i].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[0].vPlanes[i].inliers[k] % frame360.sphereCloud->width);
//                dist_center1 /= v_pbmap[0].vPlanes[i].inliers.size();
//
//                for(size_t k=0; k < v_pbmap[1].vPlanes[j].inliers.size(); k++)
//                  dist_center2 += v_pbmap[1].vPlanes[j].inliers[k] / frameRGBD_[0].getPointCloud()->width + v_pbmap[1].vPlanes[j].inliers[k] % frameRGBD_[0].getPointCloud()->width;
////                      dist_center2 += (v_pbmap[1].vPlanes[j].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[1].vPlanes[j].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[1].vPlanes[j].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[1].vPlanes[j].inliers[k] % frame360.sphereCloud->width);
//                dist_center2 /= v_pbmap[1].vPlanes[j].inliers.size();
//                calib_planes.correspondences(prevSize, 9) = std::max(dist_center1, dist_center2);



//              for(size_t sensor1=0; sensor1 < calib->num_sensors; sensor1++)
//              {
//      //          matches.mmCorrespondences[sensor1] = std::map<unsigned, mrpt::math::CMatrixDouble>();
//                for(unsigned sensor2=sensor1+1; sensor2 < calib->num_sensors; sensor2++)
//                {
//      //            cout << " sensor1 " << sensor1 << " " << v_pbmap[sensor1].vPlanes.size() << " sensor2 " << sensor2 << " " << v_pbmap[sensor2].vPlanes.size() << endl;
//      //              matches.mmCorrespondences[sensor1][sensor2] = mrpt::math::CMatrixDouble(0, 10);

//                  for(unsigned i=0; i < v_pbmap[sensor1].vPlanes.size(); i++)
//                  {
//                    unsigned planesIdx_i = planesSourceIdx[sensor1];
//                    for(unsigned j=0; j < v_pbmap[sensor2].vPlanes.size(); j++)
//                    {
//                      unsigned planesIdx_j = planesSourceIdx[sensor2];

//      //                if(sensor1 == 0 && sensor2 == 2 && i == 0 && j == 0)
//      //                {
//      //                  cout << "Inliers " << all_planes.vPlanes[planesIdx_i+i].inliers.size() << " and " << all_planes.vPlanes[planesIdx_j+j].inliers.size() << endl;
//      //                  cout << "elongation " << all_planes.vPlanes[planesIdx_i+i].elongation << " and " << all_planes.vPlanes[planesIdx_j+j].elongation << endl;
//      //                  cout << "normal " << all_planes.vPlanes[planesIdx_i+i].v3normal.transpose() << " and " << all_planes.vPlanes[planesIdx_j+j].v3normal.transpose() << endl;
//      //                  cout << "d " << all_planes.vPlanes[planesIdx_i+i].d << " and " << all_planes.vPlanes[planesIdx_j+j].d << endl;
//      //                  cout << "color " << all_planes.vPlanes[planesIdx_i+i].hasSimilarDominantColor(all_planes.vPlanes[planesIdx_j+j],0.06) << endl;
//      //                  cout << "nearby " << all_planes.vPlanes[planesIdx_i+i].isPlaneNearby(all_planes.vPlanes[planesIdx_j+j], 0.5) << endl;
//      //                }

//      //                cout << "  Check planes " << planesIdx_i+i << " and " << planesIdx_j+j << endl;

//                      if( all_planes.vPlanes[planesIdx_i+i].inliers.size() > 1000 && all_planes.vPlanes[planesIdx_j+j].inliers.size() > 1000 &&
//                          all_planes.vPlanes[planesIdx_i+i].elongation < 5 && all_planes.vPlanes[planesIdx_j+j].elongation < 5 &&
//                          all_planes.vPlanes[planesIdx_i+i].v3normal .dot (all_planes.vPlanes[planesIdx_j+j].v3normal) > 0.99 &&
//                          fabs(all_planes.vPlanes[planesIdx_i+i].d - all_planes.vPlanes[planesIdx_j+j].d) < 0.2 )//&&
//      //                    all_planes.vPlanes[planesIdx_i+i].hasSimilarDominantColor(all_planes.vPlanes[planesIdx_j+j],0.06) &&
//      //                    all_planes.vPlanes[planesIdx_i+i].isPlaneNearby(all_planes.vPlanes[planesIdx_j+j], 0.5) )
//        //                      matches.inliersUpperFringe(all_planes.vPlanes[planesIdx_i+i], 0.2) > 0.2 &&
//        //                      matches.inliersLowerFringe(all_planes.vPlanes[planesIdx_j+j], 0.2) > 0.2 ) // Assign correspondence
//                        {
//      //                  cout << "\t   Associate planes " << planesIdx_i+i << " and " << planesIdx_j+j << endl;

//                        #if VISUALIZE_POINT_CLOUD
//                          // Visualize Control Planes
//                          { boost::mutex::scoped_lock updateLock(visualizationMutex);
//                            match1 = planesIdx_i+i;
//                            match2 = planesIdx_j+j;
//                            drawMatch = true;
//                            keyDown = false;
//                          updateLock.unlock();
//                          }

//      //                    match1 = planesIdx_i+i;
//      //                    match2 = planesIdx_j+j;
//      //                    pcl::visualization::CloudViewer viewer("RGBD360_calib");
//      //                    viewer.runOnVisualizationThread (boost::bind(&GetControlPlanes::viz_cb, this, _1), "viz_cb");
//      //                    viewer.registerKeyboardCallback(&GetControlPlanes::keyboardEventOccurred, *this);

//      //                    cout << " keyDown " << keyDown << endl;
//                          while(!keyDown)
//                            boost::this_thread::sleep (boost::posix_time::milliseconds (10));
//                        #endif

//      //                  cout << "\t   Record corresp " << endl;
//                          unsigned prevSize = calib_planes.correspondences.getRowCount();
//                          calib_planes.correspondences.setSize(prevSize+1, calib_planes.correspondences.getColCount());
//                          calib_planes.correspondences(prevSize, 0) = v_pbmap[sensor1].vPlanes[i].v3normal[0];
//                          calib_planes.correspondences(prevSize, 1) = v_pbmap[sensor1].vPlanes[i].v3normal[1];
//                          calib_planes.correspondences(prevSize, 2) = v_pbmap[sensor1].vPlanes[i].v3normal[2];
//                          calib_planes.correspondences(prevSize, 3) = v_pbmap[sensor1].vPlanes[i].d;
//                          calib_planes.correspondences(prevSize, 4) = v_pbmap[sensor2].vPlanes[j].v3normal[0];
//                          calib_planes.correspondences(prevSize, 5) = v_pbmap[sensor2].vPlanes[j].v3normal[1];
//                          calib_planes.correspondences(prevSize, 6) = v_pbmap[sensor2].vPlanes[j].v3normal[2];
//                          calib_planes.correspondences(prevSize, 7) = v_pbmap[sensor2].vPlanes[j].d;
//                          calib_planes.correspondences(prevSize, 8) = std::min(v_pbmap[sensor1].vPlanes[i].inliers.size(), v_pbmap[sensor2].vPlanes[j].inliers.size());

//                          // For several sensors (more than 2)
////                          unsigned prevSize = matches.mmCorrespondences[sensor1][sensor2].getRowCount();
////                          matches.mmCorrespondences[sensor1][sensor2].setSize(prevSize+1, matches.mmCorrespondences[sensor1][sensor2].getColCount());
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 0) = v_pbmap[sensor1].vPlanes[i].v3normal[0];
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 1) = v_pbmap[sensor1].vPlanes[i].v3normal[1];
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 2) = v_pbmap[sensor1].vPlanes[i].v3normal[2];
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 3) = v_pbmap[sensor1].vPlanes[i].d;
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 4) = v_pbmap[sensor2].vPlanes[j].v3normal[0];
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 5) = v_pbmap[sensor2].vPlanes[j].v3normal[1];
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 6) = v_pbmap[sensor2].vPlanes[j].v3normal[2];
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 7) = v_pbmap[sensor2].vPlanes[j].d;
////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 8) = std::min(v_pbmap[sensor1].vPlanes[i].inliers.size(), v_pbmap[sensor2].vPlanes[j].inliers.size());


////                          float dist_center1 = 0, dist_center2 = 0;
////                          for(unsigned k=0; k < v_pbmap[sensor1].vPlanes[i].inliers.size(); k++)
////                            dist_center1 += v_pbmap[sensor1].vPlanes[i].inliers[k] / frame360.frameRGBD_[sensor1].getPointCloud()->width + v_pbmap[sensor1].vPlanes[i].inliers[k] % frame360.frameRGBD_[sensor1].getPointCloud()->width;
////      //                      dist_center1 += (v_pbmap[sensor1].vPlanes[i].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[sensor1].vPlanes[i].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[sensor1].vPlanes[i].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[sensor1].vPlanes[i].inliers[k] % frame360.sphereCloud->width);
////                          dist_center1 /= v_pbmap[sensor1].vPlanes[i].inliers.size();

////                          for(unsigned k=0; k < v_pbmap[sensor2].vPlanes[j].inliers.size(); k++)
////                            dist_center2 += v_pbmap[sensor2].vPlanes[j].inliers[k] / frame360.frameRGBD_[sensor2].getPointCloud()->width + v_pbmap[sensor2].vPlanes[j].inliers[k] % frame360.frameRGBD_[sensor2].getPointCloud()->width;
////      //                      dist_center2 += (v_pbmap[sensor2].vPlanes[j].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[sensor2].vPlanes[j].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[sensor2].vPlanes[j].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[sensor2].vPlanes[j].inliers[k] % frame360.sphereCloud->width);
////                          dist_center2 /= v_pbmap[sensor2].vPlanes[j].inliers.size();

////                          matches.mmCorrespondences[sensor1][sensor2](prevSize, 9) = std::max(dist_center1, dist_center2);
////      //                  cout << "\t Size " << matches.mmCorrespondences[sensor1][sensor2].getRowCount() << " x " << matches.mmCorrespondences[sensor1][sensor2].getColCount() << endl;

////                          if( sensor2 - sensor1 == 1 ) // Calculate conditioning
////                          {
////      //                      updateConditioning(couple_id, correspondences[couple_id].back());
////                            matches.covariances[sensor1] += all_planes.vPlanes[planesIdx_i+i].v3normal * all_planes.vPlanes[planesIdx_j+j].v3normal.transpose();
////                            matches.calcAdjacentConditioning(sensor1);
////      //                    cout << "Update " << sensor1 << endl;

////        //                    // For visualization
////        //                    plane_corresp[couple_id].push_back(pair<mrpt::pbmap::Plane*, mrpt::pbmap::Plane*>(&all_planes.vPlanes[planesIdx_i+i], &all_planes.vPlanes[planes_counter_j+j]));
////                          }
////                          else if(sensor2 - sensor1 == 7)
////                          {
////      //                      updateConditioning(couple_id, correspondences[couple_id].back());
////                            matches.covariances[sensor2] += all_planes.vPlanes[planesIdx_i+i].v3normal * all_planes.vPlanes[planesIdx_j+j].v3normal.transpose();
////                            matches.calcAdjacentConditioning(sensor2);
////      //                    cout << "Update " << sensor2 << endl;
////                          }

//      //                    break;
//                        }
//                    }
//                  }
//                }
//              }
        }
      }
    }
}



double ExtrinsicCalibPlanes::calcCorrespRotError(const std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > & Rt)
{
    double accum_error2 = 0.0;
//    double accum_error_deg = 0.0;
    size_t num_corresp = 0;
    for(unsigned sensor1=0; sensor1 < calib->num_sensors-1; sensor1++)
        for(unsigned sensor2=sensor1+1; sensor2 < calib->num_sensors; sensor2++)
        {
            mrpt::math::CMatrixDouble & correspondences = planes.mm_corresp[sensor1][sensor2];
            for(unsigned i=0; i < correspondences.rows(); i++)
            {
                //        float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
                float weight = 1.0;
                Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
                Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
                Eigen::Matrix<T,3,1> rot_error = (Rt[sensor1].block<3,3>(0,0)*n_obs_i - Rt[sensor2].block<3,3>(0,0)*n_obs_ii);
                accum_error2 += weight * fabs(rot_error.dot(rot_error));
                ++num_corresp;
                // accum_error_deg += acos(fabs(rot_error.dot(rot_error)));
            }
        }

    //      std::cout << "AvError deg " << accum_error2 / num_corresp << std::endl;
    return accum_error2 / num_corresp;
}

//    float calcCorrespTransError(Eigen::Matrix<T,3,3> &Rot_)
//    {
//      float accum_error2 = 0.0;
//      float accum_error_m = 0.0;
//      for(unsigned i=0; i < correspondences.rows(); i++)
//      {
////        float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
//        float weight = 1.0;
//        Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        float trans_error = (correspondences(i,3) - correspondences(i,7) + n_obs_i.dot());
//        accum_error2 += weight * fabs(rot_error.dot(rot_error));
//        accum_error_deg += acos(fabs(rot_error.dot(rot_error)));
//      }
//
//      std::cout << "AvError deg " << accum_error_deg/correspondences.rows() << std::endl;
//      return accum_error2/correspondences.rows();
//    }

Eigen::Matrix<T,3,3> ExtrinsicCalibPlanes::calcFIMRotation(const mrpt::math::CMatrixDouble & correspondences)
{
    Eigen::Matrix<T,3,3> FIM = Eigen::Matrix<T,3,3>::Zero();

    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);

        Eigen::Matrix<T,3,1> score = calcScoreRotation(n_obs_i, n_obs_ii);

        FIM += score * score.transpose();
    }

    return FIM;
}

Eigen::Matrix<T,3,3> ExtrinsicCalibPlanes::calcFIMTranslation()
{
    Eigen::Matrix<T,3,3> FIM = Eigen::Matrix<T,3,3>::Zero();

    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        //        Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        float d_obs_i = correspondences(i,3);
        float d_obs_ii = correspondences(i,7);

        Eigen::Matrix<T,3,1> score = calcScoreTranslation(n_obs_i, d_obs_i, d_obs_ii);

        FIM += score * score.transpose();
    }

    return FIM;
}


//    Eigen::Matrix<T,3,3> calcFisherInfMat(const int weightedLS)
//    {
//      // Calibration system
//      Eigen::Matrix<T,3,3> rotationCov = Eigen::Matrix<T,3,3>::Zero();
//
//      Eigen::Matrix<T,3,3> FIM_rot = Eigen::Matrix<T,3,3>::Zero();
//      Eigen::Matrix<T,3,3> FIM_trans = Eigen::Matrix<T,3,3>::Zero();
//      Eigen::Matrix<T,3,1> score;
//
//      float accum_error2 = 0;
////      rotationCov += v3normal2 * v3normal1.transpose();
//      for(unsigned i=0; i < correspondences.rows(); i++)
//      {
////          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
//        Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
//        Eigen::Matrix<T,3,1> n_i = n_obs_i;
//        Eigen::Matrix<T,3,1> n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
//        Eigen::Matrix<T,3,1> rot_error = (n_i - n_ii);
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
//      Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd(rotationCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
//      float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//
//      float minFIM_rot = std::min(FIM_rot(0,0), std::min(FIM_rot(1,1), FIM_rot(2,2)));
//      float minFIM_trans = std::min(FIM_trans(0,0), std::min(FIM_trans(1,1), FIM_trans(2,2)));
////      std::cout << "minFIM_rot " << minFIM_rot << " " << minFIM_trans << " conditioning " << conditioning << " numCorresp " << correspondences.rows() << std::endl;
//      std::cout << "\nFIM_rot \n" << FIM_rot << std::endl;
//      std::cout << "\nFIM_trans \n" << FIM_trans << std::endl;
//    }

Eigen::Matrix<T,3,3> ExtrinsicCalibPlanes::CalibrateRotation(int weightedLS)
{
    // Calibration system
    Eigen::Matrix<T,3,3> rotationCov = Eigen::Matrix<T,3,3>::Zero();

    //      Eigen::Matrix<T,3,3> FIM_rot = Eigen::Matrix<T,3,3>::Zero();
    //      Eigen::Matrix<T,3,3> FIM_trans = Eigen::Matrix<T,3,3>::Zero();
    //      Eigen::Matrix<T,3,1> score;

    float accum_error2 = 0;
    //      rotationCov += v3normal2 * v3normal1.transpose();
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        //          float weight = (inliers / correspondences(i,3)) / correspondences.rows()
        Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        Eigen::Matrix<T,3,1> n_i = n_obs_i;
        Eigen::Matrix<T,3,1> n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
        Eigen::Matrix<T,3,1> rot_error = (n_i - n_ii);
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
    Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd(rotationCov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    std::cout << "conditioning " << conditioning << std::endl;
    //      if(conditioning > 20000)
    if(conditioning > 100)
    {
        std::cout << "Bad conditioning " << conditioning << std::endl;
        return Eigen::Matrix<T,3,3>::Identity();
    }

    rotation = svd.matrixV() * svd.matrixU().transpose();
    double det = rotation.determinant();
    if(det != 1)
    {
        Eigen::Matrix<T,3,3> aux;
        aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
        rotation = svd.matrixV() * aux * svd.matrixU().transpose();
    }
    std::cout << "accum_rot_error2 av_deg" << acos(calcCorrespRotError(rotation)) << std::endl;

    return rotation;
}


Eigen::Matrix<T,3,3> ExtrinsicCalibPlanes::CalibrateRotationD(int weightedLS)
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
        return Eigen::Matrix<T,3,3>::Identity();
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
    //      return Eigen::Matrix<T,3,3>::Identity();
}

/*! Get the rotation of each sensor in the multisensor RGBD360 setup */
Eigen::Matrix<T,3,3> ExtrinsicCalibPlanes::CalibrateRotationManifold(int weightedLS)
{
    cout << "CalibrateRotationManifold...\n";
    Eigen::Matrix<float,3,3> hessian;
    Eigen::Matrix<float,3,1> gradient;
    Eigen::Matrix<float,3,1> update_vector;
    Eigen::Matrix<T,3,3> jacobian_rot_i, jacobian_rot_ii; // Jacobians of the rotation
    float accum_error2;
    float av_angle_error;
    unsigned numPlaneCorresp;

    // Load the extrinsic calibration from the device' specifications
    //        calib->Rt_estimated[sensor_id] = Rt_specs[sensor_id];

    Eigen::Matrix<T,4,4> calib->Rt_estimatedTemp;
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
                Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
                Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
                //            Eigen::Matrix<T,3,1> n_i = n_obs_i;
                Eigen::Matrix<T,3,1> n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
                //            jacobian_rot_i = rv::math::skew(-n_i);
                jacobian_rot_ii = skew(n_ii);
                Eigen::Matrix<T,3,1> rot_error = (n_obs_i - n_ii);
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
                    Eigen::Matrix<T,3,3> information;
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

                //            Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
                //            Eigen::Matrix<T,3,3> cov = hessian.inverse();
                //            Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd2(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

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

        //        Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        //        float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
        //        Eigen::Matrix<T,3,3> cov;
        //        svd.pinv(cov);
        //        std::cout << "hessian \n" << hessian << "inv\n" << hessian.inverse() << "\ncov \n" << cov << std::endl;

        //        std::cout << "conditioning " << conditioning << std::endl;
        //        if(conditioning > 100)
        //          return Eigen::Matrix<T,3,3>::Identity();

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
            Eigen::Matrix<T,3,3> update_rot_eig;
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

Eigen::Matrix<T,3,1> ExtrinsicCalibPlanes::CalibrateTranslation(int weightedLS)
{
    // Calibration system
    Eigen::Matrix<T,3,3> translationHessian = Eigen::Matrix<T,3,3>::Zero();
    Eigen::Matrix<T,3,1> translationGradient = Eigen::Matrix<T,3,1>::Zero();

    //      Eigen::Matrix<T,3,1> translation2 = Eigen::Matrix<T,3,1>::Zero();
    //      Eigen::Matrix<T,3,1> sumNormals = Eigen::Matrix<T,3,1>::Zero();

    //              translationHessian += v3normal1 * v3normal1.transpose();
    //  //            double error = d2 - d1;
    //              translationGradient += v3normal1 * (d2 - d1);
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        //        Eigen::Matrix<T,3,1> n_i = calib->Rt_estimated[sensor_id].block(0,0,3,3) * n_obs_i;
        //        Eigen::Matrix<T,3,1> n_ii = calib->Rt_estimated.block(0,0,3,3) * n_obs_ii;
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

    Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd(translationHessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
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

void ExtrinsicCalibPlanes::Calibrate()
{
    //      calibrated_Rt = Eigen::Matrix<T,4,4>::Identity();
    calib->Rt_estimated.block(0,0,3,3) = CalibrateRotation();
    calib->Rt_estimated.block(0,3,3,1) = CalibrateTranslation();
    //      std::cout << "calib->Rt_estimated\n" << calib->Rt_estimated << std::endl;

    // Calculate average error
    float av_rot_error = 0;
    float av_trans_error = 0;
    for(unsigned i=0; i < correspondences.rows(); i++)
    {
        Eigen::Matrix<T,3,1> n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
        Eigen::Matrix<T,3,1> n_obs_ii; n_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
        av_rot_error += fabs(acos(n_obs_i .dot( rotation * n_obs_ii ) ));
        av_trans_error += fabs(correspondences(i,3) - correspondences(i,7) - n_obs_i .dot(translation));
        //        params_error += plane_correspondences1[i] .dot( plane_correspondences2[i] );
    }
    av_rot_error /= correspondences.rows();
    av_trans_error /= correspondences.rows();
    std::cout << "Errors n.n " << calcCorrespRotError(calib->Rt_estimated) << " av deg " << av_rot_error*180/3.14159f << " av trans " << av_trans_error << std::endl;
}

/*! Print the number of correspondences and the conditioning number to the standard output */
void ExtrinsicCalibPlanes::printConditioning()
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
void ExtrinsicCalibPlanes::calcAdjacentConditioning(unsigned couple_id)
{
    //      if(conditioning_measCount[couple_id] > 3)
    //      {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariances_[couple_id], Eigen::ComputeFullU | Eigen::ComputeFullV);
    conditioning_[couple_id] = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    //      }
}
