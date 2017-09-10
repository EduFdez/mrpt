/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: camera-rot-tracker
    FILE: camera-rot-tracker_main.cpp
    AUTHOR: Eduardo Fernández-Moral <efernandezmoral@gmail.com>

    See README.txt for instructions or
          http://www.mrpt.org/Application:camera-rot-tracker
  ---------------------------------------------------------------*/

#include "camera-rot-tracker.h"
#include "../kinect-rig-calib/kinect-rig-calib_misc.h"
#include "DifOdometry_Datasets_RGBD.h"
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <boost/thread/mutex.hpp>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;
using namespace Eigen;


void CameraRotTracker::displayObservation()
{
    cout << "CameraRotTracker::displayObservation...\n";

    for(size_t i=1; i < num_sensors; i++)
    {
//        string win_name = format("rgb_%lu",v_rgb[i]);
        cv::imshow(mrpt::format("rgb_%lu",i), v_rgb[i] ); //cv::moveWindow("rgb", 20,20);
    }
    cv::waitKey(0);
}

void CameraRotTracker::loadConfiguration(const string & config_file)
{
    //Initial steps. Load configuration file
    //-------------------------------------------
    utils::CConfigFile cfg(config_file);
    rawlog_file = cfg.read_string("GLOBAL", "rawlog_file", "no file", true);
    output_dir = cfg.read_string("GLOBAL", "output_dir", "no dir", true);

    decimation = cfg.read_int("GLOBAL", "decimation", 0, true);
    display = cfg.read_bool("GLOBAL", "display", 0, true);
    verbose = cfg.read_bool("GLOBAL", "verbose", 0, true);
    min_pixels_line = cfg.read_int("GLOBAL", "min_pixels_line", 100, true);
    min_angle_diff = cfg.read_float("GLOBAL", "min_angle_diff", 1.2, true);

    if(verbose)
        cout << "loadConfiguration -> dataset: " << rawlog_file << "\toutput: " << output_dir << "\tdecimation: " << decimation << endl;

    //string sensor_label = "RGBD";
    intrinsics.loadFromConfigFile("GLOBAL", cfg);

    cout << "...CameraRotTracker::loadConfiguration\n";
}

void CameraRotTracker::setNewFrame()
{
    cout << "CameraRotTracker::setNewFrame..." << num_sensors << endl;
    for(size_t i=1; i < num_sensors; i++)
    {
        obsRGBD[i] = obsRGBD[i-1];
//        v_rgb[i] = cv::Mat(obsRGBD[i]->intensityImage.getAs<IplImage>());
//        convertRange_mrpt2cvMat(obsRGBD[i]->rangeImage, v_depth[i]);
        v_cloud[i] = v_cloud[i-1];
        vv_segments2D[i] = vv_segments2D[i-1];
        vv_segment_n[i] = vv_segment_n[i-1];
    }
}

/*
void CameraRotTracker::run2()
{
    //						Open Rawlog File
    //==================================================================
    mrpt::obs::CRawlog dataset;
    if (!dataset.loadFromRawLogFile(rawlog_file))
        throw runtime_error("\nCouldn't open rawlog dataset file for input...");

//        // Set external images directory:
//        const string imgsPath = CRawlog::detectImagesDirectory(filename);
//        CImage::IMAGES_PATH_BASE = imgsPath;

    cout << "dataset size " << dataset.size() << "\n";

    // Observation parameters
    bool first_frame = true;
    mrpt::obs::CObservation3DRangeScanPtr obsRGBD[2];  // The RGBD observation
    mrpt::vision::CUndistortMap undist_rgb;
    mrpt::vision::CUndistortMap undist_depth;
    undist_rgb.setFromCamParams(intrinsics.rightCamera);
    undist_depth.setFromCamParams(intrinsics.leftCamera);

    //==============================================================================
    //									Main operation
    //==============================================================================
    size_t n_obs = 0, n_frame_sync = 0;
    CObservationPtr observation;
    while( !b_exit && n_obs < dataset.size() )
    {
        observation = dataset.getAsObservation(n_obs);
        ++n_obs;
        if(verbose)
            cout << n_obs << " observation: " << observation->sensorLabel << endl; //<< ". Timestamp " << timestampTotime_t(observation->timestamp) << endl;

        if(!IS_CLASS(observation, CObservation3DRangeScan))
            continue;

        size_t sensor_id = 10e6; // Initialize with a non-valid sensor index
        for(size_t i=0; i < num_sensors; i++) // Identify the sensor which captured the current obs
            if(observation->sensorLabel == sensor_labels[i])
            {
                sensor_id = i;
                break;
            }
        if(sensor_id == 10e6) // This RGBD sensor is not taken into account for calibration
            continue;

        obsRGBD[0] = CObservation3DRangeScanPtr(observation);
        obsRGBD->load();


        // Get the rgb and depth images, and the point clouds
        {
            boost::mutex::scoped_lock updateLock(visualizationMutex);
//            rgb] = cv::Mat(obsRGBD]->intensityImage.getAs<IplImage>());
//            convertRange_mrpt2cvMat(obsRGBD]->rangeImage, depth]);
            // Image rectification (to reduce radial distortion)
            cv::Mat src(obsRGBD->intensityImage.getAs<IplImage>());
            undist_rgb.undistort(src, rgb[0]);
            cv::Mat raw_depth;
            convertRange_mrpt2cvMat(obsRGBD->rangeImage, raw_depth);
            undist_depth.undistort(raw_depth, depth[0]);
//            cloud = getPointCloudRegistered<PointT>(rgb, depth, intrinsics, depth_reg);
//            pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//            viewer.showCloud(cloud]);
//            while (!viewer.wasStopped ())
//                boost::this_thread::sleep (boost::posix_time::milliseconds (10));
            updateLock.unlock();
        }
        b_freeze = false; // Update 3D visualization

        // Display color and depth images
        displayObservation();

        //						Segment features
        //==================================================================
        //PbMap::pbMapFromPCloud(cloud], v_pbmap], th_dist_plane, th_angle_plane, min_inliers);
        CFeatureLines featLines;
        featLines.extractLines(rgb[sensor_id], vv_segments2D[sensor_id], min_pixels_line); //, true);
        cout << sensor_id << " lines " << vv_segments2D[sensor_id].size() << endl;
        ExtrinsicCalibLines::getSegments3D(intrinsics[sensor_id].rightCamera, cloud[sensor_id], v_pbmap[sensor_id], vv_segments2D[sensor_id], vv_segment_n[sensor_id], vv_segments3D[sensor_id], vv_line_has3D[sensor_id]);

        //==============================================================================
        //								Get Correspondences
        //==============================================================================
        cout << "Get Correspondences\n";
        b_freeze = false; // Update 3D visualization
        getCorrespondences(); // Both planes and lines according to s_type strategy

        if(first_frame)
            obsRGBD_prev = obsRGBD;

        b_show_corresp = true;
        b_freeze = false; // Update 3D visualization
        b_pause = true;
        while(b_pause)
            boost::this_thread::sleep (boost::posix_time::milliseconds (50));

        // Compute the calibration if there are enough measurements
        Matrix<T,3,3> R_planes = CalibrateRotationPair();
        Matrix<T,3,3> R_lines_n = ApproximateRotationZeroTrans();
        cout << "ROTATION diff " << RAD2DEG(acos( (trace<T,3>(R_planes * R_lines_n.transpose()) - 1) / 2)) << endl;
        cout << "ROTATION ERROR " << ExtrinsicCalibPlanes::calcRotationErrorPair(planes.mm_corresp[0][1], R_planes, true) << " lines " << ExtrinsicCalibLines::calcRotationErrorPair(lines.mm_corresp[0][1], R_planes, true) << endl;
        cout << "ROTATION ERROR " << ExtrinsicCalibPlanes::calcRotationErrorPair(planes.mm_corresp[0][1], R_lines_n, true) << " lines " << ExtrinsicCalibLines::calcRotationErrorPair(lines.mm_corresp[0][1], R_lines_n, true) << endl;
    }

    //========================== Perform calibration ===============================
    bool save_corresp = true;
    calibrate(save_corresp);
}

void CameraRotTracker::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
    //cout << "ExtrinsicRgbdCalibration::viz_cb(...) " << b_freeze << "\n";
    if(!b_viz_init)
    {
        viz.setSize(1280,960); // Set the window size
        viz.setPosition(1100,740);
        viz.setCameraPosition(0,-2,-3,1,0,0);
        //viz.setCameraPosition(0,0,-5,0,-0.707107,0.707107,1,0,0);
        b_viz_init = true;
    }
    if( b_freeze || cloud.empty() || !cloud[0] || cloud[0]->empty() )
    {
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
        return;
    }
    //cout << "   ::viz_cb(...) Update visualization \n";

    viz.removeAllShapes();
    viz.removeAllPointClouds();
    //viz.removeCoordinateSystem();

    { //mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
        boost::mutex::scoped_lock updateLock(visualizationMutex);

        char name[1024];
        Eigen::Affine3f Rt;

        sprintf (name, "%zu pts. Params ...", cloud[1]->size());
        viz.addText (name, 20, 20, "params");
        viz.addCoordinateSystem(0.1, Eigen::Affine3f::Identity());

        // Draw camera systems and point clouds
        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
            if (!viz.updatePointCloud (cloud], sensor_labels]))
            {
                Rt.matrix() = Rt_estimated.cast<float>();
                viz.addCoordinateSystem(0.2, Rt);
                viz.addPointCloud (cloud], sensor_labels]);
                viz.updatePointCloudPose(sensor_labels], Rt);
            }

        // Confirm match candidates
        if(b_confirm_visually)
        {
            if(b_wait_plane_confirm)
            {
                //cout << "Draw a pair of candidate planes to match \n";
                // Draw a pair of candidate planes to match
                for(size_t i=0; i < 2; i++)
                {
                    // Draw camera system
                    sprintf (name, "ref_%lu", i);
                    Rt.matrix() = Rt_estimated[sensor_pair[i].cast<float>();
                    viz.removeCoordinateSystem();
                    viz.addCoordinateSystem(0.05, Rt);

                    mrpt::pbmap::Plane &plane = all_planes.vPlanes[plane_candidate_all[i]];
                    sprintf (name, "m_normal_%lu", i);
                    pcl::PointXYZ pt1(plane.v3center[0], plane.v3center[1], plane.v3center[2]); // Begin and end points of normal's arrow for visualization
                    pcl::PointXYZ pt2(plane.v3center[0] + (0.2f*plane.v3normal[0]), plane.v3center[1] + (0.2f*plane.v3normal[1]), plane.v3center[2] + (0.2f*plane.v3normal[2]));
                    viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);
                    sprintf (name, "m_plane_contour_%lu", i);
                    viz.addPolygon<PointT> (plane.polygonContourPtr, red[0], grn[0], blu[0], name);
                    sprintf (name, "m_inliers_%lu", i);
                    pcl::PointCloud<PointT>::Ptr planeCloudColoured = colourPointCloud<PointT>(plane.planePointCloudPtr, red[0], grn[0], blu[0], 0.5f);
                    viz.addPointCloud (planeCloudColoured, name);
                    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
                    //pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane.planePointCloudPtr, red[0], grn[0], blu[0]);
                    //viz.addPointCloud (plane.planePointCloudPtr, color, name);
                }
                while(confirm_corresp == 0){
                    viz.spinOnce(50);
                    boost::this_thread::sleep (boost::posix_time::milliseconds (50));
                }

                updateLock.unlock();
                return;
            }

            //        if(b_wait_line_confirm)
            //        {
            //            // Draw a pair of candidate lines to match
            //            for(size_t i=0; i < 2; i++)
            //            {
            ////                if(vv_line_has3D[sensor_pair[i]][line_candidate[i]])
            //                {
            ////                    vector<mrpt::math::TLine3D> & seg3D = vv_segments3D[sensor_pair[i]][line_candidate[i]];
            //                    Eigen::Matrix<T,6,1> & seg3D = vv_segments3D[sensor_pair[i]][line_candidate[i]];

            //                    pcl::PointXYZ pt1(seg3D[0], seg3D[1], seg3D[2]);
            //                    pcl::PointXYZ pt2(seg3D[3], seg3D[4], seg3D[5]);
            //                    sprintf (name, "m_line_%lu_%u", sensor_pair[i], static_cast<unsigned>(i));
            //                    viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], name);
            //    //                viz.removeShape("line");
            //    //                viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");

            //                    viz.removeShape("pt1");
            //                    viz.addSphere<pcl::PointXYZ>(pt1, 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp1");
            //                    viz.removeShape("pt2");
            //                    viz.addSphere<pcl::PointXYZ>(pt2, 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp2");

            //        //                size_t p1 = vv_segments2D[sensor1][i][0] + vv_segments2D[sensor1][i][1] * cloud[sensor1]->width;
            //                    size_t p1 = line_match1[0] + line_match1[1] * cloud[sensor1]->width;
            //                    if(cloud[sensor1]->points[p1.z > 0.3f && cloud[sensor1]->points[p1.z < 10.f)
            //                    {
            //                        viz.removeShape("sp1");
            //                        viz.addSphere<PointT>(cloud[sensor1]->points[p1], 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp1");
            //        //                    pcl::ModelCoefficients circle_coeff;
            //        //                    circle_coeff.values.resize(3);    // We need 3 values
            //        //                    circle_coeff.values[0] = x;
            //        //                    circle_coeff.values[1] = y;
            //        //                    circle_coeff.values[2] = radius;
            //        //                    viz.addCircle<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");
            //                    }
            //                }
            //            }
            //            while(confirm_corresp == 0)
            //                boost::this_thread::sleep (boost::posix_time::milliseconds (10));

            //            size_t sensor2 = 1;
            //            Rt.matrix() = Rt_estimated[sensor2];
            //            size_t p1 = line_match2[0] + line_match2[1] * cloud[sensor2]->width;
            //            if(cloud[sensor2]->points[p1.z > 0.3f && cloud[sensor2]->points[p1.z < 10.f)
            //            {
            //                viz.removeShape("l2_sp1");
            //                viz.addSphere<PointT>(cloud[sensor2]->points[p1], 0.02, ared[1], agrn[1], ablu[1], "l2_sp1");
            //                viz.updateShapePose("l2_sp1", Rt);
            //            }
            //            size_t p2 = line_match2[2] + line_match2[3] * cloud[sensor2]->width;
            //            if(cloud[sensor2]->points[p2.z > 0.3f && cloud[sensor2]->points[p2.z < 10.f)
            //            {
            //                viz.removeShape("l2_sp2");
            //                viz.addSphere<PointT>(cloud[sensor2]->points[p2], 0.02, ared[1], agrn[1], ablu[1], "l2_sp2");
            //                viz.updateShapePose("l2_sp2", Rt);
            //            }
            //            return;
            //        }
        }

        if(b_show_corresp)
        {
//            cout << "   ::viz_cb(...) Draw correspondences \n";
//            // Draw planes
//            for(size_t i=0; i < all_planes.vPlanes.size(); i++)
//            {
//                //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                //              if(it->first == i)
//                //              {
//                mrpt::pbmap::Plane &plane = all_planes.vPlanes[i];
//                sprintf (name, "normal_%u", static_cast<unsigned>(i));
//                pcl::PointXYZ pt1(plane.v3center[0], plane.v3center[1], plane.v3center[2]); // Begin and end points of normal's arrow for visualization
//                pcl::PointXYZ pt2(plane.v3center[0] + (0.2f*plane.v3normal[0]), plane.v3center[1] + (0.2f*plane.v3normal[1]), plane.v3center[2] + (0.2f*plane.v3normal[2]));
//                viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

//                sprintf (name, "plane_contour_%02d", int (i));
//                // viz.addPolygon<PointT> (plane.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
//                viz.addPolygon<PointT> (plane.polygonContourPtr, red[i%10], grn[i%10], blu[i%10], name);

//            }
            // Draw plane matches
            set<size_t> drawn_planes;
            string matched_planes = "Matched planes:\n";
            for(map< size_t, map< size_t, map<size_t, size_t> > >::iterator it1=mmm_plane_matches_all.begin(); it1 != mmm_plane_matches_all.end(); it1++)
                for(map< size_t, map<size_t, size_t> >::iterator it2=it1->second.begin(); it2 != it1->second.end(); it2++)
                    for(map<size_t, size_t>::iterator it3=it2->second.begin(); it3 != it2->second.end(); it3++)
                    {
                        matched_planes += to_string(it3->first) + " - " + to_string(it3->second);
                        size_t col = it3->first%10; // Colour index
                        array<size_t,2> idx = {it3->first, it3->second};
                        for(size_t i=0; i < 2; i++)
                        {
                            if(drawn_planes.count(idx[i]))
                                continue;
                            drawn_planes.insert(idx[i]);
                            mrpt::pbmap::Plane &plane = all_planes.vPlanes[idx[i]];
                            sprintf (name, "normal_%lu", idx[i]);
                            pcl::PointXYZ pt1(plane.v3center[0], plane.v3center[1], plane.v3center[2]);
                            pcl::PointXYZ pt2(plane.v3center[0] + (0.2f*plane.v3normal[0]), plane.v3center[1] + (0.2f*plane.v3normal[1]), plane.v3center[2] + (0.2f*plane.v3normal[2]));
                            viz.addArrow (pt2, pt1, ared[col], agrn[col], ablu[col], false, name);

                            sprintf (name, "plane_contour_%lu", idx[i]);
                            viz.addPolygon<PointT> (plane.polygonContourPtr, red[col], grn[col], blu[col], name);

                            sprintf (name, "inliers_%lu", idx[i]);
                            pcl::PointCloud<PointT>::Ptr planeCloudColoured = colourPointCloud<PointT>(plane.planePointCloudPtr, red[col], grn[col], blu[col], 0.5f);
                            viz.addPointCloud (planeCloudColoured, name);
                        }
                    }
            viz.addText (matched_planes.c_str(), 220, 20, "matched_planes");

//            // Draw lines
//            for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
//            {
//                for(size_t i=0; i < vv_segments3D.size(); i++)
//                    //for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
//                {
//                    //vector<mrpt::math::TLine3D> seg3D = vv_segments3D];
//                    Matrix<T,3,1> seg3D_1(Rt_estimated.block<3,3>(0,0)*vv_segments3D][i.block<3,1>(0,0) + Rt_estimated.block<3,1>(0,3));
//                    Matrix<T,3,1> seg3D_2(Rt_estimated.block<3,3>(0,0)*vv_segments3D][i.block<3,1>(3,0) + Rt_estimated.block<3,1>(0,3));
//                    pcl::PointXYZ pt1(seg3D_1[0], seg3D_1[1], seg3D_1[2]);
//                    pcl::PointXYZ pt2(seg3D_2[0], seg3D_2[1], seg3D_2[2]);
//                    sprintf (name, "line_%lu_%lu", sensor_id, i);
//                    viz.removeShape(name);
//                    viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], name);
//                }
//            }
            // Draw line matches
            //cout << "   ::viz_cb(...) Draw line matches \n";
            vector< set<size_t> > drawn_lines(num_sensors);
            string matched_lines = "Matched lines:\n";
            for(map< size_t, map< size_t, map<size_t, size_t> > >::iterator it1=mmm_line_matches.begin(); it1 != mmm_line_matches.end(); it1++)
                for(map< size_t, map<size_t, size_t> >::iterator it2=it1->second.begin(); it2 != it1->second.end(); it2++)
                    for(map<size_t, size_t>::iterator it3=it2->second.begin(); it3 != it2->second.end(); it3++)
                    {
                        matched_lines += to_string(it1->first) + "." + to_string(it3->first) + " - " + to_string(it2->first) + "." + to_string(it3->second);
                        size_t col = it3->first%10; // Colour index
                        array<size_t,2> idx = {it3->first, it3->second};
                        array<size_t,2> sensor = {it1->first, it2->first};
                        for(size_t i=0; i < 2; i++)
                        {
                            //cout << i << " line " << vv_segments3D[sensor[i]][idx[i].transpose() << endl;
                            if(drawn_lines[sensor[i].count(idx[i]))
                                continue;
                            drawn_lines[sensor[i].insert(idx[i]);

                            Matrix<T,3,1> seg3D_1(Rt_estimated[sensor[i].block<3,3>(0,0)*vv_segments3D[sensor[i]][idx[i].block<3,1>(0,0) + Rt_estimated[sensor[i].block<3,1>(0,3));
                            Matrix<T,3,1> seg3D_2(Rt_estimated[sensor[i].block<3,3>(0,0)*vv_segments3D[sensor[i]][idx[i].block<3,1>(3,0) + Rt_estimated[sensor[i].block<3,1>(0,3));
                            pcl::PointXYZ pt1(seg3D_1[0], seg3D_1[1], seg3D_1[2]);
                            pcl::PointXYZ pt2(seg3D_2[0], seg3D_2[1], seg3D_2[2]);
                            sprintf (name, "line_%lu_%lu", sensor[i], idx[i]);
                            viz.removeShape(name);
                            viz.addLine<pcl::PointXYZ>(pt1, pt2, red[col], grn[col], blu[col], name);
                            sprintf (name, "sp1_%lu_%lu", sensor[i], idx[i]);
                            viz.addSphere<pcl::PointXYZ>(pt1, 0.01, red[col], grn[col], blu[col], name);
                            sprintf (name, "sp2_%lu_%lu", sensor[i], idx[i]);
                            viz.addSphere<pcl::PointXYZ>(pt2, 0.01, red[col], grn[col], blu[col], name);

                            // Draw virtual plane
                            sprintf (name, "virtual_plane_%lu_%lu", sensor[i], idx[i]);
                            pcl::PointCloud<pcl::PointXYZ>::Ptr triangle(new pcl::PointCloud<pcl::PointXYZ>());
                            triangle->points.resize(3);
                            triangle->points[0.getVector3fMap() = Rt_estimated[sensor[i].block<3,1>(0,3).cast<float>(); // Set the optical center of the camera
                            triangle->points[1] = pt1;
                            triangle->points[2] = pt2;
                            viz.addPolygon<pcl::PointXYZ> (triangle, red[col], grn[col], blu[col], name);

//                            pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
//                            plane->values.resize(4);
//                            plane->values[0] = vv_segment_n[sensor[i]][idx[i]][0];
//                            plane->values[1] = vv_segment_n[sensor[i]][idx[i]][1];
//                            plane->values[2] = vv_segment_n[sensor[i]][idx[i]][2];
//                            plane->values[3] = 0;
//                            viz.addPlane (*plane, name, 0);
//                            viz.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,  ared[col], agrn[col], ablu[col], name, 0);
//                            viz.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, name, 0);
//                            //viz.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name, 0);

//                            pcl::PointXYZ pt1_n(triangle->points[0]); // Begin and end points of normal's arrow for visualization
//                            pt1_n.getVector3fMap() += triangle->points[1.getVector3fMap() + triangle->points[2.getVector3fMap(); pt1_n.getVector3fMap() /= 3.f;
                            pcl::PointXYZ pt2_n(pt1); pt2_n.getVector3fMap() += 0.1f*(Rt_estimated[sensor[i].block<3,3>(0,0)*vv_segment_n[sensor[i]][idx[i]]).cast<float>();
                            sprintf (name, "seg_n_%lu_%lu", sensor[i], idx[i]);
                            viz.addArrow (pt2_n, pt1, ared[col], agrn[col], ablu[col], false, name);
                        }
                    }
            viz.addText (matched_lines.c_str(), 420, 20, "matched_lines");
            b_show_corresp = false;
        }

#if RECORD_VIDEO
        string screenshotFile = mrpt::format("im_%04u.png", ++numScreenshot);
        viz.saveScreenshot(screenshotFile);
#endif
        b_freeze = true;
        updateLock.unlock();
    }
}

void CameraRotTracker::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if ( event.keyDown () )
    {
        cout << "CameraRotTracker::keyboardEventOccurred : " << event.getKeySym () << endl;
        if(event.getKeySym () == "e" || event.getKeySym () == "E")
            b_exit = true;
        else if(event.getKeySym () == "k" || event.getKeySym () == "K"){
            confirm_corresp = 1;
        }
        else if(event.getKeySym () == "l" || event.getKeySym () == "L"){
            confirm_corresp = -1;
//            b_freeze = !b_freeze;
        }
        else if(event.getKeySym () == "Return"){
            cout << "PAUSE\n";
            b_pause = !b_pause;
        }
    }
}
*/
