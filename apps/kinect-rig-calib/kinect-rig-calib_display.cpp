/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinect-rig-calib_display.h"
#include <mrpt/pbmap/colors.h>
#include <boost/thread/mutex.hpp>

void KinectRigCalib_display::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
    //cout << "ExtrinsicRgbdCalibration::viz_cb(...)\n";
    if(!b_viz_init)
    {
        viz.setSize(1280,960); // Set the window size
        viz.setPosition(1100,740);
        viz.setCameraPosition(0,-2,-3,1,0,0);
        //viz.setCameraPosition(0,0,-5,0,-0.707107,0.707107,1,0,0);
        b_viz_init = true;
    }
    if( b_freeze || krc.cloud.empty() || !krc.cloud[0] || krc.cloud[0]->empty())
    {
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
        return;
    }
    //cout << "   ::viz_cb(...)\n";

    viz.removeAllShapes();
    viz.removeAllPointClouds();

    { //mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
        boost::mutex::scoped_lock krc.updateLock(visualizationMutex);

        char name[1024];
        Eigen::Affine3f Rt;

        sprintf (name, "%zu pts. Params ...", krccloud[1]->size());
        viz.addText (name, 20, 20, "params");

        // Draw camera system
        //Rt.matrix() = initOffset;
        Rt.matrix() = krc.Rt_estimated[1];
        viz.removeCoordinateSystem();
        viz.addCoordinateSystem(0.05, Rt);
        viz.addCoordinateSystem(0.1, Eigen::Affine3f::Identity());

        for(size_t sensor_id=0; sensor_id < krc.num_sensors; sensor_id++)
            if (!viz.updatePointCloud (krc.cloud[sensor_id], krc.sensor_labels[sensor_id]))
            {
                viz.addPointCloud (krc.cloud[sensor_id], krc.sensor_labels[sensor_id]);
                Rt.matrix() = krc.Rt_estimated[sensor_id];
                viz.updatePointCloudPose(krc.sensor_labels[sensor_id], Rt);
            }

        // Draw planes
        for(size_t i=0; i < krc.all_planes.vPlanes.size(); i++)
        {
            //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
            //              if(it->first == i)
            //              {
            mrpt::pbmap::Plane &plane_i = krc.all_planes.vPlanes[i];
            sprintf (name, "normal_%u", static_cast<unsigned>(i));
            pcl::PointXYZ pt1(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]); // Begin and end points of normal's arrow for visualization
            pcl::PointXYZ pt2(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]), plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]), plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
            //            viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
            viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

            sprintf (name, "approx_plane_%02d", int (i));
            //            viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
            viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);

            //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
            //              if(it->first == i)
            //              {
            //                mrpt::pbmap::Plane &plane_i = all_planes.vPlanes[i];
            //                sprintf (name, "normal_%u", static_cast<unsigned>(i));
            //                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
            //                pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
            //                pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
            //                                    plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
            //                                    plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
            //                viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
            //
            //                sprintf (name, "approx_plane_%02d", int (i));
            //                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);
            //
            ////                sprintf (name, "inliers_%02d", int (i));
            ////                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[0], grn[0], blu[0]);
            ////                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
            //              }
        }

        // Draw lines
        size_t sensor1 = 0;
        vector<mrpt::math::TLine3D> &lines = krc.v_lines3D[sensor1];
        for(size_t i=0; i < lines.size(); i++)
        {
            pcl::PointXYZ pt1(lines[i][0], lines[i][1], lines[i][2]);
            pcl::PointXYZ pt2(lines[i][3], lines[i][4], lines[i][5]);
            sprintf (name, "line_%lu_%u", sensor1, static_cast<unsigned>(i));
            //viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], name);
            viz.removeShape("line");
            viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");

//                size_t p1 = v_lines[sensor1][i][0] + v_lines[sensor1][i][1] * cloud[sensor1]->width;
            size_t p1 = krc.line_match1[0] + krc.line_match1[1] * krc.cloud[sensor1]->width;
            if(krc.cloud[sensor1]->points[p1].z > 0.3f && krc.cloud[sensor1]->points[p1].z < 10.f)
            {
                viz.removeShape("sp1");
                viz.addSphere<PointT>(krc.cloud[sensor1]->points[p1], 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp1");
//                    pcl::ModelCoefficients circle_coeff;
//                    circle_coeff.values.resize(3);    // We need 3 values
//                    circle_coeff.values[0] = x;
//                    circle_coeff.values[1] = y;
//                    circle_coeff.values[2] = radius;
//                    viz.addCircle<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");
            }
//                size_t p2 = v_lines[sensor1][i][2] + v_lines[sensor1][i][3] * cloud[sensor1]->width;
            size_t p2 = krc.line_match1[2] + krc.line_match1[3] * krc.cloud[sensor1]->width;
            if(krc.cloud[sensor1]->points[p2].z > 0.3f && krc.cloud[sensor1]->points[p2].z < 10.f)
            {
                viz.removeShape("sp2");
                viz.addSphere<PointT>(krc.cloud[sensor1]->points[p2], 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp2");
            }
        }

        size_t sensor2 = 1;
        Rt.matrix() = krc.Rt_estimated[sensor2];
        size_t p1 = krc.line_match2[0] + krc.line_match2[1] * krc.cloud[sensor2]->width;
        if(krc.cloud[sensor2]->points[p1].z > 0.3f && krc.cloud[sensor2]->points[p1].z < 10.f)
        {
            viz.removeShape("l2_sp1");
            viz.addSphere<PointT>(cloud[sensor2]->points[p1], 0.02, ared[1], agrn[1], ablu[1], "l2_sp1");
            viz.updateShapePose("l2_sp1", Rt);
        }
        size_t p2 = krc.line_match2[2] + krc.line_match2[3] * krc.cloud[sensor2]->width;
        if(krc.cloud[sensor2]->points[p2].z > 0.3f && krc.cloud[sensor2]->points[p2].z < 10.f)
        {
            viz.removeShape("l2_sp2");
            viz.addSphere<PointT>(cloud[sensor2]->points[p2], 0.02, ared[1], agrn[1], ablu[1], "l2_sp2");
            viz.updateShapePose("l2_sp2", Rt);
        }

//            //if(plane_corresp.size() > 0)
//            {
//                for(size_t i=0; i < v_pbmap[0].vPlanes.size(); i++)
//                {
//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->first == i)
//                    //              {
//                    mrpt::pbmap::Plane &plane_i = v_pbmap[0].vPlanes[i];
//                    sprintf (name, "normal_%u", static_cast<unsigned>(i));
//                    pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                            plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                            plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //            viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    viz.addArrow (pt2, pt1, ared[0], agrn[0], ablu[0], false, name);

//                    sprintf (name, "approx_plane_%02d", int (i));
//                    //            viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
//                    viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);

//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->first == i)
//                    //              {
//                    //                mrpt::pbmap::Plane &plane_i = v_pbmap[0].vPlanes[i];
//                    //                sprintf (name, "normal_%u", static_cast<unsigned>(i));
//                    //                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    //                pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    //                pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                    //                                    plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                    //                                    plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //                viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    //
//                    //                sprintf (name, "approx_plane_%02d", int (i));
//                    //                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);
//                    //
//                    ////                sprintf (name, "inliers_%02d", int (i));
//                    ////                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[0], grn[0], blu[0]);
//                    ////                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
//                    //              }
//                }
//                for(size_t i=0; i < v_pbmap[1].vPlanes.size(); i++)
//                {
//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->second == i)
//                    //              {
//                    //    cout << "   v_pbmap[1] " << i << "\n";

//                    mrpt::pbmap::Plane &plane_i = v_pbmap[1].vPlanes[i];
//                    sprintf (name, "normal_j_%u", static_cast<unsigned>(i));
//                    pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                            plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                            plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //            viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    viz.addArrow (pt2, pt1, ared[3], agrn[3], ablu[3], false, name);

//                    sprintf (name, "approx_plane_j_%02d", int (i));
//                    //            viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
//                    viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[3], grn[3], blu[3], name);

//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->second == i)
//                    //              {
//                    //                mrpt::pbmap::Plane &plane_i = v_pbmap[1].vPlanes[i];
//                    //                sprintf (name, "normal_j_%u", static_cast<unsigned>(i));
//                    //                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    //                pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    //                pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                    //                                    plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                    //                                    plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //                viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    //
//                    //                sprintf (name, "approx_plane_j_%02d", int (i));
//                    //                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[3], grn[3], blu[3], name);
//                    //
//                    ////                sprintf (name, "inliers_j_%02d", int (i));
//                    ////                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[3], grn[3], blu[3]);
//                    ////                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
//                    //              }
//                }
//          }

#if RECORD_VIDEO
        std::string screenshotFile = mrpt::format("im_%04u.png", ++numScreenshot);
        viz.saveScreenshot(screenshotFile);
#endif
        b_freeze = true;
        krc.updateLock.unlock();
    }
}

void KinectRigCalib_display::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if ( event.keyDown () )
    {
        if(event.getKeySym () == "e" || event.getKeySym () == "E")
            b_exit = true;
        else if(event.getKeySym () == "k" || event.getKeySym () == "K")
            b_confirm_visually = !b_confirm_visually;
        else if(event.getKeySym () == "l" || event.getKeySym () == "L"){
            b_freeze = !b_freeze;
            b_confirm_visually = !b_confirm_visually;
        }
    }
}

