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

#pragma once

#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/colors.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

namespace mrpt{
  namespace pbmap{

    /*! This class creates a visualizer to display a point cloud and a PbMap extracted from it. The visualizer runs in a separate thread,
     *  and can be sychronized using the vis_mutex_.
     *
     * \ingroup mrpt_pbmap_grp
     */
    class DisplayCloudPbMap
    {
    public:

        /*! Static function to display a 3D view of a RgbdFeat
         * \param frame to display
         * \param win_name window name
         */
        static inline void displayAndPause(const pcl::PointCloud<PointT>::Ptr & point_cloud, PbMap *pbmap = NULL)
        {
            DisplayCloudPbMap display(point_cloud, pbmap, "Cloud_and_PbMap");
            while( !display.viewer_.wasStopped() )
                boost::this_thread::sleep( boost::posix_time::milliseconds(10) );
        }

        /*! Visualizer object */
        pcl::visualization::CloudViewer viewer_;

        /*! Constructor. It starts the visualization in a separate thread */
        DisplayCloudPbMap(const pcl::PointCloud<PointT>::Ptr & point_cloud, PbMap *pbmap = NULL, std::string win_name = "RgbdFeat") :
            viewer_(win_name),
            cloud_(point_cloud),
            pbmap_(pbmap),
            b_init_viewer_(true),
//            show_normals_(true),
            show_planes_(true),
            color_planes_(true)
        {
            b_update_ = true;
            viewer_.runOnVisualizationThread(boost::bind(&DisplayCloudPbMap::viz_cb, this, _1), "viz_cb");
            viewer_.registerKeyboardCallback(&DisplayCloudPbMap::keyboardEventOccurred, *this);

            std::cout << "Initialize DisplayCloudPbMap visualizer \n";
            if(pbmap_)
                std::cout << "Features to display: " << pbmap_->vPlanes.size() << " planes, 0 lines and 0 points.\n";
        }

      private:

        /*! The point cloud to display. */
        pcl::PointCloud<PointT>::Ptr cloud_;

        /*! The PbMap cloud to display. */
        PbMap * pbmap_;

        /*! This variable controls some visualization settings. */
        bool b_init_viewer_;

        /*! This variable controls some visualization settings. */
        bool b_update_;

        /*! Show the normal cloud. */
//        bool show_normals_;

        /*! Show the PbMap's planes. It is control through keyboard event. */
        bool show_planes_;

        /*! Show the PbMap's planes filled with different colors. It is control through keyboard event. */
        bool color_planes_;

        /*! Visualization callback */
        void viz_cb(pcl::visualization::PCLVisualizer & viz)
        {
            if( cloud_->empty() )
            {
                boost::this_thread::sleep (boost::posix_time::milliseconds (10));
                return;
            }

            if(b_init_viewer_)
            {
                b_init_viewer_ = false;
                viz.setCameraPosition (
                                        0, -5, -8,		// Position
                                        0, 0.8, 1,		// Viewpoint
                                        0, -1,  0 );    // Up

                viz.setSize(800,600); // Set the window size
                viz.setBackgroundColor(0.5, 0.5, 0.5); //(1.0, 1.0, 1.0);
                viz.addCoordinateSystem(0.3, "global");
            }

            // Render the data
            if(b_update_)
            {
                //std::cout << "b_update_b_update_ " << endl;
                viz.removeAllPointClouds();
                viz.removeAllShapes();

                if(!viz.updatePointCloud (cloud_, "point_cloud"))
                    viz.addPointCloud (cloud_, "point_cloud");

//                if(show_normals_ && !frame_->normal_cloud_->empty())
//                {
//                    //viewer_->removePointCloud("normal_cloud", 0);
//                    viz.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud_, frame_->normal_cloud_, 10, 0.05, "normal_cloud");
//                }

                if(pbmap_)
                {
                    char name[1024];
                    size_t planes_inliers = 0;
                    for(size_t i=0; i < pbmap_->vPlanes.size(); i++)
                    {
                        planes_inliers += pbmap_->vPlanes[i].inliers.size();
                        if(pbmap_->vPlanes[i].inliers.size() != pbmap_->vPlanes[i].planePointCloudPtr->size())
                        {
                            std::cerr << "ERROR in DisplayCloudPbMap::viz_cb \n\n";
                            throw;
                        }
                    }
                    sprintf(name, "Frame %lu, planes inliers %lu", cloud_->size(), planes_inliers);
                    if(!viz.updateText(name, 20, 20, "info") )
                        viz.addText(name, 20, 20, "info");

                    if(show_planes_)
                    {
                        // Draw planes
                        for(size_t i=0; i < pbmap_->vPlanes.size(); i++)
                        {
                            mrpt::pbmap::Plane &plane_i = pbmap_->vPlanes[i];
                            sprintf (name, "normal_%u", static_cast<unsigned>(i));
                            pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
                            pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
                            pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.5f * plane_i.v3normal[0]),
                                    plane_i.v3center[1] + (0.5f * plane_i.v3normal[1]),
                                    plane_i.v3center[2] + (0.5f * plane_i.v3normal[2]));
                            viz.addArrow(pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

                            {
                                sprintf(name, "n%u %s", static_cast<unsigned>(i), plane_i.label.c_str());
                                //            sprintf (name, "n%u %.1f %.2f", static_cast<unsigned>(i), plane_i.curvature*1000, plane_i.areaHull);
                                viz.addText3D(name, pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
                            }

                            sprintf (name, "approx_plane_%02d", int (i));
                            viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[i%10], 0.5 * grn[i%10], 0.5 * blu[i%10], name);

                            if(color_planes_)
                            {
                                sprintf (name, "plane_%02u", static_cast<unsigned>(i));
                                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[i%10], grn[i%10], blu[i%10]);
                                if (!viz.updatePointCloud (plane_i.planePointCloudPtr, color, name))
                                    viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
                                viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
                            }

                        }
                        //b_update_ = false;
                    }
                }
            }

            viz.spinOnce();
            //viz.spin();
            boost::this_thread::sleep(boost::posix_time::milliseconds (10));

//    #if RECORD_VIDEO
//            std::string screenshotFile = mrpt::format("im_%04u.png", ++n_screenshot_);
//            viz.saveScreenshot (screenshotFile);
//    #endif
        }

        /*! Get events from the keyboard */
        void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
        {
            if ( event.keyDown() )
            {
                // std::cout << "Key pressed " << event.getKeySym () << endl;
                if(event.getKeySym() == "k" || event.getKeySym() == "K"){std::cout << " Press K: Show/hide planes\n";
                    show_planes_ = !show_planes_;
                    b_update_ = true;}
                else if(event.getKeySym() == "l" || event.getKeySym() == "L"){std::cout << " Press L: fill/unfill plane colors\n";
                    color_planes_ = !color_planes_;
                    b_update_ = true;}
                else if(event.getKeySym() == "n" || event.getKeySym() == "N"){std::cout << " Press L: show/hide normal directions";
//                    show_normals_ = !show_normals_;
                    b_update_ = true;}
            }
        }
    };

  }
}
