/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CFeatureLines_H
#define CFeatureLines_H

#include <mrpt/vision/utils.h>
//#include <mrpt/utils/CImage.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/eigen.hpp>
//#include <pcl/point_types.h>

namespace mrpt
{
    namespace vision
    {
        /**  This class wraps different line detectors and descriptors from OpenCV.
          *
          *  \ingroup mrpt_vision_grp
          */
        class VISION_IMPEXP CFeatureLines
        {
          public:

            /** Execution time (ms) */
            float time;

            /*! This function calls the functions defined below according the the parameter "method" */
            void extractLines ( const cv::Mat & image,
                                std::vector<cv::Vec4f> & segments,
                                size_t th_length,
                                const int method = 0,
                                const bool display = false);

            /*! Extract lines using OpenCV methods: LSD, or BinaryDescriptor */
            void extractLines_cv2 ( const cv::Mat & image,
                                    std::vector<cv::Vec4f> & segments,
                                    const float min_length = 10, const int method = 0, const bool display = false);

            /*! Extract lines by applying sequentially the methods of Canny, Hough and Bresenham */
            void extractLines_CHB ( const cv::Mat & image,
                                    std::vector<cv::Vec4f> & segments,
                                    size_t th_length , const bool display = false);

            void extractLines_CannyHough(const cv::Mat & canny_image,
                                         const std::vector<cv::Vec2f> lines,
                                         std::vector<cv::Vec4f> & segments,
                                         size_t th_length );
        }; // end of class

    } // end of namespace
} // end of namespace

#endif
