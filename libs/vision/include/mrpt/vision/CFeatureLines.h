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

            static const size_t band_contrast = 2;

            /*! This function calls the functions defined below according the the parameter "method" */
            void extractLines (const cv::Mat & image,
                                std::vector<cv::Vec4f> & segments,
                                const int method = 0,
                                const size_t th_length = 0,
                                const size_t max_lines = 20,
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

            /*! Compute the contrast of line segments by comparing the brightness of 2 parallel bands to the segment at a distance "band_dist" */
            void computeContrastOfSegments(const cv::Mat & image, std::vector<cv::Vec4f> & segments, std::vector<int> &v_contrast, const size_t band_dist = band_contrast, const bool display = false);

            /*! This function calls the functions defined below according the the parameter "method" */
            inline void extractLines(const cv::Mat          & image,
                                    std::vector<cv::Vec4f>  & segments,
                                    std::vector<float>      & v_length,
                                    std::vector<int>        & v_contrast,
                                    const int method = 0,
                                    const size_t th_length = 0,
                                    const size_t max_lines = 20,
                                    const bool display = false)
            {
                std::cout << "CFeatureLines::extractLines... display " << display << std::endl;

                extractLines (image, segments, method, th_length, max_lines, display);
                computeContrastOfSegments (image, segments, v_contrast, band_contrast, display);

                v_length.resize( segments.size() );
                for(size_t i=0; i < segments.size(); ++i)
                    v_length[i] = sqrt((segments[i][2]-segments[i][0])*(segments[i][2]-segments[i][0]) + (segments[i][3]-segments[i][1])*(segments[i][3]-segments[i][1]));
            }

            void computeContrastOfSegments(const cv::Mat & image, std::vector<cv::Vec6f> & segments, const size_t band_dist = band_contrast);

            /*! This function calls the functions defined below according the the parameter "method" */
            void extractLinesDesc ( const cv::Mat & image,
                                    std::vector<cv::Vec4f> & segments,
                                    std::vector<cv::Vec6f> & segmentsDesc,
                                    const int method = 0,
                                    const size_t th_length = 0,
                                    const size_t max_lines = 20,
                                    const bool display = false);

        }; // end of class

    } // end of namespace
} // end of namespace

#endif
