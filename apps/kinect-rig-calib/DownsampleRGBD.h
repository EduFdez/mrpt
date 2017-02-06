/*
 *  Copyright (c) 2012, Universidad de Málaga - Grupo MAPIR
 *
 *  All rights reserved.
 *
 *  Redistribution and use in input_img and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of input_img code must retain the above copyright
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

#ifndef DOWNSAMPLERGBD_H
#define DOWNSAMPLERGBD_H

#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/filters/filter.h>
//#include <pcl/filters/fast_bilateral.h>
//#include <pcl/filters/bilateral.h>

#define ENABLE_OPENMP_MULTITHREADING 0

class DownsampleRGBD
{

private:

    int downsamplingStep;
    double minDepth, maxDepth;
    bool pointCloudAvailable;
    bool filteredCloudAvailable;
    bool intensityImageAvailable;
    bool rgbImageAvailable;
    bool depthImageAvailable;

    /*! RGB image */
    cv::Mat m_rgbImage;

    /*! Intensity image (grayscale version of the RGB image) */
    cv::Mat m_intensityImage;

    /*! Depth image */
    cv::Mat m_depthImage;

    /*! Downsampled point cloud */
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr mDownsampledPointCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_DownsampledPointCloudPtr;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_FilteredPointCloudPtr;

public:

    /*! Constructor */
    DownsampleRGBD(int step = 2) :
        downsamplingStep(step),
        minDepth(0.3),
        maxDepth(5.0)
    {
        //std::cout << "DownsampleRGBD::Ctor...\n";
        pointCloudAvailable=false;
        pointCloudAvailable=false;
        intensityImageAvailable=false;
        rgbImageAvailable=false;
        depthImageAvailable=false;
    }

    /*! Destructor */
    virtual ~DownsampleRGBD()
    {};

    /*! Set the valid depth range (minD, maxD) of a range image. Values out of this range are considered outliers for downsampling.
     * \param step is the downsamplingStep.
     */
    inline void setDownsamplingStep(const int step){downsamplingStep = step;};

    /*! Set the valid depth range (minD, maxD) of a range image. Values out of this range are considered outliers for downsampling.
     * \param minD the minimum valid depth.
     * \param maxD the maximum valid depth.
     */
    inline void setMinMaxDepth(const int minD, const int maxD){minDepth = minD; maxDepth = maxD;};

    /*! Downsample an opencv image according to the parameter \a <downsamplingStep>. This method keeps only one point per each square of
     * \a <downsamplingStep>* \a <downsamplingStep> pixels.
     * \param input_img is the image to downsample.
     * \param step if provided, it updates the value of the parameter \a <downsamplingStep>.
     * \return the downsampled point cloud
     */
    cv::Mat downsampleDepth(cv::Mat &input_img, const int step = -1)
    {
        if(step != -1)
            downsamplingStep = step;

        if(!depthImageAvailable)
        {
            if(downsamplingStep % 2 != 0 || input_img.cols % downsamplingStep != 0 || input_img.rows % downsamplingStep != 0)
                throw std::runtime_error("\nERROR... downsampleDepth \n\n");

            IplImage aux(input_img);
            IplImage *input_img_ = &aux;

            // declare a destination IplImage object with correct size, depth and channels
            IplImage *destination = cvCreateImage( cvSize((int)(input_img_->width/downsamplingStep), (int)(input_img_->height/downsamplingStep) ), input_img_->depth, input_img_->nChannels );

            //use cvResize to resize input_img_ to a destination image
            cvResize(input_img_, destination);

            //m_depthImage = cv::Mat(destination);
            m_depthImage = cv::cvarrToMat(destination);

            depthImageAvailable = true;
        }

        return m_depthImage;
    }

    /*! Downsample an opencv image according to the parameter \a <downsamplingStep>. This method keeps only one point per each square of
     * \a <downsamplingStep>* \a <downsamplingStep> pixels.
     * \param input_img is the input image to downsample.
     * \param step If provided, it updates the value of the parameter \a <downsamplingStep>.
     * \return The downsampled point cloud
     */
    cv::Mat downsampleIntensity(cv::Mat &input_img)
    {
        if(!intensityImageAvailable)
        {
            if(input_img.cols % downsamplingStep != 0 || input_img.rows % downsamplingStep != 0)
                throw std::runtime_error("\nERROR... downsampleIntensity \n\n");

            IplImage aux(input_img);
            IplImage *input_img_ = &aux;

            // declare a destination IplImage object with correct size, depth and channels
            IplImage *destination = cvCreateImage( cvSize((int)(input_img_->width/downsamplingStep), (int)(input_img_->height/downsamplingStep) ), input_img_->depth, input_img_->nChannels );

            //use cvResize to resize input_img_ to a destination image
            cvResize(input_img_, destination);

            //m_intensityImage = cv::Mat(destination);
            m_depthImage = cv::cvarrToMat(destination);
        }

        return m_intensityImage;
    }

    /*! Downsample an opencv image according to the parameter \a <downsamplingStep>. This method keeps only one point per each square of
     * \a <downsamplingStep>* \a <downsamplingStep> pixels.
     * \param input_img is the input image to downsample.
     * \param step If provided, it updates the value of the parameter \a <downsamplingStep>.
     * \return The downsampled point cloud
     */
    inline cv::Mat downsampleRGB(cv::Mat &input_img)
    //cv::Mat DownsampleRGBD::buildPyramid(cv::Mat & img,std::vector<cv::Mat>& pyramid,int levels,bool applyBlur)
    {
        if(!rgbImageAvailable)
        {
            if(input_img.cols % downsamplingStep != 0 || input_img.rows % downsamplingStep != 0)
                throw std::runtime_error("\nERROR... downsampleRGB \n\n");

            IplImage aux(input_img);
            IplImage *input_img_ = &aux;

            // declare a destination IplImage object with correct size, depth and channels
            IplImage *destination = cvCreateImage( cvSize((int)(input_img_->width/downsamplingStep), (int)(input_img_->height/downsamplingStep) ), input_img_->depth, input_img_->nChannels );

            //use cvResize to resize input_img_ to a destination image
            cvResize(input_img_, destination);

            //m_rgbImage = cv::Mat(destination);
            m_rgbImage = cv::cvarrToMat(destination);

            //    pyrDown(InputArray src, OutputArray dst, const Size& dstsize=Size(), int borderType=BORDER_DEFAULT )
            //
            //    //Create space for all the images
            //    pyramid.resize(levels);
            //
            //    double factor = 1;
            //    for(int level=0;level<levels;level++)
            //    {
            //        //Create an auxiliar image of factor times the size of the original image
            //        cv::Mat imgAux;
            //        if(level!=0)
            //        {
            //            cv::resize(img,imgAux,cv::Size(0,0),factor,factor);
            //        }
            //        else
            //        {
            //            imgAux = img;
            //        }
            //
            //        //Blur the resized image with different filter size depending on the current pyramid level
            //        if(applyBlur)
            //        {
            //            #if ENABLE_GAUSSIAN_BLUR
            //            if(blurFilterSize[level]>0)
            //            {
            //                cv::GaussianBlur(imgAux,imgAux,cv::Size(blurFilterSize[level],blurFilterSize[level]),3);
            //                cv::GaussianBlur(imgAux,imgAux,cv::Size(blurFilterSize[level],blurFilterSize[level]),3);
            //            }
            //            #elif ENABLE_BOX_FILTER_BLUR
            //            if(blurFilterSize[level]>0)
            //            {
            //                cv::blur(imgAux,imgAux,cv::Size(blurFilterSize[level],blurFilterSize[level]));
            //                cv::blur(imgAux,imgAux,cv::Size(blurFilterSize[level],blurFilterSize[level]));
            //            }
            //            #endif
            //        }
            //
            //        //Assign the resized image to the current level of the pyramid
            //        pyramid[level]=imgAux;
            //
            //        factor = factor/2;
            //    }
            rgbImageAvailable = true;
        }

        return m_rgbImage;
    }

    /*! Downsample an organized point cloud according to the parameter \a <downsamplingStep>. This method keeps only one point per each square of
     * \a <downsamplingStep>* \a <downsamplingStep> pixels.
     * \param input_cloud is the input point cloud to downsample.
     * \param step If provided, it updates the value of the parameter \a <downsamplingStep>.
     * \return The downsampled point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsamplePointCloud( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud)
    {
        //std::cout << "DownsampleRGBD::downsamplePointCloud...\n";

        if(!pointCloudAvailable)
        {
            //  std::cout << "Downsampling\n";
            m_DownsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
            m_DownsampledPointCloudPtr->points.resize(input_cloud->size()/(downsamplingStep*downsamplingStep));
            m_DownsampledPointCloudPtr->width = input_cloud->width/downsamplingStep;
            m_DownsampledPointCloudPtr->height = input_cloud->height/downsamplingStep;
            m_DownsampledPointCloudPtr->is_dense = false;

            static int j;j=0;
            std::vector<double> xV;
            std::vector<double> yV;
            std::vector<double> zV;
            //  std::cout << "Downsampling1\n";

#if ENABLE_OPENMP_MULTITHREADING
#pragma omp parallel for
#endif
            for(unsigned r=0;r<input_cloud->height;r=r+downsamplingStep)
            {
                for(unsigned c=0;c<input_cloud->width;c=c+downsamplingStep)
                {
                    unsigned nPoints=0;
                    xV.resize(downsamplingStep*downsamplingStep);
                    yV.resize(downsamplingStep*downsamplingStep);
                    zV.resize(downsamplingStep*downsamplingStep);

                    unsigned centerPatch = (r+downsamplingStep/2)*input_cloud->width+c+downsamplingStep/2;

                    for(unsigned r2=r;r2<r+downsamplingStep;r2++)
                    {
                        for(unsigned c2=c;c2<c+downsamplingStep;c2++)
                        {
                            //Check if the point has valid data
                            if(pcl_isfinite (input_cloud->points[r2*input_cloud->width+c2].x) &&
                                    //                       pcl_isfinite (input_cloud->points[r2*input_cloud->width+c2].y) &&
                                    //                       pcl_isfinite (input_cloud->points[r2*input_cloud->width+c2].z) &&
                                    minDepth < input_cloud->points[r2*input_cloud->width+c2].z &&
                                    input_cloud->points[r2*input_cloud->width+c2].z < maxDepth)
                            {
                                //Create a vector with the x, y and z coordinates of the square region
                                xV[nPoints]=input_cloud->points[r2*input_cloud->width+c2].x;
                                yV[nPoints]=input_cloud->points[r2*input_cloud->width+c2].y;
                                zV[nPoints]=input_cloud->points[r2*input_cloud->width+c2].z;

                                nPoints++;
                            }
                        }
                    }

                    //Check if there are points in the region
                    if(nPoints>0)
                    {
                        xV.resize(nPoints);
                        yV.resize(nPoints);
                        zV.resize(nPoints);

                        //Compute the mean 3D point
                        std::sort(xV.begin(),xV.end());
                        std::sort(yV.begin(),yV.end());
                        std::sort(zV.begin(),zV.end());

                        pcl::PointXYZRGBA point;
                        point.x=xV[nPoints/2];
                        point.y=yV[nPoints/2];
                        point.z=zV[nPoints/2];

                        point.r = input_cloud->points[centerPatch].r;
                        point.g = input_cloud->points[centerPatch].g;
                        point.b = input_cloud->points[centerPatch].b;

                        //Set the mean point as the representative point of the region
#if ENABLE_OPENMP_MULTITHREADING
#pragma omp critical
#endif
                        {
                            m_DownsampledPointCloudPtr->points[j]=point;
                            j++;
                        }
                    }
                    else
                    {
                        //Set a nan point to keep the m_DownsampledPointCloudPtr organised
#if ENABLE_OPENMP_MULTITHREADING
#pragma omp critical
#endif
                        {
                            m_DownsampledPointCloudPtr->points[j] = input_cloud->points[centerPatch];
                            j++;
                        }
                    }
                }
            }

            pointCloudAvailable = true;
        }
        return m_DownsampledPointCloudPtr;
    }

};

#endif // DOWNSAMPLERGBD_H
