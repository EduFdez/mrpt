/*
 *  Copyright (c) 2015,   INRIA Sophia Antipolis - LAGADIC Team
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

#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <Eigen/Core>

//namespace sensor
//{
//    /*! This class encapsulates different projection models including both perspective and spherical.
//     *  It implements the functionality to project and reproject from the image domain to 3D and viceversa.
//     *
//     *  \ingroup sensor_group
//     *  \sa ProjectionModel
//     */
//    class PinholeModel
//    {
//      protected:

////        /*! Camera intrinsic parameters */
////        mrpt::utils::TCamera cam;

//      public:

//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//        /*!  Constructor */
//        PinholeModel(){};

//        /*! Destructor */
//        virtual ~PinholeModel() {}

        /*! Return the translation vector of the input pose
         *  \param[in] pose
         */
        template<typename Scalar> inline
        Eigen::Matrix<Scalar,3,1> getTranslationVector(const Eigen::Matrix<Scalar,4,4> pose)
        {
            Eigen::Matrix<Scalar,3,1> translation = pose.block(0,3,3,1);
            return  translation;
        }

        /*! Return the rotation vector of the input pose
         *  \param[in] pose
         */
        template<typename Scalar, size_t N> inline
        Eigen::Matrix<Scalar,3,1> getRotationVector_mrpt(const Eigen::Matrix<Scalar,N,N> & pose)
        {
            if(N != 3 and N!= 4)
                throw std::runtime_error("\n ERROR getRotationVector_mrpt: input matrix is not a rotation (3x3) neather a pose (4x4)");

            Eigen::Matrix<Scalar,3,1> rot_vec;
            mrpt::poses::CPose3D pose_mrpt;
            if(N == 4)
            {
                mrpt::math::CMatrixDouble44 mat_mrpt(pose);
                pose_mrpt = mrpt::poses::CPose3D(mat_mrpt);
            }
            else
                pose_mrpt.setRotationMatrix(pose);
            mrpt::math::CArrayDouble<3> vRot_mrpt = pose_mrpt.ln_rotation();
            rot_vec = Eigen::Matrix<Scalar,3,1>(vRot_mrpt[0],vRot_mrpt[1],vRot_mrpt[2]);

            return rot_vec;
        }


        /*! Return the rotation vector of the input pose. This code has been extracted from TooN: http://www.edwardrosten.com/cvd/toon.html
         *  \param[in] pose
         */
        template <typename Scalar, size_t N>
        inline Eigen::Matrix<Scalar,3,1> getRotationVector(const Eigen::Matrix<Scalar,N,N> & pose)
        {
            using std::sqrt;
            if(N != 3 and N!= 4)
                throw std::runtime_error("\n ERROR getRotationVector: input matrix is not a rotation (3x3) neather a pose (4x4)");


            Eigen::Matrix<Scalar,3,1> result;

            const Scalar cos_angle = (pose(0,0) + pose(1,1) + pose(2,2) - 1.0) * 0.5;
            result[0] = (pose(2,1)-pose(1,2))/2;
            result[1] = (pose(0,2)-pose(2,0))/2;
            result[2] = (pose(1,0)-pose(0,1))/2;

            Scalar sin_angle_abs = sqrt(result.dot(result));
            if (cos_angle > M_SQRT1_2) {            // [0 - Pi/4[ use asin
                if(sin_angle_abs > 0){
                    result *= asin(sin_angle_abs) / sin_angle_abs;
                }
            } else if( cos_angle > -M_SQRT1_2) {    // [Pi/4 - 3Pi/4[ use acos, but antisymmetric part
                const Scalar angle = acos(cos_angle);
                result *= angle / sin_angle_abs;
            } else {  // rest use symmetric part
                // antisymmetric part vanishes, but still large rotation, need information from symmetric part
                const Scalar angle = M_PI - asin(sin_angle_abs);
                const Scalar d0 = pose(0,0) - cos_angle,
                        d1 = pose(1,1) - cos_angle,
                        d2 = pose(2,2) - cos_angle;
                Eigen::Matrix<Scalar,3,1> r2;
                if(d0*d0 > d1*d1 && d0*d0 > d2*d2){ // first is largest, fill with first column
                    r2[0] = d0;
                    r2[1] = (pose(1,0)+pose(0,1))/2;
                    r2[2] = (pose(0,2)+pose(2,0))/2;
                } else if(d1*d1 > d2*d2) { 			    // second is largest, fill with second column
                    r2[0] = (pose(1,0)+pose(0,1))/2;
                    r2[1] = d1;
                    r2[2] = (pose(2,1)+pose(1,2))/2;
                } else {							    // third is largest, fill with third column
                    r2[0] = (pose(0,2)+pose(2,0))/2;
                    r2[1] = (pose(2,1)+pose(1,2))/2;
                    r2[2] = d2;
                }
                // flip, if we point in the wrong direction!
                if(r2.dot(result) < 0)
                    r2 *= -1;
                r2 = r2 / r2.norm();
                result = angle*r2;
            }
            return result;
        }

        void warp(const cv::Mat & gray, const cv::Mat & depth, const mrpt::utils::TCamera & cam, const Eigen::Matrix4f & Rt, cv::Mat & warped_gray, const float min_depth_ = 0.3f, const float max_depth_ = 10.f)
        {
            //std::cout << "PinholeModel::warp...\n";
            if( gray.empty() || depth.empty() || gray.rows != depth.rows || int(cam.nrows) != depth.rows || gray.cols != depth.cols || int(cam.ncols) != depth.cols)
            {
                cerr << "\n ERROR in ProjectionModel::warpRgbdMap " << gray.empty() << " " << depth.empty() << " \n";
                throw;
            }

            Eigen::VectorXi valid_pixels;
            Eigen::MatrixXf xyz, xyz_tf;//, warp_pixels;
            size_t img_size_ = cam.nrows*cam.ncols;
            size_t row_stride_ = depth.step / sizeof(depth.type());
            float inv_fx_ = 1 / cam.fx();
            float inv_fy_ = 1 / cam.fy();

            xyz.resize(img_size_,3);
            valid_pixels.resize(img_size_);
            int *_validPixel = &valid_pixels(0);
            float *_x = &xyz(0,0);
            float *_y = &xyz(0,1);
            float *_z = &xyz(0,2);

        //    #if _ENABLE_OPENMP
        //    #pragma omp parallel for
        //    #endif
            for(size_t r=0; r < cam.nrows; r++)
            {
                int row_pix = r*row_stride_;
                float *_depth = reinterpret_cast<float*>(depth.data) + row_pix;
                for(size_t c=0; c < cam.ncols; c++)
                {
                    //Compute the 3D coordinates of the pij of the source frame
                    //cout << depthSrcPyr[pyrLevel].type() << " xyz " << i << " x " << xyz(i,2) << " thres " << min_depth_ << " " << max_depth_ << endl;
                    if(min_depth_ < *_depth && *_depth < max_depth_) //Compute the jacobian only for the valid points
                    {
                        *(_validPixel++) = row_pix + c;
                        *_z = *_depth; //xyz(i,2) = 0.001f*depthSrcPyr[pyrLevel].at<unsigned short>(r,c);
                        *_x = (c - cam.cx()) * inv_fx_ * (*_depth);
                        *_y = (r - cam.cy()) * inv_fy_ * (*_depth);
                    }
                    else
                        *(_validPixel++) = -1;

                    ++_depth; ++_x; ++_y; ++_z;

//                    std::cout << row_pix+c << " pt " << xyz.block(row_pix+c,0,1,3) << " c " << c << " cam.cx() " << cam.cx() << " inv_fx_ " << inv_fx_
//                              << " min_depth_ " << min_depth_ << " max_depth_ " << max_depth_ << std::endl;
//                    mrpt::system::pause();
                }
            }

            Eigen::MatrixXf input_xyz_transp = Eigen::MatrixXf::Ones(4,img_size_);
            input_xyz_transp.block(0,0,3,img_size_) = xyz.block(0,0,img_size_,3).transpose();
            Eigen::MatrixXf aux = Rt * input_xyz_transp;
            xyz_tf = aux.block(0,0,3,img_size_).transpose();

            //cout << "row_stride_ " << row_stride_ << " " << cam.ncols << endl;
            warped_gray = cv::Mat::zeros(gray.rows, gray.cols, gray.type());
            uchar *_warped_gray = warped_gray.ptr<uchar>(0);
            uchar *_gray = const_cast<uchar*>(gray.ptr<uchar>(0));
            for(size_t i=0; i < img_size_; i++)
            {
                if(valid_pixels(i) == -1)
                    continue;

                Eigen::Vector3f pt_tf = xyz_tf.block(i,0,1,3).transpose();
                //Project the 3D point to the 2D plane
                float inv_z = 1.f/pt_tf(2);
                // 2D coordinates of the transformed pixel(r,c) of frame 1
                float c_tf = (pt_tf(0) * cam.fx())*inv_z + cam.cx(); //transformed x (2D)
                float r_tf = (pt_tf(1) * cam.fy())*inv_z + cam.cy(); //transformed y (2D)
                int c_transf = round(c_tf);
                int r_transf = round(r_tf);
                // cout << i << " Pixel transform " << i/cam.ncols << " " << i%cam.ncols << " " << r_transf << " " << c_transf << endl;
                if( c_transf >= 0 && c_transf < int(cam.ncols) && r_transf >= 0 && r_transf < int(cam.nrows) )
                {
                    _warped_gray[r_transf * row_stride_ + c_transf] = _gray[i];
                    if(c_transf+1 < int(cam.ncols))
                        _warped_gray[r_transf * row_stride_ + c_transf+1] = _gray[i];
                    if(r_transf+1 < int(cam.nrows))
                        _warped_gray[(r_transf+1) * row_stride_ + c_transf+1] = _gray[i];
                }

                //cout << i << " Pixel transform " << warp_map_lin(i) << " " << r_transf << " " << c_transf << " from pt " << xyz.block(i,0,1,3) << endl;
            }
        }
//    };
//}
