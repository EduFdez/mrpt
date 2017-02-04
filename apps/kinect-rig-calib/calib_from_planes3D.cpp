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

#include "calib_from_planes3D.h"
//#include <rv/utils/file.h>

using namespace std;
//using namespace rv::utils;


PlaneCorresp::PlaneCorresp(size_t n_sensors) :
    n_sensors_(n_sensors)
{
    std::cout << "PlaneCorresp... n_sensors_ " << n_sensors_ << std::endl;
    for(unsigned sensor_id1=0; sensor_id1 < n_sensors_; sensor_id1++)
    {
        mm_corresp_[sensor_id1] = std::map<unsigned, mrpt::math::CMatrixDouble>();
        for(unsigned sensor_id2=sensor_id1+1; sensor_id2 < n_sensors_; sensor_id2++)
            mm_corresp_[sensor_id1][sensor_id2] = mrpt::math::CMatrixDouble(0, 10);
    }
    conditioning_ = std::vector<float>(n_sensors_, 9999.9);
    covariances_ = std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >(n_sensors_, Eigen::Matrix3f::Zero());
}

/*! Load the plane correspondences between the different Asus sensors from file */
void PlaneCorresp::saveCorrespondences(const std::string &planeCorrespDirectory)
{
    for(std::map<unsigned, std::map<unsigned, mrpt::math::CMatrixDouble> >::iterator it_pair1=mm_corresp_.begin();
        it_pair1 != mm_corresp_.end();
        it_pair1++)
    {
        for(std::map<unsigned, mrpt::math::CMatrixDouble>::iterator it_pair2=it_pair1->second.begin();
            it_pair2 != it_pair1->second.end(); it_pair2++)
        {
            if(it_pair2->second.rows() > 0)
                it_pair2->second.saveToTextFile( mrpt::format("%s/correspondences_%u_%u.txt", planeCorrespDirectory.c_str(), it_pair1->first, it_pair2->first) );
        }
    }
}

/*! Load the plane correspondences between a pair of Asus sensors from file */
mrpt::math::CMatrixDouble PlaneCorresp::getPlaneCorrespondences(const std::string matchedPlanesFile)
{
    if( !fileExists(matchedPlanesFile) )
        throw std::runtime_error("ERROR PlaneCorresp::getPlaneCorrespondences -> wrong matchedPlanesFile\n\n");

    mrpt::math::CMatrixDouble correspMat;
    correspMat.loadFromTextFile(matchedPlanesFile);
    //      std::cout << "Load PlaneCorresp " << sensor_id << " and " << sensor_corresp << std::endl;
    std::cout << correspMat.rows() << " correspondences " << std::endl;

    return correspMat;
}

/*! Load the plane correspondences between the different Asus sensors from file */
void PlaneCorresp::loadPlaneCorrespondences(const std::string planeCorrespDirectory)
{
    mm_corresp_.clear();
    for(unsigned sensor_id = 0; sensor_id < n_sensors_-1; sensor_id++)
    {
        mm_corresp_[sensor_id] = std::map<unsigned, mrpt::math::CMatrixDouble>();
        for(unsigned sensor_corresp = sensor_id+1; sensor_corresp < n_sensors_; sensor_corresp++)
        {
            std::string fileCorresp = mrpt::format("%s/correspondences_%u_%u.txt", planeCorrespDirectory.c_str(), sensor_id, sensor_corresp);
            if( fileExists(fileCorresp) )
            {
                mrpt::math::CMatrixDouble correspMat;
                correspMat.loadFromTextFile(fileCorresp);
                mm_corresp_[sensor_id][sensor_corresp] = correspMat;
                //          std::cout << "Load PlaneCorresp " << sensor_id << " and " << sensor_corresp << std::endl;
                //          std::cout << correspMat.rows() << " correspondences " << std::endl;
            }
        }
    }
    std::cout << "PlaneCorresp::loadPlaneCorrespondences -> " << mm_corresp_.size() << " sensors \n";
}

/*! Get the rate of inliers near the border of the sensor (the border nearer to the next lower index Asus sensor) */
float PlaneCorresp::inliersUpperFringe(mrpt::pbmap::Plane &plane, float fringeWidth) // This only works for QVGA resolution
{
    size_t count = 0;
    size_t im_size = 320 * 240;
    int limit = fringeWidth * im_size;
    for(size_t i=0; i < plane.inliers.size(); ++i)
        if(plane.inliers[i] < limit)
            ++count;

    return float(count) / (im_size*fringeWidth);
}

/*! Get the rate of inliers near the border of the sensor (the border nearer to the next upper index Asus sensor) */
float PlaneCorresp::inliersLowerFringe(mrpt::pbmap::Plane &plane, float fringeWidth)
{
    size_t count = 0;
    size_t im_size = 320 * 240;
    int limit = (1 - fringeWidth) * im_size;
    for(unsigned i=0; i < plane.inliers.size(); ++i)
        if(plane.inliers[i] > limit)
            ++count;

    return float(count) / (im_size*fringeWidth);
}

/*! Print the number of correspondences and the conditioning number to the standard output */
void PlaneCorresp::printConditioning()
{
    std::cout << "Conditioning\n";
    for(unsigned sensor_id = 0; sensor_id < n_sensors_-1; sensor_id++)
        std::cout << mm_corresp_[sensor_id][sensor_id+1].rows() << "\t";
    std::cout << mm_corresp_[0][n_sensors_-1].rows() << "\t";
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
void PlaneCorresp::calcAdjacentConditioning(unsigned couple_id)
{
    //      if(conditioning_measCount[couple_id] > 3)
    //      {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariances_[couple_id], Eigen::ComputeFullU | Eigen::ComputeFullV);
    conditioning_[couple_id] = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
    //      }
}
