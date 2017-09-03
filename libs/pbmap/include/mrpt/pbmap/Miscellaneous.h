/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#ifndef __MISCELLANEOUS_H
#define __MISCELLANEOUS_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils/types_math.h> // Eigen
#include <map>
#include <string>
#include <iostream>
#include <iterator>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mrpt/pbmap/link_pragmas.h>
#include <mrpt/math.h>

/*! Sort a vector and retrieve the indexes of teh sorted values.*/
template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> & v)
{
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;

    // sort indexes based on comparing values in v
    std::sort( idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] > v[i2];} );

    return idx;
}

template <typename T>
std::vector<size_t> sort_vector(std::vector<T> & v)
{
    // initialize original index locations
    std::vector<size_t> idx = sort_indexes(v);

    std::vector<T> sorted_vector(v.size());
    for (size_t i = 0; i != idx.size(); ++i)
        sorted_vector[i] = v[idx[i]];
    v = sorted_vector;

    return idx;
}

/*! Sort a vector and retrieve the indexes of the sorted values.*/
template <typename T>
std::vector<size_t> sort_weights(const std::vector<T> & v)
{
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::set<size_t> sorted;
    for (size_t j = 0; j < v.size(); ++j)
    {
        T largest_element; // we assume that all the elements (weights) are positive
        size_t largest_idx;
        size_t i = 0;
        for ( ; i < v.size(); ++i)
        {
            if(sorted.count(i))
                continue;
            else
            {
                largest_element = v[i];
                largest_idx = i;
                break;
            }
        }
        for ( i++; i < v.size(); ++i)
        {
            if(sorted.count(i))
                continue;
            if(v[i] > largest_element)
            {
                largest_element = v[i];
                largest_idx = i;
            }
        }
        idx[j] = largest_idx;
        sorted.insert(largest_idx);
    }
    //  std::cout << "\n sort_indexes: ";
    //  for(size_t i = 0; i < idx.size(); i++)
    //      std::cout << idx[i] << " ";
    //  std::cout << "\n";

    return idx;
}

/*! Compute the pseudoinverse of an Eigen::Matrix.*/
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        else
            singularValuesInv(i, i) = Scalar{0};
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

namespace mrpt {
namespace pbmap {
    typedef pcl::PointXYZRGBA PointT;

    template <class POINT>
    inline Eigen::Vector3f diffPoints(const POINT &P1, const POINT &P2)
    {
        Eigen::Vector3f diff;
        diff[0] = P1.x - P2.x;
        diff[1] = P1.y - P2.y;
        diff[2] = P1.z - P2.z;
        return diff;
    }

    /*! Compose a 3D-point with a pose.*/
    template<class dataType>
    Eigen::Matrix<dataType,3,1> compose(Eigen::Matrix<dataType,4,4> &pose, Eigen::Matrix<dataType,3,1> &point)
    {
        Eigen::Matrix<dataType,3,1> transformedPoint = pose.block(0,0,3,3) * point + pose.block(0,3,3,1);
        return transformedPoint;
    }

    /*! Compose two poses.*/
    template<class dataType>
    Eigen::Matrix<dataType,4,4> compose(Eigen::Matrix<dataType,4,4> &pose1, Eigen::Matrix<dataType,4,4> &pose2)
    {
        Eigen::Matrix<dataType,4,4> transformedPose;
        transformedPose.block(0,0,3,3) = pose1.block(0,0,3,3) * pose2.block(0,0,3,3);
        transformedPose.block(0,3,3,1) = pose1.block(0,3,3,1) + pose1.block(0,0,3,3)*pose2.block(0,3,3,1);
        transformedPose.row(3) << 0,0,0,1;
        return transformedPose;
    }

    /*! Get the pose's inverse.*/
    template<class dataType>
    Eigen::Matrix<dataType,4,4> inverse(Eigen::Matrix<dataType,4,4> &pose)
    {
        Eigen::Matrix<dataType,4,4> inverse;
        inverse.block(0,0,3,3) = pose.block(0,0,3,3).transpose();
        inverse.block(0,3,3,1) = -(inverse.block(0,0,3,3) * pose.block(0,3,3,1));
        inverse.row(3) << 0,0,0,1;
        return inverse;
    }

    struct Segment
    {
        Segment(PointT p0, PointT p1) :
            P0(p0), P1(p1)
        {};

        PointT P0, P1;
    };

    /*!  Square of the distance between two segments */
    float PBMAP_IMPEXP dist3D_Segment_to_Segment2( Segment S1, Segment S2);

    /*!  Check if a point lays inside a convex hull */
    bool PBMAP_IMPEXP isInHull(PointT &point3D, pcl::PointCloud<PointT>::Ptr hull3D);

    /**
    * Bhattacharyya histogram distance function
    * @param hist1
    * @param hist2
    * @return the histogram distance output
    */
    double PBMAP_IMPEXP BhattacharyyaDist(std::vector<float> &hist1, std::vector<float> &hist2);

    template<typename dataType>
    dataType getMode(std::vector<dataType> data, dataType range)
    {
        float normalizeConst = 255.0/range;
        std::vector<int> data2(data.size() );
        for(size_t i=0; i < data.size(); i++)
            data2[i] = (int)(data[i]*normalizeConst);

        std::map<int,int> histogram;
        for(size_t i=0; i < data2.size(); i++)
            if(histogram.count(data2[i]) == 0)
                histogram[data2[i] ] = 1;
            else
                histogram[data2[i] ]++;

        int mode = 0, count = 0;
        for(std::map<int,int>::iterator bin = histogram.begin(); bin != histogram.end(); bin++)
            if(bin->second > count)
            {
                count = bin->second;
                mode = bin->first;
            }

        return (dataType)mode/normalizeConst;
    }

    // Gets the center of a single-mode distribution, it performs variable mean shift
    template<typename dataType>
    Eigen::Vector4f getMultiDimMeanShift_color(std::vector<Eigen::Vector4f> &data, dataType &stdDevHist, dataType &concentration)
    {
        //    cout << "Do meanShift\n";

        std::vector<Eigen::Vector4f> dataTemp = data;
        size_t size = data.size();

        //    This one is specific for normalized color
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for(size_t i=0; i < data.size(); i++)
        {
            sum += data[i].head(3);
        }
        Eigen::Vector3f meanShift = sum/size;
        //cout << "First meanShift " << meanShift.transpose() << endl;

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(size_t i=0; i < data.size(); i++)
        {
            Eigen::Vector3f diff = data[i].head(3) - meanShift;
            cov += diff * diff.transpose();
        }
        //    cov /= size;
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov);
        stdDevHist = svd.singularValues().maxCoeff() / sqrt((double) size);
        //    stdDevHist = 0.05;

        double shift = 1000; // Large limit
        int iteration_counter = 0;
        double convergence = 0.001;
        while(2*dataTemp.size() > size && shift > convergence)
        {
            //  std::cout << "iteration " << iteration_counter << " Std " << stdDevHist << " maxEig " << svd.singularValues().maxCoeff() << std::endl;
            for(typename std::vector<Eigen::Vector4f>::iterator it=dataTemp.begin(); it != dataTemp.end(); )
            {
                //        cout << "CHeck\n";
                Eigen::Vector3f diff = (*it).head(3) - meanShift;
                if(diff.norm() > stdDevHist)
                {
                    sum -= (*it).head(3);
                    cov -= diff * diff.transpose();
                    dataTemp.erase(it);
                }
                else
                    it++;
            }
            //    cout << "sum " << sum.transpose() << " newdatasize " << dataTemp.size() << endl;
            Eigen::Vector3f meanUpdated = sum / dataTemp.size();
            shift = (meanUpdated - meanShift).norm();
            meanShift = meanUpdated;
            svd = Eigen::JacobiSVD<Eigen::Matrix3f>(cov);
            //      stdDevHist = svd.singularValues().maxCoeff() / dataTemp.size();
            stdDevHist = svd.singularValues().maxCoeff() / sqrt((double) dataTemp.size());

            iteration_counter++;
        }
        //  std::cout << "Number of iterations: " << iteration_counter << " shift " << shift
        //            << " size " << (float)dataTemp.size() / size << " in " << clock.Tac() * 1e3 << " ms." << std::endl;

        //  stdDevHist = calcStdDev(data, meanShift);

        Eigen::Vector4f dominantColor;
        dominantColor.head(3) = meanShift;
        float averageIntensity = 0;
        for(unsigned i=0; i < dataTemp.size(); i++)
            averageIntensity += dataTemp[i][3];
        averageIntensity /= dataTemp.size();
        dominantColor(3) = averageIntensity;

        //    concentration = float(dataTemp.size()) / size;
        int countFringe05 = 0;
        for(typename std::vector<Eigen::Vector4f>::iterator it=data.begin(); it != data.end(); it++)
            if((it->head(3) - meanShift).norm() < 0.05 ) //&& *it(3) - averageIntensity < 0.3)
                ++countFringe05;
        concentration = static_cast<dataType>(countFringe05) / data.size();

        return dominantColor;
    }

    // Gets the center of a single-mode distribution, it performs variable mean shift
    template<typename dataType>
    dataType getHistogramMeanShift(std::vector<dataType> &data, double range, dataType &stdDevHist_out)//, dataType &concentration05)
    {
        //    cout << "Do meanShift\n";
        //  mrpt::utils::CTicTac clock;
        //  clock.Tic();
        size_t size = data.size();
        std::vector<dataType> dataTemp = data;

        dataType sum = 0;
        for(size_t i=0; i < data.size(); i++){
            sum += data[i];}
        dataType meanShift =sum/size;
        dataType stdDevHist = mrpt::math::stddev(data);

        ////    dataType meanShift;
        //    double meanShift, stdDevHist;
        //    mrpt::math::meanAndStd(data,meanShift,stdDevHist);
        //    double sum = meanShift*data.size();
        //cout << "mean " << meanShift << endl;

        //dataType step = 1;
        double shift = 1000;
        int iteration_counter = 0;
        double convergence = range * 0.001;
        while(2*dataTemp.size() > size && shift > convergence)
        {
            //  std::cout << "iteration " << iteration_counter << " Std " << stdDevHist << std::endl;
            for(typename std::vector<dataType>::iterator it=dataTemp.begin(); it != dataTemp.end(); )
            {
                //        cout << "CHeck\n";
                if(fabs(*it - meanShift) > stdDevHist)
                {
                    sum -= *it;
                    dataTemp.erase(it);
                }
                else
                    it++;
            }
            //    cout << "sum " << sum << " newdatasize " << dataTemp.size() << endl;
            double meanUpdated = sum / dataTemp.size();
            shift = fabs(meanUpdated - meanShift);
            meanShift = meanUpdated;
            stdDevHist = mrpt::math::stddev(dataTemp);

            iteration_counter++;
        }
        //  std::cout << "Number of iterations: " << iteration_counter << " shift " << shift
        //            << " size " << (float)dataTemp.size() / size << " in " << clock.Tac() * 1e3 << " ms." << std::endl;

        //  stdDevHist = calcStdDev(data, meanShift);

        //    // Calculate concentration05
        ////    stdDevHist_out = float(dataTemp.size()) / size;
        //    int countFringe05 = 0;
        //    for(typename std::vector<dataType>::iterator it=data.begin(); it != data.end(); it++)
        //        if(fabs(*it - meanShift) < 0.05)
        //            ++countFringe05;
        //    concentration05 = static_cast<dataType>(countFringe05) / data.size();

        return static_cast<dataType>(meanShift);
    }

    template <class POINT>
    typename pcl::PointCloud<POINT>::Ptr colourPointCloud(const typename pcl::PointCloud<POINT>::Ptr &in, const unsigned char r, const unsigned char g, const unsigned char b, float alpha)
    {
        if( !std::is_same<POINT,pcl::PointXYZRGB>::value && !std::is_same<POINT,pcl::PointXYZRGBA>::value )
            throw std::runtime_error("\nERROR: mrpt::pbmap::colourPointCloud called without an input colour cloud\n");

        // Maxe sure that: 0 <= alpha <= 1
        if( alpha < 0.f ) alpha = 0.f;
        else if( alpha > 1.f ) alpha = 1.f;

        const float beta = 1.f - alpha;
        float alpha_r(alpha*r), alpha_g(alpha*g), alpha_b(alpha*b);
        typename pcl::PointCloud<POINT>::Ptr out(new typename pcl::PointCloud<POINT>(*in));
        for(size_t i=0; i < in->size(); i++)
        {
            out->points[i].r = (unsigned char)(beta*out->points[i].r + alpha_r);
            out->points[i].g = (unsigned char)(beta*out->points[i].g + alpha_g);
            out->points[i].b = (unsigned char)(beta*out->points[i].b + alpha_b);
        }

        return out;
    }

    /**
    * Output a vector as a stream that is space separated.
    * @param os Output stream
    * @param v Vector to output
    * @return the stream output
    */
    template<typename T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
    {
        std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
        return os;
    }

} } // End of namespaces

#endif

#endif
