/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef MetricRegistration_H
#define MetricRegistration_H

#include <mrpt/math/CMatrix.h>
#include <mrpt/poses/poses_frwds.h>
#include <Eigen/Dense>

#include <mrpt/utils/bits.h>
#include <mrpt/utils/COutputLogger.h>

#include <mrpt/slam/link_pragmas.h>

#include <numeric>

namespace mrpt
{
namespace slam
{
    /**  This class implements different algorithms to register geometric features (e.g. points, lines and planes).
	 */
    template <typename T>
    class SLAM_IMPEXP  MetricRegistration : public mrpt::utils::COutputLogger
	{
	public:
        MetricRegistration() : mrpt::utils::COutputLogger("MetricRegistration") {}
		/** Dtor */
        virtual ~MetricRegistration() { }

//		/** The method for aligning a pair of metric maps, aligning only 2D + orientation.
//		 *   The meaning of some parameters and the kind of the maps to be aligned are implementation dependant,
//		 *    so look into the derived classes for instructions.
//		 *  The target is to find a PDF for the pose displacement between
//		 *   maps, <b>thus the pose of m2 relative to m1</b>. This pose
//		 *   is returned as a PDF rather than a single value.
//		 *
//		 * \param m1			[IN] The first map
//		 * \param m2			[IN] The second map. The pose of this map respect to m1 is to be estimated.
//		 * \param grossEst		[IN] An initial gross estimation for the displacement. If a given algorithm doesn't need it, set to <code>CPose2D(0,0,0)</code> for example.
//		 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
//		 * \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
//		 *
//		 * \return A smart pointer to the output estimated pose PDF.
//		 * \sa CICP
//		 */
//		mrpt::poses::CPosePDFPtr Align(
//				const mrpt::maps::CMetricMap		*m1,
//				const mrpt::maps::CMetricMap		*m2,
//				const mrpt::poses::CPose2D			&grossEst,
//				float					*runningTime = NULL,
//				void					*info = NULL );

        /*! Calculate the rotation from the covariance matrix of a set of corresponding normal vectors */
        static inline T calcRotationFromNormalVectorCov ( const Eigen::Matrix<T,3,3> & covariance, Eigen::Matrix<T,3,3> & rotation)
        {
    //        std::cout << "ExtrinsicCalib::calcRotationFromNormalVectorCov...\n";

            Eigen::JacobiSVD<Eigen::Matrix<T,3,3> > svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
            float conditioning = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();

            rotation = svd.matrixV() * svd.matrixU().transpose();
            double det = rotation.determinant();
            if(det != 1)
            {
                Eigen::Matrix<T,3,3> aux;
                aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
                rotation = svd.matrixV() * aux * svd.matrixU().transpose();
            }
            //std::cout << "    Estimation took " << 1000*clock.Tac() << " ms. Conditioning " << conditioning << "\n";
            //std::cout << "rotation \n" << rotation << "\nconditioning " << conditioning << "\n";
            //std::cout << "conditioning " << conditioning << "\n";

            return conditioning;
        }


        static std::map<size_t, size_t> registerNormalVectors ( const std::vector<Eigen::Matrix<T,3,1> > & n_i, std::vector<Eigen::Matrix<T,3,1> > & n_j,
                                                                Eigen::Matrix<T,3,3> & rotation, T & conditioning, const T max_angle_diff = 0.0174533, const T min_angle_pair = 0.174533)
        {
            //std::cout << "\nmrpt::slam::MetricRegistration<T>::registerNormalVectors... \n";

            // Find a set of normal vector correspondences among n_i and n_j and compute the relative rotation and conditioning number
            std::map<size_t,size_t> best_matches;
            std::vector<std::map<size_t,size_t> > discarded_matches; // with at least 3 matches
//            T best_score = 10e6;
            const T max_angle_diff_cos = cos(max_angle_diff);
            const T max_angle_diff_1st_pair = 0.9*max_angle_diff;
            //const T min_conditioning = 0.01;

            // Exhaustive search
            for(size_t i1=0; i1 < n_i.size(); i1++)
            {
                for(size_t i2=i1+1; i2 < n_i.size(); i2++)
                {
                    T angle_i = acos( n_i[i1].dot(n_i[i2]) );
                    if( angle_i < min_angle_pair )
                        continue;
                    for(size_t j1=0; j1 < n_j.size(); j1++)
                    {
                        for(size_t j2=0; j2 < n_j.size(); j2++)
                        {
                            if( j1 == j2 )
                                continue;
                            bool already_checked = false;
                            for(size_t k=0; k < discarded_matches.size(); k++)
                                if(discarded_matches[k].count(i1) && discarded_matches[k][i1] == j1 && discarded_matches[k].count(i2) && discarded_matches[k][i2] == j2)
                                {
                                    already_checked = true;
                                    break;
                                }
                            if( already_checked )
                                continue;
                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error angle " << utils::RAD2DEG(acos( n_j[j1].dot(n_j[j2])) - angle_i) << " " << utils::RAD2DEG(max_angle_diff_1st_pair) << "\n";
                            if( fabs( acos( n_j[j1].dot(n_j[j2]) ) - angle_i ) > max_angle_diff_1st_pair )
                                continue;

                            // Compute rotation
                            Eigen::Matrix<T,3,3> rot;
                            Eigen::Matrix<T,3,3> cov = n_j[j1]*n_i[i1].transpose() + n_j[j2]*n_i[i2].transpose() + (n_j[j1].cross(n_j[j2]))*(n_i[i1].cross(n_i[i2])).transpose();
                            T cond = calcRotationFromNormalVectorCov(cov, rot);
                            //if(cond < min_conditioning){ //std::cout << "ExtrinsicCalibLines::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
                            //    continue; }
                            //if( n_i[i1].dot(rot*n_j[j1]) < max_angle_diff_cos ) // Check if the rotation is consistent
                            //    continue;

                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << n_i[i1].dot(rot*n_j[j1]) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n"; // << "\nR\n" << rot << "\n";

                            // Check how many lines are consistent with this rotation
                            std::map<size_t,size_t> matches;
                            matches[i1] = j1;
                            matches[i2] = j2;
                            std::set<size_t> matches2; matches2.insert(j1); matches2.insert(j2);
                            for(size_t i=0; i < n_i.size(); i++)
                            {
                                if( i==i1 || i==i2 )
                                    continue;
//                                size_t best_j = 1e6;
//                                T angle_best_j = max_angle_diff_cos;
                                for(size_t j=0; j < n_j.size(); j++)
                                {
                                    if( matches2.count(j) )
                                        continue;
                                    //std::cout << "error " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff_cos << "\n";
                                    if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
                                    {
                                        cov += n_j[j]*n_i[i].transpose();
                                        for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
                                            cov += (n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
                                        matches[i] = j;
                                        matches2.insert(j);
                                        break;
                                    }
//                                    if( n_i[i].dot(rot*n_j[j]) > angle_best_j )
//                                    {
//                                        //std::cout << "True\n";
//                                        best_j = j;
//                                        angle_best_j = n_i[i].dot(rot*n_j[j]);
//                                        break;
//                                    }
                                }
//                                if( best_j != 1e6)
//                                {
//                                    cov += n_j[best_j]*n_i[i].transpose();
//                                    for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
//                                        cov += (n_j[best_j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
//                                    matches[i] = best_j;
//                                    matches2.insert(best_j);
//                                }
                            }

                            // Compute the rotation from the inliers
                            cond = calcRotationFromNormalVectorCov(cov, rot);

                            // Perform a 2nd search of inliers after the rotation computation
                            if( matches.size() > 2 && best_matches.size() < 0.8*matches.size() )
                                for(size_t i=0; i < n_i.size(); i++)
                                {
                                    if( matches.count(i) )
                                        continue;
                                    for(size_t j=0; j < n_j.size(); j++)
                                    {
                                        if( matches2.count(j) )
                                            continue;
                                        //std::cout << "error " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff << "\n";
                                        if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
                                        {
                                            cov += n_j[j]*n_i[i].transpose();
                                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
                                                cov += (n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
                                            matches[i] = j;
                                            matches2.insert(j);
                                        }
                                    }
                                }
                            cond = calcRotationFromNormalVectorCov(cov, rot);
//                            T error = 0;
//                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++){ //std::cout << "match " << it->first << " - " << it->second << " error " << utils::RAD2DEG(acos(n_i[it->first] .dot (rot*n_j[it->second]))) << "\n";
//                                error += acos(n_i[it->first] .dot (rot*n_j[it->second]));}

                            //std::cout << "Num corresp " << matches.size() << " conditioning " << cond << " error " << error << " average error " << utils::RAD2DEG(error/matches.size()) << " deg.\n";

                            if(best_matches.size() < matches.size() && matches.size() > 2)
                            {
                                //if(matches.size() > 2)
                                    discarded_matches.push_back(matches);

                                best_matches = matches;
//                                best_score = error;
                                rotation = rot;
                                conditioning = cond;
                            }
                        }
                    }
                }
            }
//            std::cout << " ...matchNormalVectors took " << 1000*clock.Tac() << " ms " << best_matches.size() << " matches, best_score " << utils::RAD2DEG(best_score) << " conditioning " << conditioning << "\n" << rotation << "\n";
        //    for(map<size_t,size_t>::iterator it=best_matches.begin(); it != best_matches.end(); it++)
        //        std::cout << "match " << it->first << " - " << it->second << "\n";

            return best_matches;
        }


        static std::map<size_t, size_t> registerNormalVectorsStop ( const std::vector<Eigen::Matrix<T,3,1> > & n_i, std::vector<Eigen::Matrix<T,3,1> > & n_j,
                                                                Eigen::Matrix<T,3,3> & rotation, T & conditioning, const T max_angle_diff = 0.0174533, const T min_angle_pair = 0.174533)
        {
            // Find a set of normal vector correspondences among n_i and n_j and compute the relative rotation and conditioning number
            std::map<size_t,size_t> best_matches;
            std::vector<std::map<size_t,size_t> > discarded_matches; // with at least 3 matches
//            T best_score = 10e6;
            const T max_angle_diff_cos = cos(max_angle_diff);
            const T max_angle_diff_1st_pair = 0.9*max_angle_diff;
            //const T min_conditioning = 0.01;

            // Exhaustive search
            for(size_t i1=0; i1 < n_i.size(); i1++)
            {
                for(size_t i2=i1+1; i2 < n_i.size(); i2++)
                {
                    T angle_i = acos( n_i[i1].dot(n_i[i2]) );
                    if( angle_i < min_angle_pair )
                        continue;
                    for(size_t j1=0; j1 < n_j.size(); j1++)
                    {
                        for(size_t j2=0; j2 < n_j.size(); j2++)
                        {
                            if( j1 == j2 )
                                continue;
                            bool already_checked = false;
                            for(size_t k=0; k < discarded_matches.size(); k++)
                                if(discarded_matches[k].count(i1) && discarded_matches[k][i1] == j1 && discarded_matches[k].count(i2) && discarded_matches[k][i2] == j2)
                                {
                                    already_checked = true;
                                    break;
                                }
                            if( already_checked )
                                continue;
                            if( fabs( acos( n_j[j1].dot(n_j[j2]) ) - angle_i ) > max_angle_diff_1st_pair )
                                continue;

                            // Compute rotation
                            Eigen::Matrix<T,3,3> rot;
                            Eigen::Matrix<T,3,3> cov = n_j[j1]*n_i[i1].transpose() + n_j[j2]*n_i[i2].transpose() + (n_j[j1].cross(n_j[j2]))*(n_i[i1].cross(n_i[i2])).transpose();
                            T cond = calcRotationFromNormalVectorCov(cov, rot);
                            //if(cond < min_conditioning){ //std::cout << "ExtrinsicCalibLines::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
                            //    continue; }
                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << n_i[i1].dot(rot*n_j[j1]) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n";
                            //if( n_i[i1].dot(rot*n_j[j1]) < max_angle_diff_cos ) // Check if the rotation is consistent
                            //    continue;

                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << utils::RAD2DEG(acos(n_i[i1].dot(rot*n_j[j1]))) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n";

                            // Check how many lines are consistent with this rotation
                            std::map<size_t,size_t> matches;
                            matches[i1] = j1;
                            matches[i2] = j2;
                            std::set<size_t> matches2; matches2.insert(j1); matches2.insert(j2);
                            for(size_t i=0; i < n_i.size(); i++)
                            {
                                if( i==i1 || i==i2 )
                                    continue;
                                if( i > 0 ) // Check early stopping of recursive search
                                    if( n_i.size()-i+size_t(i>i1)+size_t(i>i2) < best_matches.size() ){ //std::cout << "iSTOP exhaustive search: max length reached\n";
                                        break;}
                                for(size_t j=0; j < n_j.size(); j++)
                                {
                                    if( matches2.count(j) )
                                        continue;
                                    //std::cout << "error " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff << "\n";
                                    if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
                                    {
                                        cov += n_j[j]*n_i[i].transpose();
                                        for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
                                            cov += (n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
                                        matches[i] = j;
                                        matches2.insert(j);
                                        break;
                                    }
                                }
                            }

                            // Compute the rotation from the inliers
                            cond = calcRotationFromNormalVectorCov(cov, rot);

                            // Perform a 2nd search of inliers after the rotation computation
                            if( matches.size() > 2 && best_matches.size() < 0.8*matches.size() )
                                for(size_t i=0; i < n_i.size(); i++)
                                {
                                    if( matches.count(i) )
                                        continue;
                                    for(size_t j=0; j < n_j.size(); j++)
                                    {
                                        if( matches2.count(j) )
                                            continue;
                                        //std::cout << "error " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff << "\n";
                                        if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
                                        {
                                            cov += n_j[j]*n_i[i].transpose();
                                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
                                                cov += (n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
                                            matches[i] = j;
                                            matches2.insert(j);
                                        }
                                    }
                                }
                            cond = calcRotationFromNormalVectorCov(cov, rot);
//                            T error = 0;
//                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++){ //std::cout << "match " << it->first << " - " << it->second << " error " << utils::RAD2DEG(acos(n_i[it->first] .dot (rot*n_j[it->second]))) << "\n";
//                                error += acos(n_i[it->first] .dot (rot*n_j[it->second]));}

                            //std::cout << "Num corresp " << matches.size() << " conditioning " << cond << " error " << error << " average error " << utils::RAD2DEG(error/matches.size()) << " deg.\n";

                            if(best_matches.size() < matches.size() && matches.size() > 2)
                            {
                                //if(matches.size() > 2)
                                    discarded_matches.push_back(matches);

                                best_matches = matches;
//                                best_score = error;
                                rotation = rot;
                                conditioning = cond;
                            }
                        }
                    }
                }
            }
//            std::cout << " ...matchNormalVectors took " << 1000*clock.Tac() << " ms " << best_matches.size() << " matches, best_score " << utils::RAD2DEG(best_score) << " conditioning " << conditioning << "\n" << rotation << "\n";
        //    for(map<size_t,size_t>::iterator it=best_matches.begin(); it != best_matches.end(); it++)
        //        std::cout << "match " << it->first << " - " << it->second << "\n";

            return best_matches;
        }

        static std::map<size_t, size_t> registerNormalVectorsContrast ( const std::vector<Eigen::Matrix<T,3,1> > & n_i, const std::vector<float> & length_i, const std::vector<int> & contrast_i,
                                                                        const std::vector<Eigen::Matrix<T,3,1> > & n_j, const std::vector<float> & length_j, const std::vector<int> & contrast_j,
                                                                        Eigen::Matrix<T,3,3> & rotation, T & conditioning,
                                                                        const T max_angle_diff = 0.0174533, const T min_angle_pair = 0.174533, const int th_contrast = 80)
        {
            std::cout << "\nmrpt::slam::MetricRegistration<T>::registerNormalVectorsContrast... \n";

            // Find a set of normal vector correspondences among n_i and n_j and compute the relative rotation and conditioning number
            std::map<size_t,size_t> best_matches;
            std::vector<std::map<size_t,size_t> > discarded_matches; // with at least 3 matches
            T best_score(0);
            const T max_angle_diff_cos = cos(max_angle_diff);
            const T max_angle_diff_1st_pair = max_angle_diff;
            //const T min_conditioning = 0.01;

            // Exhaustive search
            for(size_t i1=0; i1 < n_i.size(); i1++)
            {
                for(size_t i2=i1+1; i2 < n_i.size(); i2++)
                {
                    T angle_i = acos( n_i[i1].dot(n_i[i2]) );
                    if( angle_i < min_angle_pair )
                        continue;
                    for(size_t j1=0; j1 < n_j.size(); j1++)
                    {
                        if( abs(contrast_i[i1]-contrast_j[j1]) > th_contrast ) // Check scalar contrast descriptor
                            continue;

                        for(size_t j2=0; j2 < n_j.size(); j2++)
                        {
                            if( j1 == j2 )
                                continue;
                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << "\n";
                            bool already_checked = false;
                            for(size_t k=0; k < discarded_matches.size(); k++)
                                if(discarded_matches[k].count(i1) && discarded_matches[k][i1] == j1 && discarded_matches[k].count(i2) && discarded_matches[k][i2] == j2)
                                {
                                    already_checked = true;
                                    break;
                                }
                            if( already_checked )
                                continue;
                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " contrast " << abs( contrast_i[i2]-contrast_j[j2] ) << " " << th_contrast << "\n";
                            if( abs( contrast_i[i2]-contrast_j[j2] ) > th_contrast ) // Check scalar contrast descriptor
                                continue;
                            T angle_diff = fabs( acos( n_j[j1].dot(n_j[j2]) ) - angle_i );
                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error angle " << utils::RAD2DEG(angle_diff) << " " << utils::RAD2DEG(max_angle_diff_1st_pair) << "\n";
                            if( angle_diff > max_angle_diff_1st_pair )
                                continue;

                            // Compute rotation
                            Eigen::Matrix<T,3,3> rot;
//                            Eigen::Matrix<T,3,3> cov = n_j[j1]*n_i[i1].transpose() + n_j[j2]*n_i[i2].transpose() +
//                                                      (n_j[j1].cross(n_j[j2]))*(n_i[i1].cross(n_i[i2])).transpose();

                            std::vector<T> w(2);  // weights
                            w[0] = length_j[j1]+length_i[i1]; //(length_j[j1]*length_i[i1]) / (length_j[j1]+length_i[i1]);
                            w[1] = length_j[j2]+length_i[i2];
                            Eigen::Matrix<T,3,3> cov = w[0]*n_j[j1]*n_i[i1].transpose() + w[1]*n_j[j2]*n_i[i2].transpose() +
                                                       (w[0]*w[1]/(w[0]+w[1]))*(n_j[j1].cross(n_j[j2]))*(n_i[i1].cross(n_i[i2])).transpose();
                            T cond = calcRotationFromNormalVectorCov(cov, rot);
                            //const T max_angle_diff_cos = std::max(cos(2*max_angle_diff), std::min(max_angle_diff_cos_init, cos(2*angle_diff)));

                            //if(cond < min_conditioning){ //std::cout << "ExtrinsicCalibLines::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
                            //    continue; }
                            //if( n_i[i1].dot(rot*n_j[j1]) < max_angle_diff_cos ){ // Check if the rotation is consistent
                            //    std::cout << "Second pair check not validated\n";
                            //    continue;}

                            std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << n_i[i1].dot(rot*n_j[j1]) << " error " << n_i[i2].dot(rot*n_j[j2]) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n"; // << "\nR\n" << rot << "\n";
                            std::cout << "Rotation\n" << rot << "\n";

                            // Check how many lines are consistent with this rotation
                            std::map<size_t,size_t> matches;
                            matches[i1] = j1;
                            matches[i2] = j2;
                            std::set<size_t> matches2; matches2.insert(j1); matches2.insert(j2);
                            T score = w[0]+w[1];
                            for(size_t i=0; i < n_i.size(); i++)
                            {
                                if( i==i1 || i==i2 )
                                    continue;
                                for(size_t j=0; j < n_j.size(); j++)
                                {
                                    if( matches2.count(j) )
                                        continue;
                                    if( abs(contrast_i[i]-contrast_j[j]) > th_contrast ) // Check scalar contrast descriptor
                                        continue;

                                    //std::cout << "error " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff_cos << "\n";
                                    if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
                                    {
                                        //std::cout << "True\n";
                                        T w_ij = length_j[j]+length_i[i];
                                        cov += w_ij*n_j[j]*n_i[i].transpose();
                                        size_t match_id(0);
                                        for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++, match_id++)
                                            cov += (w[match_id]*w_ij/(w[match_id]+w_ij))*(n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
                                        matches[i] = j;
                                        matches2.insert(j);
                                        w.push_back(w_ij);
                                        score += w_ij;
                                        break;
                                    }
                                }
                            }

                            // Compute the rotation from the inliers
                            cond = calcRotationFromNormalVectorCov(cov, rot);

                            // Perform a 2nd search of inliers after the rotation computation
                            if( matches.size() > 2 && best_matches.size() < 0.8*matches.size() )
                                for(size_t i=0; i < n_i.size(); i++)
                                {
                                    if( matches.count(i) )
                                        continue;
                                    for(size_t j=0; j < n_j.size(); j++)
                                    {
                                        if( matches2.count(j) )
                                            continue;

                                        //std::cout << "error 2nd round " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff_cos << "\n";
                                        if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
                                        {
                                            //std::cout << "Add match 2nd round " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff_cos << "\n";
                                            cov += n_j[j]*n_i[i].transpose();
                                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
                                                cov += (n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
                                            matches[i] = j;
                                            matches2.insert(j);
                                        }
                                    }
                                }
                            cond = calcRotationFromNormalVectorCov(cov, rot);

                            T error(0);
                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
                            {
                                std::cout << "match " << it->first << " - " << it->second << " error " << utils::RAD2DEG(acos(n_i[it->first] .dot (rot*n_j[it->second]))) << "\n";
                                error += acos(n_i[it->first] .dot (rot*n_j[it->second]));
                            }

                            //std::cout << "best_score " << best_score << " score " << score << "\n";
                            //if(best_matches.size() < matches.size() && matches.size() > 2)
                            if(score > best_score) // && matches.size() > 2)
                            {
                                //std::cout << "Num corresp " << matches.size() << " conditioning " << cond << " error " << error << " average error " << utils::RAD2DEG(error/matches.size()) << " deg.\n";
                                //if(matches.size() > 2)
                                    discarded_matches.push_back(matches);

                                best_matches = matches;
                                best_score = score;
                                rotation = rot;
                                conditioning = cond;
                            }
                        }
                    }
                }
            }
//            std::cout << " ...matchNormalVectors took " << 1000*clock.Tac() << " ms " << best_matches.size() << " matches, best_score " << utils::RAD2DEG(best_score) << " conditioning " << conditioning << "\n" << rotation << "\n";
        //    for(map<size_t,size_t>::iterator it=best_matches.begin(); it != best_matches.end(); it++)
        //        std::cout << "match " << it->first << " - " << it->second << "\n";

            return best_matches;
        }

        static std::map<size_t, size_t> registerNormalVectorsContrastStop ( const std::vector<Eigen::Matrix<T,3,1> > & n_i, std::vector<float> & length_i, const std::vector<int> & contrast_i,
                                                                            const std::vector<Eigen::Matrix<T,3,1> > & n_j, std::vector<float> & length_j, const std::vector<int> & contrast_j,
                                                                            Eigen::Matrix<T,3,3> & rotation, T & conditioning,
                                                                            const T max_angle_diff = 0.0174533, const T min_angle_pair = 0.174533, const int th_contrast = 80)
        {
            // Find a set of normal vector correspondences among n_i and n_j and compute the relative rotation and conditioning number
            std::map<size_t,size_t> best_matches;
            std::vector<std::map<size_t,size_t> > discarded_matches; // with at least 3 matches
            T best_score(0);
            const T max_angle_diff_cos = cos(max_angle_diff);
            //const T min_conditioning = 0.01;

            // Exhaustive search
            for(size_t i1=0; i1 < n_i.size(); i1++)
            {
                for(size_t i2=i1+1; i2 < n_i.size(); i2++)
                {
                    T angle_i = acos( n_i[i1].dot(n_i[i2]) );
                    if( angle_i < min_angle_pair )
                        continue;
                    for(size_t j1=0; j1 < n_j.size(); j1++)
                    {
                        if( abs(contrast_i[i1]-contrast_j[j1]) > th_contrast ) // Check scalar contrast descriptor
                            continue;

                        for(size_t j2=0; j2 < n_j.size(); j2++)
                        {
                            if( j1 == j2 )
                                continue;
                            bool already_checked = false;
                            for(size_t k=0; k < discarded_matches.size(); k++)
                                if(discarded_matches[k].count(i1) && discarded_matches[k][i1] == j1 && discarded_matches[k].count(i2) && discarded_matches[k][i2] == j2)
                                {
                                    already_checked = true;
                                    break;
                                }
                            if( already_checked )
                                continue;
                            if( abs(contrast_i[i2]-contrast_j[j2]) > th_contrast ) // Check scalar contrast descriptor
                                continue;
                            if( fabs( acos( n_j[j1].dot(n_j[j2]) ) - angle_i ) > max_angle_diff )
                                continue;

                            // Compute rotation
                            Eigen::Matrix<T,3,3> rot;
//                            Eigen::Matrix<T,3,3> cov = n_j[j1]*n_i[i1].transpose() + n_j[j2]*n_i[i2].transpose() +
//                                                      (n_j[j1].cross(n_j[j2]))*(n_i[i1].cross(n_i[i2])).transpose();
                            std::vector<T> w(2);  // weights
                            w[0] = length_j[j1]+length_i[i1]; //(length_j[j1]*length_i[i1]) / (length_j[j1]+length_i[i1]);
                            w[1] = length_j[j2]+length_i[i2];
                            Eigen::Matrix<T,3,3> cov = w[0]*n_j[j1]*n_i[i1].transpose() + w[1]*n_j[j2]*n_i[i2].transpose() +
                                                       (w[0]*w[1]/(w[0]+w[1]))*(n_j[j1].cross(n_j[j2]))*(n_i[i1].cross(n_i[i2])).transpose();
                            T cond = calcRotationFromNormalVectorCov(cov, rot);
                            //if(cond < min_conditioning){ //std::cout << "ExtrinsicCalibLines::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
                            //    continue; }
                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << n_i[i1].dot(rot*n_j[j1]) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n";
                            //if( n_i[i1].dot(rot*n_j[j1]) < max_angle_diff_cos ) // Check if the rotation is consistent
                            //    continue;

                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << utils::RAD2DEG(acos(n_i[i1].dot(rot*n_j[j1]))) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n";

                            // Check how many lines are consistent with this rotation
                            std::map<size_t,size_t> matches;
                            matches[i1] = j1;
                            matches[i2] = j2;
                            std::set<size_t> matches_j; matches_j.insert(j1); matches_j.insert(j2);
                            T score = w[0]+w[1];
                            float len_prev_match_i(0.f);
                            float accum_i( std::accumulate(length_i.begin(), length_i.end(), 0.f) );
                            for(size_t i=0; i < n_i.size(); i++, accum_i -= length_i[i])
                            {
                                float accum_j( std::accumulate(length_j.begin(), length_j.end(), 0.f) );
                                if( i==i1 )
                                {
                                    len_prev_match_i = length_i[i];
                                    continue;
                                }
                                if( i==i2 )
                                {
                                    len_prev_match_i += length_i[i];
                                    continue;
                                }
                                if( i > 0 ) // Check early stopping of recursive search
                                    if( len_prev_match_i + accum_i + accum_j < best_score ){ //std::cout << "iSTOP exhaustive search: max length reached\n";
                                        break;}

                                float len_prev_match_j(0.f);
                                for(size_t j=0; j < n_j.size(); j++, accum_j -= length_j[j])
                                {
                                    if( matches_j.count(j) )
                                    {
                                        len_prev_match_j += length_j[j];
                                        continue;
                                    }
                                    if( abs(contrast_i[i]-contrast_j[j]) > th_contrast ) // Check scalar contrast descriptor
                                        continue;
                                    if( j > 0 ) // Check early stopping of recursive search
                                        if( len_prev_match_i + len_prev_match_j + accum_i + accum_j < best_score ){ //std::cout << "jSTOP exhaustive search: max length reached\n";
                                            break;}

                                    //std::cout << "error " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff << "\n";
                                    if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
                                    {
                                        matches_j.insert(j);
                                        T w_ij = length_j[j]+length_i[i];
                                        cov += w_ij*n_j[j]*n_i[i].transpose();
                                        size_t match_id(0);
                                        for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++, match_id++)
                                            cov += (w[match_id]*w_ij/(w[match_id]+w_ij))*(n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
                                        matches[i] = j;
                                        matches_j.insert(j);
                                        w.push_back(w_ij);
                                        score += w_ij;
                                        break;
                                    }
                                }
                            }

                            // Compute the rotation from the inliers
                            cond = calcRotationFromNormalVectorCov(cov, rot);
//                            T error(0);
//                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
//                            {
//                                //std::cout << "match " << it->first << " - " << it->second << " error " << utils::RAD2DEG(acos(n_i[it->first] .dot (rot*n_j[it->second]))) << "\n";
//                                error += acos(n_i[it->first] .dot (rot*n_j[it->second]));
//                            }
                            //std::cout << "Num corresp " << matches.size() << " conditioning " << cond << " error " << error << " average error " << utils::RAD2DEG(error/matches.size()) << " deg.\n";

                            //if(best_matches.size() < matches.size() && matches.size() > 2)
                            if(score > best_score)
                            {
                                //if(matches.size() > 2)
                                    discarded_matches.push_back(matches);

                                best_matches = matches;
                                best_score = score;
                                rotation = rot;
                                conditioning = cond;
                            }
                        }
                    }
                }
            }
//            std::cout << " ...matchNormalVectors took " << 1000*clock.Tac() << " ms " << best_matches.size() << " matches, best_score " << utils::RAD2DEG(best_score) << " conditioning " << conditioning << "\n" << rotation << "\n";
        //    for(map<size_t,size_t>::iterator it=best_matches.begin(); it != best_matches.end(); it++)
        //        std::cout << "match " << it->first << " - " << it->second << "\n";

            return best_matches;
        }

//        static std::map<size_t, size_t> findIsometryLinesContrast ( const std::vector<Eigen::Vector4f> & l_i, const std::vector<int> & contrast_i,
//                                                                    const std::vector<Eigen::Vector4f> & l_j, const std::vector<int> & contrast_j,
//                                                                    Eigen::Matrix<T,3,3> & homography, T & conditioning,
//                                                                    const T max_angle_diff = 0.0174533, const T min_angle_pair = 0.174533, const int th_contrast = 80)
//        {
//            // Find a set of normal vector correspondences among n_i and n_j and compute the relative rotation and conditioning number
//            std::map<size_t,size_t> best_matches;
//            std::vector<std::map<size_t,size_t> > discarded_matches; // with at least 3 matches
////            T best_score = 10e6;
//            const T max_angle_diff_cos = cos(max_angle_diff);
//            //const T min_conditioning = 0.01;
//            homography.setIdentity();

//            // Exhaustive search
//            for(size_t i1=0; i1 < n_i.size(); i1++)
//            {
//                Eigen::Vector2f l_i1_n(l_i[i1][2]-l_i[i1][0], l_i[i1][3]-l_i[i1][1]); l_i1_n.normalize();
//                for(size_t i2=i1+1; i2 < n_i.size(); i2++)
//                {
//                    Eigen::Vector2f l_i2_n(l_i[i2][2]-l_i[i2][0], l_i[i2][3]-l_i[i2][1]); l_i2_n.normalize();
//                    T angle_i = acos(l_i1_n.dot(l_i2_n));
//                    Eigen::Vector2f n_i1(-l_i1_n[1],l_i1_n[0]);
//                    Eigen::Vector2f diff_e1(l_i[i2][0]-l_i[i1][0],l_i[i2][1]-l_i[i1][1]);
//                    T d_i = fabs(n_i1 .dot( diff_e1 ));
//                    if( angle_i < min_angle_pair && d_i < 40 ) // Check that the lines are not alomost coincident
//                        continue;
//                    for(size_t j1=0; j1 < n_j.size(); j1++)
//                    {
//                        if( abs(contrast_i[i1]-contrast_j[j1]) > th_contrast ) // Check scalar contrast descriptor
//                            continue;

//                        for(size_t j2=0; j2 < n_j.size(); j2++)
//                        {
//                            if( j1 == j2 )
//                                continue;
//                            bool already_checked = false;
//                            for(size_t k=0; k < discarded_matches.size(); k++)
//                                if(discarded_matches[k].count(i1) && discarded_matches[k][i1] == j1 && discarded_matches[k].count(i2) && discarded_matches[k][i2] == j2)
//                                {
//                                    already_checked = true;
//                                    break;
//                                }
//                            if( already_checked )
//                                continue;
//                            if( abs(contrast_i[i2]-contrast_j[j2]) > th_contrast ) // Check scalar contrast descriptor
//                                continue;

//                            T angle_i1 = atan2(l_i1_n[1], l_i1_n[0]);
//                            T angle_i2 = atan2(l_i2_n[1], l_i2_n[0]);
//                            T diff_angle_i = angle_i2- angle_i1;
//                            Eigen::Vector2f l_j1_n(l_j[j1][2]-l_j[j1][0], l_j[j1][3]-l_j[j1][1]); l_j1_n.normalize();
//                            Eigen::Vector2f l_j2_n(l_j[j2][2]-l_j[j2][0], l_j[j2][3]-l_j[j2][1]); l_j2_n.normalize();
//                            T angle_j1 = atan2(l_j1_n[1], l_j1_n[0]);
//                            T angle_j2 = atan2(l_j2_n[1], l_j2_n[0]);
//                            T diff_angle_j = angle_j2 - angle_j1;
//                            if( fabs( diff_angle_j - diff_angle_i ) > max_angle_diff )
//                                continue;

//                            T angle_1 = angle_j1 - angle_i1;
//                            T angle_2 = angle_j2 - angle_i2;
//                            T angle = (angle_1 + angle_2) * 0.5;
//                            H(0,0) = H(1,1) = cos(angle);
//                            H(1,0) = sin(angle);
//                            H(0,1) = -H(1,0);

//                            Eigen::Matrix<T,2,2> A;
//                            A.row(0) << -l_i1_n[1], l_i1_n[0];
//                            A.row(1) << -l_i2_n[1], l_i2_n[0];
//                            Eigen::Matrix<T,2,1> b;
//                            Eigen::Matrix<T,2,1> ej1_rot = H.block<2,2>(0,0)*l_j[j1].head(2);
//                            b(0) = -l_i1_n[1]*(l_i[i1][0] - ej1_rot(0)) + l_i1_n[0]*(l_i[i1][1] - ej1_rot(1));
//                            Eigen::Matrix<T,2,1> ej2_rot = H.block<2,2>(0,0)*l_j[j2].head(2);
//                            b(1) = -l_i2_n[1]*(l_i[i2][0] - ej2_rot(0)) + l_i2_n[0]*(l_i[i2][1] - ej2_rot(1));
//                            Eigen::Matrix<T,2,2> A_inv; A << A(1,1), -A(0,1), A(1,0) << A(0,0);
//                            A_inv /= (A(0,0)*A(1,1) - A(1,0)*A(0,1));
//                            H.block<2,1>(0,2) = A_inv*b;

////                            H = K*R*K_inv;


//                            // Compute rotation
//                            Eigen::Matrix<T,3,3> rot;
//                            Eigen::Matrix<T,3,3> cov = n_j[j1]*n_i[i1].transpose() + n_j[j2]*n_i[i2].transpose() + (n_j[j1].cross(n_j[j2]))*(n_i[i1].cross(n_i[i2])).transpose();
//                            T cond = calcRotationFromNormalVectorCov(cov, rot);
//                            //if(cond < min_conditioning){ //std::cout << "ExtrinsicCalibLines::matchNormalVectors: JacobiSVD bad conditioning " << cond << " < " << min_conditioning << "\n";
//                            //    continue; }
//                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << n_i[i1].dot(rot*n_j[j1]) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n";
//                            //if( n_i[i1].dot(rot*n_j[j1]) < max_angle_diff_cos ) // Check if the rotation is consistent
//                            //    continue;

//                            //std::cout << "Candidate pair " << i1 << "-" << i2 << " vs " << j1 << "-" << j2 << " error " << utils::RAD2DEG(acos(n_i[i1].dot(rot*n_j[j1]))) << " min_cos " << max_angle_diff_cos << " conditioning " << cond << "\n";

//                            // Check how many lines are consistent with this rotation
//                            std::map<size_t,size_t> matches;
//                            matches[i1] = j1;
//                            matches[i2] = j2;
//                            std::set<size_t> matches2; matches2.insert(j1); matches2.insert(j2);
//                            for(size_t i=0; i < n_i.size(); i++)
//                            {
//                                if( i==i1 || i==i2 )
//                                    continue;
//                                for(size_t j=0; j < n_j.size(); j++)
//                                {
//                                    if( matches2.count(j) )
//                                        continue;
//                                    if( abs(contrast_i[i]-contrast_j[j]) > th_contrast ) // Check scalar contrast descriptor
//                                        continue;

//                                    //std::cout << "error " << i << " vs " << j << " : " << n_i[i].dot(rot*n_j[j]) << " min " << max_angle_diff << "\n";
//                                    if( n_i[i].dot(rot*n_j[j]) > max_angle_diff_cos )
//                                    {
//                                        cov += n_j[j]*n_i[i].transpose();
//                                        for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++)
//                                            cov += (n_j[j].cross(n_j[it->second]))*(n_i[i].cross(n_i[it->first])).transpose();
//                                        matches[i] = j;
//                                        matches2.insert(j);
//                                    }
//                                }
//                            }

//                            // Compute the rotation from the inliers
//                            T error = 0;
//                            cond = calcRotationFromNormalVectorCov(cov, rot);
//                            for(std::map<size_t,size_t>::iterator it=matches.begin(); it != matches.end(); it++){ //std::cout << "match " << it->first << " - " << it->second << " error " << utils::RAD2DEG(acos(n_i[it->first] .dot (rot*n_j[it->second]))) << "\n";
//                                error += acos(n_i[it->first] .dot (rot*n_j[it->second]));}

//                            //std::cout << "Num corresp " << matches.size() << " conditioning " << cond << " error " << error << " average error " << utils::RAD2DEG(error/matches.size()) << " deg.\n";

//                            if(best_matches.size() < matches.size() && matches.size() > 2)
//                            {
//                                //if(matches.size() > 2)
//                                    discarded_matches.push_back(matches);

//                                best_matches = matches;
////                                best_score = error;
//                                rotation = rot;
//                                conditioning = cond;
//                            }
//                        }
//                    }
//                }
//            }
////            std::cout << " ...matchNormalVectors took " << 1000*clock.Tac() << " ms " << best_matches.size() << " matches, best_score " << utils::RAD2DEG(best_score) << " conditioning " << conditioning << "\n" << rotation << "\n";
//        //    for(map<size_t,size_t>::iterator it=best_matches.begin(); it != best_matches.end(); it++)
//        //        std::cout << "match " << it->first << " - " << it->second << "\n";

//            return best_matches;
//        }

        static Eigen::Matrix<T,3,1> computeTranslationSM(const Eigen::Matrix<T,3,3> & R, std::map<size_t, size_t> & sm_matches,
                                                         const std::vector<Eigen::Matrix<T,3,1> > & n_i, const std::vector<float> & depth_i, const std::vector<float> & depth_j)
        {
            // Calculate translation
            Eigen::Matrix<T,3,1> translation;
            Eigen::Matrix<T,3,3> hessian = Eigen::Matrix<T,3,3>::Zero();
            Eigen::Matrix<T,3,1> gradient = Eigen::Matrix<T,3,1>::Zero();
            T accum_error2 = 0.0;
            for(auto it = sm_matches.begin(); it != sm_matches.end(); it++)
            {
                T trans_error = (sm_matches[it->first] - sm_matches[it->second]); //+n*t
                accum_error2 += trans_error * trans_error;
                hessian += n_i[it->first].v3normal * n_i[it->first].v3normal.transpose();
                gradient += n_i[it->first].v3normal * trans_error;
            }
            translation = -hessian.inverse() * gradient;

            return translation;
        }

//        static Eigen::Matrix<T,3,1> computePoseSM(const Eigen::Matrix<T,3,3> & R, std::map<size_t, size_t> & sm_matches,
//                                                         const std::vector<Eigen::Matrix<T,3,1> > & n_i, const std::vector<float> & depth_i,
//                                                         const std::vector<Eigen::Matrix<T,3,1> > & n_j, const std::vector<float> & depth_j)
//        {
//            // Calculate translation
//            Eigen::Matrix<T,3,1> translation;
//            Eigen::Matrix<T,3,3> hessian = Eigen::Matrix<T,3,3>::Zero();
//            Eigen::Matrix<T,3,1> gradient = Eigen::Matrix<T,3,1>::Zero();
//            float accum_error2 = 0.0;
//            for(auto it = sm_matches.begin(); it != sm_matches.end(); it++)
//            {
//                float trans_error = (sm_matches[it->first] - sm_matches[it->second]); //+n*t
//                accum_error2 += trans_error * trans_error;
//                hessian += n_i[it->first].v3normal * n_i[it->first].v3normal.transpose();
//                gradient += n_i[it->first].v3normal * trans_error;
//            }
//            translation = -hessian.inverse() * gradient;

//            return translation;
//        }

        inline static void rotation2homography(const Eigen::Matrix<T,3,3> & R, const Eigen::Matrix<T,3,3> & K, Eigen::Matrix<T,3,3> & H)
        {
            Eigen::Matrix<T,3,3> K_inv;
            T fx_inv = 1/K(0,0);
            T fy_inv = 1/K(1,1);
            K_inv << fx_inv, 0, -K(0,2)*fx_inv, 0, fy_inv, -K(1,2)*fy_inv, 0, 0, 1;
            H = K*R*K_inv;
            H /= H(2,2);
        }

        inline static void homography2rotation(const Eigen::Matrix<T,3,3> & H, const Eigen::Matrix<T,3,3> & K, Eigen::Matrix<T,3,3> & R)
        {
            Eigen::Matrix<T,3,3> K_inv;
            T fx_inv = 1/K(0,0);
            T fy_inv = 1/K(1,1);
            K_inv << fx_inv, 0, -K(0,2)*fx_inv, 0, fy_inv, -K(1,2)*fy_inv, 0, 0, 1;
            R = K_inv*H*K;
            //normalize
        }
	};

  } // End of namespace
} // End of namespace

#endif
