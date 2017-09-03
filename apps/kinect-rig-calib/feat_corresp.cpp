/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "feat_corresp.h"
#include <mrpt/system/filesystem.h>
#include <iostream>

//using namespace std;
//using namespace mrpt::math;

/*! Constructor */
FeatCorresp::FeatCorresp(const size_t sensors, const size_t length) : n_sensors(sensors)
{
    std::cout << "PlaneCorresp... n_sensors " << n_sensors << std::endl;
    for(size_t sensor1=0; sensor1 < n_sensors; sensor1++)
    {
        mm_corresp[sensor1] = std::map<size_t, mrpt::math::CMatrixDouble>();
        for(size_t sensor2=sensor1+1; sensor2 < n_sensors; sensor2++)
            mm_corresp[sensor1][sensor2] = mrpt::math::CMatrixDouble(0, length);
    }
}

/*! Load the plane correspondences between the different Asus sensors from file */
void FeatCorresp::saveCorrespondences(const std::string & dir, const std::string name)
{
    for(std::map<size_t, std::map<size_t, mrpt::math::CMatrixDouble> >::iterator it_pair1=mm_corresp.begin();
        it_pair1 != mm_corresp.end();
        it_pair1++)
    {
        for(std::map<size_t, mrpt::math::CMatrixDouble>::iterator it_pair2=it_pair1->second.begin();
            it_pair2 != it_pair1->second.end(); it_pair2++)
        {
            if(it_pair2->second.rows() > 0)
                it_pair2->second.saveToTextFile( mrpt::format("%s/corresp_%s_%u_%u.txt", dir.c_str(), name.c_str(), it_pair1->first, it_pair2->first) );
        }
    }
}

/*! Load the plane correspondences between the different Asus sensors from file */
void FeatCorresp::loadCorrespondences(const std::string dir, const std::string name)
{
    mm_corresp.clear();
    for(size_t sensor1 = 0; sensor1 < n_sensors-1; sensor1++)
    {
        mm_corresp[sensor1] = std::map<size_t, mrpt::math::CMatrixDouble>();
        for(size_t sensor2 = sensor1+1; sensor2 < n_sensors; sensor2++)
        {
            std::string fileCorresp = mrpt::format("%s/corresp_%s_%u_%u.txt", dir.c_str(), name.c_str(), sensor1, sensor2);
            if( mrpt::system::fileExists(fileCorresp) )
            {
                mrpt::math::CMatrixDouble correspMat;
                correspMat.loadFromTextFile(fileCorresp);
                mm_corresp[sensor1][sensor2] = correspMat;
                //          std::cout << "Load FeatCorresp " << sensor1 << " and " << sensor2 << std::endl;
                //          std::cout << correspMat.rows() << " correspondences " << std::endl;
            }
        }
    }
    std::cout << "FeatCorresp::loadFeatCorrespondences -> " << mm_corresp.size() << " sensors \n";
}

/*! Load the plane correspondences between a pair of Asus sensors from file */
mrpt::math::CMatrixDouble FeatCorresp::loadCorrespondencesPair(const std::string file)
{
    if( !mrpt::system::fileExists(file) )
        throw std::runtime_error("ERROR FeatCorresp::getFeatCorrespondences -> wrong matchedPlanesFile\n\n");

    mrpt::math::CMatrixDouble correspMat;
    correspMat.loadFromTextFile(file);
    std::cout << "FeatCorresp::loadCorrespondencesPair " << correspMat.rows() << " correspondences " << std::endl;

    return correspMat;
}
