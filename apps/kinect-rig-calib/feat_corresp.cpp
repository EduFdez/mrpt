/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "feat_corresp.h"

//using namespace std;
//using namespace mrpt::math;

/*! Load the plane correspondences between the different Asus sensors from file */
void FeatCorresp::saveCorrespondences(const std::string & dir, const std::string name)
{
    for(std::map<unsigned, std::map<unsigned, mrpt::math::CMatrixDouble> >::iterator it_pair1=mm_corresp.begin();
        it_pair1 != mm_corresp.end();
        it_pair1++)
    {
        for(std::map<unsigned, mrpt::math::CMatrixDouble>::iterator it_pair2=it_pair1->second.begin();
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
    for(unsigned sensor_id = 0; sensor_id < n_sensors_-1; sensor_id++)
    {
        mm_corresp[sensor_id] = std::map<unsigned, mrpt::math::CMatrixDouble>();
        for(unsigned sensor_corresp = sensor_id+1; sensor_corresp < n_sensors_; sensor_corresp++)
        {
            std::string fileCorresp = mrpt::format("%s/corresp_%s_%u_%u.txt", dir.c_str(), name.c_str(), sensor_id, sensor_corresp);
            if( fileExists(fileCorresp) )
            {
                mrpt::math::CMatrixDouble correspMat;
                correspMat.loadFromTextFile(fileCorresp);
                mm_corresp[sensor_id][sensor_corresp] = correspMat;
                //          std::cout << "Load FeatCorresp " << sensor_id << " and " << sensor_corresp << std::endl;
                //          std::cout << correspMat.rows() << " correspondences " << std::endl;
            }
        }
    }
    std::cout << "FeatCorresp::loadFeatCorrespondences -> " << mm_corresp.size() << " sensors \n";
}

/*! Load the plane correspondences between a pair of Asus sensors from file */
mrpt::math::CMatrixDouble FeatCorresp::loadCorrespondencesPair(const std::string file)
{
    if( !fileExists(file) )
        throw std::runtime_error("ERROR FeatCorresp::getFeatCorrespondences -> wrong matchedPlanesFile\n\n");

    mrpt::math::CMatrixDouble correspMat;
    correspMat.loadFromTextFile(file);
    std::cout << "FeatCorresp::loadCorrespondencesPair " << correspMat.rows() << " correspondences " << std::endl;

    return correspMat;
}
