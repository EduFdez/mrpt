/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "line_corresp.h"

using namespace std;

LineCorresp::LineCorresp(size_t sensors) : FeatCorresp(sensors)
{
    std::cout << "LineCorresp... n_sensors " << n_sensors << std::endl;
    for(size_t sensor1=0; sensor1 < n_sensors; sensor1++)
    {
        mm_corresp[sensor1] = std::map<size_t, mrpt::math::CMatrixDouble>();
        for(size_t sensor2=sensor1+1; sensor2 < n_sensors; sensor2++)
            mm_corresp[sensor1][sensor2] = mrpt::math::CMatrixDouble(0, 10);
    }
}
