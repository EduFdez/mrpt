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

LineCorresp::LineCorresp(size_t sensors) :
    n_sensors(sensors)
{
    std::cout << "LineCorresp... n_sensors_ " << n_sensors_ << std::endl;
    for(unsigned sensor_id1=0; sensor_id1 < n_sensors_; sensor_id1++)
    {
        mm_corresp[sensor_id1] = std::map<unsigned, mrpt::math::CMatrixDouble>();
        for(unsigned sensor_id2=sensor_id1+1; sensor_id2 < n_sensors_; sensor_id2++)
            mm_corresp[sensor_id1][sensor_id2] = mrpt::math::CMatrixDouble(0, 10);
    }

}
