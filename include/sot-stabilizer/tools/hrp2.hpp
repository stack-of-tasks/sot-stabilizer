/*
 * Copyright 2014,
 * Alexis Mifsud
 *
 * CNRS
 *
 * This file is part of sot-stabilizer.
 * sot-stabilizer is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * dynamic-graph-tutorial is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with dynamic-graph-tutorial.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HRP2CONSTANTS
#define HRP2CONSTANTS

#include <vector>
#include <deque>

#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
#   include <iostream>
#endif

#include <boost/assert.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sotStabilizer
{

    namespace hrp2
    {
        const double m=59.9; //56.8679920;//59.8;//59.8; // 58 ds la doc
        const double H=0.807;//1.54;
        const double R=0.31;

        const double angKe=600;//510;
        const double angKv=60;
    }

}

#endif //HRP2CONSTANTS
