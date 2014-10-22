

/*
 * Copyright 2014,
 * Mehdi Benallegue
 *
 * CNRS
 *
 * This file is part of dynamic-graph-tutorial.
 * dynamic-graph-tutorial is free software: you can redistribute it and/or
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


#ifndef DISCRETETIMENONALGEBRAICRICCATIEQUATION
#define DISCRETETIMENONALGEBRAICRICCATIEQUATION

#include <state-observation/tools/definitions.hpp>


namespace sotStabilizer {
namespace tools {
void discreteTimeNonAlgRiccatiEqn (
       const stateObservation::Matrix & A, const stateObservation::Matrix & B,
       const stateObservation::Matrix & Q, const stateObservation::Matrix & R,
       const stateObservation::Matrix & Qn, stateObservation::IndexedMatrixArray &P,
       unsigned n);
}
}

#endif//DISCRETETIMENONALGEBRAICRICCATIEQUATION
