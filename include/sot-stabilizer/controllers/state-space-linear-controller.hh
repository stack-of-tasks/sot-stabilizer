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

#ifndef STATESPACELINEARCONTROLLER_H
#define STATESPACELINEARCONTROLLER_H

#include <state-observation/tools/definitions.hpp>
#include <sot-stabilizer/controllers/controller-base.hh>

namespace sotStabilizer
{
namespace controller
{

class StateSpaceLinearController: public ControllerBase
{
public:

    StateSpaceLinearController(unsigned stateSize, unsigned controlSize);

    virtual ~StateSpaceLinearController(){}

    /// available methods
    // - setState
    // - getState
    // - getTime

    /// virtual methods inherited from parent

    virtual unsigned getStateSize() const ;
    virtual unsigned getControlSize() const ;
    virtual stateObservation::Vector getControl(int time);

    /// \brief Get the state at time+1 if the system were exactly the caracteristic polynomial in a_
    virtual stateObservation::Vector getTheoricalState(stateObservation::Vector &u, const double dt);


    /// overladen methods



    /// new methods

    virtual void setCaracteristicPolynomial(const stateObservation::Vector &a);
    virtual void setCaracteristicPolynomialFromPoles(const stateObservation::Matrix &p);
    virtual stateObservation::Vector getCaracteristicPolynomial();

protected:

    /// available methods and attributes
    // - checkState()
    // - x_
    // - time_

    /// new methods and attributes

    unsigned stateSize_;
    unsigned controlSize_;
    bool computedInput_;

    stateObservation::Vector u_;

    // Caracteristic polynomial coefficients
    stateObservation::Vector a_;

};
}
}

#endif // STATESPACELINEARCONTROLLER_H
