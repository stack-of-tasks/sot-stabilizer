//
// Copyright (c) 2012,
// Mehdi Benallegue
//
// CNRS
//
// This file is part of sot-dynamic.
// sot-dynamic is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// sot-dynamic is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
//

#include <sstream>

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>

#include <dynamic-graph/factory.h>
#include <jrl/mal/matrixabstractlayervector3jrlmath.hh>
#include <sot/core/vector-utheta.hh>

#include <sot/core/flags.hh>
#include <state-observation/tools/definitions.hpp>

#include <sot-stabilizer/prototyping/zmp-to-com-algebraic-controller.hh>

namespace sotStabilizer
{

using dynamicgraph::command::docDirectSetter;
using dynamicgraph::command::makeDirectSetter;
using dynamicgraph::command::docDirectGetter;
using dynamicgraph::command::makeDirectGetter;
using dynamicgraph::command::makeCommandVoid0;
using dynamicgraph::command::docCommandVoid0;
using dynamicgraph::Vector;
using dynamicgraph::Matrix;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( ZMPtoCoMAlgerbraicController,  "ZMPtoCoMAlgerbraicController" );

ZMPtoCoMAlgerbraicController::ZMPtoCoMAlgerbraicController(const std::string& inName) :
    dynamicgraph::Entity(inName),
    comSIN_ (0x0, "ZMPtoCoMAlgerbraicController("+inName+")::input(vector)::comIn"),
    comHeightSIN_ (0x0, "ZMPtoCoMAlgerbraicController("+inName+")::input(vector)::comHeight"),
    comddotSIN_ (0x0, "ZMPtoCoMAlgerbraicController("+inName+")::input(vector)::comddotIN"),
    zmprefSIN_ (0x0, "ZMPtoCoMAlgerbraicController("+inName+")::input(vector)::zmpref"),
    comddotSOUT_ ("ZMPtoCoMAlgerbraicController("+inName+")::output(vector)::comddot")
{
    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (comHeightSIN_);
    signalRegistration (comddotSIN_);
    signalRegistration (zmprefSIN_);
    signalRegistration (comddotSOUT_);

    comddotSOUT_.addDependency(comSIN_);
    comddotSOUT_.addDependency(comHeightSIN_);
    comddotSOUT_.addDependency(comddotSIN_);
    comddotSOUT_.addDependency(zmprefSIN_);

    comddotSOUT_.setFunction (boost::bind(&ZMPtoCoMAlgerbraicController::computeddCom,
                                          this,_1,_2));

    dynamicgraph::Vector zeroV(1); zeroV.setZero();
    //comddotSOUT_.setConstant(zeroV);
}



/// Compute the control law
Vector&
ZMPtoCoMAlgerbraicController::computeddCom(Vector& comddot,
                                   const int& time)
{
    comddot.resize(1);

    comddot(0) = (stateObservation::cst::gravityConstant/comHeightSIN_(time)(0))*
                   (comSIN_(time)(0) - zmprefSIN_(time)(0) - comddotSIN_(time)(0));

    return comddot;

}

} // namespace sotStabilizer
