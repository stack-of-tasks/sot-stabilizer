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

#ifndef SOT_DECOUPLED_ELASTIC_INV_PENDULUM_SIMULATOR
# define SOT_DECOUPLED_ELASTIC_INV_PENDULUM_SIMULATOR

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-homogeneous.hh>

#include <sot-state-observation/tools/definitions.hh>

#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <state-observation/dynamical-system/bidim-elastic-inv-pendulum-dyn-sys.hpp>



/// Decoupled elastic inverted pendulum simulator
///
/// This entity takes as input
/// \li com accleration of the center of mass (COM)

/// and provides as output
/// \li com new position of the CoM
/// \li the new deformation (rotation vector)
/// \li the deformation velocity (angular velocity vector)

namespace sotStabilizer
{

class ZMPtoCoMAlgerbraicController : public dynamicgraph::Entity
{

public:
    // Constant values

    /// Constructor by name
    ZMPtoCoMAlgerbraicController(const std::string& inName);
    ~ZMPtoCoMAlgerbraicController() {}

    /// Documentation of the entity
    virtual std::string getDocString () const
    {
        std::string doc =
            "simple controller which generates the CoM acceleration\n"
            "from the CoM position and the current acceleration.\n"
            "The output acceleration is the difference between the\n"
            "current acceleration and the necessary one.";
        return doc;
    }

    /// Each entity should provide the name of the class it belongs to
    virtual const std::string& getClassName (void) const
    {
        return CLASS_NAME;
    }

    /// @}
    /// \name Sampling time period
    /// @{

protected:
            /*
            \brief Class name
            */
    static const std::string CLASS_NAME;

private:

    /// position for the center of mass reset
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comSIN_;
    /// height of the center of mass
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comHeightSIN_;
    /// Current acceleration of center of mass
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comddotSIN_;
    /// Reference position of the ZMP
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> zmprefSIN_;
    /// Additional acceleration of center of mass
    dynamicgraph::Signal < ::dynamicgraph::Vector, int> comddotSOUT_;

        /// Compute the control law
    ::dynamicgraph::Vector& computeddCom(::dynamicgraph::Vector& com,
            const int& time);

}; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DECOUPLED_ELASTIC_INV_PENDULUM_SIMULATOR

