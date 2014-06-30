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
#include <state-observation/dynamical-system/bidim-elastic-inv-pendulum.hpp>



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

class DecoupledElasticInvPendulum : public dynamicgraph::Entity
{

public:
    // Constant values

    /// Constructor by name
    DecoupledElasticInvPendulum(const std::string& inName);
    ~DecoupledElasticInvPendulum() {}

    /// Documentation of the entity
    virtual std::string getDocString () const
    {
        std::string doc =
            "Decoupled elastic inverted pendulum \n"
            "\n"
            "This entity takes as input\n"
            "  -com accleration of the center of mass (COM)\n"
            "  -number of contacts"
            "  -position of contact1"
            "  -position of contact2"
            "and provides as output\n"
            "  -com new position of the CoM\n"
            "  -the new deformation (rotation vector)\n"
            "  -the deformation velocity (angular velocity vector)";
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

    /// \brief Set sampling time period
    virtual void setTimePeriod(const double& inTimePeriod)
    {
        dt_ = inTimePeriod;
    }
    /// \brief Get sampling time period
    virtual double getTimePeriod() const
    {
        return dt_;
    }
    /// @}

    virtual void resetCoM()
    {

        com_=comSIN_ (comSOUT_.getTime());
    }

protected:
            /*
            \brief Class name
            */
    static const std::string CLASS_NAME;

private:

    /// Compute the control law
    ::dynamicgraph::Vector& computeCom(::dynamicgraph::Vector& com,
            const int& time);

    double dt_;
    double kth_;
    double kz_;
    double m_;
    double comh_;

    ::dynamicgraph::Vector com_;
    ::dynamicgraph::Vector comdot_;
    ::dynamicgraph::Vector flex_;
    ::dynamicgraph::Vector flexdot_;


    /// position for the center of mass reset
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comSIN_;
    /// Acceleration of center of mass
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comddotSIN_;
    /// Acceleration of center of mass
    dynamicgraph::SignalPtr < unsigned, int> contactNbrSIN_;
    /// Acceleration of center of mass
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> contact1SIN_;
    /// Acceleration of center of mass
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> contact2SIN_;


    /// Position of center of mass
    dynamicgraph::Signal <dynamicgraph::Vector, int> comSOUT_;
    /// Position of center of mass
    dynamicgraph::Signal <dynamicgraph::Vector, int> comdotSOUT_;
    /// Flexibility orientation
    dynamicgraph::Signal <dynamicgraph::Vector, int> flexSOUT_;
    /// Flexibility velocity
    dynamicgraph::Signal <dynamicgraph::Vector, int> flexdotSOUT_;


    stateObservation::BidimElasticInvPendulum p;


}; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DECOUPLED_ELASTIC_INV_PENDULUM_SIMULATOR

