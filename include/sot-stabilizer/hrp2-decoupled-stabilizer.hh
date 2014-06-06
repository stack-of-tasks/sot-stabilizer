//
// Copyright (c) 2012,
// Florent Lamiraux
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

#ifndef SOT_DYNAMIC_STABILIZER_HH
# define SOT_DYNAMIC_STABILIZER_HH

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>

#include <sot/core/task-abstract.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/multi-bound.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

namespace sotStabilizer
{
using dynamicgraph::sot::TaskAbstract;
using dynamicgraph::Signal;
using dynamicgraph::SignalPtr;
using dynamicgraph::SignalTimeDependent;
using dynamicgraph::Vector;
using dynamicgraph::Matrix;
using dynamicgraph::Entity;
using dynamicgraph::sot::VectorMultiBound;
using dynamicgraph::sot::MatrixHomogeneous;
using dynamicgraph::sot::MatrixRotation;
using dynamicgraph::sot::VectorUTheta;
using dynamicgraph::sot::VectorRollPitchYaw;

/// Dynamic balance stabilizer
///
/// This task takes as input four signals
/// \li comSIN, the position of the center of mass (COM)
/// \li comDesSIN, the the desired position of the center of mass,
/// \li zmpSIN, the position of the center of pressure (ZMP),
/// \li zmpDesSIN, the desired position of the center of pressure,
/// \li jacobianSIN, the jacobian of the center of mass,
/// \li comdotSIN, reference velocity of the center of mass,
/// and provides as output two signals
/// \li taskSOUT, the desired time derivative of the center of mass,
/// \li jacobianSOUT, the jacobian of the center of mass
class HRP2DecoupledStabilizer : public TaskAbstract
{
    DYNAMIC_GRAPH_ENTITY_DECL ();
public:
    // Constant values
    static double m_;
    static double g_;

    /// Constructor by name
    HRP2DecoupledStabilizer(const std::string& inName);
    ~HRP2DecoupledStabilizer() {}

    /// Documentation of the entity
    virtual std::string getDocString () const
    {
        std::string doc =
            "Dynamic balance humanoid robot stabilizer\n"
            "\n"
            "This task aims at controlling balance for a walking legged humanoid"
            " robot.\n"
            "The entity takes 6 signals as input:\n"
            "  - deltaCom: the difference between the position of the center of "
            "mass and the\n"
            " reference,\n"
            "  - Jcom: the Jacobian of the center of mass wrt the robot "
            "configuration,\n"
            "  - comdot: the reference velocity of the center of mass \n"
            "  \n"
            "As any task, the entity provide two output signals:\n"
            "  - task: the velocity of the center of mass so as to cope with\n"
            "          perturbations,\n"
            "  - jacobian: the Jacobian of the center of mass with respect to "
            "robot\n"
            "              configuration.\n";
        return doc;
    }

    /// Start stabilizer
    void start ()
    {
        on_ = true;
    }

    /// Start stabilizer
    void stop ()
    {
        on_ = false;
    }

    /// @}
    /// \name Sampling time period
    /// @{

    /// \brief Set sampling time period
    void setTimePeriod(const double& inTimePeriod)
    {
        dt_ = inTimePeriod;
    }
    /// \brief Get sampling time period
    double getTimePeriod() const
    {
        return dt_;
    }
    /// @}

private:

    /// Compute the control law
    VectorMultiBound& computeControlFeedback(VectorMultiBound& comdot,
            const int& time);

    Matrix& computeJacobianCom(Matrix& jacobian, const int& time);

    /// Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comSIN_;
    /// Reference position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comRefSIN_;
    /// Position of center of mass
    SignalPtr < dynamicgraph::Matrix, int> jacobianSIN_;
    /// Reference velocity of the center of mass
    SignalPtr < dynamicgraph::Vector, int> comdotSIN_;
    // Position of left foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
    // Position of right foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
    // Force in left foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceLeftFootSIN_;
    // Force in right foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceRightFootSIN_;
    // State of the flexibility
    SignalPtr <MatrixHomogeneous, int> stateFlexSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> stateFlexDotSIN_;
    SignalPtr <double, int> controlGainSIN_;
    // Acceleration of center of mass
    SignalTimeDependent <dynamicgraph::Vector, int> d2comSOUT_;
    // Number of support feet
    SignalTimeDependent <unsigned int, int> nbSupportSOUT_;

    // Contact Position
    SignalTimeDependent <Vector, int> supportPos1SOUT_;
    SignalTimeDependent <Vector, int> supportPos2SOUT_;

    //error output
    SignalTimeDependent <Vector, int> errorSOUT_;

    //error output
    SignalTimeDependent <Vector, int> debugSOUT_;


    Vector computeGainsFromPoles(const Vector & pole, double comHeight, double kth, double mass) const;


    /// Gains single support
    Vector gain1_;
    /// Gains double support
    Vector gain2_;
    /// Gains z correction
    Vector gainz_;
    /// Gains lateral correction
    Vector gainLat_;
    /// Store center of mass for finite-difference evaluation of velocity
    Vector prevCom_;
    /// coordinates of center of mass velocity in moving frame
    Vector dcom_;
    // Time sampling period
    double dt_;
    // Whether stabilizer is on
    bool on_;
    // Threshold on normal force above which the foot is considered in contact
    double forceThreshold_;
    // Angular stiffness of flexibility of each foot
    double angularStiffness_;
    // Number of feet in support
    unsigned int nbSupport_;
    // Acceleration of the center of mass computed by stabilizer
    Vector d2com_;
    // Deviation of center of mass
    Vector deltaCom_;

    MatrixHomogeneous flexPosition_;
    MatrixHomogeneous flexPositionLf_;
    MatrixHomogeneous flexPositionRf_;
    MatrixHomogeneous flexPositionLat_;
    Vector flexVelocity_;
    Vector flexVelocityLf_;
    Vector flexVelocityRf_;
    Vector flexVelocityLat_;

    // Observation of vertical flexibility: (\zeta - \zeta_{ref}, Fz)
    Vector flexZobs_;
    // Control of lateral flexibility in double support
    Vector flexLatControl_;
    // Observation of lateral flexibility in double support
    Vector flexLatObs_;

    double timeBeforeFlyingFootCorrection_;
    unsigned int iterationsSinceLastSupportLf_;
    unsigned int iterationsSinceLastSupportRf_;
    unsigned int supportCandidateLf_;
    unsigned int supportCandidateRf_;
    // Temporary variables for internal computation
    VectorUTheta uth_;
    MatrixRotation R_;
    Vector translation_;
    Vector zmp_;
    double u2x_, u2y_, u1x_, u1y_;
    // Lateral deflection in double support
    double theta1Ref_;
    double theta1RefPrev_;
    double dtheta1Ref_;
    Vector debug_;

    double kth_;
    double kz_;

    Vector poles1_;
    Vector poles2_;
    Vector polesLat_;

    double hrp2Mass_;
}; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DYNAMIC_STABILIZER_HH

