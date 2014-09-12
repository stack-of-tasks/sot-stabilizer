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

#ifndef SOT_DYNAMIC_STABILIZER_HRP2_LQR_HH
# define SOT_DYNAMIC_STABILIZER_HRP2_LQR_HH

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

#include <sot-stabilizer/controllers/discrete-time-lti-lqr.hh>
#include <sot-state-observation/tools/definitions.hh>

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
  class HRP2LQRDecoupledStabilizer : public TaskAbstract
  {
    DYNAMIC_GRAPH_ENTITY_DECL ();
  public:
    // Constant values
    static double constm_;
    static double constcomHeight_;
    static double conststepLength_;

    /// Constructor by name
    HRP2LQRDecoupledStabilizer(const std::string& inName);
    ~HRP2LQRDecoupledStabilizer() {}

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

    void setFixedGains(const bool & b)
    {
      fixedGains_=b;
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


    void setStateCost1(const Matrix & Q)
    {
      Q1_=sotStateObservation::convertMatrix<stateObservation::Matrix>(Q);
      controller1x_.setCostMatrices(Q1_,R_);
      controller1y_.setCostMatrices(Q1_,R_);
    }

    void setStateCost2(const Matrix & Q)
    {
      Q2_=sotStateObservation::convertMatrix<stateObservation::Matrix>(Q);
      controller2_.setCostMatrices(Q2_,R_);
    }

    void setStateCostLat(const Matrix & Q)
    {
      Qlat_=sotStateObservation::convertMatrix<stateObservation::Matrix>(Q);
      controllerLat_.setCostMatrices(Qlat_,R_);

    }

    void setHorizon(const int & horizon)
    {
      controller1x_.setHorizonLength(horizon);
      controller1y_.setHorizonLength(horizon);
      controller2_.setHorizonLength(horizon);
      controllerLat_.setHorizonLength(horizon);
    }


    void setInputCost(const Matrix & R)
    {
      R_=sotStateObservation::convertMatrix<stateObservation::Matrix>(R);
      controllerLat_.setCostMatrices(Qlat_,R_);
      controller2_.setCostMatrices(Q2_,R_);
      controller1x_.setCostMatrices(Q1_,R_);
      controller1y_.setCostMatrices(Q1_,R_);

    }

    void setZMPMode(const bool & mode)
    {
      zmpMode_=mode;
      if (zmpMode_)
      {
        taskSOUT.addDependency(zmpRefSIN_);
        taskSOUT.removeDependency(comddotRefSIN_);
      }
      else
      {
        taskSOUT.removeDependency(zmpRefSIN_);
        taskSOUT.addDependency(comddotRefSIN_);
      }
    }

    Matrix getStateCost1() const
    {
      return sotStateObservation::convertMatrix<Matrix>(Q1_);
    }

    Matrix getStateCost2() const
    {
      return sotStateObservation::convertMatrix<Matrix>(Q2_);
    }

    Matrix getStateCostLat() const
    {
      return sotStateObservation::convertMatrix<Matrix>(Qlat_);
    }

    Matrix getInputCost() const
    {
      return sotStateObservation::convertMatrix<Matrix>(R_);
    }


    Matrix getLastGain1x() const
    {
      return sotStateObservation::convertMatrix<Matrix>(controller1x_.getLastGain());
    }

    Matrix getLastGain1y() const
    {
      return sotStateObservation::convertMatrix<Matrix>(controller1y_.getLastGain());
    }

    Matrix getLastGain2() const
    {
      return sotStateObservation::convertMatrix<Matrix>(controller2_.getLastGain());
    }

    Matrix getLastGainLat() const
    {
      return sotStateObservation::convertMatrix<Matrix>(controllerLat_.getLastGain());
    }

    inline stateObservation::Matrix computeDynamicsMatrix
      (double comHeight, double x, double xdot, double kth, double kdth, double mass);

    inline stateObservation::Matrix computeInputMatrix
      (double comHeight, double x, double kth, double kdth, double mass);


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
    SignalPtr < dynamicgraph::Vector, int> comdotRefSIN_;

    ///Reference com ddot
    SignalPtr < dynamicgraph::Vector, int> comddotRefSIN_;

    ///Reference ZMP
    SignalPtr < dynamicgraph::Vector, int> zmpRefSIN_;

    /// Position of left foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
    /// Position of right foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
    /// Force in left foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceLeftFootSIN_;
    /// Force in right foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceRightFootSIN_;
    /// State of the flexibility
    SignalPtr <MatrixHomogeneous, int> stateFlexSIN_;
    /// Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> stateFlexDotSIN_;

    /// Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> stateFlexDDotSIN_;

    SignalPtr <double, int> controlGainSIN_;

    /// Acceleration of center of mass
    SignalTimeDependent <dynamicgraph::Vector, int> comdotSOUT_;

    /// Acceleration of center of mass
    SignalTimeDependent <dynamicgraph::Vector, int> comddotSOUT_;

    /// Acceleration of center of mass
    SignalTimeDependent <dynamicgraph::Vector, int> comddotRefSOUT_;

    /// Number of support feet
    SignalTimeDependent <unsigned int, int> nbSupportSOUT_;

    /// Contact Position
    SignalTimeDependent <Vector, int> supportPos1SOUT_;
    SignalTimeDependent <Vector, int> supportPos2SOUT_;

    ///error output
    SignalTimeDependent <Vector, int> errorSOUT_;

    SignalTimeDependent <Vector, int> debugSOUT_;


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

    Vector comddotRef_;

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

    double timeBeforeFlyingFootCorrection_;
    unsigned int iterationsSinceLastSupportLf_;
    unsigned int iterationsSinceLastSupportRf_;
    unsigned int supportCandidateLf_;
    unsigned int supportCandidateRf_;
    // Temporary variables for internal computation
    VectorUTheta uth_;
    Vector translation_;
    Vector zmp_;
    double u2x_, u2y_, u1x_, u1y_;
    // Lateral deflection in double support
    double theta1Ref_;
    double theta1RefPrev_;
    double dtheta1Ref_;
    Vector debug_;

    stateObservation::Matrix Q1_,Q2_,Qlat_;
    stateObservation::Matrix R_;

    stateObservation::Matrix A1x_;
    stateObservation::Matrix A1y_;
    stateObservation::Matrix A2_;
    stateObservation::Matrix ALat_;

    stateObservation::Matrix Bx_;
    stateObservation::Matrix By_;

    controller::DiscreteTimeLTILQR controller1x_;
    controller::DiscreteTimeLTILQR controller1y_;
    controller::DiscreteTimeLTILQR controller2_;
    controller::DiscreteTimeLTILQR controllerLat_;


    double kth_;
    double kdth_;
    double kz_;

    bool fixedGains_;

    bool zmpMode_;

    double hrp2Mass_;
  }; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DYNAMIC_STABILIZER_HRP2_LQR_HH

