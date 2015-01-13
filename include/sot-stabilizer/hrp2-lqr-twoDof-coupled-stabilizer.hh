//
// Copyright (c) 2014,
// Alexis Mifsud
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

#ifndef SOT_DYNAMIC_STABILIZER_HRP2_TWODOF_LQR_HH
# define SOT_DYNAMIC_STABILIZER_HRP2_TWODOF_LQR_HH

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

  using namespace sotStateObservation;
  using namespace stateObservation;

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
  class HRP2LQRTwoDofCoupledStabilizer : public TaskAbstract
  {
    DYNAMIC_GRAPH_ENTITY_DECL ();
  public:
    // Constant values
    static double constm_;
    static double constcomHeight_;
    static double conststepLength_;

    /// Constructor by name
    HRP2LQRTwoDofCoupledStabilizer(const std::string& inName);
    ~HRP2LQRTwoDofCoupledStabilizer() {}

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


    void setStateCost(const Matrix & Q)
    {
      Q_=sotStateObservation::convertMatrix<stateObservation::Matrix>(Q);
      controller_.setCostMatrices(Q_,R_);
    }

      void setHorizon(const int & horizon)
    {
      controller_.setHorizonLength(horizon);
    }


    void setInputCost(const Matrix & R)
    {
      R_=sotStateObservation::convertMatrix<stateObservation::Matrix>(R);
      controller_.setCostMatrices(Q_,R_);
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

    Matrix getStateCost() const
    {
      return sotStateObservation::convertMatrix<Matrix>(Q_);
    }

    Matrix getInputCost() const
    {
      return sotStateObservation::convertMatrix<Matrix>(R_);
    }


    Matrix getLastGain() const
    {
      return sotStateObservation::convertMatrix<Matrix>(controller_.getLastGain());
    }


    void setKth(const dynamicgraph::Matrix & m)
    {
        Kth_=convertMatrix<stateObservation::Matrix>(m);
    }

    void setKdth(const dynamicgraph::Matrix & m)
    {
        Kdth_=convertMatrix<stateObservation::Matrix>(m);
    }


    inline void computeDynamicsMatrix(const stateObservation::Vector cl, const stateObservation::Matrix Kth, const stateObservation::Matrix Kdth, const int& time);

  private:

    /// Compute the number of supports
    unsigned int computeNbSupport(const int& time);

    /// Compute the control law
    VectorMultiBound& computeControlFeedback(VectorMultiBound& comdot,
        const int& time);

    Matrix& computeJacobian(Matrix& jacobian, const int& time);

    stateObservation::Matrix3 computeInert(const stateObservation::Vector& com, const int& time);

    /// References
    // Reference position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comRefgSIN_;
    // Reference orientation of the waist
    SignalPtr < dynamicgraph::Vector, int > waistOriRefgSIN_;

    // Reference velocity of the center of mass
    SignalPtr < dynamicgraph::Vector, int> comdotRefSIN_;
    // Reference com ddot
    SignalPtr < dynamicgraph::Vector, int> comddotRefSIN_;

    /// Com
    // Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comSIN_;
    // Velocity of center of mass
    SignalPtr < dynamicgraph::Vector, int> comDotSIN_;

    /// Waist
    // Orientation of the waist
    SignalPtr < dynamicgraph::Vector, int > waistOriSIN_;
        // Refrence angular velocity of the waist
    SignalPtr < dynamicgraph::Vector , int > waistAngVelSIN_;
    // Reference  angular acceleration of the waist
    SignalPtr < dynamicgraph::Vector, int> waistAngAccSIN_;

    /// Flexibility
    // State of the flexibility
    SignalPtr <MatrixHomogeneous, int> stateFlexSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> flexAngVelVectSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> stateFlexDDotSIN_;
    // Orientation of the flexibility on vector form
    SignalPtr <dynamicgraph::Vector, int> flexOriVectSIN_;



    SignalPtr < dynamicgraph::Matrix, int> jacobianComSIN_;
    SignalPtr < dynamicgraph::Matrix, int> jacobianWaistSIN_;
    SignalPtr <double, int> controlGainSIN_;

    ///Reference ZMP
    SignalPtr < dynamicgraph::Vector, int> zmpRefSIN_;

    /// Signals to compute number of supports
    // Position of left foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
    // Position of right foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
    // Force in left foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceLeftFootSIN_;
    // Force in right foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceRightFootSIN_;


    /// Signals to compute dynamics model
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> inertiaSIN;
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> positionWaistSIN;

    // Velocity of center of mass
    SignalTimeDependent <dynamicgraph::Vector, int> comdotSOUT_;
    // Acceleration of center of mass
    SignalTimeDependent <dynamicgraph::Vector, int> comddotSOUT_;
    // Reference acceleration of center of mass
    SignalTimeDependent <dynamicgraph::Vector, int> comddotRefSOUT_;

    /// Number and position of supports
    // Number of support feet
    SignalTimeDependent <unsigned int, int> nbSupportSOUT_;
    // Contact Position
    SignalTimeDependent <Vector, int> supportPos1SOUT_;
    SignalTimeDependent <Vector, int> supportPos2SOUT_;

    ///error output
    SignalTimeDependent <Vector, int> errorSOUT_;

    ///Reference ZMP
    SignalTimeDependent <Vector, int> zmpRefSOUT_;

    // Time sampling period
    double dt_;
    // Whether stabilizer is on
    bool on_;

    // Number of feet in support
    unsigned int nbSupport_;
    double forceThreshold_; // Threshold on normal force above which the foot is considered in contact
    unsigned int iterationsSinceLastSupportLf_;
    unsigned int iterationsSinceLastSupportRf_;
    unsigned int supportCandidateLf_;
    unsigned int supportCandidateRf_;

    Vector zmp_;

    stateObservation::Matrix A_;
    stateObservation::Matrix B_;
    stateObservation::Matrix Q_;
    stateObservation::Matrix R_;

    controller::DiscreteTimeLTILQR controller_;

    stateObservation::Matrix Kth_;
    stateObservation::Matrix Kdth_;

    bool fixedGains_;
    bool zmpMode_;

    double hrp2Mass_;

  }; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DYNAMIC_STABILIZER_HRP2_TWODOF_LQR_HH

