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

//#define NDEBUG
#include <iostream>

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

      //flexOriRefSIN_.setConstant(flexOriVectSIN_);
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

    void setInertia(const dynamicgraph::Matrix & I)
    {
        I_=I;
    }

    void constantInertia(const bool & b)
    {
        constantInertia_=b;
    }


    Vector& getControl(Vector& control, const int& time);
    inline void computeDynamicsMatrix(const stateObservation::Vector3 cl, const stateObservation::Matrix Kth, const stateObservation::Matrix Kdth, const int& time);

  private:

    /// Methods
    // Compute the number of supports
    unsigned int computeNbSupport(const int& time);
    // Compute the task
    VectorMultiBound& computeControlFeedback(VectorMultiBound& comdot, const int& time);
    // Compute the jacobian
    Matrix& computeJacobian(Matrix& jacobian, const int& time);
    // Compute the inertia tensor
    stateObservation::Matrix3 computeInert(const stateObservation::Vector& com, const int& time);

    /// Signals
        /// State
    // Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comSIN_;
    // Homogeneous representation of the waist position
    SignalPtr <dynamicgraph::Matrix, int> waistHomoSIN_;
    // Orientation of the flexibility on vector form
    SignalPtr <dynamicgraph::Vector, int> flexOriVectSIN_;
    // Velocity of center of mass
    SignalPtr < dynamicgraph::Vector, int> comDotSIN_;
    // Refrence angular velocity of the waist
    SignalPtr < dynamicgraph::Vector , int > waistVelSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> flexAngVelVectSIN_;

        /// Translational part of flexibility
    // Position
    SignalPtr < dynamicgraph::Vector, int> tflexSIN_;
    // Vitesse
    SignalPtr < dynamicgraph::Vector, int> dtflexSIN_;
    // Acceleration
    SignalPtr < dynamicgraph::Vector, int> ddtflexSIN_;

        /// Reference state
    // Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comRefSIN_;
    // Homogeneous representation of the waist position
    SignalPtr <dynamicgraph::Vector, int> waistOriRefSIN_;
    // Orientation of the flexibility on vector form
    SignalPtr <dynamicgraph::Vector, int> flexOriRefSIN_;
    // Velocity of center of mass
    SignalPtr < dynamicgraph::Vector, int> comDotRefSIN_;
    // Refrence angular velocity of the waist
    SignalPtr < dynamicgraph::Vector , int > waistVelRefSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> flexAngVelRefSIN_;

        /// Jacobians
    SignalPtr < dynamicgraph::Matrix, int> jacobianComSIN_;
    SignalPtr < dynamicgraph::Matrix, int> jacobianWaistSIN_;

        /// Signals to compute number of supports
    // Position of left foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
    // Position of right foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
    // Force in left foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceLeftFootSIN_;
    // Force in right foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceRightFootSIN_;

        /// Control gain for 0 supports case
    SignalPtr <double, int> controlGainSIN_;

        /// Signals to compute dynamics model
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> inertiaSIN;

        /// Outputs
    // state output
    SignalTimeDependent <Vector, int> stateSOUT_;
    // state output in the world frame
    SignalTimeDependent <Vector, int> stateWorldSOUT_;
    // state reference output
    SignalTimeDependent <Vector, int> stateRefSOUT_;
    // error state output
    SignalTimeDependent <Vector, int> stateErrorSOUT_;
    // predicted state
    SignalTimeDependent <Vector, int> stateSimulationSOUT_;
    // extended state output
    SignalTimeDependent <Vector, int> stateExtendedSOUT_;
    // state model error
    SignalTimeDependent <Vector, int> stateModelErrorSOUT_;
    // error output
    SignalTimeDependent <Vector, int> errorSOUT_;
    // control output
    SignalTimeDependent <Vector, int> controlSOUT_;
    // gains computed by the LQR controller
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> gainSOUT;

    // Number of support feet
    SignalTimeDependent <unsigned int, int> nbSupportSOUT_;
    // Contact Position
    SignalTimeDependent <Vector, int> supportPos1SOUT_;
    SignalTimeDependent <Vector, int> supportPos2SOUT_;

    // A and B matrices computed by the computeDynamics method
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> AmatrixSOUT;
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> BmatrixSOUT;
    // Inertia computed by the computeInert method
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> inertiaSOUT;

        /// Unused signals
    // angular acceleration of the waist
    SignalPtr < dynamicgraph::Vector, int> waistAngAccSIN_;
    // Reference com ddot
    SignalPtr < dynamicgraph::Vector, int> comddotRefSIN_;
    ///Reference ZMP
    SignalPtr < dynamicgraph::Vector, int> zmpRefSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> stateFlexDDotSIN_;
    ///Reference ZMP
    SignalTimeDependent <Vector, int> zmpRefSOUT_;

    /// Parameters
    // Time sampling period
    double dt_;
    // Whether stabilizer is on
    bool on_;
    // Number of feet in support
    unsigned int nbSupport_;
    double forceThreshold_; // Threshold on normal force above which the foot is considered in contact
    Vector supportPos1_, supportPos2_;

    stateObservation::Matrix A_;
    stateObservation::Matrix B_;
    stateObservation::Matrix Q_;
    stateObservation::Matrix R_;

    dynamicgraph::Matrix I_;
    bool constantInertia_;

    controller::DiscreteTimeLTILQR controller_;

    double kth_, kdth_;
    stateObservation::Matrix Kth_;
    stateObservation::Matrix Kdth_;

    stateObservation::Vector preTask_;
    stateObservation::Vector xpredicted_;

    stateObservation::Vector comRef_;

    bool fixedGains_;
    bool computed_;

    double hrp2Mass_;

    Vector zmp_;
    bool zmpMode_;

  }; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DYNAMIC_STABILIZER_HRP2_TWODOF_LQR_HH

