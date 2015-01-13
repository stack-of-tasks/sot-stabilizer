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

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>

#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-stabilizer/hrp2-lqr-twoDof-coupled-stabilizer.hh>
#include <stdexcept>

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
  using dynamicgraph::command::makeCommandVoid0;
  using dynamicgraph::command::docCommandVoid0;
  using dynamicgraph::command::docDirectSetter;
  using dynamicgraph::command::makeDirectSetter;
  using dynamicgraph::command::docDirectGetter;
  using dynamicgraph::command::makeDirectGetter;
  using dynamicgraph::sot::MatrixHomogeneous;
  using dynamicgraph::sot::MatrixRotation;
  using dynamicgraph::sot::VectorUTheta;

  double HRP2LQRTwoDofCoupledStabilizer::constm_ = 59.8;
  double HRP2LQRTwoDofCoupledStabilizer::constcomHeight_ = 0.807;
  double HRP2LQRTwoDofCoupledStabilizer::conststepLength_ = 0.19;

  const unsigned stateSize_=6;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HRP2LQRTwoDofCoupledStabilizer, "HRP2LQRTwoDofCoupledStabilizer");

/// Dynamic balance stabilizer
///
/// This task takes as input four signals
/// \li comSIN, the position of the center of mass (COM)
/// \li comRefSIN, the the desired position of the center of mass,
/// \li zmpSIN, the position of the center of pressure (ZMP),
/// \li zmpDesSIN, the desired position of the center of pressure,
/// \li jacobianSIN, the jacobian of the center of mass,
/// \li comdotSIN, reference velocity of the center of mass,
/// and provides as output two signals
/// \li taskSOUT, the desired time derivative of the center of mass,
/// \li jacobianSOUT, the jacobian of the center of mass
  HRP2LQRTwoDofCoupledStabilizer::HRP2LQRTwoDofCoupledStabilizer(const std::string& inName) :
    TaskAbstract(inName),
    comSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::com"),
    comDotSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comDot"),
    comRefgSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comRefg"),
    jacobianSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jcom"),
    comdotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comdotRef"),
    comddotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comddotRef"),
    waistOriSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistOri"),
    waistOriRefgSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistOriRefg"),
    waistAngVelSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistAngVel"),
    waistAngAccSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistAngAcc"),
    zmpRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::zmpRef"),
    leftFootPositionSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(HomoMatrix)::leftFootPosition"),
    rightFootPositionSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(HomoMatrix)::rightFootPosition"),
    forceLeftFootSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::force_lf"),
    forceRightFootSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::force_rf"),
    stateFlexSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(HomoMatrix)::stateFlex"),
    flexOriVectSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexOriVect"),
    stateFlexDotSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::stateFlexDot"),
    stateFlexDDotSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::stateFlexDDot"),
    controlGainSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(double)::controlGain"),
    inertiaSIN(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::inertia"),
    positionWaistSIN(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::positionWaist"),
    comdotSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comdot"),
    comddotSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comddot"),
    comddotRefSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comddotRefOUT"),
    nbSupportSOUT_
    ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(unsigned)::nbSupport"),
    zmpRefSOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::zmpRefOUT"),
    errorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::error"),
    supportPos1SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos1"),
    supportPos2SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos2"),
    dt_ (.005), on_ (false),
    forceThreshold_ (.036 * constm_*stateObservation::cst::gravityConstant),
    iterationsSinceLastSupportLf_ (0), iterationsSinceLastSupportRf_ (0),
    supportCandidateLf_ (0), supportCandidateRf_ (0),
    fixedGains_(false), zmpMode_(true),
    zmp_ (3),
    controller_(stateSize_,1),
    A_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    B_(stateObservation::Matrix::Zero(stateSize_,1)),
    Q_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    R_(stateObservation::Matrix::Zero(1,1))
  {
    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (comDotSIN_);
    signalRegistration (comRefgSIN_);
    signalRegistration (jacobianSIN_);
    signalRegistration (comdotRefSIN_);
    signalRegistration (zmpRefSIN_);
    signalRegistration (comddotRefSIN_);
    signalRegistration (leftFootPositionSIN_ << rightFootPositionSIN_
                        << forceRightFootSIN_ << forceLeftFootSIN_);
    signalRegistration (stateFlexSIN_ << stateFlexDotSIN_ << stateFlexDDotSIN_);
    signalRegistration (controlGainSIN_);
    signalRegistration (comdotSOUT_);
    signalRegistration (comddotSOUT_ <<comddotRefSOUT_);
    signalRegistration (nbSupportSOUT_ << supportPos1SOUT_ << supportPos2SOUT_);
    signalRegistration (errorSOUT_);
    signalRegistration (zmpRefSOUT_);
    signalRegistration (inertiaSIN);
    dynamicgraph::Matrix inertia(6);
    inertiaSIN.setConstant(inertia);

    signalRegistration (waistOriSIN_);
    signalRegistration (waistOriRefgSIN_);
    signalRegistration (waistAngVelSIN_);
    signalRegistration (waistAngAccSIN_);

    signalRegistration (flexOriVectSIN_);

    signalRegistration (positionWaistSIN);
    dynamicgraph::Matrix positionWaist;
    positionWaistSIN.setConstant(positionWaist);

    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (comRefgSIN_);
    taskSOUT.addDependency (comdotRefSIN_);

    taskSOUT.addDependency (stateFlexSIN_);
    taskSOUT.addDependency (stateFlexDotSIN_);
    taskSOUT.addDependency (stateFlexDDotSIN_);

    if (zmpMode_)
    {
      taskSOUT.addDependency(zmpRefSIN_);
    }
    else
    {
      taskSOUT.addDependency(comddotRefSIN_);
    }

    taskSOUT.addDependency (leftFootPositionSIN_);
    taskSOUT.addDependency (rightFootPositionSIN_);
    taskSOUT.addDependency (forceRightFootSIN_);
    taskSOUT.addDependency (forceLeftFootSIN_);
    taskSOUT.addDependency (stateFlexSIN_);
    taskSOUT.addDependency (stateFlexDotSIN_);
    taskSOUT.addDependency (stateFlexDDotSIN_);
    taskSOUT.addDependency (controlGainSIN_);

    jacobianSOUT.addDependency (jacobianSIN_);

    taskSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeControlFeedback,this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeJacobianCom,
                                          this,_1,_2));
    nbSupportSOUT_.addDependency (taskSOUT);


    Vector rfconf(6);
    rfconf.setZero();

    Vector lfconf(6);
    lfconf.setZero();

    rfconf(0) = 0.009490463094;
    rfconf(1) = -0.095000000000;

    lfconf(0) = 0.009490463094;
    lfconf(1) = 0.095000000000;

    supportPos1SOUT_.setConstant (lfconf);
    supportPos1SOUT_.setTime (0);
    supportPos2SOUT_.setConstant (rfconf);
    supportPos2SOUT_.setTime (0);

    nbSupportSOUT_.setConstant (2);
    nbSupportSOUT_.setTime (0);





    std::string docstring;
    docstring =
      "\n"
      "    Set sampling time period task\n"
      "\n"
      "      input:\n"
      "        a floating point number\n"
      "\n";
    addCommand("setTimePeriod",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, double>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setTimePeriod, docstring));

    docstring =
      "\n"
      "    Set if yes or no the gains are fixed\n"
      "\n"
      "      input:\n"
      "        boolean\n"
      "\n";
    addCommand("setFixedGains",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, bool>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setFixedGains, docstring));


    docstring =
      "\n"
      "    Get sampling time period task\n"
      "\n"
      "      return:\n"
      "        a floating point number\n"
      "\n";
    addCommand("getTimePeriod",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, double>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getTimePeriod, docstring));

    addCommand ("start",
                makeCommandVoid0 (*this, &HRP2LQRTwoDofCoupledStabilizer::start,
                                  docCommandVoid0 ("Start stabilizer")));

    addCommand ("stop",
                makeCommandVoid0 (*this, &HRP2LQRTwoDofCoupledStabilizer::stop,
                                  docCommandVoid0 ("Stop stabilizer")));


    docstring =
      "\n"
      "    Set the cost Matrix for Input\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setInputCost",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setInputCost, docstring));

    docstring =
      "\n"
      "    Set the cost Matrix for state (single support)\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setStateCost",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setStateCost, docstring));


    docstring =
      "\n"
      "    Set if we use zmp reference or CoM acceleration\n"
      "\n"
      "      input:\n"
      "        Bool\n"
      "\n";
    addCommand("setZmpMode",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, bool>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setZMPMode, docstring));


    docstring =
      "\n"
      "    Get the cost Matrix for Input\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getInputCost",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getInputCost, docstring));

    docstring =
      "\n"
      "    Get the cost Matrix for state (simple support)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getStateCost",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getStateCost, docstring));

    docstring =
      "\n"
      "    Get the last Gain matrix for simple support (x axis)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getLastGains",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getLastGain, docstring));

    docstring =
      "\n"
      "    Set the horizon for the LQR\n"
      "\n"
      "      input:\n"
      "        int\n"
      "\n";
    addCommand("setHorizon",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, int>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setHorizon, docstring));



    docstring  =
            "\n"
            "    Sets the angular stifness of the flexibility \n"
            "\n";

    addCommand(std::string("setKth"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,dynamicgraph::Matrix>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setKth ,docstring));

    docstring  =
            "\n"
            "    Sets the angular damping of the flexibility \n"
            "\n";

    addCommand(std::string("setKdth"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,dynamicgraph::Matrix>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setKdth ,docstring));



    Kth_ << 600,0,0,
            0,600,0,
            0,0,600;

    Kdth_ <<    65,0,0,
                0,65,0,
                0,0,65;


    Q_(0,0)=Q_(2,2)=Q_(4,4)=10;
    R_(0,0)=1;

    zmp_.setZero ();

    int horizon = 200;

    controller_.setHorizonLength(horizon);

    computeDynamicsMatrix(Kth_,Kdth_,0);
    controller_.setDynamicsMatrices(A_, B_);
    controller_.setCostMatrices(Q_,R_);

    hrp2Mass_ = 58;
  }

  /// Determine number of support : to be put in stateObervation in my opinion
  unsigned int HRP2LQRTwoDofCoupledStabilizer::computeNbSupport(const int& time)//const MatrixHomogeneous& leftFootPosition, const MatrixHomogeneous& rightFootPosition, const Vector& forceLf, const Vector& forceRf, const int& time)
  {
      /// Forces signals
      const MatrixHomogeneous& leftFootPosition = leftFootPositionSIN_.access (time);
      const MatrixHomogeneous& rightFootPosition = rightFootPositionSIN_.access (time);
      const Vector& forceLf = forceLeftFootSIN_.access (time);
      const Vector& forceRf = forceRightFootSIN_.access (time);

      //feet position
      Vector rfpos(3);
      Vector lfpos(3);

      leftFootPosition.extract(rfpos);
      rightFootPosition.extract(lfpos);

      // Express vertical component of force in global basis
      double flz = leftFootPosition (2,0) * forceLf (0) +
                   leftFootPosition(2,1) * forceLf (1) +
                   leftFootPosition (2,2) * forceLf (2);
      double frz = rightFootPosition (2,0) * forceRf (0) +
                   rightFootPosition(2,1) * forceRf (1) +
                   rightFootPosition (2,2) * forceRf (2);


      //compute the number of supports
      nbSupport_ = 0;
      if (frz >= forceThreshold_)
      {
        rightFootPosition.extract(rfpos);
        nbSupport_++;
        supportCandidateRf_++;
        supportPos1SOUT_.setConstant (rfpos);
        nbSupportSOUT_.setTime (time);
        if (supportCandidateRf_ >= 3)
        {
          iterationsSinceLastSupportRf_ = 0;
        }
      }
      else
      {
        supportCandidateRf_ = 0;
        iterationsSinceLastSupportRf_ ++;
      }

      if (flz >= forceThreshold_)
      {
        leftFootPosition.extract(rfpos);
        nbSupport_++;
        supportCandidateLf_++;
        if (nbSupport_==0)
        {
          supportPos1SOUT_.setConstant (rfpos);
          supportPos1SOUT_.setTime (time);
        }
        else
        {
          supportPos2SOUT_.setConstant (rfpos);
          supportPos2SOUT_.setTime (time);
        }
        if (supportCandidateLf_ >= 3)
        {
          iterationsSinceLastSupportLf_ = 0;
        }
      }
      else
      {
        supportCandidateLf_ = 0;
        iterationsSinceLastSupportLf_++;
      }
      nbSupportSOUT_.setConstant (nbSupport_);
      nbSupportSOUT_.setTime (time);

      if (!on_)
      {

        nbSupport_=0;

      }

      return nbSupport_;
  }


/// Compute the control law
  VectorMultiBound&
  HRP2LQRTwoDofCoupledStabilizer::computeControlFeedback(VectorMultiBound& task,
      const int& time)
  {
    // Control gain
    const double& gain = controlGainSIN_.access (time);

    // Determination of the number of support
    nbSupport_=computeNbSupport(time);

    // Reference in the global frame
    const stateObservation::Vector3 & comRefg = convertVector<stateObservation::Vector>(comRefgSIN_ (time));
    const stateObservation::Vector3 & waistOriRefg = convertVector<stateObservation::Vector>(waistOriRefgSIN_ (time));
        // References of velocities and acceleration are equal to zero
        // References for the flexibility are set to zero

    // Com
    const stateObservation::Vector3 & com = convertVector<stateObservation::Vector>(comSIN_ (time));
    const stateObservation::Vector3 & comDot = convertVector<stateObservation::Vector>(comDotSIN_ (time));

    // Waist orientation
    const stateObservation::Vector3 & waistOri = convertVector<stateObservation::Vector>(waistOriSIN_ (time));
    const stateObservation::Vector3 & waistAngVel = convertVector<stateObservation::Vector>(waistAngVelSIN_ (time));

    // Flexibility
    const stateObservation::Vector3 & flexOriVect = convertVector<stateObservation::Vector>(flexOriVectSIN_.access(time));
    const stateObservation::Vector3 & flexDot = convertVector<stateObservation::Vector>(stateFlexDotSIN_.access(time));

    // Error
    stateObservation::Vector dCom = com - comRefg;
    stateObservation::Vector dWaistOri = waistOri - waistOriRefg;

    stateObservation::Vector xk;
    xk.resize(stateSize_);
    xk <<   dCom,
            dWaistOri,
            flexOriVect,
            comDot,
            waistAngVel,
            flexDot;

    stateObservation::Vector u;
    stateObservation::Matrix3 Kth, Kdth;
    stateObservation::Vector preTask;

    switch (nbSupport_)
    {
        case 0: // No support
        {
            preTask <<  -gain*dCom,
                        -gain*dWaistOri;
        }
        break;
        case 1: // Single support
        {
             computeDynamicsMatrix(Kth_,Kdth_,time);
             controller_.setDynamicsMatrices(A_,B_);
             controller_.setState(xk,time);
             u=controller_.getControl(time);
             preTask=dt_*u;
        }
        break;
        case 2 : // Double support
        {
              Kth <<    0.5*Kth_(0,0)*Kth_(0,0) ,0,0,
                        0,Kth_(1,1),0,
                        0,0,0.5*Kth_(2,2)*Kth_(2,2);
              Kdth <<    0.5*Kdth_(0,0)*Kdth_(0,0) ,0,0,
                        0,Kdth_(1,1),0,
                        0,0,0.5*Kdth_(2,2)*Kdth_(2,2);

              // TODO: when feet are not aligned along the y axis

              computeDynamicsMatrix(Kth,Kdth,time);
              controller_.setDynamicsMatrices(A_,B_);
              controller_.setState(xk,time);
              u=controller_.getControl(time);
              preTask=dt_*u;
        }
        break;
        default: throw std::invalid_argument("Only 0, 1 and 2 number of supports cases are developped");
    };

    task.resize (3);
    task [0].setSingleBound (preTask(0));
    task [1].setSingleBound (preTask (1));
    task [2].setSingleBound (preTask (2));
    return task;
  }

  Matrix& HRP2LQRTwoDofCoupledStabilizer::computeJacobianCom(Matrix& jacobian, const int& time)
  {
    typedef unsigned int size_t;
    jacobian = jacobianSIN_ (time);
    return jacobian;
  }


  void HRP2LQRTwoDofCoupledStabilizer::computeDynamicsMatrix(const stateObservation::Matrix Kth, const stateObservation::Matrix Kdth, const int& time)
  {
    double g = stateObservation::cst::gravityConstant;
    double m = hrp2Mass_;
    stateObservation::Vector cl = convertVector<stateObservation::Vector>(comSIN_ (time));
    stateObservation::Matrix I = computeInert(cl,time);

    stateObservation::Matrix3 identity;
    identity.setIdentity();

    stateObservation::Matrix3 ddomega_cl, ddomega_omegach, ddomega_omega, ddomega_dcl,
                              ddomega_domegach, ddomega_domega, ddomega_ddcl, ddomega_ddomegach;

    // usefull variables for code factorisation
    stateObservation::Vector3 uz;
    uz <<     0,
              0,
              1;

    stateObservation::Matrix Inertia;
    Inertia = I;
    Inertia -= m * kine::skewSymmetric(cl)*kine::skewSymmetric(cl);
    Inertia = Inertia.inverse();

    stateObservation::Vector3 v;
    v=-g*m*Inertia*kine::skewSymmetric(cl)*uz;

    // Caracteristic polynomial
    ddomega_cl=Inertia*(m*(-kine::skewSymmetric(kine::skewSymmetric(cl)*v)-kine::skewSymmetric(cl)*kine::skewSymmetric(v))+g*m*kine::skewSymmetric(uz));
    ddomega_omegach=-Inertia*(-kine::skewSymmetric(I*v)+I*kine::skewSymmetric(v));
    ddomega_omega=-kine::skewSymmetric(v)+Inertia*(-Kth-Kdth-g*m*kine::skewSymmetric(cl)*kine::skewSymmetric(uz));
    ddomega_dcl.setZero();
    ddomega_domegach.setZero();
    ddomega_domega=-Inertia*Kdth;

    ddomega_ddcl=-m*Inertia*kine::skewSymmetric(cl);
    ddomega_ddomegach=Inertia*I;

    // A_ and B_ computation
    A_.block(0,9,2,2)=identity;
    A_.block(3,12,2,2)=identity;
    A_.block(6,15,2,2)=identity;
    A_.block(15,0,2,2)=ddomega_cl;
    A_.block(15,3,2,2)=ddomega_omegach;
    A_.block(15,6,2,2)=ddomega_omega;
    A_.block(15,9,2,2)=ddomega_dcl;
    A_.block(15,12,2,2)=ddomega_domegach;
    A_.block(15,15,2,2)=ddomega_domega;

    stateObservation::Matrix Identity(stateObservation::Matrix::Zero(stateSize_,stateSize_));
    Identity.setIdentity();

    A_.noalias() = dt_ * A_ + Identity;

    B_.block(9,0,2,2)=identity;
    B_.block(12,3,2,2)=identity;
    B_.block(15,0,2,2)=ddomega_ddcl;
    B_.block(15,3,2,2)=ddomega_ddomegach;

    B_.noalias() = dt_* B_;

  }


  stateObservation::Matrix3 HRP2LQRTwoDofCoupledStabilizer::computeInert(const stateObservation::Vector& cl, const int& inTime)
{
    const dynamicgraph::Matrix& inertia=inertiaSIN.access(inTime);
    const dynamicgraph::Matrix& homoWaist=positionWaistSIN.access(inTime);

    double m=inertia(0,0); //<=== donne 56.8;

    dynamicgraph::Vector com=convertVector<dynamicgraph::Vector>(cl);
    dynamicgraph::Vector inert;
    inert.resize(6);

    dynamicgraph::Vector waist;
    waist.resize(3);
    waist.elementAt(0)=homoWaist(0,3);
    waist.elementAt(1)=homoWaist(1,3);
    waist.elementAt(2)=homoWaist(2,3);

    // Inertia expressed at waist
    inert.elementAt(0)=inertia(3,3);
    inert.elementAt(1)=inertia(4,4);
    inert.elementAt(2)=inertia(5,5);
    inert.elementAt(3)=inertia(3,4);
    inert.elementAt(4)=inertia(3,5);
    inert.elementAt(5)=inertia(4,5);

    // From waist to com
    inert.elementAt(0) += -m*((com.elementAt(1)-waist.elementAt(1))*(com.elementAt(1)-waist.elementAt(1))+(com.elementAt(2)-waist.elementAt(2))*(com.elementAt(2)-waist.elementAt(2)));
    inert.elementAt(1) += -m*((com.elementAt(0)-waist.elementAt(0))*(com.elementAt(0)-waist.elementAt(0))+(com.elementAt(2)-waist.elementAt(2))*(com.elementAt(2)-waist.elementAt(2)));
    inert.elementAt(2) += -m*((com.elementAt(0)-waist.elementAt(0))*(com.elementAt(0)-waist.elementAt(0))+(com.elementAt(1)-waist.elementAt(1))*(com.elementAt(1)-waist.elementAt(1)));
    inert.elementAt(3) += m*(com.elementAt(0)-waist.elementAt(0))*(com.elementAt(1)-waist.elementAt(1));
    inert.elementAt(4) += m*(com.elementAt(0)-waist.elementAt(0))*(com.elementAt(2)-waist.elementAt(2));
    inert.elementAt(5) += m*(com.elementAt(1)-waist.elementAt(1))*(com.elementAt(2)-waist.elementAt(2));

    // From com to local frame
    inert.elementAt(0) -= -m*((com.elementAt(1))*(com.elementAt(1))+(com.elementAt(2))*(com.elementAt(2)));
    inert.elementAt(1) -= -m*((com.elementAt(0))*(com.elementAt(0))+(com.elementAt(2))*(com.elementAt(2)));
    inert.elementAt(2) -= -m*((com.elementAt(0))*(com.elementAt(0))+(com.elementAt(1))*(com.elementAt(1)));
    inert.elementAt(3) -= m*(com.elementAt(0))*(com.elementAt(1));
    inert.elementAt(4) -= m*(com.elementAt(0))*(com.elementAt(2));
    inert.elementAt(5) -= m*(com.elementAt(1))*(com.elementAt(2));

    return kine::computeInertiaTensor(convertVector<stateObservation::Vector>(inert));
}

} // namespace sotStabilizer
