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

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>

#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-stabilizer/hrp2-lqr-twoDof-coupled-stabilizer.hh>

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
    comRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comRef"),
    jacobianSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jcom"),
    comdotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comdotRef"),
    comddotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comddotRef"),
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
    stateFlexDotSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::stateFlexDot"),
    stateFlexDDotSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::stateFlexDDot"),
    controlGainSIN_
    (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(double)::controlGain"),
    comdotSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comdot"),
    comddotSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comddot"),
    comddotRefSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comddotRefOUT"),
    nbSupportSOUT_
    ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(unsigned)::nbSupport"),
    zmpRefSOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::zmpRefOUT"),
    errorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::error"),
    debugSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::debug"),
    supportPos1SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos1"),
    supportPos2SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos2"),
    prevCom_(3), dcom_ (3), dt_ (.005), on_ (false),
    forceThreshold_ (.036 * constm_*stateObservation::cst::gravityConstant),
    angularStiffness_ (425.), d2com_ (3),
    deltaCom_ (3),
    flexPosition_ (), flexPositionLf_ (), flexPositionRf_ (),
    flexPositionLat_ (),
    flexVelocity_ (6), flexVelocityLf_ (6), flexVelocityRf_ (6),
    flexVelocityLat_ (6),
    timeBeforeFlyingFootCorrection_ (.1),
    iterationsSinceLastSupportLf_ (0), iterationsSinceLastSupportRf_ (0),
    supportCandidateLf_ (0), supportCandidateRf_ (0),
    uth_ (),fixedGains_(false), zmpMode_(true),
    translation_ (3), zmp_ (3),comddotRef_(3),
    theta1Ref_ (0), theta1RefPrev_ (0), dtheta1Ref_ (0), debug_(14),
    controller_(stateSize_,1),
    A_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    B_(stateObservation::Matrix::Zero(stateSize_,1)),
    Q_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    R_(stateObservation::Matrix::Zero(1,1))
  {
    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (comRefSIN_);
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
    signalRegistration (debugSOUT_);


    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (comRefSIN_);
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

    taskSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeControlFeedback,
                                      this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeJacobianCom,
                                          this,_1,_2));
    nbSupportSOUT_.addDependency (taskSOUT);

    d2com_.setZero ();

    dcom_.setZero ();
    deltaCom_.setZero ();
    comddotSOUT_.setConstant (d2com_);
    comdotSOUT_.setConstant (d2com_);
    comddotRefSOUT_.setConstant (d2com_);
    flexVelocity_.setZero ();
    flexVelocityLat_.setZero ();


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


    addCommand ("getKth",
                makeDirectGetter (*this, &kth_,
                                  docDirectGetter
                                  ("Set angular elasticity","float")));

    addCommand ("getKz",
                makeDirectGetter (*this, &kz_,
                                  docDirectGetter
                                  ("Set linear elasticity","float")));

    addCommand ("setKth",
                makeDirectSetter (*this, &kth_,
                                  docDirectSetter
                                  ("Set angular elasticity","float")));

    addCommand ("setKz",
                makeDirectSetter (*this, &kz_,
                                  docDirectSetter
                                  ("Set linear elasticity","float")));

    addCommand ("getKdth",
                makeDirectGetter (*this, &kdth_,
                                  docDirectGetter
                                  ("Set angular viscosity","float")));

    addCommand ("setKdth",
                makeDirectSetter (*this, &kdth_,
                                  docDirectSetter
                                  ("Set angular viscosity","float")));

    addCommand ("setRFOffset",
                makeDirectSetter (*this, &kdth_,
                                  docDirectSetter
                                  ("Set angular viscosity","float")));



    prevCom_.fill (0.);


    kth_ = 600;
    kdth_ = 65;
    kz_= 53200;//150000;

    Q_(0,0)=Q_(2,2)=Q_(4,4)=10;
    R_(0,0)=1;

    debug_.setZero();

    zmp_.setZero ();

    int horizon = 200;

    controller_.setHorizonLength(horizon);

    A_=computeDynamicsMatrix(kth_, kdth_, constm_);
    B_=computeInputMatrix(kth_, kdth_, constm_);

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

    const Vector & com = comSIN_ (time);
    const Vector & comref = comRefSIN_ (time);
    const Vector& comdotRef = comdotRefSIN_ (time);
    const MatrixHomogeneous& flexibilityMatrix = stateFlexSIN_.access(time);
    const Vector& flexDot = stateFlexDotSIN_.access(time);
    const Vector& flexDDot = stateFlexDDotSIN_.access(time);

    switch (nbSupport_)
    {
        case 0: break;
        case 1: //single support
        {
        }
        break;
        default: //double support or more
        {
        }
        break;
    };

    return task;
  }

  Matrix& HRP2LQRTwoDofCoupledStabilizer::computeJacobianCom(Matrix& jacobian, const int& time)
  {
    typedef unsigned int size_t;
    jacobian = jacobianSIN_ (time);
    return jacobian;
  }


  stateObservation::Matrix HRP2LQRTwoDofCoupledStabilizer::computeDynamicsMatrix
    (double kth, double kdth, double mass)
  {
    stateObservation::Matrix A(stateObservation::Matrix::Zero(stateSize_,stateSize_));


    return A;
  }


  stateObservation::Matrix HRP2LQRTwoDofCoupledStabilizer::computeInputMatrix(double kth, double kdth, double mass)
  {
    stateObservation::Matrix B(stateObservation::Matrix::Zero(stateSize_,1));

    return B;
  }

} // namespace sotStabilizer

