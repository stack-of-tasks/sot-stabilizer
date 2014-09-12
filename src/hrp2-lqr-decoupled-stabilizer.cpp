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

#include <sot-stabilizer/hrp2-lqr-decoupled-stabilizer.hh>

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

  double HRP2LQRDecoupledStabilizer::constm_ = 59.8;
  double HRP2LQRDecoupledStabilizer::constcomHeight_ = 0.807;
  double HRP2LQRDecoupledStabilizer::conststepLength_ = 0.19;

  const unsigned stateSize_=6;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HRP2LQRDecoupledStabilizer, "HRP2LQRDecoupledStabilizer");

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
  HRP2LQRDecoupledStabilizer::HRP2LQRDecoupledStabilizer(const std::string& inName) :
    TaskAbstract(inName),
    comSIN_ (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::com"),
    comRefSIN_ (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::comRef"),
    jacobianSIN_ (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(matrix)::Jcom"),
    comdotRefSIN_ (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::comdotRef"),
    comddotRefSIN_ (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::comddotRef"),
    zmpRefSIN_ (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::zmpRef"),
    leftFootPositionSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(HomoMatrix)::leftFootPosition"),
    rightFootPositionSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(HomoMatrix)::rightFootPosition"),
    forceLeftFootSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::force_lf"),
    forceRightFootSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::force_rf"),
    stateFlexSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(HomoMatrix)::stateFlex"),
    stateFlexDotSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::stateFlexDot"),
    stateFlexDDotSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::stateFlexDDot"),
    controlGainSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(double)::controlGain"),
    comdotSOUT_ ("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::comdot"),
    comddotSOUT_ ("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::comddot"),
    comddotRefSOUT_ ("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::comddotRefOUT"),
    nbSupportSOUT_
    ("HRP2LQRDecoupledStabilizer("+inName+")::output(unsigned)::nbSupport"),
    errorSOUT_ ("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::error"),
    debugSOUT_ ("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::debug"),
    supportPos1SOUT_("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::supportPos1"),
    supportPos2SOUT_("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::supportPos2"),
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
    controller1_(stateSize_,1),controller2_(stateSize_,1),controllerLat_(stateSize_,1),
    A1_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    A2_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    ALat_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    B_(stateObservation::Matrix::Zero(stateSize_,1)),
    Q1_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    Q2_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    Qlat_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
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

    taskSOUT.setFunction (boost::bind(&HRP2LQRDecoupledStabilizer::computeControlFeedback,
                                      this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2LQRDecoupledStabilizer::computeJacobianCom,
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




    std::string docstring;
    docstring =
      "\n"
      "    Set sampling time period task\n"
      "\n"
      "      input:\n"
      "        a floating point number\n"
      "\n";
    addCommand("setTimePeriod",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, double>
               (*this, &HRP2LQRDecoupledStabilizer::setTimePeriod, docstring));

    docstring =
      "\n"
      "    Set if yes or no the gains are fixed\n"
      "\n"
      "      input:\n"
      "        boolean\n"
      "\n";
    addCommand("setFixedGains",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, bool>
               (*this, &HRP2LQRDecoupledStabilizer::setFixedGains, docstring));


    docstring =
      "\n"
      "    Get sampling time period task\n"
      "\n"
      "      return:\n"
      "        a floating point number\n"
      "\n";
    addCommand("getTimePeriod",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, double>
               (*this, &HRP2LQRDecoupledStabilizer::getTimePeriod, docstring));

    addCommand ("start",
                makeCommandVoid0 (*this, &HRP2LQRDecoupledStabilizer::start,
                                  docCommandVoid0 ("Start stabilizer")));

    addCommand ("stop",
                makeCommandVoid0 (*this, &HRP2LQRDecoupledStabilizer::stop,
                                  docCommandVoid0 ("Stop stabilizer")));


    docstring =
      "\n"
      "    Set the cost Matrix for Input\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setInputCost",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::setInputCost, docstring));

    docstring =
      "\n"
      "    Set the cost Matrix for state (single support)\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setStateCost1",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::setStateCost1, docstring));

    docstring =
      "\n"
      "    Set the cost Matrix for state (double support frontal)\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setStateCost2",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::setStateCost2, docstring));

    docstring =
      "\n"
      "    Set the cost Matrix for state (double support lateral)\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setStateCostLat",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::setStateCostLat, docstring));

    docstring =
      "\n"
      "    Set if we use zmp reference or CoM acceleration\n"
      "\n"
      "      input:\n"
      "        Bool\n"
      "\n";
    addCommand("setZmpMode",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, bool>
               (*this, &HRP2LQRDecoupledStabilizer::setZMPMode, docstring));


    docstring =
      "\n"
      "    Get the cost Matrix for Input\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getInputCost",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getInputCost, docstring));

    docstring =
      "\n"
      "    Get the cost Matrix for state (simple support)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getStateCost1",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getStateCost1, docstring));

    docstring =
      "\n"
      "    Get the cost Matrix for state (double support frontal)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getStateCost2",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getStateCost2, docstring));



    docstring =
      "\n"
      "    Get the cost Matrix for state (lateral)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getStateCostLat",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getStateCostLat, docstring));

    docstring =
      "\n"
      "    Get the last Gain matrix for simple support\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getLastGains1",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getLastGain1, docstring));

    docstring =
      "\n"
      "    Get the last Gain matrix for double support sagittal\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getLastGains2",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getLastGain2, docstring));

    docstring =
      "\n"
      "    Get the last Gain matrix for double support lateral\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getLastGainsLat",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getLastGainLat, docstring));

    docstring =
      "\n"
      "    Set the horizon for the LQR\n"
      "\n"
      "      input:\n"
      "        int\n"
      "\n";
    addCommand("setHorizon",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, int>
               (*this, &HRP2LQRDecoupledStabilizer::setHorizon, docstring));


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



    prevCom_.fill (0.);


    kth_ = 600;
    kdth_ = 65;
    kz_= 53200;//150000;

    B_(5,0)=1/constcomHeight_;
    Q1_(0,0)=Q1_(2,2)=Q1_(4,4)=10;
    Q1_(1,1)=Q1_(3,3)=3;
    Q2_ = Qlat_ = Q1_;
    R_(0,0)=1;

    debug_.setZero();

    zmp_.setZero ();

    int horizon = 200;

    controller1_.setHorizonLength(horizon);
    controller2_.setHorizonLength(horizon);
    controllerLat_.setHorizonLength(horizon);


    A1_=computeDynamicsMatrix(constcomHeight_, kth_, kdth_, constm_);
    A2_=computeDynamicsMatrix(constcomHeight_, 2 * kth_, 2*kdth_, constm_);
    ALat_=computeDynamicsMatrix(constcomHeight_, 2*kth_ + 2*kz_*conststepLength_/2, 2*kdth_, constm_);
    B_=computeInputMatrix(constcomHeight_, kth_, kdth_, constm_);

    controller1_.setDynamicsMatrices(A1_, B_);
    controller2_.setDynamicsMatrices (A2_, B_);
    controllerLat_.setDynamicsMatrices (ALat_, B_);

    controllerLat_.setCostMatrices(Qlat_,R_);
    controller2_.setCostMatrices(Q2_,R_);
    controller1_.setCostMatrices(Q1_,R_);

    hrp2Mass_ = 58;
  }

/// Compute the control law
  VectorMultiBound&
  HRP2LQRDecoupledStabilizer::computeControlFeedback(VectorMultiBound& comdot,
      const int& time)
  {
    const Vector & com = comSIN_ (time);
    const Vector & comref = comRefSIN_ (time);

    const Vector& comdotRef = comdotRefSIN_ (time);

    const MatrixHomogeneous& flexibilityMatrix = stateFlexSIN_.access(time);
    const Vector& flexDot = stateFlexDotSIN_.access(time);
    const Vector& flexDDot = stateFlexDDotSIN_.access(time);

    const MatrixHomogeneous& leftFootPosition = leftFootPositionSIN_.access (time);
    const MatrixHomogeneous& rightFootPosition = rightFootPositionSIN_.access (time);
    const double& gain = controlGainSIN_.access (time);
    const Vector& forceLf = forceLeftFootSIN_.access (time);
    const Vector& forceRf = forceRightFootSIN_.access (time);



    Vector flexibilityPos(3);
    MatrixRotation flexibilityRot;

    flexibilityMatrix.extract(flexibilityRot);
    flexibilityMatrix.extract(flexibilityPos);

    VectorUTheta flexibility;
    flexibility.fromMatrix(flexibilityRot);

    Vector realcom(com);
    realcom(0)-= com(2)*flexibility (1);
    realcom(1)+= com(2)*flexibility (0);

    deltaCom_ = com - comref;



    double x = deltaCom_ (0);
    double y = deltaCom_ (1);
    double z = deltaCom_ (2);

    double dx = dcom_(0) - comdotRef (0);
    double dy = dcom_(1) - comdotRef (1);
    double dz = dcom_(2) - comdotRef (2);

    double theta0, dtheta0, ddtheta0;
    double theta1, dtheta1, ddtheta1;
    double xi, dxi, ddxi, lat, dlat, ddlat;
    //double thetaz;
    //double dthetaz;
    double fzRef, Zrefx, Zrefy, fz, Zx, Zy;



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

    switch (nbSupport_)
    {
    case 0:
      dcom_ (0) = -gain * x + comdotRef(0);
      dcom_ (1) = -gain * y + comdotRef(1);
      dcom_ (2) = -gain * z + comdotRef(2);

      debug_.setZero();
      debug_(0)=x;
      debug_(2)=dcom_(0);

      debug_(5)=y;
      debug_(7)=dcom_(1);
      break;
    case 1: //single support
    {
//      std::cout << "x " <<x << " y " <<y << " dx "<< dx <<" dy "<< dy;

      x -= com(2)*flexibility (1);
      y += com(2)*flexibility (0);

      dx -= com(2)*flexDot (1);
      dy += com(2)*flexDot (0);

      if ((!fixedGains_))
      {
        A1_=computeDynamicsMatrix(com(2), kth_, kdth_, constm_);
        B_=computeInputMatrix(com(2), kth_, kdth_, constm_);
        controller1_.setDynamicsMatrices(A1_, B_);
      }



      if (zmpMode_) //computing the reference acceleration of the CoM
      {
        const Vector &zmpref=  zmpRefSIN_(time);
        comddotRef_(0)= (stateObservation::cst::gravityConstant/com(2))*
                   (comref(0) - zmpref(0));
        comddotRef_(1)=(stateObservation::cst::gravityConstant/com(2))*
                   (comref(1) - zmpref(1));

//        std::cout << " comddotRefx " <<comddotRef_(0) << " comddotRefy " <<comddotRef_(1) ;
//        std::cout << " ddcomx " << d2com_(0)<<" ddcomy " << d2com_(1)<<std::endl;
      }
      else
      {
        comddotRef_ = comddotRefSIN_(time);
      }

      double ddx = d2com_(0) - com(2)*flexDDot (1) - comddotRef_(0) ;
      double ddy = d2com_(1) + com(2)*flexDDot (0) - comddotRef_(1) ;

//      std::cout << " update x " <<x << " y " <<y << " dx "<< dx <<" dy "<< dy;





      //along x
      theta0 = flexibility (1);
      dtheta0 = flexDot (1);
      ddtheta0 = flexDDot (1);


      stateObservation::Vector xVector (stateSize_);
      xVector[0]=x;
      xVector[1]=theta0;
      xVector[2]=dx;
      xVector[3]=dtheta0;
      xVector[4]=ddx;
      xVector[5]=ddtheta0;

//      std::cout << "Simple Support x" << std::endl;
      controller1_.setState(xVector,time);
      d2com_ (0)= controller1_.getControl(time)[0];

      // along y
      theta1 = -flexibility (0);
      dtheta1 = -flexDot (0);
      xVector[0]=y;
      xVector[1]=theta1;
      xVector[2]=dy;
      xVector[3]=dtheta1;
      xVector[4]=ddy;
      xVector[5]=ddtheta1;

//      std::cout << "Simple Support y" << std::endl;
      controller1_.setState(xVector,time);
      d2com_ (1)= controller1_.getControl(time)[0];


      debug_(0)=x;
      debug_(1)=theta0;
      debug_(2)=dx;
      debug_(3)=dtheta0;
      debug_(4)=ddx;
      debug_(5)=ddtheta0;
      debug_(6)=d2com_(0);

      debug_(7)=y;
      debug_(8)=theta1;
      debug_(9)=dy;
      debug_(10)=dtheta1;
      debug_(11)=ddy;
      debug_(12)=ddtheta1;
      debug_(13)=d2com_(1);

      d2com_ += comddotRef_;

      dcom_ (0) += dt_ * d2com_ (0);
      dcom_ (1) += dt_ * d2com_ (1);

      // along z
      dcom_ (2) = -gain * z + comdotRef(2);

    }
    break;
    default: //double support or more
    {

//      std::cout << "x " <<x << " y " <<y << " dx "<< dx <<" dy "<< dy;
      x = x - com(2)*flexibility (1);
      y = y + com(2)*flexibility (0);

      dx = dx - com(2)*flexDot (1);
      dy = dy + com(2)*flexDot (0);

      if (zmpMode_) //computing the reference acceleration of the CoM
      {
        const Vector &zmpref=  zmpRefSIN_(time);
        comddotRef_(0)= (stateObservation::cst::gravityConstant/com(2))*
                   (comref(0) - zmpref(0));
        comddotRef_(1)=(stateObservation::cst::gravityConstant/com(2))*
                   (comref(1) - zmpref(1));


//        std::cout << " comddotRefx " <<comddotRef_(0) << " comddotRefy " <<comddotRef_(1) ;
//        std::cout << " ddcomx " << d2com_(0)<<" ddcomy " << d2com_(1)<<std::endl;
      }
      else
      {
        comddotRef_ = comddotRefSIN_(time);
      }

      double ddx = d2com_(0) - com(2)*flexDDot (1) - comddotRef_(0) ;

      double ddy = d2com_(1) + com(2)*flexDDot (0) - comddotRef_(1) ;

//      std::cout << " update x " <<x << " y " <<y << " dx "<< dx <<" dy "<< dy;

      // compute component of angle orthogonal to the line joining the feet
      double delta_x = rfpos (0) - lfpos (0) ;
      double delta_y = rfpos (1) - lfpos (1) ;

      double stepLength = sqrt (delta_x*delta_x+delta_y*delta_y);

      if ((!fixedGains_))
      {
        A2_=computeDynamicsMatrix(com(2), 2 * kth_, 2*kdth_, constm_);
        B_ = computeInputMatrix(com(2), 2 * kth_, 2*kdth_, constm_);
        ALat_=computeDynamicsMatrix(com(2), 2*kth_ + 2*kz_*stepLength/2, 2*kdth_, constm_);

        controller2_.setDynamicsMatrices (A2_, B_);
        controllerLat_.setDynamicsMatrices (ALat_, B_);

      }

      u2x_ = delta_x/stepLength;
      u2y_ = delta_y/stepLength;
      u1x_ = u2y_;
      u1y_ = -u2x_;

      //along the orthogonal to the contacts line
      theta0 = + u1x_ * flexibility (1) - u1y_ * flexibility (0);
      dtheta0 = + u1x_ * flexDot (1) - u1y_ * flexDot (0);
      ddtheta0 = + u1x_ * flexDDot (1) - u1y_ * flexDDot (0);
      xi = u1x_*x + u1y_*y;
      dxi = u1x_*dx + u1y_*dy;
      ddxi = u1x_*ddx + u1y_*ddy;



      stateObservation::Vector xVector (stateSize_);
      xVector[0]=xi;
      xVector[1]=theta0;
      xVector[2]=dxi;
      xVector[3]=dtheta0;
      xVector[4]=ddxi;
      xVector[5]=ddtheta0;

//      std::cout << std::endl << "Double Support Frontal ===" << std::endl;
      controller2_.setState(xVector,time);
      ddxi= controller2_.getControl(time)[0];


      //along the contacts line
      theta1 = + u2x_ * flexibility (1) - u2y_ * flexibility (0);
      dtheta1 = + u2x_ * flexDot (1) - u2y_ * flexDot (0);
      ddtheta1 = + u2x_ * flexDDot (1) - u2y_ * flexDDot (0);
      lat = u2x_*x + u2y_*y;
      dlat = u2x_*dx + u2y_*dy;
      ddlat = u2x_*ddx + u2y_*ddy;

      xVector[0]=lat;
      xVector[1]=theta1;
      xVector[2]=dlat;
      xVector[3]=dtheta1;
      xVector[4]=ddlat;
      xVector[5]=ddtheta1;

//      std::cout << std::endl << "Double Support Lateral ===" << std::endl;
      controllerLat_.setState(xVector,time);
      ddlat= controllerLat_.getControl(time)[0];



      //fusion

      d2com_ (0) = ddxi * u1x_ + ddlat*u2x_;
      d2com_ (1) = ddxi * u1y_ + ddlat*u2y_;

      d2com_ += comddotRef_;

      dcom_ (0) += dt_ * d2com_ (0);
      dcom_ (1) += dt_ * d2com_ (1);

//      std::cout << " Out: ddcomx " << d2com_(0)<<" ddcomy " << d2com_(1)<<std::endl;


      // along z
      dcom_ (2) = -gain * z + comdotRef(2);

      debug_(0)=xi;
      debug_(1)=theta0;
      debug_(2)=dxi;
      debug_(3)=dtheta0;
      debug_(4)=ddxi;
      debug_(5)=ddtheta0;
      debug_(6)=ddxi;

      debug_(7)=lat;
      debug_(8)=theta1;
      debug_(9)=dlat;
      debug_(10)=dtheta1;
      debug_(11)=ddlat;
      debug_(12)=ddtheta1;
      debug_(13)=ddlat;
    }
    break;
    };

    comdot.resize (3);
    comdot [0].setSingleBound (dcom_ (0));
    comdot [1].setSingleBound (dcom_ (1));
    comdot [2].setSingleBound (dcom_ (2));

    comddotRefSOUT_.setConstant (comddotRef_);
    comddotRefSOUT_.setTime (time);

    comdotSOUT_.setConstant (dcom_);
    comdotSOUT_.setTime (time);

    comddotSOUT_.setConstant (d2com_);
    comddotSOUT_.setTime (time);

    errorSOUT_.setConstant (deltaCom_);
    errorSOUT_.setTime (time);

    debugSOUT_.setConstant (debug_);
    debugSOUT_.setTime (time);

    return comdot;
  }

  Matrix& HRP2LQRDecoupledStabilizer::computeJacobianCom(Matrix& jacobian, const int& time)
  {
    typedef unsigned int size_t;
    jacobian = jacobianSIN_ (time);
    return jacobian;
  }


  stateObservation::Matrix HRP2LQRDecoupledStabilizer::computeDynamicsMatrix(double comHeight, double kth, double kdth, double mass)
  {
    const double & g = stateObservation::cst::gravityConstant;

    stateObservation::Matrix A(stateObservation::Matrix::Zero(stateSize_,stateSize_));

    A(0,0)=1;
    A(0,2)=dt_;

    A(1,1)=1;
    A(1,3)=dt_;

    A(2,2)=1;
    A(2,4)=dt_;

    A(3,3)=1;
    A(3,5)=dt_;

    A(5,0)= -g/stateObservation::tools::square(comHeight);
    A(5,1)= -kth/(constm_*stateObservation::tools::square(comHeight));
    A(5,3)= 0;//-kdth /(constm_*stateObservation::tools::square(comHeight));

    A(4,0) =-comHeight*A(5,0);
    A(4,1) =-comHeight*A(5,1);
    A(4,3) =-comHeight*A(5,3);


    return A;
  }


  stateObservation::Matrix HRP2LQRDecoupledStabilizer::computeInputMatrix(double comHeight, double kth, double kdth, double mass)
  {
    stateObservation::Matrix B(stateObservation::Matrix::Zero(stateSize_,1));
    B(5,0) = 1/comHeight;

    return B;

  }

} // namespace sotStabilizer

