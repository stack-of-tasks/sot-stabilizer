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

  const unsigned stateSize_=14;
  const unsigned controlSize_=5;
  const unsigned taskSize_=7;

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
    waistHomoSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::waistHomo"),
    flexOriVectSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexOriVect"),
    comDotSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comDot"),
    waistVelSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistVel"),
    flexAngVelVectSIN_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexAngVelVect"),
    tflexSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::tflex"),
    dtflexSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::dtflex"),
    ddtflexSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::ddtflex"),
    comRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comRef"),
    comRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comRef"),
    waistOriRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistOriRef"),
    flexOriRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexOriRef"),
    comDotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comDotRef"),
    waistVelRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistVelRef"),
    flexAngVelRefSIN_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexAngVelRef"),
    jacobianComSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jcom"),
    jacobianWaistSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jwaist"),
    jacobianChestSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jchest"),
    leftFootPositionSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(HomoMatrix)::leftFootPosition"),
    rightFootPositionSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(HomoMatrix)::rightFootPosition"),
    forceLeftFootSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::force_lf"),
    forceRightFootSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::force_rf"),
    controlGainSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(double)::controlGain"),
    inertiaSIN(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::inertia"),
    angularmomentumSIN(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::angularmomentum"),
    stateSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::state"),
    stateWorldSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateWorld"),
    stateErrorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateError"),
    stateRefSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateRef"),
    stateSimulationSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateSimulation"),
    stateExtendedSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector):stateExtended"),
    stateModelErrorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector):stateModelError"),
    errorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::error"),
    controlSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::control"),
    gainSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::gain"),
    nbSupportSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(unsigned)::nbSupport"),
    supportPos1SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos1"),
    supportPos2SOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::supportPos2"),
    AmatrixSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::Amatrix"),
    BmatrixSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::Bmatrix"),
    inertiaSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::inertiaOut"), 
    waistAngAccSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistAngAcc"),
        comddotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comddotRef"),
    zmpRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::zmpRef"),
    stateFlexDDotSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::stateFlexDDot"),
    zmpRefSOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::zmpRefOUT"),
    energySOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::energy"),
    dt_ (.005), on_ (false),
    forceThreshold_ (.036 * constm_*stateObservation::cst::gravityConstant),
    supportPos1_(3), supportPos2_(3),
    fixedGains_(true), zmpMode_(true), computed_(false),
    zmp_ (3), xpredicted_(stateSize_),
    controller_(stateSize_,controlSize_),
    A_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    B_(stateObservation::Matrix::Zero(stateSize_,controlSize_)),
    Q_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    R_(stateObservation::Matrix::Zero(controlSize_,controlSize_)),
    I_(3,3), constantInertia_(false)
  {

    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (waistHomoSIN_);
    signalRegistration (flexOriVectSIN_);
    signalRegistration (comDotSIN_);
    signalRegistration (waistVelSIN_);
    signalRegistration (flexAngVelVectSIN_);
    signalRegistration (tflexSIN_);
    signalRegistration (dtflexSIN_);
    signalRegistration (ddtflexSIN_);
    signalRegistration (comRefSIN_);
    signalRegistration (waistOriRefSIN_);
    signalRegistration (flexOriRefSIN_);
    signalRegistration (comDotRefSIN_);
    signalRegistration (waistVelRefSIN_);
    signalRegistration (flexAngVelRefSIN_);
    signalRegistration (jacobianComSIN_);
    signalRegistration (jacobianWaistSIN_);
    signalRegistration (jacobianChestSIN_);
    signalRegistration (leftFootPositionSIN_ << rightFootPositionSIN_ << forceRightFootSIN_ << forceLeftFootSIN_);
    signalRegistration (controlGainSIN_);
    signalRegistration (inertiaSIN);
    signalRegistration (angularmomentumSIN);
    signalRegistration (stateSOUT_);
    signalRegistration (stateWorldSOUT_);
    signalRegistration (stateRefSOUT_);
    signalRegistration (stateSimulationSOUT_);
    signalRegistration (stateErrorSOUT_);
    signalRegistration (stateExtendedSOUT_);
    signalRegistration (stateModelErrorSOUT_);
    signalRegistration (errorSOUT_);
    signalRegistration (controlSOUT_);
    signalRegistration (gainSOUT);
    signalRegistration (nbSupportSOUT_ << supportPos1SOUT_ << supportPos2SOUT_);
    signalRegistration (AmatrixSOUT);
    signalRegistration (BmatrixSOUT);
    signalRegistration (inertiaSOUT);
    signalRegistration (waistAngAccSIN_);
    signalRegistration (comddotRefSIN_);
    signalRegistration (zmpRefSIN_);
    signalRegistration (stateFlexDDotSIN_);
    signalRegistration (zmpRefSOUT_);
    signalRegistration (energySOUT_);

    // Set dependencies
        // taskSOUT dependencies
    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (waistHomoSIN_);
    taskSOUT.addDependency (flexOriVectSIN_);
    taskSOUT.addDependency (comDotSIN_);
    taskSOUT.addDependency (waistVelSIN_);
    taskSOUT.addDependency (flexAngVelVectSIN_);
    taskSOUT.addDependency (comRefSIN_);
    taskSOUT.addDependency (waistOriRefSIN_);
    taskSOUT.addDependency (flexOriRefSIN_);
    taskSOUT.addDependency (comDotRefSIN_);
    taskSOUT.addDependency (waistVelRefSIN_);
    taskSOUT.addDependency (flexAngVelRefSIN_);
    taskSOUT.addDependency (jacobianComSIN_);
    taskSOUT.addDependency (jacobianWaistSIN_);
    taskSOUT.addDependency (leftFootPositionSIN_);
    taskSOUT.addDependency (rightFootPositionSIN_);
    taskSOUT.addDependency (forceRightFootSIN_);
    taskSOUT.addDependency (forceLeftFootSIN_);
    taskSOUT.addDependency (controlGainSIN_);
    taskSOUT.addDependency (inertiaSIN);
        // jacobianSOUT dependencies
    jacobianSOUT.addDependency (jacobianComSIN_);
    jacobianSOUT.addDependency (jacobianWaistSIN_);
        // nbSupportSOUT dependency
    nbSupportSOUT_.addDependency (taskSOUT);
        // supports position dependencies
    supportPos1SOUT_.addDependency (taskSOUT);
    supportPos2SOUT_.addDependency (taskSOUT);

    controlSOUT_.addDependency (taskSOUT);

    if (zmpMode_){
      taskSOUT.addDependency(zmpRefSIN_);
    }else{
      taskSOUT.addDependency(comddotRefSIN_);
    }

    // Settinf methods for output signals
    taskSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeControlFeedback,this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeJacobian,this,_1,_2));

    controlSOUT_.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::getControl,this,_1,_2));

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
      "    Get the last floored Gain matrix for simple support (x axis)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getLastFloorGains",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getLastFloorGain, docstring));

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

    docstring  =
            "\n"
            "    Sets the inertia \n"
            "\n";

    addCommand(std::string("setInertia"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,dynamicgraph::Matrix>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setInertia ,docstring));

    docstring  =
            "\n"
            "    constnant inertia \n"
            "\n";

    addCommand(std::string("constantInertia"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,bool>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::constantInertia ,docstring));


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
    supportPos1_=lfconf;

    supportPos2SOUT_.setConstant (rfconf);
    supportPos2SOUT_.setTime (0);
    supportPos2_=rfconf;

    nbSupportSOUT_.setConstant (2);
    nbSupportSOUT_.setTime (0);

    stateObservation::Vector com;
    com.resize(3);
    com <<  0.00949,
            0,
            0.80771;
    comSIN_.setConstant(convertVector<dynamicgraph::Vector>(com));

    stateObservation::Matrix4 homoWaist;
    homoWaist <<      0.99998573432883131, -0.0053403256847235764, 0.00010981989355530105, -1.651929567364003e-05,
                      0.0053403915800877009, 0.99998555471196726, -0.00060875707006170711, 0.0008733516988761506,
                      -0.00010656734615829933, 0.00060933486696839291, 0.99999980867719196, 0.64869730032049466,
                      0.0, 0.0, 0.0, 1.0;
    waistHomoSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(homoWaist));

    stateObservation::Matrix inertia;
    inertia.resize(6,6);
    inertia <<  56.867992000000001, 1.082595017423981e-17, -1.0227922838095677e-19, 0.012404467020412754, 9.0440811406882826, -0.087341805362338931,
                2.1471441204328374e-17, 56.867991999999994, -6.6915602833089727e-20, -9.0441155822106154, 0.012400591461492968, 0.77639199232843892,
                -1.6877131474041934e-19, 7.0557844506283218e-19, 56.867992000000001, 0.086277191499897876, -0.77651101919284393, -6.6174449004242214e-22,
                0.012404467020412752, -9.0441155822106154, 0.086277191499897876, 9.5962629755728397, -0.0033323767643302234, 0.15070837823024946,
                9.0440811406882808, 0.012400591461492966, -0.77651101919284393, -0.0033323567633330438, 8.3906225580324634, -0.038677900732410085,
                -0.087341805362338931, 0.77639199232843892, 6.6174449004242214e-22, 0.15070833849597523, -0.03867792067753683, 1.7394745168929315;
    inertiaSIN.setConstant(convertMatrix<dynamicgraph::Matrix>(inertia));

    stateObservation::Matrix leftFootPos;
    leftFootPos.resize(4,4);
    leftFootPos <<  1,1.94301e-07,2.363e-10,0.00949046,
                    -1.94301e-07,1,-2.70566e-12,0.095,
                    -2.363e-10,2.70562e-12,1,3.03755e-06,
                    0,0,0,1;
    leftFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(leftFootPos));

    stateObservation::Matrix rightFootPos;
    rightFootPos.resize(4,4);
    rightFootPos <<  1,-9.18094e-18,-1.52169e-16,0.009496046,
                    9.184e-18,1,-1.10345e-16,-0.095,
                    1.68756e-16,1.10345e-16,1,2.55006e-07,
                    0,0,0,1;
    rightFootPositionSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(rightFootPos));

    stateObservation::Vector forceRightFoot;
    forceRightFoot.resize(6);
    forceRightFoot <<   45.1262,
                        -21.367,
                        361.344,
                        1.12135,
                        -14.5562,
                        1.89125;
    forceRightFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(forceRightFoot));

    stateObservation::Vector forceLeftFoot;
    forceLeftFoot.resize(6);
    forceLeftFoot <<    44.6005,
                        21.7871,
                        352.85,
                        -1.00715,
                        -14.5158,
                        -1.72017;
    forceLeftFootSIN_.setConstant(convertVector<dynamicgraph::Vector>(forceLeftFoot));

    Vector vect;
    stateObservation::Vector vec;
    vec.resize(2); vec.setZero();
    vect.resize(3); vect.setZero();
    comDotSIN_.setConstant(vect);
    flexOriVectSIN_.setConstant(vect);
    flexAngVelVectSIN_.setConstant(vect);
    vect.resize(6); vect.setZero();
    waistVelSIN_.setConstant(vect);
    tflexSIN_.setConstant(vect);
    dtflexSIN_.setConstant(vect);
    ddtflexSIN_.setConstant(vect);


    Kth_.resize(3,3);
    Kdth_.resize(3,3);

    kth_=60;
    kdth_=6.5;

    stateObservation::Vector Qvec;
    stateObservation::Matrix Qglob;
    Qglob.resize(stateSize_,stateSize_);
    Qvec.resize(stateSize_);
    Qglob.setIdentity();
    Qglob.noalias()=0.00001*Qglob;
    Qvec    <<  1,     // com
                1,
                1,
                1,     // ori waist
                1,
                1,     // ori flex
                1,
                1,     // dcom
                1,
                1,
                1,     // d ori waist
                1,
                1,     // d ori flex
                1;
    Q_ = Qvec.asDiagonal()*Qglob;

    stateObservation::Vector Rvec;
    stateObservation::Matrix Rglob;
    Rglob.resize(controlSize_,controlSize_);
    Rvec.resize(controlSize_);
    Rglob.setIdentity();
    Rglob.noalias()=1*Rglob;
    Rvec    <<  1,     // dd com
                1,
                1,
                1,     // dd ori waist
                1;

    Q_ = Qvec.asDiagonal()*Qglob;
    R_ = Rvec.asDiagonal()*Rglob;

    zmp_.setZero ();

    int horizon = 4000;

    nbSupport_=computeNbSupport(0);

    Kth_ <<    0.5*kth_*kth_,0,0,
              0,kth_,0,
              0,0,0.5*kth_*kth_;
    Kdth_ <<    0.5*kdth_ ,0,0,
              0,kdth_,0,
              0,0,0.5*kdth_*kdth_;

    controller_.setHorizonLength(horizon);
    computeDynamicsMatrix(com,Kth_,Kdth_,0);
    controller_.setDynamicsMatrices(A_, B_);
    controller_.setCostMatrices(Q_,R_);

    Vector v(stateSize_);
    v.setZero();
    stateWorldSOUT_.setConstant(v);

    preTask_.resize(controlSize_);
    preTask_.setZero();

    hrp2Mass_ = 58;

    dynamicgraph::Matrix::Matrix gains;
    gains.resize(controlSize_,stateSize_);
    gains.setZero();
    gainSOUT.setConstant (gains);
    AmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(A_));
    BmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(B_));

    dynamicgraph::Vector zero(6); zero.setZero();
    controlSOUT_.setConstant(zero);

    vect.resize(4);
    energySOUT_.setConstant(vect);

  }

  Vector& HRP2LQRTwoDofCoupledStabilizer::getControl(Vector& control, const int& time)
  {
      taskSOUT.access(time);
      control=controlSOUT_.access (time);
      return control;
  }

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

      MatrixRotation rfrot;
      MatrixRotation lfrot;

      VectorUTheta rfuth;
      VectorUTheta lfuth;

      rightFootPosition.extract(rfpos);
      rightFootPosition.extract(rfrot);
      rfuth.fromMatrix(rfrot);

      leftFootPosition.extract(lfpos);
      leftFootPosition.extract(lfrot);
      lfuth.fromMatrix(lfrot);

      Vector rfconf(6);
      Vector lfconf(6);

      for (size_t i=0; i<3; ++i)
      {
        rfconf(i)   = rfpos(i);
        rfconf(i+3) = rfuth(i);
        lfconf(i)   = lfpos(i);
        lfconf(i+3) = lfuth(i);
      }

      // Express vertical component of force in global basis
      double flz = leftFootPosition (2,0) * forceLf (0) +
                   leftFootPosition(2,1) * forceLf (1) +
                   leftFootPosition (2,2) * forceLf (2);
      double frz = rightFootPosition (2,0) * forceRf (0) +
                   rightFootPosition(2,1) * forceRf (1) +
                   rightFootPosition (2,2) * forceRf (2);

      //compute the number of supports
      unsigned int nbSupport = 0;
      if (frz >= forceThreshold_)
      {
        supportPos1SOUT_.setConstant (rfconf);
        supportPos1SOUT_.setTime (time);
        supportPos1_=rfconf;
        nbSupport++;
      }

      if (flz >= forceThreshold_)
      {
        if (nbSupport==0)
        {
          supportPos1SOUT_.setConstant (lfconf);
          supportPos1SOUT_.setTime (time);
          supportPos1_=lfconf;
        }
        else
        {
          supportPos2SOUT_.setConstant (lfconf);
          supportPos2SOUT_.setTime (time);
          supportPos2_=lfconf;
        }
        nbSupport++;
      }

      nbSupportSOUT_.setConstant (nbSupport);
      nbSupportSOUT_.setTime (time);

      if (!on_)
      {
        nbSupport=0;
      }

      return nbSupport;
  }


/// Compute the control law
  VectorMultiBound&
  HRP2LQRTwoDofCoupledStabilizer::computeControlFeedback(VectorMultiBound& task,
      const int& time)
  {

    //std::cout << "\n\n time: " << time << std::endl;

    // State
    const stateObservation::Vector & com = convertVector<stateObservation::Vector>(comSIN_.access(time));
    const Matrix4& waistHomo = convertMatrix<stateObservation::Matrix>(waistHomoSIN_ (time));
    const stateObservation::Vector & flexOriVect = convertVector<stateObservation::Vector>(flexOriVectSIN_.access(time));
    const stateObservation::Vector & comDot = convertVector<stateObservation::Vector>(comDotSIN_ (time));
    const stateObservation::Vector & waistVel = convertVector<stateObservation::Vector>(waistVelSIN_ (time));
    const stateObservation::Vector & flexAngVelVect = convertVector<stateObservation::Vector>(flexAngVelVectSIN_.access(time));

    // Translational part of the flexibility
    const stateObservation::Vector & tflex = convertVector<stateObservation::Vector>(tflexSIN_.access(time));
    const stateObservation::Vector & dtflex = convertVector<stateObservation::Vector>(dtflexSIN_.access(time));
    const stateObservation::Vector & ddtflex = convertVector<stateObservation::Vector>(ddtflexSIN_.access(time));

    // State Reference
        // References of velocities and acceleration are equal to zero
    const stateObservation::Vector & comRef = convertVector<stateObservation::Vector>(comRefSIN_ (time));
    const stateObservation::Vector & waistOriRef = convertVector<stateObservation::Vector>(waistOriRefSIN_.access(time));
    const stateObservation::Vector & flexOriRef = convertVector<stateObservation::Vector>(flexOriRefSIN_.access(time));
    const stateObservation::Vector & comDotRef = convertVector<stateObservation::Vector>(comDotRefSIN_ (time));
    const stateObservation::Vector & waistAngVelRef = convertVector<stateObservation::Vector>(waistVelRefSIN_ (time));
    const stateObservation::Vector & flexAngVelRef = convertVector<stateObservation::Vector>(flexAngVelRefSIN_.access(time));

    // Determination of the number of support
    unsigned int nbSupport=computeNbSupport(time);


    // Control gain
    const double& gain = controlGainSIN_.access (time);

    // Waist orientation
    Matrix3 waistOri=waistHomo.block(0,0,3,3);
    stateObservation::Vector3 waistOriVect;
    waistOriVect=kine::rotationMatrixToRotationVector(waistOri);
    stateObservation::Vector3 waistAngVel = waistVel.tail(3);

    // flex orientation
    Matrix3 flexOri=kine::rotationVectorToRotationMatrix(flexOriVect);

    // Angular momentum
    const stateObservation::Vector & angularmomentum = convertVector<stateObservation::Vector>(angularmomentumSIN.access(time));

    /// State in the local frame

    // State reconstruction
    stateObservation::Vector xk;
    xk.resize(stateSize_);
    xk <<   com,
            (waistOriVect).block(0,0,2,1),
            (flexOriVect).block(0,0,2,1),
            comDot,
            (waistAngVel).block(0,0,2,1),
            (flexAngVelVect).block(0,0,2,1);
    stateSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xk));

    /// State in the world frame

//    // State reconstruction
//    stateObservation::Vector xk;
//    xk.resize(stateSize_);
//    xk <<   flexOri*com,
//            (flexOriVect+waistOri).block(0,0,2,1),
//            (flexOriVect).block(0,0,2,1),
//            kine::skewSymmetric(flexOriVect)*flexOri*com+flexOri*comDot,
//            (flexAngVelVect+flexOri*waistAngVel).block(0,0,2,1),
//            (flexAngVelVect).block(0,0,2,1);
//    stateSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xk));

    // Extended state reconstruction
    stateObservation::Vector extxk;
    extxk.resize(stateSize_+4);
    extxk <<    com,
                waistOriVect,
                flexOriVect,
                comDot,
                waistAngVel,
                flexAngVelVect;
    stateExtendedSOUT_.setConstant (convertVector<dynamicgraph::Vector>(extxk));
    stateExtendedSOUT_.setTime (time);

    // State in the world frame
    stateObservation::Vector xkworld;
    xkworld.resize(stateSize_);
    xkworld <<  flexOri*com+tflex,
                (kine::rotationMatrixToRotationVector(flexOri*waistOri)).block(0,0,2,1),
                (flexOriVect).block(0,0,2,1),
                kine::skewSymmetric(flexAngVelVect)*flexOri*com+flexOri*comDot+dtflex,
                (flexAngVelVect+flexOri*waistAngVel).block(0,0,2,1),
                (flexAngVelVect).block(0,0,2,1);
    stateWorldSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xkworld));

    // Reference reconstruction
    stateObservation::Vector xkRef;
    xkRef.resize(stateSize_);
    xkRef <<   comRef,
               waistOriRef.block(0,0,2,1),
               flexOriRef.block(0,0,2,1),
               comDotRef,
               waistAngVelRef.block(0,0,2,1),
               flexAngVelRef.block(0,0,2,1);
    stateRefSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xkRef));

    stateObservation::Vector controlDref;
    controlDref.resize(controlSize_);
    controlDref <<  comDotRef,
                    waistAngVelRef.block(0,0,2,1);

    // State error
    stateObservation::Vector dxk;
    dxk.resize(stateSize_);
    dxk=xk-xkRef;
    stateErrorSOUT_.setConstant (convertVector<dynamicgraph::Vector>(dxk));

    stateObservation::Vector u;
    u.resize(controlSize_);
    u.setZero();

    switch (nbSupport)
    {
        case 0: // No support
        {
            preTask_ <<  -gain*dxk.block(0,0,5,1)+controlDref;
        }
        break;
        case 1: // Single support
        {
             if(nbSupport!=nbSupport_ || computed_==false || fixedGains_!=true) // || comRef!=comRef_)
             {
                Kth_ <<   kth_,0,0,
                          0,kth_,0,
                          0,0,kth_;
                Kdth_ <<  kdth_,0,0,
                          0,kdth_,0,
                          0,0,kdth_;

                computeDynamicsMatrix(xkRef.block(0,0,3,1),Kth_,Kdth_,time);
                controller_.setDynamicsMatrices(A_,B_);
                nbSupport_=nbSupport;
                computed_=true;
                comRef_=comRef;
             }
             controller_.setState(dxk,time);
             u=controller_.getControl(time);
             preTask_+=dt_*u+controlDref;
        }
        break;
        case 2 : // Double support
        {
              if(nbSupport!=nbSupport_ || computed_ == false || fixedGains_!=true) // || comRef!=comRef_)
              {
                  Kth_ <<    0.5*kth_*kth_,0,0,
                            0,kth_,0,
                            0,0,0.5*kth_*kth_;
                  Kdth_ <<  0.5*kdth_*kdth_ ,0,0,
                            0,kdth_,0,
                            0,0,0.5*kdth_*kdth_;

                  // TODO: when feet are not aligned along the y axis

                  computeDynamicsMatrix(xkRef.block(0,0,3,1),Kth_,Kdth_,time);
                  controller_.setDynamicsMatrices(A_,B_);
                  nbSupport_=nbSupport;
                  computed_=true;
                  comRef_=comRef;
              }
              controller_.setState(dxk,time);
              u=controller_.getControl(time);
              preTask_+=dt_*u+controlDref;
        }
        break;
        default: throw std::invalid_argument("Only 0, 1 and 2 number of supports cases are developped");
    };

    // Energy
    double Etot, Eflex, Ecom, Ewaist;
    Eflex=0.5*Kth_(0,0)*flexOriVect.squaredNorm();
    Ecom=0.5*constm_*(xkworld.block(7,0,3,1)).squaredNorm();
    Ewaist=0.5*angularmomentum.dot(waistAngVel);
    Etot=Eflex+Ecom+Ewaist;
    stateObservation::Vector energy;
    energy.resize(4);
    energy <<   Etot,
                Ecom,
                Eflex,
                Ewaist;
    energySOUT_.setConstant(convertVector<dynamicgraph::Vector>(energy));


    task.resize (taskSize_);
    int i;
    for (i=0;i<controlSize_;i++)
    {
        task [i].setSingleBound (preTask_(i));
    }
    for (i=controlSize_;i<taskSize_;i++)
    {
        task [i].setSingleBound (preTask_(i-taskSize_+controlSize_));
    }

    stateObservation::Vector error;
    error.resize(controlSize_);
    error.block(0,0,5,1)=dxk.block(0,0,5,1);
    errorSOUT_.setConstant (convertVector<dynamicgraph::Vector>(error));
    errorSOUT_.setTime (time);

    controlSOUT_.setConstant (convertVector<dynamicgraph::Vector>(u));
    controlSOUT_.setTime (time);
    gainSOUT.setConstant (convertMatrix<dynamicgraph::Matrix>(controller_.getLastGain()));
    AmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(A_));
    BmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(B_));
   // std::cout << "A: " << A_ << std::endl;
   // std::cout << "B: " << B_ << std::endl;

    // Validation of the model
    stateObservation::Vector modelError;
    modelError.resize(stateSize_);
    modelError.setZero();
    xpredicted_=A_*dxk+B_*u;
    stateSimulationSOUT_.setConstant(convertVector<dynamicgraph::Vector>(xpredicted_));
    modelError=xpredicted_-dxk;
    stateModelErrorSOUT_.setConstant(convertVector<dynamicgraph::Vector>(modelError));

    return task;
  }

  Matrix& HRP2LQRTwoDofCoupledStabilizer::computeJacobian(Matrix& jacobian, const int& time)
  {
    typedef unsigned int size_t;

    const stateObservation::Matrix & jacobianCom=convertMatrix<stateObservation::Matrix>(jacobianComSIN_(time));
    const stateObservation::Matrix & jacobianWaist=convertMatrix<stateObservation::Matrix>(jacobianWaistSIN_(time));
    const stateObservation::Matrix & jacobianChest=convertMatrix<stateObservation::Matrix>(jacobianChestSIN_(time));

    stateObservation::Matrix jacobianWaistOri = jacobianWaist.block(3,0,2,jacobianWaist.cols());
    stateObservation::Matrix jacobianChestOri = jacobianChest.block(3,0,2,jacobianChest.cols());

    stateObservation::Matrix preJacobian;
    preJacobian.resize(jacobianCom.rows()+jacobianWaistOri.rows()+jacobianChestOri.rows(),jacobianCom.cols());

    preJacobian.block(0,0,jacobianCom.rows(),jacobianCom.cols())= jacobianCom;
    preJacobian.block(jacobianCom.rows(),0,jacobianWaistOri.rows(),jacobianWaistOri.cols())= jacobianWaistOri;
    preJacobian.block(jacobianCom.rows()+jacobianWaistOri.rows(),0,jacobianChestOri.rows(),jacobianChestOri.cols())= jacobianChestOri;

    jacobian = convertMatrix<dynamicgraph::Matrix>(preJacobian);

    return jacobian;
  }


  void HRP2LQRTwoDofCoupledStabilizer::computeDynamicsMatrix(const stateObservation::Vector3 cl, const stateObservation::Matrix Kth, const stateObservation::Matrix Kdth, const int& time)
  {
    double g = stateObservation::cst::gravityConstant;
    double m = hrp2Mass_;

    /// State in the local frame

    stateObservation::Matrix I;
    if(constantInertia_!=true)
    {
        I = computeInert(cl,time);
    }
    else
    {
        I=I_;
    }
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
    Inertia -= m * kine::skewSymmetric2(cl);
    Inertia = Inertia.inverse();

    stateObservation::Vector3 v;
    v=-g*m*Inertia*kine::skewSymmetric(cl)*uz;

    // Caracteristic polynomial
    ddomega_cl=Inertia*m*(2*kine::skewSymmetric(cl)*kine::skewSymmetric(v)-kine::skewSymmetric(v)*kine::skewSymmetric(cl)-g*kine::skewSymmetric(uz));
    ddomega_omegach=Inertia*I*kine::skewSymmetric(v)-Inertia*kine::skewSymmetric(I*v);
    ddomega_omega=kine::skewSymmetric(v)-Inertia*(Kth-g*m*kine::skewSymmetric(cl)*kine::skewSymmetric(uz)); //
    ddomega_dcl.setZero(); //
    ddomega_domegach.setZero(); //
    ddomega_domega=-Inertia*Kdth; //

    ddomega_ddcl=-m*Inertia*kine::skewSymmetric(cl); //
    ddomega_ddomegach=-Inertia*I; //

    /// State in the world frame

//    stateObservation::Matrix I = computeInert(cl,time);
//    stateObservation::Matrix3 identity;
//    identity.setIdentity();
//
//    stateObservation::Matrix3 ddomega_cl, ddomega_omegach, ddomega_omega, ddomega_dcl,
//                              ddomega_domegach, ddomega_domega, ddomega_ddcl, ddomega_ddomegach;
//
//    // usefull variables for code factorisation
//    stateObservation::Vector3 uz;
//    uz <<     0,
//              0,
//              1;
//
//    stateObservation::Matrix Inertia;
//    Inertia = I;
//    Inertia = Inertia.inverse();
//
//    // Caracteristic polynomial
//    ddomega_cl=Inertia*g*m*kine::skewSymmetric(uz);
//    ddomega_omegach=Inertia*kine::skewSymmetric(g*m*kine::skewSymmetric(cl)*uz)-kine::skewSymmetric(Inertia*g*m*kine::skewSymmetric(cl)*uz);
//    ddomega_omega=-Inertia*Kth; //
//    ddomega_dcl.setZero(); //
//    ddomega_domegach.setZero(); //
//    ddomega_domega=-Inertia*Kdth; //
//
//    ddomega_ddcl=-m*Inertia*kine::skewSymmetric(cl); //
//    ddomega_ddomegach=-identity; //

    // A_ and B_ computation
    A_.block(0,7,3,3)=identity;
    A_.block(3,10,2,2)=identity.block(0,0,2,2);
    A_.block(5,12,2,2)=identity.block(0,0,2,2);
    A_.block(12,0,2,3)=ddomega_cl.block(0,0,2,3);
    A_.block(12,3,2,2)=ddomega_omegach.block(0,0,2,2);
    A_.block(12,5,2,2)=ddomega_omega.block(0,0,2,2);
    A_.block(12,7,2,3)=ddomega_dcl.block(0,0,2,3);
    A_.block(12,10,2,2)=ddomega_domegach.block(0,0,2,2);
    A_.block(12,12,2,2)=ddomega_domega.block(0,0,2,2);

    stateObservation::Matrix Identity(stateObservation::Matrix::Zero(stateSize_,stateSize_));
    Identity.setIdentity();

    A_.noalias() = dt_ * A_ + Identity;

    B_.block(7,0,3,3)=identity;
    B_.block(10,3,2,2)=identity.block(0,0,2,2);
    B_.block(12,0,2,3)=ddomega_ddcl.block(0,0,2,3);
    B_.block(12,3,2,2)=ddomega_ddomegach.block(0,0,2,2);

    B_.noalias() = dt_* B_;

  }


  stateObservation::Matrix3 HRP2LQRTwoDofCoupledStabilizer::computeInert(const stateObservation::Vector& cl, const int& inTime)
{
    const stateObservation::Matrix& inertiaSotWaistFrame=convertMatrix<stateObservation::Matrix>(inertiaSIN.access(inTime));
    const MatrixHomogeneous& waistHomo = waistHomoSIN_.access(inTime);
    Vector waistPos(3);
    waistHomo.extract(waistPos);

    double m=inertiaSotWaistFrame(0,0); //<=== give 56.8;
    stateObservation::Vector3 wl=convertVector<stateObservation::Vector>(waistPos);


    stateObservation::Matrix3 inertiaControlFrame, inertiaComFrame, inertiaWaistFrame;

    inertiaWaistFrame = inertiaSotWaistFrame.block(3,3,3,3);
    inertiaComFrame = inertiaWaistFrame + m * kine::skewSymmetric2(cl-wl);

    inertiaSOUT.setConstant (convertMatrix<dynamicgraph::Matrix>(inertiaComFrame));
    I_=inertiaComFrame;
    return inertiaComFrame;
}

} // namespace sotStabilizer
