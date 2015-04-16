/*
 *  Copyright 2014 CNRS
 *
 *  Alexis Mifsud
 */

#include <boost/format.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <sot-stabilizer/prototyping/non-linear-rotational-table-cart-device.hh>
#include <dynamic-graph/command-bind.h>
#include "command-increment_NonLinearRotationalTableCartDevice.hh"

#include <iostream>

using dynamicgraph::command::makeDirectSetter;
using dynamicgraph::command::makeDirectGetter;

namespace sotStabilizer
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NonLinearRotationalTableCartDevice, "NonLinearRotationalTableCartDevice");

NonLinearRotationalTableCartDevice::NonLinearRotationalTableCartDevice(const std::string& inName) :
  Entity(inName),
  controlSIN_(NULL, "NonLinearRotationalTableCartDevice("+inName+")::input(vector)::control"),
  stateSOUT_("NonLinearRotationalTableCartDevice("+inName+")::output(vector)::state"),
  controlStateSOUT_("NonLinearRotationalTableCartDevice("+inName+")::output(vector)::controlState"),
  comSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::com"),
  waistHomoSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::waistHomo"),
  flexOriVectSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexOriVect"),
  comDotSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comDot"),
  waistVelSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::waistVel"),
  flexAngVelVectSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexAngVelVect"),
  flexAngAccVectSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexAngAccVect"),
  flexLinAccSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexLinAcc"),
  robotMass_(58.0), robotMassInv_(1/58.0), I_ (3,3), cl_(3), contactsNumber_(2)
{

  Kfe_.resize(3,3);
  Kfv_.resize(3,3);
  Kte_.resize(3,3);
  Ktv_.resize(3,3);
  I_.setZero();

  Kfe_ <<   100,0,0,
            0,100,0,
            0,0,100;

  Kfv_ <<     0.1,0,0,
              0,0.1,0,
              0,0,0.1;

  Kte_ <<    100,0,0,
             0,100,0,
             0,0,100;

  Ktv_ <<     0.1,0,0,
              0,0.1,0,
              0,0,0.1;

  // Register signals into the entity.
  signalRegistration (controlSIN_);
  signalRegistration (stateSOUT_);
  signalRegistration (comSOUT_);
  signalRegistration (waistHomoSOUT_);
  signalRegistration (flexOriVectSOUT_);
  signalRegistration (comDotSOUT_);
  signalRegistration (waistVelSOUT_);
  signalRegistration (flexAngVelVectSOUT_);
  signalRegistration (flexAngAccVectSOUT_);
  signalRegistration (flexLinAccSOUT_);


//  comSOUT_.addDependency (controlSIN_);
//  waistHomoSOUT_.addDependency (controlSIN_);
//  flexOriVectSOUT_.addDependency (controlSIN_);
//  comDotSOUT_.addDependency (controlSIN_);
//  waistVelSOUT_.addDependency (controlSIN_);
//  flexAngVelVectSOUT_.addDependency (controlSIN_);

  // Set signals as constant to size them
  stateObservation::Vector state;
  state.resize(24);
//  state <<  0.00949046,
//            0,
//            0.80771,
  state <<  0.00949046,
            0,
            0.80777699999999997,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            -0.0073,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0;
  stateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(state));
  stateSOUT_.setTime(0);

  stateObservation::Vector control; control.resize(5);
  control.setZero();
  controlSIN_.setConstant(convertVector<dynamicgraph::Vector>(control));
  controlSIN_.setTime(0);

  stateObservation::Vector controlState; controlState.resize(14);
  controlState.setZero();
  controlStateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(controlState));
  controlStateSOUT_.setTime(0);

  stateObservation::Vector com;
  com.resize(3);
//  com <<  0.00949046,
//          0,
//          0.80771;
  com <<  0.00965,
          0,
          0.80777699999999997;
  comSOUT_.setConstant(convertVector<dynamicgraph::Vector>(com));
  comSOUT_.setTime(0);

  stateObservation::Matrix4 homoWaist;
  homoWaist <<      0.99998573432883131, -0.0053403256847235764, 0.00010981989355530105, -1.651929567364003e-05,
                    0.0053403915800877009, 0.99998555471196726, -0.00060875707006170711, 0.0008733516988761506,
                    -0.00010656734615829933, 0.00060933486696839291, 0.99999980867719196, 0.64869730032049466,
                    0.0, 0.0, 0.0, 1.0;
  waistHomoSOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(homoWaist));
  waistHomoSOUT_.setTime(0);

  dynamicgraph::Vector vect;
  stateObservation::Vector vec;
  vec.resize(2); vec.setZero();
  vect.resize(3); vect.setZero();
  comDotSOUT_.setConstant(vect);
  comDotSOUT_.setTime(0);
  flexOriVectSOUT_.setConstant(vect);
  flexOriVectSOUT_.setTime(0);
  flexAngVelVectSOUT_.setConstant(vect);
  flexAngVelVectSOUT_.setTime(0);
  vect.resize(6); vect.setZero();
  waistVelSOUT_.setConstant(vect);
  waistVelSOUT_.setTime(0);

  flexAngAccVectSOUT_.setConstant(vect);
  flexAngAccVectSOUT_.setTime(0);
  flexLinAccSOUT_.setConstant(vect);
  flexLinAccSOUT_.setTime(0);

  stateObservation::Matrix4 positionHomo;
  positionHomo  <<  1,-9.18094e-18,-1.52169e-16,0.00949046,
                    9.184e-18,1,-1.10345e-16,-0.095,
                    1.68756e-16,1.10345e-16,1,2.55006e-07,
                    0,0,0,1;
  setContactPosition(0, positionHomo);
  positionHomo  <<  1,1.94301e-07,2.363e-10,0.00949046,
                    -1.94301e-07,1,-2.70566e-12,0.095,
                    -2.363e-10,2.70562e-12,1,3.03755e-06,
                    0,0,0,1;
  setContactPosition(1, positionHomo);

  // Commands
  std::string docstring;

  // Incr
  docstring =
    "\n"
    "    Integrate dynamics for time step provided as input\n"
    "\n"
    "      take one floating point number as input\n"
    "\n";
  addCommand(std::string("incr"),
             new command::Increment1(*this, docstring));

  // setCartMass
  docstring =
    "\n"
    "    Set robot mass\n"
    "\n";
  addCommand(std::string("setRobotMass"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, double>
             (*this, &NonLinearRotationalTableCartDevice::setRobotMass, docstring));

  // getCartMass
  docstring =
    "\n"
    "    Get robot mass\n"
    "\n";
  addCommand(std::string("getRobotMass"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, double>
             (*this, &NonLinearRotationalTableCartDevice::getRobotMass, docstring));


  // setStiffness
  docstring =
    "\n"
    "    Set stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("setKfe"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKfe, docstring));

  // getStiffness
  docstring =
    "\n"
    "    Get cart stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("getKfe"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKfe, docstring));
  // setViscosity
  docstring =
    "\n"
    "    Set viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("setKfv"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKfv, docstring));

  // getViscosity
  docstring =
    "\n"
    "    Get cart viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("getKfv"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKfv, docstring));


 // setStiffness
  docstring =
    "\n"
    "    Set stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("setKte"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKte, docstring));

  // getStiffness
  docstring =
    "\n"
    "    Get cart stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("getKte"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKte, docstring));
  // setViscosity
  docstring =
    "\n"
    "    Set viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("setKtv"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKtv, docstring));

  // getViscosity
  docstring =
    "\n"
    "    Get cart viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("getKtv"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKtv, docstring));


  // setMomentOfInertia
  docstring =
    "\n"
    "    Set moment of inertia around y axis\n"
    "\n";

    addCommand(std::string("setMomentOfInertia"),
               new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
               (*this, &NonLinearRotationalTableCartDevice::setMomentOfInertia, docstring));

  // setMomentOfInertia
  docstring =
    "\n"
    "    Get moment of inertia around y axis\n"
    "\n";

    addCommand(std::string("getMomentOfInertia"),
               new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
               (*this, &NonLinearRotationalTableCartDevice::getMomentOfInertia, docstring));

}

NonLinearRotationalTableCartDevice::~NonLinearRotationalTableCartDevice()
{
}

void NonLinearRotationalTableCartDevice::setContactPosition
                                    (unsigned i, const Matrix4 & positionHomo)
{
    stateObservation::Vector6 positionVect;
    positionVect=kine::homogeneousMatrixToVector6(positionHomo);
    opti_.contactPosV.setValue(positionVect.block(0,0,3,1),i);
    opti_.contactOriV.setValue(positionVect.block(3,0,3,1),i);
}

Vector6 NonLinearRotationalTableCartDevice::getContactPosition(unsigned i)
{
    stateObservation::Vector6 contactPosVect;
    contactPosVect  <<  opti_.contactPosV[i],
                        opti_.contactOriV[i];
    return contactPosVect;
}

void NonLinearRotationalTableCartDevice::computeAccelerations
   (const Vector3& positionCom, const Vector3& velocityCom,
    const Vector3& accelerationCom, const Vector3& AngMomentum,
    const Vector3& dotAngMomentum,
    const Matrix3& Inertia, const Matrix3& dotInertia,
    const IndexedMatrixArray& contactPosV,
    const IndexedMatrixArray& contactOriV,
    const Vector3& position, const Vector3& linVelocity, Vector3& linearAcceleration,
    const Vector3 &oriVector ,const Matrix3& orientation,
    const Vector3& angularVel, Vector3& angularAcceleration)
{
    opti_.skewV=kine::skewSymmetric(angularVel);
    opti_.skewV2=kine::skewSymmetric2(angularVel);
    opti_.skewVR=opti_.skewV * orientation;
    opti_.skewV2R=opti_.skewV2 * orientation;

    opti_.rFlexT=orientation.transpose();

    computeElastContactForcesAndMoments (contactPosV, contactOriV,
                      position, linVelocity, oriVector, orientation,
                         angularVel, opti_.fc, opti_.tc);

    opti_.wx2Rc=opti_.skewV2R*positionCom;
    opti_._2wxRv=2*opti_.skewVR*velocityCom;
    opti_.Ra = orientation*accelerationCom;
    opti_.Rc = orientation * positionCom;
    opti_.Rcp = opti_.Rc+position;
    opti_.RIRT = orientation*Inertia*opti_.rFlexT;

    opti_.vf =robotMassInv_*opti_.fc;
    opti_.vf.noalias() -= opti_.Ra;
    opti_.vf.noalias() -= opti_._2wxRv;
    opti_.vf.noalias() -= opti_.wx2Rc;
    opti_.vf.noalias() -= cst::gravity;

    opti_.vt =opti_.tc;
    opti_.vt.noalias() -= opti_.skewV * (opti_.RIRT * angularVel);
    opti_.vt.noalias() -= orientation * (dotInertia * (opti_.rFlexT * angularVel)+dotAngMomentum) ;
    opti_.vt.noalias() -= opti_.skewVR * AngMomentum;
    opti_.vt.noalias() -= robotMass_* (kine::skewSymmetric(position) *
                        (opti_.wx2Rc+ opti_._2wxRv + opti_.Ra ));
    opti_.vt.noalias() -= robotMass_* kine::skewSymmetric(opti_.Rcp) * cst::gravity;

    angularAcceleration = ( opti_.RIRT + robotMass_*kine::skewSymmetric2(opti_.Rc)).llt().solve(
                            (opti_.vt - robotMass_*kine::skewSymmetric(opti_.Rcp)*opti_.vf));

    linearAcceleration = opti_.vf;
    linearAcceleration += kine::skewSymmetric(opti_.Rc)*angularAcceleration;

}

Matrix3& NonLinearRotationalTableCartDevice::computeRotation_
            (const Vector3 & x, int i)
{
    Vector3 & oriV = opti_.orientationVector(i);
    Matrix3 & oriR = opti_.curRotation(i);
    if (oriV!=x)
    {
        oriV = x;
        oriR = kine::rotationVectorToAngleAxis(x).toRotationMatrix();
    }

    return oriR;
}

void NonLinearRotationalTableCartDevice::computeElastContactForcesAndMoments
                          (const IndexedMatrixArray& contactPosArray,
                           const IndexedMatrixArray& contactOriArray,
                           const Vector3& position, const Vector3& linVelocity,
                           const Vector3& oriVector, const Matrix3& orientation,
                           const Vector3& angVel,
                           Vector3& forces, Vector3& moments)
{
    unsigned nbContacts=contactsNumber_;
    fc_.resize(nbContacts*3);
    tc_.resize(nbContacts*3);
    forces.setZero();
    moments.setZero();


  for (unsigned i = 0; i<nbContacts ; ++i)
  {
    opti_.contactPos = contactPosArray[i];

    opti_.Rci = computeRotation_(contactOriArray[i],i+2);
    opti_.Rcit.noalias()= opti_.Rci.transpose();

    opti_.RciContactPos.noalias()= orientation*opti_.contactPos;

    opti_.globalContactPos = position;
    opti_.globalContactPos.noalias() += opti_.RciContactPos ;

    opti_.forcei.noalias() = - opti_.Rci*Kfe_*opti_.Rcit*(opti_.globalContactPos-opti_.contactPos);
    opti_.forcei.noalias() += - opti_.Rci*Kfv_*opti_.Rcit*(kine::skewSymmetric(angVel)*opti_.RciContactPos
                              +linVelocity);

    fc_.segment<3>(3*i)= opti_.forcei;

    forces += opti_.forcei;

    opti_.momenti.noalias() = -opti_.Rci*Kte_*opti_.Rcit*oriVector;
    opti_.momenti.noalias() += -opti_.Rci*Ktv_*opti_.Rcit*angVel;

    tc_.segment<3>(3*i)= opti_.momenti;

    moments.noalias() += opti_.momenti + kine::skewSymmetric(opti_.globalContactPos)*opti_.forcei;;

    }

}

stateObservation::Vector NonLinearRotationalTableCartDevice::computeDynamics(
                                  double inTimeStep,
                                  stateObservation::Vector &xn,
                                  stateObservation::Vector &un)
{
  double dt = inTimeStep;

  Vector3 linearAcceleration;
  Vector3 angularAcceleration;
  linearAcceleration.setZero();
  angularAcceleration.setZero();
  Vector3 positionCom=xn.block(0,0,3,1);
  Vector3 velocityCom=xn.block(12,0,3,1);
  Vector3 accelerationCom=un.block(0,0,3,1);
  Vector3 AngMomentum;
  Vector3 dotAngMomentum;
  Matrix3 Inertia;
  Matrix3 dotInertia;
  Vector3 position=xn.block(9,0,3,1);
  Vector3 linVelocity=xn.block(21,0,3,1);
  Vector3 oriVector=xn.block(6,0,3,1);
  Matrix3 orientation;
  stateObservation::Vector oriVect;
  oriVect.resize(6);
  oriVect <<    0,
                0,
                0,
                oriVector;
  orientation=(kine::vector6ToHomogeneousMatrix(oriVect)).block(0,0,3,3);
  Vector3 angularVel=xn.block(18,0,3,1);
  Vector6 waistOriVect;
  waistOriVect <<   xn.block(3,0,3,1),
                    0,
                    0,
                    0;
  Matrix3 waistOri=(kine::vector6ToHomogeneousMatrix(waistOriVect)).block(0,0,3,3);
  Vector3 waistAngVel=xn.block(15,0,3,1);
  Vector3 waistAngAcc;
  waistAngAcc   <<  un.block(3,0,2,1),
                    0;
  Inertia=waistOri*I_*waistOri.transpose()-robotMass_*kine::skewSymmetric2(positionCom);
  dotInertia.setZero();
  AngMomentum=waistOri*I_*waistOri.transpose()*waistAngVel+robotMass_*kine::skewSymmetric(positionCom)*velocityCom;
  dotAngMomentum.setZero();

  computeAccelerations
     (positionCom, velocityCom,
      accelerationCom, AngMomentum,
      dotAngMomentum,
      Inertia, dotInertia,
      opti_.contactPosV,
      opti_.contactOriV,
      position, linVelocity, linearAcceleration,
      oriVector , orientation,
      angularVel, angularAcceleration);

  stateObservation::Vector xn1;
  xn1.resize(24);

  xn1   <<  velocityCom,
            waistAngVel,
            angularVel,
            linVelocity,
            accelerationCom,
            waistAngAcc,
            angularAcceleration,
            linearAcceleration;

  //const dynamicgraph::Vector& k2 = A_*(xn + k1*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k3 = A_*(xn + k2*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k4 = A_*(xn + k3*dt) + B_*inControl;
  //dynamicgraph::Vector dx = (k1 + k2*2 + k3*2 + k4)*(dt/6.);//Runge Kutta 4

  stateObservation::Vector dx = xn1* dt;
  stateObservation::Vector nextState = xn + dx;

  flexAngAccVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(angularAcceleration));
  flexLinAccSOUT_.setConstant(convertVector<dynamicgraph::Vector>(linearAcceleration));

  return nextState;
}

void NonLinearRotationalTableCartDevice::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();

  stateObservation::Vector xn = convertVector<stateObservation::Vector>(stateSOUT_ (t));
  stateObservation::Vector un = convertVector<stateObservation::Vector>(controlSIN_ (t));

  // Compute dynamics
  stateObservation::Vector nextState = computeDynamics(inTimeStep,xn, un);
  stateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextState));
  stateSOUT_.setTime (t+1);

  // Compute control state
  stateObservation::Vector nextControlState;
  nextControlState.resize(18);
  nextControlState  <<  nextState.block(0,0,3,1),
                        nextState.block(3,0,3,1),
                        nextState.block(6,0,3,1),
                        nextState.block(12,0,3,1),
                        nextState.block(15,0,3,1),
                        nextState.block(18,0,3,1);
  controlStateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextControlState));
  controlStateSOUT_.setTime(t+1);

  // Model output
  stateObservation::Vector6 waistPosVect;
  waistPosVect <<   nextControlState.block(3,0,3,1),
                    0,
                    0,
                    0;
  comSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextControlState.block(0,0,3,1)));
  comSOUT_.setTime(t+1);
  waistHomoSOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(kine::vector6ToHomogeneousMatrix(waistPosVect)));
  waistHomoSOUT_.setTime(t+1);
  flexOriVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextControlState.block(6,0,3,1)));
  flexOriVectSOUT_.setTime(t+1);
  comDotSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextControlState.block(9,0,3,1)));
  comDotSOUT_.setTime(t+1);
  stateObservation::Vector6 waistVel;
  waistVel  <<  0,
                0,
                0,
                nextControlState.block(12,0,3,1);
  waistVelSOUT_.setConstant(convertVector<dynamicgraph::Vector>(waistVel));
  waistVelSOUT_.setTime(t+1);
  flexAngVelVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextControlState.block(15,0,3,1)));
  flexAngVelVectSOUT_.setTime(t+1);
}

}
