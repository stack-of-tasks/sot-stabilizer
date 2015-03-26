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
#include "command-increment.hh"

#include <iostream>

namespace sotStabilizer
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NonLinearRotationalTableCartDevice, "NonLinearRotationalTableCartDevice");

NonLinearRotationalTableCartDevice::NonLinearRotationalTableCartDevice(const std::string& inName) :
  Entity(inName),
  controlSIN_(NULL, "NonLinearRotationalTableCartDevice("+inName+")::input(vector)::control"),
  stateSOUT_("NonLinearRotationalTableCartDevice("+inName+")::output(vector)::state"),
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

  // Set signals as constant to size them
  dynamicgraph::Vector state (20); state.fill (0.);
  stateSOUT_.setConstant(state);
  dynamicgraph::Vector control (6); control.setZero ();
  controlSIN_.setConstant (control);

  // Commands
  std::string docstring;

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

void NonLinearRotationalTableCartDevice::computeAccelerations
   (const Vector3& positionCom, const Vector3& velocityCom,
    const Vector3& accelerationCom, const Vector3& AngMomentum,
    const Vector3& dotAngMomentum,
    const Matrix3& Inertia, const Matrix3& dotInertia,
    const Vector3& contactPosV,
    const Vector3& contactOriV,
    const Vector3& position, const Vector3& linVelocity, Vector3& linearAcceleration,
    const Vector3 &oriVector ,const Matrix3& orientation,
    const Vector3& angularVel, Vector3& angularAcceleration)
{
    op_.skewV=kine::skewSymmetric(angularVel);
    op_.skewV2=kine::skewSymmetric2(angularVel);
    op_.skewVR=op_.skewV * orientation;
    op_.skewV2R=op_.skewV2 * orientation;

    op_.rFlexT=orientation.transpose();

    computeElastContactForcesAndMoments (contactPosV, contactOriV,
                      position, linVelocity, oriVector, orientation,
                         angularVel, op_.fc, op_.tc);

    op_.wx2Rc=op_.skewV2R*positionCom;
    op_._2wxRv=2*op_.skewVR*velocityCom;
    op_.Ra = orientation*accelerationCom;
    op_.Rc = orientation * positionCom;
    op_.Rcp = op_.Rc+position;
    op_.RIRT = orientation*Inertia*op_.rFlexT;

    op_.vf =robotMassInv_*op_.fc;
    op_.vf.noalias() -= op_.Ra;
    op_.vf.noalias() -= op_._2wxRv;
    op_.vf.noalias() -= op_.wx2Rc;
    op_.vf.noalias() -= cst::gravity;

    op_.vt =op_.tc;
    op_.vt.noalias() -= op_.skewV * (op_.RIRT * angularVel);
    op_.vt.noalias() -= orientation * (dotInertia * (op_.rFlexT * angularVel)+dotAngMomentum) ;
    op_.vt.noalias() -= op_.skewVR * AngMomentum;
    op_.vt.noalias() -= robotMass_* (kine::skewSymmetric(position) *
                        (op_.wx2Rc+ op_._2wxRv + op_.Ra ));
    op_.vt.noalias() -= robotMass_* kine::skewSymmetric(op_.Rcp) * cst::gravity;

    angularAcceleration = ( op_.RIRT + robotMass_*kine::skewSymmetric2(op_.Rc)).llt().solve(
                            (op_.vt - robotMass_*kine::skewSymmetric(op_.Rcp)*op_.vf));

    linearAcceleration = op_.vf;
    linearAcceleration += kine::skewSymmetric(op_.Rc)*angularAcceleration;

}

Matrix3& NonLinearRotationalTableCartDevice::computeRotation_
            (const Vector3 & x, int i)
{
    Vector3 & oriV = op_.orientationVector(i);
    Matrix3 & oriR = op_.curRotation(i);
    if (oriV!=x)
    {
        oriV = x;
        oriR = kine::rotationVectorToAngleAxis(x).toRotationMatrix();
    }

    return oriR;
}

void NonLinearRotationalTableCartDevice::computeElastContactForcesAndMoments
                          (const Vector3& contactPosArray,
                           const Vector3& contactOriArray,
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
    op_.contactPos = contactPosArray;

    op_.Rci = computeRotation_(contactOriArray,i+2);
    op_.Rcit.noalias()= op_.Rci.transpose();

    op_.RciContactPos.noalias()= orientation*op_.contactPos;

    op_.globalContactPos = position;
    op_.globalContactPos.noalias() += op_.RciContactPos ;

    op_.forcei.noalias() = - op_.Rci*Kfe_*op_.Rcit*(op_.globalContactPos-op_.contactPos);
    op_.forcei.noalias() += - op_.Rci*Kfv_*op_.Rcit*(kine::skewSymmetric(angVel)*op_.RciContactPos
                              +linVelocity);

    fc_.segment<3>(3*i)= op_.forcei;

    forces += op_.forcei;

    op_.momenti.noalias() = -op_.Rci*Kte_*op_.Rcit*oriVector;
    op_.momenti.noalias() += -op_.Rci*Ktv_*op_.Rcit*angVel;

    tc_.segment<3>(3*i)= op_.momenti;

    moments.noalias() += op_.momenti + kine::skewSymmetric(op_.globalContactPos)*op_.forcei;;

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

  Vector3 positionCom=xn.block(0,0,3,1);
  Vector3 velocityCom=xn.block(12,0,3,1);
  Vector3 accelerationCom=un.block(0,0,3,1);
  Vector3 AngMomentum;
  Vector3 dotAngMomentum;
  Matrix3 Inertia;
  Matrix3 dotInertia;
  Vector3 contactPosV;
  Vector3 contactOriV;
  Vector3 position=xn.block(9,0,3,1);
  Vector3 linVelocity=xn.block(21,0,3,1);
  Vector3 oriVector=xn.block(6,0,3,1);
  Matrix3 orientation;
  Vector3 angularVel=xn.block(18,0,3,1);

  Vector6 waistOriVect;
  waistOriVect <<   xn.block(3,0,3,1),
                    0,
                    0,
                    0;
  Matrix3 waistOri=(kine::vector6ToHomogeneousMatrix(waistOriVect)).block(0,0,3,3);
  Vector3 waistAngVel=xn.block(15,0,3,1);

  Vector3 waistAngAcc=un.block(3,0,3,1);

  contactPosV.setZero();
  contactOriV.setZero();

  Inertia=waistOri*I_*waistOri.transpose()-robotMass_*kine::skewSymmetric2(positionCom);
  dotInertia.setZero();
  AngMomentum=waistOri*I_*waistOri.transpose()*waistAngVel+robotMass_*kine::skewSymmetric(positionCom)*velocityCom;
  dotAngMomentum.setZero();

  computeAccelerations
     (positionCom, velocityCom,
      accelerationCom, AngMomentum,
      dotAngMomentum,
      Inertia, dotInertia,
      contactPosV,
      contactOriV,
      position, linVelocity, linearAcceleration,
      oriVector , orientation,
      angularVel, angularAcceleration);


  stateObservation::Vector xn1;
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

  return nextState;
}

void NonLinearRotationalTableCartDevice::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();
  stateObservation::Vector cl;

  stateObservation::Vector xn = convertVector<stateObservation::Vector>(stateSOUT_ (t));
  stateObservation::Vector un = convertVector<stateObservation::Vector>(controlSIN_ (t));

  stateObservation::Vector nextState = computeDynamics(
        inTimeStep,
        xn, un);

  cl.resize(3);
  cl=cl_;
  stateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextState));
  stateSOUT_.setTime (t+1);

}

}
