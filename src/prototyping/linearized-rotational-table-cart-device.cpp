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
#include <sot-stabilizer/prototyping/linearized-rotational-table-cart-device.hh>
#include <dynamic-graph/command-bind.h>
#include "command-increment.hh"

namespace sotStabilizer
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(LinearizedRotationalTableCartDevice, "LinearizedRotationalTableCartDevice");

LinearizedRotationalTableCartDevice::LinearizedRotationalTableCartDevice(const std::string& inName) :
  Entity(inName),
  forceSIN_(NULL, "LinearizedRotationalTableCartDevice("+inName+")::input(double)::force"),
  controlSIN_(NULL, "LinearizedRotationalTableCartDevice("+inName+")::input(vector)::control"),
  stateSOUT_("LinearizedRotationalTableCartDevice("+inName+")::output(vector)::state"),
  outputSOUT_ ("LinearizedRotationalTableCartDevice("+inName+")::output(vector)::output"),
  comrealposSOUT_ ("LinearizedRotationalTableCartDevice("+inName+")::output(vector)::comreal"),
  flexcomddotSOUT_ ("LinearizedRotationalTableCartDevice("+inName+")::output(vector)::flexcomddot"),
  clSOUT_("LinearizedRotationalTableCartDevice("+inName+")::output(vector)::cl"),
  zmpSOUT_("LinearizedRotationalTableCartDevice("+inName+")::output(vector)::zmp"),
  cartMass_(58.0), I_ (3,3)
{
  A_.resize(18,18); B_.resize(18,18);
  A_.setZero (); B_.setZero (); I_.setZero();

  stiffness_ <<  100,0,0,
                0,100,0,
                0,0,100;

  viscosity_ <<     0.1,0,0,
                    0,0.1,0,
                    0,0,0.1;

  // Register signals into the entity.
  signalRegistration (forceSIN_);
  signalRegistration (controlSIN_);
  signalRegistration (stateSOUT_);
  signalRegistration (outputSOUT_);
  signalRegistration (comrealposSOUT_);
  signalRegistration (flexcomddotSOUT_);
  signalRegistration (clSOUT_);
  signalRegistration (zmpSOUT_);

  // Set signals as constant to size them
  dynamicgraph::Vector state (18); state.fill (0.);
  stateSOUT_.setConstant(state);
  double force = 0;
  forceSIN_.setConstant(force);
  dynamicgraph::Vector control (2); control.setZero ();
  controlSIN_.setConstant (control);
  dynamicgraph::Vector output (4); output.setZero ();
  outputSOUT_.setConstant (output);
  dynamicgraph::Vector zeroV (3); zeroV.setZero ();
  comrealposSOUT_.setConstant(zeroV);
  flexcomddotSOUT_.setConstant (zeroV);
  zmpSOUT_.setConstant(zeroV);

  // Commands
  std::string docstring;

  // setCartMass
  docstring =
    "\n"
    "    Set cart mass\n"
    "\n";
  addCommand(std::string("setCartMass"),
             new ::dynamicgraph::command::Setter<LinearizedRotationalTableCartDevice, double>
             (*this, &LinearizedRotationalTableCartDevice::setCartMass, docstring));

  // getCartMass
  docstring =
    "\n"
    "    Get cart mass\n"
    "\n";
  addCommand(std::string("getCartMass"),
             new ::dynamicgraph::command::Getter<LinearizedRotationalTableCartDevice, double>
             (*this, &LinearizedRotationalTableCartDevice::getCartMass, docstring));


  // setStiffness
  docstring =
    "\n"
    "    Set stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("setStiffness"),
             new ::dynamicgraph::command::Setter<LinearizedRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &LinearizedRotationalTableCartDevice::setStiffness, docstring));

  // getStiffness
  docstring =
    "\n"
    "    Get cart stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("getStiffness"),
             new ::dynamicgraph::command::Getter<LinearizedRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &LinearizedRotationalTableCartDevice::getStiffness, docstring));
  // setViscosity
  docstring =
    "\n"
    "    Set viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("setViscosity"),
             new ::dynamicgraph::command::Setter<LinearizedRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &LinearizedRotationalTableCartDevice::setViscosity, docstring));

  // getViscosity
  docstring =
    "\n"
    "    Get cart viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("getViscosity"),
             new ::dynamicgraph::command::Getter<LinearizedRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &LinearizedRotationalTableCartDevice::getViscosity, docstring));

  // setMomentOfInertia
  docstring =
    "\n"
    "    Set moment of inertia around y axis\n"
    "\n";

    addCommand(std::string("setMomentOfInertia"),
               new ::dynamicgraph::command::Setter<LinearizedRotationalTableCartDevice, dynamicgraph::Matrix>
               (*this, &LinearizedRotationalTableCartDevice::setMomentOfInertia, docstring));

  // setMomentOfInertia
  docstring =
    "\n"
    "    Get moment of inertia around y axis\n"
    "\n";

    addCommand(std::string("getMomentOfInertia"),
               new ::dynamicgraph::command::Getter<LinearizedRotationalTableCartDevice, dynamicgraph::Matrix>
               (*this, &LinearizedRotationalTableCartDevice::getMomentOfInertia, docstring));

    // recomputeMatrices
  docstring =
    "\n"
    "    Recomputes matrices A and B\n"
    "\n";
  addCommand ("recomputeMatrices",
              dynamicgraph::command::makeCommandVoid0
                (*this, &LinearizedRotationalTableCartDevice::recomputeMatrices, docstring));



}

void LinearizedRotationalTableCartDevice::recomputeMatrices()
{

  double g = stateObservation::cst::gravityConstant;
  double m = cartMass_;
  stateObservation::Matrix3 Kth = stiffness_;
  stateObservation::Matrix3 Kdth = viscosity_;
  stateObservation::Matrix I = I_;
  stateObservation::Vector cl = cl_;

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

  B_.block(9,0,2,2)=identity;
  B_.block(12,3,2,2)=identity;
  B_.block(15,0,2,2)=ddomega_ddcl;
  B_.block(15,3,2,2)=ddomega_ddomegach;
}

LinearizedRotationalTableCartDevice::~LinearizedRotationalTableCartDevice()
{
}

stateObservation::Vector LinearizedRotationalTableCartDevice::computeDynamics(
                                    const dynamicgraph::Vector& inState,
                                  const dynamicgraph::Vector& inControl,
                                  const double&,
                                  double inTimeStep,
                                  stateObservation::Vector & flexcomddot,
                                  stateObservation::Vector & realCom,
                                  stateObservation::Vector & zmp,
                                  stateObservation::Vector& output)
{
  if (inState.size() != 18)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
                                        "state signal size is ",
                                        "%d, should be 18.",
                                        inState.size());

  double dt = inTimeStep;

  recomputeMatrices();

  const stateObservation::Vector & xn = convertVector<stateObservation::Vector>(inState);
  const stateObservation::Vector & un = convertVector<stateObservation::Vector>(inControl);
  const stateObservation::Vector & xn1 = A_*xn + B_*un;

  //const dynamicgraph::Vector& k2 = A_*(xn + k1*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k3 = A_*(xn + k2*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k4 = A_*(xn + k3*dt) + B_*inControl;
  //dynamicgraph::Vector dx = (k1 + k2*2 + k3*2 + k4)*(dt/6.);//Runge Kutta 4

  stateObservation::Vector dx = xn1* dt;
  stateObservation::Vector nextState = xn + dx;

  return nextState;
}

void LinearizedRotationalTableCartDevice::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();
  stateObservation::Vector output;
  stateObservation::Vector realCom;
  stateObservation::Vector flexcomddot;
  stateObservation::Vector cl;
  stateObservation::Vector zmp;

  stateObservation::Vector nextState = computeDynamics (stateSOUT_ (t), controlSIN_ (t),
                                      forceSIN_ (t), inTimeStep, flexcomddot, realCom, zmp,
                                      output);

  cl.resize(3);
  cl=cl_;
  stateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextState));
  stateSOUT_.setTime (t+1);
  comrealposSOUT_.setConstant(convertVector<dynamicgraph::Vector>(realCom));
  comrealposSOUT_.setTime (t+1);
  flexcomddotSOUT_.setConstant(convertVector<dynamicgraph::Vector>(flexcomddot));
  flexcomddotSOUT_.setTime (t+1);
  outputSOUT_.setConstant (convertVector<dynamicgraph::Vector>(output));
  outputSOUT_.setTime (t+1);
  clSOUT_.setConstant(convertVector<dynamicgraph::Vector>(cl));
  clSOUT_.setTime(t+1);
  zmpSOUT_.setConstant(convertVector<dynamicgraph::Vector>(zmp));
  zmpSOUT_.setTime(t+1);
  forceSIN_.setTime (t+1);
}

}
