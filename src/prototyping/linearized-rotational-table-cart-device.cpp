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
//  dynamicgraph::Vector height (1); height(0)=cartHeight_;
//  comHeightSOUT_ = height;


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
  stateObservation::Matrix Kdth = viscosity_;
  stateObservation::Matrix I = I_;
  stateObservation::Vector cl = cl_;

  A_ (0, 10) = 1.;
  A_ (1, 11) = 1.;
  A_ (2, 12) = 1.;

  A_ (3, 13) = 1.;
  A_ (4, 14) = 1.;
  A_ (5, 15) = 1.;

  A_ (6, 16) = 1.;
  A_ (7, 17) = 1.;
  A_ (8, 18) = 1.;

  B_(10,0) = 1.;
  B_(11,1) = 1.;
  B_(12,2) = 1.;

  B_(13,3) = 1.;
  B_(14,4) = 1.;
  B_(15,5) = 1.;


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
//  if (inState.size() != 18)
//    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
//                                        "state signal size is ",
//                                        "%d, should be 18.",
//                                        inState.size());

  double dt = inTimeStep;
  double g = stateObservation::cst::gravityConstant;
  double m = cartMass_;
  stateObservation::Matrix Kth = stiffness_;
  stateObservation::Matrix Kdth = viscosity_;
  stateObservation::Matrix I = I_;

  stateObservation::Vector cl=cl_;

//  A_ (0, 2) = 1.;
//  A_ (1, 3) = 1.;
//  A_ (3, 0) = - m*g/(m*zeta*zeta + Iyy);
//  A_ (3, 1) = (m*g*zeta - kth)/(m*zeta*zeta + Iyy);
//  A_ (3, 3) = -kdth/(m*zeta*zeta + Iyy);
//  B_ (2, 0) = 1.; B_ (3, 0) = m*zeta/(m*zeta*zeta + Iyy);
//
//  double xi = inState (0);
//  double th = inState (1);
//  double dth = inState (3);
//
//  const dynamicgraph::Vector& xn = inState;
//  const dynamicgraph::Vector& k1 = A_*xn + B_*inControl;
//
//  //const dynamicgraph::Vector& k2 = A_*(xn + k1*(dt/2)) + B_*inControl;
//  //const dynamicgraph::Vector& k3 = A_*(xn + k2*(dt/2)) + B_*inControl;
//  //const dynamicgraph::Vector& k4 = A_*(xn + k3*dt) + B_*inControl;
//  //dynamicgraph::Vector dx = (k1 + k2*2 + k3*2 + k4)*(dt/6.);//Runge Kutta 4
//
//  dynamicgraph::Vector dx = k1* dt;
//
  stateObservation::Vector nextState;// = xn + dx;
//
//  double testconst = - g * xn(0) + zeta * inControl(0);
//
//  th = nextState (1);
//  dth = nextState (3);
//
//  double My = kth*th + kdth*dth;
//
//  realCom.resize(1);
//  realCom(0) = nextState(0)-nextState(1)*zeta;
//
//  flexcomddot.resize(1);
//  flexcomddot(0) = dx(3)*cartHeight_;
//
//  zmp.resize(1);
//  zmp(0) = -My / (g*m);
//
//  double zmpnoflex = xn(0)-
//                    zeta/stateObservation::cst::gravityConstant*inControl(0);
//
//  output.resize (2);
//  output (0) = xi;
//  output (1) = My;
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
