/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <boost/format.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
//#include <sot-stabilizer/prototyping/linearized-table-cart.hh>
#include <dynamic-graph/command-bind.h>

#include <state-observation/tools/definitions.hpp>

#include "command-increment.hh"


using namespace sotStabilizer;
using dynamicgraph::command::makeDirectSetter;
using dynamicgraph::command::makeDirectGetter;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(LinearizedTableCartDevice, "LinearizedTableCartDevice");

LinearizedTableCartDevice::LinearizedTableCartDevice(const std::string& inName) :
  Entity(inName),
  forceSIN_(NULL, "LinearizedTableCartDevice("+inName+")::input(double)::force"),
  controlSIN_(NULL, "LinearizedTableCartDevice("+inName+")::input(vector)::control"),
  stateSOUT_("LinearizedTableCartDevice("+inName+")::output(vector)::state"),
  outputSOUT_ ("LinearizedTableCartDevice("+inName+")::output(vector)::output"),
  comrealposSOUT_ ("LinearizedTableCartDevice("+inName+")::output(vector)::comreal"),
  flexcomddotSOUT_ ("LinearizedTableCartDevice("+inName+")::output(vector)::flexcomddot"),
  comHeightSOUT_("LinearizedTableCartDevice("+inName+")::output(vector)::comHeight"),
  zmpSOUT_("LinearizedTableCartDevice("+inName+")::output(vector)::zmp"),
  cartMass_(58.0), stiffness_ (100.), viscosity_(0.1), Iyy_ (0.),
  A_ (4, 4), B_ (4, 1)
{
  A_.setZero (); B_.setZero ();
  // Register signals into the entity.
  signalRegistration (forceSIN_);
  signalRegistration (controlSIN_);
  signalRegistration (stateSOUT_);
  signalRegistration (outputSOUT_);
  signalRegistration (comrealposSOUT_);
  signalRegistration (flexcomddotSOUT_);
  signalRegistration (comHeightSOUT_);
  signalRegistration (zmpSOUT_);

  // Set signals as constant to size them
  dynamicgraph::Vector state (4); state.fill (0.);
  stateSOUT_.setConstant(state);
  double force = 0;
  forceSIN_.setConstant(force);
  dynamicgraph::Vector control (1); control.setZero ();
  controlSIN_.setConstant (control);
  dynamicgraph::Vector output (2); output.setZero ();
  outputSOUT_.setConstant (output);
  dynamicgraph::Vector zeroV (1); zeroV.setZero ();
  comrealposSOUT_.setConstant(zeroV);
  flexcomddotSOUT_.setConstant (zeroV);
  zmpSOUT_.setConstant(zeroV);
  dynamicgraph::Vector height (1); height(0)=cartHeight_;
  comHeightSOUT_ = height;


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
	     new command::Increment(*this, docstring));

  // setCartMass
  docstring =
    "\n"
    "    Set cart mass\n"
    "\n";
  addCommand(std::string("setCartMass"),
	     new ::dynamicgraph::command::Setter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::setCartMass, docstring));

  // getCartMass
  docstring =
    "\n"
    "    Get cart mass\n"
    "\n";
  addCommand(std::string("getCartMass"),
	     new ::dynamicgraph::command::Getter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::getCartMass, docstring));
  // setCartHeight
  docstring =
    "\n"
    "    Set cart height\n"
    "\n";
  addCommand(std::string("setCartHeight"),
	     new ::dynamicgraph::command::Setter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::setCartHeight, docstring));

  // getCartHeight
  docstring =
    "\n"
    "    Get cart height\n"
    "\n";
  addCommand(std::string("getCartHeight"),
	     new ::dynamicgraph::command::Getter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::getCartHeight, docstring));
  // setStiffness
  docstring =
    "\n"
    "    Set stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("setStiffness"),
	     new ::dynamicgraph::command::Setter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::setStiffness, docstring));

  // getStiffness
  docstring =
    "\n"
    "    Get cart stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("getStiffness"),
	     new ::dynamicgraph::command::Getter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::getStiffness, docstring));
  // setViscosity
  docstring =
    "\n"
    "    Set viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("setViscosity"),
	     new ::dynamicgraph::command::Setter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::setViscosity, docstring));

  // getViscosity
  docstring =
    "\n"
    "    Get cart viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("getViscosity"),
	     new ::dynamicgraph::command::Getter<LinearizedTableCartDevice, double>
	     (*this, &LinearizedTableCartDevice::getViscosity, docstring));

  // setMomentOfInertia
  docstring =
    "\n"
    "    Set moment of inertia around y axis\n"
    "\n";
  addCommand ("setMomentOfInertia",
	      makeDirectSetter (*this, &Iyy_, docstring));

  // setMomentOfInertia
  docstring =
    "\n"
    "    Get moment of inertia around y axis\n"
    "\n";
  addCommand ("getMomentOfInertia",
	      makeDirectGetter (*this, &Iyy_, docstring));

    // recomputeMatrices
  docstring =
    "\n"
    "    Recomputes matrices A and B\n"
    "\n";
  addCommand ("recomputeMatrices",
	      dynamicgraph::command::makeCommandVoid0
                (*this, &LinearizedTableCartDevice::recomputeMatrices, docstring));



}

void LinearizedTableCartDevice::recomputeMatrices()
{

  double g = stateObservation::cst::gravityConstant;
  double m = cartMass_;
  double kth = stiffness_;
  double kdth = viscosity_;
  double Iyy = Iyy_;
  double zeta = cartHeight_;

  A_ (0, 2) = 1.;
  A_ (1, 3) = 1.;
  A_ (3, 0) = - m*g/(m*zeta*zeta + Iyy);
  A_ (3, 1) = (m*g*zeta - kth)/(m*zeta*zeta + Iyy);
  A_ (3, 3) = -kdth/(m*zeta*zeta + Iyy);
  B_ (2, 0) = 1.; B_ (3, 0) = m*zeta/(m*zeta*zeta + Iyy);
}

LinearizedTableCartDevice::~LinearizedTableCartDevice()
{
}

dynamicgraph::Vector LinearizedTableCartDevice::computeDynamics(
                  const dynamicgraph::Vector& inState,
				  const dynamicgraph::Vector& inControl,
				  const double&,
				  double inTimeStep,
				  dynamicgraph::Vector & flexcomddot,
				  dynamicgraph::Vector & realCom,
				  dynamicgraph::Vector & zmp,
				  dynamicgraph::Vector& output)
{
  if (inState.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC,
					"state signal size is ",
					"%d, should be 4.",
					inState.size());

  double dt = inTimeStep;
  double g = stateObservation::cst::gravityConstant;
  double m = cartMass_;
  double kth = stiffness_;
  double kdth = viscosity_;
  double Iyy = Iyy_;
  double zeta = cartHeight_;

  A_ (0, 2) = 1.;
  A_ (1, 3) = 1.;
  A_ (3, 0) = - m*g/(m*zeta*zeta + Iyy);
  A_ (3, 1) = (m*g*zeta - kth)/(m*zeta*zeta + Iyy);
  A_ (3, 3) = -kdth/(m*zeta*zeta + Iyy);
  B_ (2, 0) = 1.; B_ (3, 0) = m*zeta/(m*zeta*zeta + Iyy);

  double xi = inState (0);
  double th = inState (1);
  double dth = inState (3);

  const dynamicgraph::Vector& xn = inState;
  const dynamicgraph::Vector& k1 = A_*xn + B_*inControl;

  //const dynamicgraph::Vector& k2 = A_*(xn + k1*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k3 = A_*(xn + k2*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k4 = A_*(xn + k3*dt) + B_*inControl;
  //dynamicgraph::Vector dx = (k1 + k2*2 + k3*2 + k4)*(dt/6.);//Runge Kutta 4

  dynamicgraph::Vector dx = k1* dt;

  dynamicgraph::Vector nextState = xn + dx;

  double testconst = - g * xn(0) + zeta * inControl(0);

  th = nextState (1);
  dth = nextState (3);

  double My = kth*th + kdth*dth;

  realCom.resize(1);
  realCom(0) = nextState(0)-nextState(1)*zeta;

  flexcomddot.resize(1);
  flexcomddot(0) = dx(3)*cartHeight_;

  zmp.resize(1);
  zmp(0) = -My / (g*m);

  double zmpnoflex = xn(0)-
                    zeta/stateObservation::cst::gravityConstant*inControl(0);

  output.resize (2);
  output (0) = xi;
  output (1) = My;
  return nextState;
}

void LinearizedTableCartDevice::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();
  dynamicgraph::Vector output;
  dynamicgraph::Vector realCom;
  dynamicgraph::Vector flexcomddot;
  dynamicgraph::Vector comheight;
  dynamicgraph::Vector zmp;

  dynamicgraph::Vector nextState = computeDynamics (stateSOUT_ (t), controlSIN_ (t),
				      forceSIN_ (t), inTimeStep, flexcomddot, realCom, zmp,
				      output);
  comheight.resize(1);
  comheight(0) = cartHeight_;
  stateSOUT_.setConstant (nextState);
  stateSOUT_.setTime (t+1);
  comrealposSOUT_.setConstant(realCom);
  comrealposSOUT_.setTime (t+1);
  flexcomddotSOUT_.setConstant(flexcomddot);
  flexcomddotSOUT_.setTime (t+1);
  outputSOUT_.setConstant (output);
  outputSOUT_.setTime (t+1);
  comHeightSOUT_.setConstant(comheight);
  comHeightSOUT_.setTime(t+1);
  zmpSOUT_.setConstant(zmp);
  zmpSOUT_.setTime(t+1);
  forceSIN_.setTime (t+1);
}
