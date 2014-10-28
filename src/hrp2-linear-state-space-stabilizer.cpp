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

#include <sot-stabilizer/hrp2-linear-state-space-stabilizer.hh>

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

  double HRP2LinearStateSpaceStabilizer::constm_ = 59.8;

  const unsigned stateSize_=6;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HRP2LinearStateSpaceStabilizer, "HRP2LinearStateSpaceStabilizer");

/// Dynamic balance stabilizer

  HRP2LinearStateSpaceStabilizer::HRP2LinearStateSpaceStabilizer(const std::string& inName) :
    TaskAbstract(inName),
    comSIN_ (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(vector)::com"),
    comRefSIN_ (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(vector)::comRef"),
    jacobianSIN_ (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(matrix)::Jcom"),
    controlGainSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(double)::controlGain"),
    stateFlexSIN_
    (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(HomoMatrix)::stateFlex"),
    stateFlexDotSIN_
    (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(HomoMatrix)::stateFlexDot"),
    leftFootPositionSIN_
    (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(HomoMatrix)::leftFootPosition"),
    rightFootPositionSIN_
    (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(HomoMatrix)::rightFootPosition"),
    forceLeftFootSIN_
    (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(vector)::force_lf"),
    forceRightFootSIN_
    (NULL, "HRP2LinearStateSpaceStabilizer("+inName+")::input(vector)::force_rf"),
    supportPos1SOUT_("HRP2LinearStateSpaceStabilizer("+inName+")::output(vector)::supportPos1"),
    supportPos2SOUT_("HRP2LinearStateSpaceStabilizer("+inName+")::output(vector)::supportPos2"),
    nbSupportSOUT_
    ("HRP2LinearStateSpaceStabilizer("+inName+")::output(unsigned)::nbSupport"),
    errorSOUT_ ("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::error"),
    deltaCom_ (3), dcom_ (3), d2com_ (3), dt_ (.005), on_ (false),
    forceThreshold_ (.036 * constm_*stateObservation::cst::gravityConstant),
    supportCandidateLf_ (0), supportCandidateRf_ (0),
    controller0_(stateSize_,1),controller1_(stateSize_,1)
  {
    /// Register signals into the entity.
    signalRegistration (comSIN_); // position of the com in the loàcal frame
    signalRegistration (comRefSIN_); // reference position of the com in the global frame
    signalRegistration (controlGainSIN_);
    signalRegistration (jacobianSIN_); // Jacobian of the com
    signalRegistration (stateFlexSIN_); // state of the flexibility from the estimator
    signalRegistration (stateFlexDotSIN_); // derivative of the state of the flexibility from the estimator

    signalRegistration (nbSupportSOUT_ << supportPos1SOUT_ << supportPos2SOUT_);
    signalRegistration (leftFootPositionSIN_ << rightFootPositionSIN_ << forceRightFootSIN_ << forceLeftFootSIN_);
    signalRegistration (nbSupportSOUT_ << supportPos1SOUT_ << supportPos2SOUT_);
    signalRegistration (errorSOUT_);


    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (comRefSIN_);
    taskSOUT.addDependency (stateFlexSIN_);
    taskSOUT.addDependency (leftFootPositionSIN_);
    taskSOUT.addDependency (rightFootPositionSIN_);
    taskSOUT.addDependency (forceRightFootSIN_);
    taskSOUT.addDependency (forceLeftFootSIN_);
    jacobianSOUT.addDependency (jacobianSIN_);
    nbSupportSOUT_.addDependency (taskSOUT);


    taskSOUT.setFunction (boost::bind(&HRP2LinearStateSpaceStabilizer::computeControlFeedback,
                                      this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2LinearStateSpaceStabilizer::computeJacobianCom,
                                          this,_1,_2));

    deltaCom_.setZero ();

    std::string docstring;
    docstring =
      "\n"
      "    Set sampling time period task\n"
      "\n"
      "      input:\n"
      "        a floating point number\n"
      "\n";
    addCommand("setTimePeriod",
               new dynamicgraph::command::Setter<HRP2LinearStateSpaceStabilizer, double>
               (*this, &HRP2LinearStateSpaceStabilizer::setTimePeriod, docstring));


    docstring =
      "\n"
      "    Get sampling time period task\n"
      "\n"
      "      return:\n"
      "        a floating point number\n"
      "\n";
    addCommand("getTimePeriod",
               new dynamicgraph::command::Getter<HRP2LinearStateSpaceStabilizer, double>
               (*this, &HRP2LinearStateSpaceStabilizer::getTimePeriod, docstring));

    addCommand ("start",
                makeCommandVoid0 (*this, &HRP2LinearStateSpaceStabilizer::start,
                                  docCommandVoid0 ("Start stabilizer")));

    addCommand ("stop",
                makeCommandVoid0 (*this, &HRP2LinearStateSpaceStabilizer::stop,
                                  docCommandVoid0 ("Stop stabilizer")));

  }

  /// Determine number of support : to be put in stateObervation in my opinion
  unsigned int HRP2LinearStateSpaceStabilizer::computeNbSupport(const int& time)//const MatrixHomogeneous& leftFootPosition, const MatrixHomogeneous& rightFootPosition, const Vector& forceLf, const Vector& forceRf, const int& time)
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
  HRP2LinearStateSpaceStabilizer::computeControlFeedback(VectorMultiBound& comdot,
      const int& time)
  {
    // Control gain
    const double& gain = controlGainSIN_.access (time);

    // From global frame to local frame
    const MatrixHomogeneous& flexibilityMatrix = stateFlexSIN_.access(time);
    const Vector& flexDot = stateFlexDotSIN_.access(time);
    const Vector & comrefg = comRefSIN_ (time);
    const Vector & com = comSIN_ (time);

    Vector flexibilityPos(3);
    VectorUTheta flexibility;
    MatrixRotation flexibilityRot;
    Vector comref;
    stateObservation::Vector3 theta, dtheta;

    flexibilityMatrix.extract(flexibilityRot);
    flexibility.fromMatrix(flexibilityRot);
    flexibilityMatrix.extract(flexibilityPos);    
    theta.setZero();
    dtheta.setZero();

    // Construction of the desired com in the local frame
    comref=flexibilityMatrix.inverse()*comrefg;

    // Error of the position of the com in the local frame
    deltaCom_=comref-com;
    Vector ddeltaCom=dcom_-comdotRef_;

    // Determination of the number of support
    nbSupport_=computeNbSupport(time);

    // Compute acceleration of the state
    if(nbSupport_ < 3 && nbSupport_ !=0){ // rotation around y (double and simple support)
        theta[0]= -flexibility(1);
        dtheta[0] = -flexDot (1);

        stateObservation::Vector x0Vector (stateSize_);
        x0Vector[0]=deltaCom_(0);
        x0Vector[1]=theta[0];
        x0Vector[2]=ddeltaCom(0);
        x0Vector[3]=dtheta[0];

        controller0_.setState(x0Vector,time);
        d2com_ (0)= controller0_.getControl(time)[0];
        dcom_ (0) = dt_ * d2com_ (0); // + conditions initiales

        // Along z
        dcom_ (2) = -gain * deltaCom_(3);
    }

    if(nbSupport_ < 2 && nbSupport_ !=0){ // rotation around x (simple support only)
        theta[1]= -flexibility(1);
        dtheta[1] = -flexDot (1);

        stateObservation::Vector x1Vector (stateSize_);
        x1Vector[0]=deltaCom_(1);
        x1Vector[1]=theta[0];
        x1Vector[2]=ddeltaCom(1);
        x1Vector[3]=dtheta[0];

        controller1_.setState(x1Vector,time);
        d2com_ (1)= controller1_.getControl(time)[0];
        dcom_ (1) = dt_ * d2com_ (1); // + conditions initiales
    }

    if(nbSupport_ == 0){
        dcom_ (0) = -gain * deltaCom_(0);
        dcom_ (1) = -gain * deltaCom_(1);
        dcom_ (2) = -gain * deltaCom_(2);
    }

    // error of the position of the Com
    errorSOUT_.setConstant (deltaCom_);
    errorSOUT_.setTime (time);

    // Value returned to the sot that wil make it converge to zero
    comdot.resize (3);
    comdot [0].setSingleBound (dcom_ (0));
    comdot [1].setSingleBound (dcom_ (1));
    comdot [2].setSingleBound (dcom_ (2));
    return comdot;
  }

  Matrix& HRP2LinearStateSpaceStabilizer::computeJacobianCom(Matrix& jacobian, const int& time)
  {
    typedef unsigned int size_t;
    jacobian = jacobianSIN_ (time);
    return jacobian;
  }

} // namespace sotStabilizer


