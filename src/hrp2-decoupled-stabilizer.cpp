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

#include <sot-stabilizer/hrp2-decoupled-stabilizer.hh>

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

double HRP2DecoupledStabilizer::m_ = 59.8;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HRP2DecoupledStabilizer, "HRP2DecoupledStabilizer");

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
HRP2DecoupledStabilizer::HRP2DecoupledStabilizer(const std::string& inName) :
    TaskAbstract(inName),
    comSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::com"),
    comRefSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::comRef"),
    jacobianSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(matrix)::Jcom"),
    comdotSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::comdot"),
    leftFootPositionSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(HomoMatrix)::leftFootPosition"),
    rightFootPositionSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(HomoMatrix)::rightFootPosition"),
    forceLeftFootSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::force_lf"),
    forceRightFootSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::force_rf"),
    stateFlexSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(HomoMatrix)::stateFlex"),
    stateFlexDotSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::stateFlexDot"),
    controlGainSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(double)::controlGain"),
    d2comSOUT_ ("HRP2DecoupledStabilizer("+inName+")::output(vector)::d2com"),
    nbSupportSOUT_
    ("HRP2DecoupledStabilizer("+inName+")::output(unsigned)::nbSupport"),
    errorSOUT_ ("HRP2DecoupledStabilizer("+inName+")::output(vector)::error"),
    debugSOUT_ ("HRP2DecoupledStabilizer("+inName+")::output(vector)::debug"),
    supportPos1SOUT_("HRP2DecoupledStabilizer("+inName+")::output(vector)::supportPos1"),
    supportPos2SOUT_("HRP2DecoupledStabilizer("+inName+")::output(vector)::supportPos2"),
    gain1_ (4), gain2_ (4), gainz_ (4), gainLat_ (4),
    poles1_ (4),poles2_ (4),polesLat_ (4),
    prevCom_(3), dcom_ (3), dt_ (.005), on_ (false),
    forceThreshold_ (.036 * m_*stateObservation::cst::gravityConstant),
    angularStiffness_ (425.), d2com_ (3),
    deltaCom_ (3),
    flexPosition_ (), flexPositionLf_ (), flexPositionRf_ (),
    flexPositionLat_ (),
    flexVelocity_ (6), flexVelocityLf_ (6), flexVelocityRf_ (6),
    flexVelocityLat_ (6),
    flexZobs_ (2), flexLatControl_ (1), flexLatObs_ (2),
    timeBeforeFlyingFootCorrection_ (.1),
    iterationsSinceLastSupportLf_ (0), iterationsSinceLastSupportRf_ (0),
    supportCandidateLf_ (0), supportCandidateRf_ (0),
    uth_ (),
    R_ (), translation_ (3), zmp_ (3),
    theta1Ref_ (0), theta1RefPrev_ (0), dtheta1Ref_ (0),
    debug_(10)
{
    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (comRefSIN_);
    signalRegistration (jacobianSIN_);
    signalRegistration (comdotSIN_);
    signalRegistration (leftFootPositionSIN_ << rightFootPositionSIN_
                        << forceRightFootSIN_ << forceLeftFootSIN_);
    signalRegistration (stateFlexSIN_ << stateFlexDotSIN_);
    signalRegistration (controlGainSIN_);
    signalRegistration (d2comSOUT_);
    signalRegistration (nbSupportSOUT_ << supportPos1SOUT_ << supportPos2SOUT_);
    signalRegistration (errorSOUT_);
    signalRegistration (debugSOUT_);


    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (comRefSIN_);
    taskSOUT.addDependency (comdotSIN_);
    taskSOUT.addDependency (leftFootPositionSIN_);
    taskSOUT.addDependency (rightFootPositionSIN_);
    taskSOUT.addDependency (forceRightFootSIN_);
    taskSOUT.addDependency (forceLeftFootSIN_);
    taskSOUT.addDependency (stateFlexSIN_);
    taskSOUT.addDependency (stateFlexDotSIN_);
    taskSOUT.addDependency (controlGainSIN_);

    jacobianSOUT.addDependency (jacobianSIN_);

    taskSOUT.setFunction (boost::bind(&HRP2DecoupledStabilizer::computeControlFeedback,
                                      this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2DecoupledStabilizer::computeJacobianCom,
                                          this,_1,_2));
    nbSupportSOUT_.addDependency (taskSOUT);

    d2com_.setZero ();
    dcom_.setZero ();
    deltaCom_.setZero ();
    d2comSOUT_.setConstant (d2com_);
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
               new dynamicgraph::command::Setter<HRP2DecoupledStabilizer, double>
               (*this, &HRP2DecoupledStabilizer::setTimePeriod, docstring));
    docstring =
        "\n"
        "    Get sampling time period task\n"
        "\n"
        "      return:\n"
        "        a floating point number\n"
        "\n";
    addCommand("getTimePeriod",
               new dynamicgraph::command::Getter<HRP2DecoupledStabilizer, double>
               (*this, &HRP2DecoupledStabilizer::getTimePeriod, docstring));

    addCommand ("start",
                makeCommandVoid0 (*this, &HRP2DecoupledStabilizer::start,
                                  docCommandVoid0 ("Start stabilizer")));

    addCommand ("stop",
                makeCommandVoid0 (*this, &HRP2DecoupledStabilizer::stop,
                                  docCommandVoid0 ("Stop stabilizer")));

    addCommand ("setPoles1",
                makeDirectSetter (*this, &poles1_,
                                  docDirectSetter
                                  ("Set poles single support",
                                   "vector")));
    addCommand ("getPoles1",
                makeDirectGetter (*this, &poles1_,
                                  docDirectGetter
                                  ("Get poles single support",
                                   "vector")));

    addCommand ("getGain1",
                makeDirectGetter (*this, &gain1_,
                                  docDirectGetter
                                  ("Get gains single support",
                                   "vector")));

    addCommand ("setPoles2",
                makeDirectSetter (*this, &poles2_,
                                  docDirectSetter
                                  ("Set poles double support",
                                   "vector")));

    addCommand ("getPoles2",
                makeDirectGetter (*this, &poles2_,
                                  docDirectGetter
                                  ("Get poles double support",
                                   "vector")));

    addCommand ("getGain2",
                makeDirectGetter (*this, &gain2_,
                                  docDirectGetter
                                  ("Get gains double support",
                                   "vector")));

    addCommand ("setPolesLateral",
                makeDirectSetter (*this, &polesLat_,
                                  docDirectSetter
                                  ("Set poles of lateral flexibility",
                                   "vector")));

    addCommand ("getPolesLateral",
                makeDirectGetter (*this, &polesLat_,
                                  docDirectGetter
                                  ("Get poles of lateral flexibility",
                                   "vector")));

    addCommand ("getGainLateral",
                makeDirectGetter (*this, &gainLat_,
                                  docDirectGetter
                                  ("Get gains of lateral flexibility",
                                   "vector")));

    addCommand ("setKth",
                makeDirectSetter (*this, &kth_,
                                  docDirectSetter
                                  ("Set angular elasticity","float")));

    prevCom_.fill (0.);

    // Single support gains for
    //  - kth = 510,
    //  - zeta = .8
    //  - m = 56
    //  - eigen values = 3, 3, 6, 6.
    gain1_ (0) = 121.27893435294121;
    gain1_ (1) = -19.899754625210093;
    gain1_ (2) = 38.133760000000009;
    gain1_ (3) = -17.707008000000005;

    // Double support gains for
    //  - kth = 2*510,
    //  - zeta = .8
    //  - m = 56
    //  - eigen values = 4, 4, 8, 8.
    gain2_ (0) = 118.62268196078432;
    gain2_ (1) = 58.543997288515406;
    gain2_ (2) = 37.326305882352941;
    gain2_ (3) = -10.661044705882359;

    // Lateral gains for
    //  - kth = 3000 (kz = 160000, h=.19)
    //  - m = 56
    //  - eigen values = 8, 8, 8, 8.
    gainLat_ (0) = 94.721917866666672;
    gainLat_ (1) = 174.26817999238096;
    gainLat_ (2) = 29.154645333333335;
    gainLat_ (3) = 2.2762837333333308;


    //computed by hand on simulation
    //gainLat_ (0) = 100.62268196078432;
    //gainLat_ (1) = -200.543997288515406;
    //gainLat_ (2) = 37.326305882352941;
    //gainLat_ (3) = -0.661044705882359;

    kth_ = 510;
    kz_= 150000;

    poles1_(0) = -3 ;
    poles1_(1) = -3 ;
    poles1_(2) = -6 ;
    poles1_(3) = -6 ;

    poles2_(0) = -4 ;
    poles2_(1) = -4 ;
    poles2_(2) = -8 ;
    poles2_(3) = -8 ;

    polesLat_(0) = -8 ;
    polesLat_(1) = -8 ;
    polesLat_(2) = -8 ;
    polesLat_(3) = -8 ;

    debug_.setZero();

    zmp_.setZero ();

    hrp2Mass_ = 58;
}

/// Compute the control law
VectorMultiBound&
HRP2DecoupledStabilizer::computeControlFeedback(VectorMultiBound& comdot,
                                   const int& time)
{
    const Vector & com = comSIN_ (time);
    const Vector deltaCom = comSIN_ (time) - comRefSIN_ (time);
    const Vector& comdotRef = comdotSIN_ (time);
    const MatrixHomogeneous& flexibilityMatrix = stateFlexSIN_.access(time);
    const Vector& flexDot = stateFlexDotSIN_.access(time);
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


    deltaCom_ = comSIN_ (time) - comRefSIN_ (time);
    //(flexibilityRot*comRefSIN_ (time)+flexibilityPos);

    double x = deltaCom_ (0);
    double y = deltaCom_ (1);
    double z = deltaCom_ (2);



    // z-component of center of mass deviation in global frame
    flexZobs_ (0) = deltaCom (2);
    flexLatObs_ (0) = 0;

    double theta0, dtheta0;
    double theta1, dtheta1, ddxi;
    double xi, dxi, lat, dlat, ddlat;
    //double thetaz;
    //double dthetaz;
    double fzRef, Zrefx, Zrefy, fz, Zx, Zy;

    flexLatControl_ (0) = 0.;

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
        dcom_ (0) = -gain * x;
        dcom_ (1) = -gain * y;
        dcom_ (2) = -gain * z;

        debug_.setZero();
        break;
    case 1: //single support
    {
            gain1_ = computeGainsFromPoles(poles1_, com(2), kth_, hrp2Mass_);

        //along x
        theta0 = flexibility (1);
        dtheta0 = flexDot (1);
        d2com_ (0)= -(gain1_ (0)*x + gain1_ (1)*theta0 +
                      gain1_ (2)*dcom_ (0) + gain1_ (3)*dtheta0);
        dcom_ (0) += dt_ * d2com_ (0);

        // along y
        theta1 = flexibility (0);
        dtheta1 = flexDot (0);
        d2com_ (1) = - (gain1_ (0)*y + gain1_ (1)*theta1 +
                        gain1_ (2)*dcom_ (1) + gain1_ (3)*dtheta1);
        dcom_ (1) += dt_ * d2com_ (1);

        // along z
        dcom_ (2) = -gain * z;

        //d2com_ (2) = - (gainz_ (0)*z + gainz_ (1)*thetaz +
        //		gainz_ (2)*dcom_ (2) + gainz_ (3)*dthetaz);
        //dcom_ (2) += dt_ * d2com_ (2);

        debug_(0)=theta0;
        debug_(1)=theta1;
        debug_(2)=d2com_(0);
        debug_(3)=d2com_(1);
        debug_(4)=dcom_(0);
        debug_(5)=dcom_(0);
    }
        break;
    default: //double support or more
    {


        // compute component of angle orthogonal to the line joining the feet
        double delta_x = lfpos (0) - rfpos (0);
        double delta_y = lfpos (1) - rfpos (1);
        double stepLength = sqrt (delta_x*delta_x+delta_y*delta_y);

        gain2_ = computeGainsFromPoles(poles2_, com(2), 2 * kth_, hrp2Mass_);
        gainLat_ = computeGainsFromPoles(polesLat_, com(2), 2*kth_ + kz_*stepLength, hrp2Mass_);

        u2x_ = delta_x/stepLength;
        u2y_ = delta_y/stepLength;
        u1x_ = u2y_;
        u1y_ = -u2x_;

        //along the orthogonal to the contacts line
        theta0 = u1x_ * flexibility (1) + u1y_ * flexibility (0);
        dtheta0 = u1x_ * flexDot (1) + u1y_ * flexDot (0);
        xi = u1x_*x + u1y_*y;
        dxi = u1x_*dcom_ (0) + u1y_*dcom_ (1);
        ddxi = - (gain2_ (0)*xi + gain2_ (1)*theta0 + gain2_ (2)*dxi +
                  gain2_ (3)*dtheta0);



        //along the contacts line
        theta1 = u2x_ * flexibility (1) + u2y_ * flexibility (0);
        dtheta1 = u2x_ * flexDot (1) + u2y_ * flexDot (0);
        lat = u2x_*x + u2y_*y;
        dlat = u2x_*dcom_ (0) + u2y_*dcom_ (1);
        ddlat = - (gainLat_ (0)*lat + gainLat_ (1)*(theta1)
                   + gainLat_ (2)*dlat + gainLat_ (3)*(dtheta1));
        flexLatControl_ (0) = ddlat;

        d2com_ (0) = ddxi * u1x_ + ddlat*u2x_;
        d2com_ (1) = ddxi * u1y_ + ddlat*u2y_;
        dcom_ (0) += dt_ * d2com_ (0);
        dcom_ (1) += dt_ * d2com_ (1);

        // along z
        dcom_ (2) = -gain * z;
        //d2com_ (2) = - (gainz_ (0)*z + gainz_ (1)*thetaz +
        //		gainz_ (2)*dcom_ (2) + gainz_ (3)*dthetaz);
        //dcom_ (2) += dt_ * d2com_ (2);

        debug_(0)=theta0;
        debug_(1)=theta1;
        debug_(2)=ddxi;
        debug_(3)=ddlat;
        debug_(4)=dxi;
        debug_(5)=dlat;
    }
    break;
    };

    comdot.resize (3);
    comdot [0].setSingleBound (comdotRef (0) + dcom_ (0));
    comdot [1].setSingleBound (comdotRef (1) + dcom_ (1));
    comdot [2].setSingleBound (comdotRef (2) + dcom_ (2));

    d2comSOUT_.setConstant (d2com_);
    d2comSOUT_.setTime (time);

    errorSOUT_.setConstant (deltaCom_);
    errorSOUT_.setTime (time);

    debugSOUT_.setConstant (debug_);
    debugSOUT_.setTime (time);

    return comdot;
}

Matrix& HRP2DecoupledStabilizer::computeJacobianCom(Matrix& jacobian, const int& time)
{
    typedef unsigned int size_t;
    jacobian = jacobianSIN_ (time);
    return jacobian;
}

Vector HRP2DecoupledStabilizer::computeGainsFromPoles
                            (const Vector & pole, double comHeight, double kth, double mass) const
{

    const double & g = stateObservation::cst::gravityConstant;

    //the characteristic polynomial coefficients
    double a0 =   pole(0)*pole(1)*pole(2)*pole(3);
    double a1 = - pole(0)*pole(1)*pole(2) - pole(0)*pole(1)*pole(3)
                - pole(0)*pole(2)*pole(3) - pole(1)*pole(2)*pole(3);
    double a2 =   pole(0)*pole(1) + pole(0)*pole(2) + pole(0)*pole(3)
                + pole(1)*pole(2) + pole(1)*pole(3) + pole(2)*pole(3);
    double a3 = - pole(0) - pole(1) - pole(2) - pole(3);

    double mu = kth / (mass * comHeight) - g;

    Vector gains(4);

    gains(0) = mass*comHeight*(a0*comHeight + a2*g - g/comHeight) / kth;
    gains(1) = mass*comHeight*(a2*mu*comHeight - mu*mu - a0*comHeight*comHeight) / kth;
    gains(2) = mass*comHeight*(a3*g + a1*comHeight) / kth;
    gains(3) = mass*comHeight*comHeight*(a3*mu - a1*comHeight) / kth;

    return gains;
}


} // namespace sotStabilizer

