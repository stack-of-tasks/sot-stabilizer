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
    comdotSIN_ (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(vector)::comdot"),
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
    controlGainSIN_
    (NULL, "HRP2LQRDecoupledStabilizer("+inName+")::input(double)::controlGain"),
    d2comSOUT_ ("HRP2LQRDecoupledStabilizer("+inName+")::output(vector)::d2com"),
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
    flexZobs_ (2), flexLatControl_ (1), flexLatObs_ (2),
    timeBeforeFlyingFootCorrection_ (.1),
    iterationsSinceLastSupportLf_ (0), iterationsSinceLastSupportRf_ (0),
    supportCandidateLf_ (0), supportCandidateRf_ (0),
    uth_ (),fixedGains_(false),
    translation_ (3), zmp_ (3),
    theta1Ref_ (0), theta1RefPrev_ (0), dtheta1Ref_ (0),
    debug_(10),
    controller1_(4,1),controller2_(4,1),controllerLat_(4,1),
    A1_(stateObservation::Matrix::Zero(4,4)),
    A2_(stateObservation::Matrix::Zero(4,4)),
    ALat_(stateObservation::Matrix::Zero(4,4)),
    B_(stateObservation::Matrix::Zero(4,1)),
    Q_(stateObservation::Matrix::Zero(4,4)),
    R_(stateObservation::Matrix::Zero(1,1))
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

    taskSOUT.setFunction (boost::bind(&HRP2LQRDecoupledStabilizer::computeControlFeedback,
                                      this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2LQRDecoupledStabilizer::computeJacobianCom,
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
        "    Set the cost Matrix for state\n"
        "\n"
        "      input:\n"
        "        Matrix\n"
        "\n";
    addCommand("setStateCost",
               new dynamicgraph::command::Setter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::setStateCost, docstring));


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
        "    Get the cost Matrix for state\n"
        "\n"
        "      output:\n"
        "        Matrix\n"
        "\n";
    addCommand("getStateCost",
               new dynamicgraph::command::Getter<HRP2LQRDecoupledStabilizer, Matrix>
               (*this, &HRP2LQRDecoupledStabilizer::getStateCost, docstring));

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


    kth_ = 1000;
    kdth_ = 65;
    kz_= 53200;//150000;

    B_(2,0)=0;
    B_(3,0)=1/constcomHeight_;
    Q_(0,0)=Q_(2,2)=10;
    Q_(1,1)=Q_(3,3)=3;
    R_(0,0)=1;

    debug_.setZero();

    zmp_.setZero ();

    int horizon = 200;

    controller1_.setHorizonLength(horizon);
    controller2_.setHorizonLength(horizon);
    controllerLat_.setHorizonLength(horizon);


    A1_=computeDynamicsMatrix(constcomHeight_, kth_, kdth_, constm_);
    controller1_.setDynamicsMatrices
    (stateObservation::Matrix::Identity(4,4) + dt_* A1_, dt_*B_);

    A2_=computeDynamicsMatrix(constcomHeight_, 2 * kth_, 2*kdth_, constm_);
    ALat_=computeDynamicsMatrix(constcomHeight_, 2*kth_ + 2*kz_*conststepLength_/2, 2*kdth_, constm_);

    controller2_.setDynamicsMatrices
    (stateObservation::Matrix::Identity(4,4)+ dt_*A2_, dt_*B_);

    controllerLat_.setDynamicsMatrices
    (stateObservation::Matrix::Identity(4,4)+ dt_*ALat_,B_);
    controllerLat_.setCostMatrices(Q_,R_);
    controller2_.setCostMatrices(Q_,R_);
    controller1_.setCostMatrices(Q_,R_);

    hrp2Mass_ = 58;
}

/// Compute the control law
VectorMultiBound&
HRP2LQRDecoupledStabilizer::computeControlFeedback(VectorMultiBound& comdot,
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

    double dx = dcom_(0) - comdotRef (0);
    double dy = dcom_(1) - comdotRef (1);
    double dz = dcom_(2) - comdotRef (2);

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
        x = x - com(2)*flexibility (1);
        y = y + com(2)*flexibility (0);

        dx = dx - com(2)*flexDot (1);
        dy = dy + com(2)*flexDot (0);

        if ((!fixedGains_))
        {


            A1_=computeDynamicsMatrix(com(2), kth_, kdth_, constm_);
            B_(3,0)=1/com(2);
            controller1_.setDynamicsMatrices
            (stateObservation::Matrix::Identity(4,4) + dt_* A1_, dt_*B_);
        }

        //along x
        theta0 = flexibility (1);
        dtheta0 = flexDot (1);

        stateObservation::Vector xVector (4);
        xVector[0]=x;
        xVector[1]=theta0;
        xVector[2]=dx;
        xVector[3]=dtheta0;

        std::cout << "Simple Support x" << std::endl;
        controller1_.setState(xVector,time);
        d2com_ (0)= controller1_.getControl(time)[0];

        // along y
        theta1 = -flexibility (0);
        dtheta1 = -flexDot (0);
        xVector[0]=y;
        xVector[1]=theta1;
        xVector[2]=dy;
        xVector[3]=dtheta1;

        std::cout << "Simple Support y" << std::endl;
        controller1_.setState(xVector,time);
        d2com_ (1)= controller1_.getControl(time)[0];


        debug_(0)=x;
        debug_(1)=theta0;
        debug_(2)=dx;
        debug_(3)=dtheta0;
        debug_(4)=d2com_(0);

        debug_(5)=y;
        debug_(6)=theta1;
        debug_(7)=dy;
        debug_(8)=dtheta1;
        debug_(9)=d2com_(1);

        dcom_ (0) += dt_ * d2com_ (0);
        dcom_ (1) += dt_ * d2com_ (1);
        // along z
        dcom_ (2) = -gain * z + comdotRef(2);

    }
    break;
    default: //double support or more
    {
        x = x - com(2)*flexibility (1);
        y = y + com(2)*flexibility (0);

        dx = dx - com(2)*flexDot (1);
        dy = dy + com(2)*flexDot (0);

        // compute component of angle orthogonal to the line joining the feet
        double delta_x = lfpos (0) - rfpos (0);
        double delta_y = lfpos (1) - rfpos (1);
        double stepLength = sqrt (delta_x*delta_x+delta_y*delta_y);

        if ((!fixedGains_))
        {
            A2_=computeDynamicsMatrix(com(2), 2 * kth_, 2*kdth_, constm_);
            ALat_=computeDynamicsMatrix(com(2), 2*kth_ + 2*kz_*stepLength/2, 2*kdth_, constm_);

            B_(3,0)=1/com(2);



            controller2_.setDynamicsMatrices
            (stateObservation::Matrix::Identity(4,4)+ dt_*A2_, dt_*B_);



            controllerLat_.setDynamicsMatrices
            (stateObservation::Matrix::Identity(4,4)+ dt_*ALat_,dt_*B_);

        }

        u2x_ = delta_x/stepLength;
        u2y_ = delta_y/stepLength;
        u1x_ = u2y_;
        u1y_ = -u2x_;

        //along the orthogonal to the contacts line
        theta0 = + u1x_ * flexibility (1) - u1y_ * flexibility (0);
        dtheta0 = + u1x_ * flexDot (1) - u1y_ * flexDot (0);
        xi = u1x_*x + u1y_*y;
        dxi = u1x_*dx + u1y_*dy;


        stateObservation::Vector xVector (4);
        xVector[0]=xi;
        xVector[1]=theta0;
        xVector[2]=dxi;
        xVector[3]=dtheta0;

        std::cout << std::endl << "Double Support Frontal ===" << std::endl;
        controller2_.setState(xVector,time);
        ddxi= controller2_.getControl(time)[0];


        //along the contacts line
        theta1 = + u2x_ * flexibility (1) - u2y_ * flexibility (0);
        dtheta1 = + u2x_ * flexDot (1) - u2y_ * flexDot (0);
        lat = u2x_*x + u2y_*y;
        dlat = u2x_*dx + u2y_*dy;

        xVector[0]=lat;
        xVector[1]=theta1;
        xVector[2]=dlat;
        xVector[3]=dtheta1;

        std::cout << std::endl << "Double Support Lateral ===" << std::endl;
        controllerLat_.setState(xVector,time);
        ddlat= controllerLat_.getControl(time)[0];


        flexLatControl_ (0) = ddlat;

        d2com_ (0) = ddxi * u1x_ + ddlat*u2x_;
        d2com_ (1) = ddxi * u1y_ + ddlat*u2y_;
        dcom_ (0) += dt_ * d2com_ (0);
        dcom_ (1) += dt_ * d2com_ (1);

        // along z
        dcom_ (2) = -gain * z + comdotRef(2);

        debug_(0)=xi;
        debug_(1)=theta0;
        debug_(2)=dxi;
        debug_(3)=dtheta0;
        debug_(4)=ddxi;

        debug_(5)=lat;
        debug_(6)=theta1;
        debug_(7)=dlat;
        debug_(8)=dtheta1;
        debug_(9)=ddlat;
    }
    break;
    };

    comdot.resize (3);
    comdot [0].setSingleBound (dcom_ (0));
    comdot [1].setSingleBound (dcom_ (1));
    comdot [2].setSingleBound (dcom_ (2));

    d2comSOUT_.setConstant (d2com_);
    d2comSOUT_.setTime (time);

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

    stateObservation::Matrix A(stateObservation::Matrix::Zero(4,4));

    A(0,2)=1;
    A(1,3)=1;
    A(3,0)=-g/stateObservation::tools::square(comHeight);
    A(3,1)= (constm_*g*comHeight-kth)/(constm_*stateObservation::tools::square(comHeight));
    A(3,3)= -kdth /(constm_*stateObservation::tools::square(comHeight));

    A(2,0) =-comHeight*A(3,0);
    A(2,1) =-comHeight*A(3,1);
    A(2,3) =-comHeight*A(3,3);


    return A;
}

} // namespace sotStabilizer

