//
// Copyright (c) 2012,
// Mehdi Benallegue
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

#include <sstream>

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>

#include <dynamic-graph/factory.h>
#include <jrl/mal/matrixabstractlayervector3jrlmath.hh>
#include <sot/core/vector-utheta.hh>

#include <sot/core/flags.hh>

#include <sot-stabilizer/prototyping/decoupled-elastic-inv-pendulum-simulator.hh>

namespace sotStabilizer
{

using dynamicgraph::command::docDirectSetter;
using dynamicgraph::command::makeDirectSetter;
using dynamicgraph::command::docDirectGetter;
using dynamicgraph::command::makeDirectGetter;
using dynamicgraph::command::makeCommandVoid0;
using dynamicgraph::command::docCommandVoid0;
using dynamicgraph::Vector;
using dynamicgraph::Matrix;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DecoupledElasticInvPendulum,  "DecoupledElasticInvPendulum" );

DecoupledElasticInvPendulum::DecoupledElasticInvPendulum(const std::string& inName) :
    dynamicgraph::Entity(inName),
    comSIN_ (0x0, "DecoupledElasticInvPendulum("+inName+")::input(vector)::comIn"),
    comddotSIN_ (0x0, "DecoupledElasticInvPendulum("+inName+")::input(vector)::comddot"),
    contactNbrSIN_ (0x0, "DecoupledElasticInvPendulum("+inName+")::input(unsigned)::contactNbr"),
    contact1SIN_ (0x0, "DecoupledElasticInvPendulum("+inName+")::input(vector)::contact1"),
    contact2SIN_ (0x0, "DecoupledElasticInvPendulum("+inName+")::input(vector)::contact2"),
    comSOUT_ ("DecoupledElasticInvPendulum("+inName+")::output(vector)::com"),
    comdotSOUT_ ("DecoupledElasticInvPendulum("+inName+")::output(vector)::comdot"),
    flexSOUT_ ("DecoupledElasticInvPendulum("+inName+")::output(vector)::flex"),
    flexdotSOUT_("DecoupledElasticInvPendulum("+inName+")::output(vector)::flexdot"),
    dt_(0.005), com_(3), comdot_(3),
    flex_(3), flexdot_(3)
{
    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (comddotSIN_);
    signalRegistration (contactNbrSIN_);
    signalRegistration (contact1SIN_);
    signalRegistration (contact2SIN_);

    signalRegistration (comSOUT_);
    signalRegistration (comdotSOUT_);
    signalRegistration (flexSOUT_);
    signalRegistration (flexdotSOUT_);

    comSOUT_.addDependency      (comddotSIN_);
    comSOUT_.addDependency      (contactNbrSIN_);
    comSOUT_.addDependency      (contact1SIN_);
    comSOUT_.addDependency      (contact2SIN_);
    comdotSOUT_.addDependency   (comSOUT_);
    flexSOUT_.addDependency     (comSOUT_);
    flexdotSOUT_.addDependency  (comSOUT_);

    comSOUT_.setFunction (boost::bind(&DecoupledElasticInvPendulum::computeCom,
                                          this,_1,_2));



    std::string docstring;
    docstring =
        "\n"
        "    Set sampling time period task\n"
        "\n"
        "      input:\n"
        "        a floating point number\n"
        "\n";
    addCommand("setTimePeriod",
               new dynamicgraph::command::Setter<DecoupledElasticInvPendulum, double>
               (*this, &DecoupledElasticInvPendulum::setTimePeriod, docstring));

    addCommand("getTimePeriod",
               makeDirectGetter (*this, &kth_,
                                  docDirectGetter
                                  ("Get sampling time period","float")));

    addCommand ("getKth",
                makeDirectGetter (*this, &kth_,
                                  docDirectGetter
                                  ("Get angular elasticity","float")));

    addCommand ("getKz",
                makeDirectGetter (*this, &kz_,
                                  docDirectGetter
                                  ("Get linear elasticity","float")));

    addCommand ("setKth",
                makeDirectSetter (*this, &kth_,
                                  docDirectSetter
                                  ("Set angular elasticity","float")));

    addCommand ("setKz",
                makeDirectSetter (*this, &kz_,
                                  docDirectSetter
                                  ("Set linear elasticity","float")));

    addCommand ("setMass",
                makeDirectSetter (*this, &m_,
                                  docDirectSetter
                                  ("Set pendulum mass","float")));

    addCommand ("getMass",
                makeDirectGetter (*this, &m_,
                                  docDirectGetter
                                  ("Get pendulum mass","float")));

    addCommand ("resetCoM",
                makeCommandVoid0 (*this, &DecoupledElasticInvPendulum::resetCoM,
                                  docCommandVoid0 ("Set the internal position of the CoM to comIn signal")));


    kth_ = 510;
    kz_= 53200;//150000;
    m_ = 59.8;
    comh_ = 0.8;



    com_.setZero ();
    com_(2) = 0.8;
    comdot_.setZero ();
    flex_.setZero ();
    flexdot_.setZero ();


}



/// Compute the control law
Vector&
DecoupledElasticInvPendulum::computeCom(Vector& com,
                                   const int& time)
{
    const Vector & comddot = comddotSIN_ (time);
    const int& contactNbr   = contactNbrSIN_ (time);
    const Vector& contact1 = contact1SIN_ (time);
    const Vector& contact2 = contact2SIN_ (time);

    com.resize(3);

    switch (contactNbr)
    {
    case 0:
        com_ = com_ + dt_* comdot_;
        comdot_ = comdot_ + dt_ * comddot;
        flex_ = flex_ + dt_ * flexdot_;
        flexdot_ = flexdot_;

    case 1: //single support
    {
        double x     = com_(0) - contact1(0);
        double y     = com_(1) - contact1(1);
        comh_ = com_(2) - contact1(2);


        p.setHeight(comh_);
        p.setElasticity(kth_);
        p.setMass(m_);

        //Along x
        stateObservation::Vector xk (stateObservation::Vector::Zero(4,1));
        xk(0) = x;
        xk(1) = -flex_ (1);
        xk(2) = comdot_(0);
        xk(3) = -flexdot_(1);

        stateObservation::Vector uk ( stateObservation::Vector::Zero(1,1));
        uk[0] = comddot(0);

        stateObservation::Vector xk1 (p.stateDynamics(xk,uk,time));



        //Along y
        stateObservation::Vector yk (stateObservation::Vector::Zero(4,1));
        yk(0) = y;
        yk(1) = flex_ (0);
        yk(2) = comdot_(1);
        yk(3) = flexdot_(0);

        uk = stateObservation::Vector::Zero(1,1);
        uk[0] = comddot(1);

        stateObservation::Vector yk1 (p.stateDynamics(yk,uk,time));


        com_(0)     = xk(0)+contact1(0);
        com_(1)     = yk(0)+contact1(1);

        flex_(1)    = - xk(1);
        flex_(0)    = yk(1);

        comdot_(1)  = yk(2);
        comdot_(0)  = xk(2);

        flexdot_(1) = - xk(3);
        flexdot_(0) = xk(3);
    }
        break;
    default: //double support or more
    {

        // compute component of angle orthogonal to the line joining the feet
        double delta_x = contact1 (0) - contact2 (0);
        double delta_y = contact1 (1) - contact2 (1);

        dynamicgraph::Vector contact =  ( contact1 + contact2 )*0.5;

        p.setMass(m_);
        p.setHeight(comh_);

        double x     = com_(0) - contact(0);
        double y     = com_(1) - contact(1);
        comh_        = com_(2) - contact(2);

        double stepLength = sqrt (delta_x*delta_x+delta_y*delta_y);

        double u2x = delta_x/stepLength;
        double u2y = delta_y/stepLength;
        double u1x = u2y;
        double u1y = -u2x;

        //along the orthogonal to the contacts line

        p.setElasticity(2*kth_);

        double theta0 = - u1x * flex_ (1) + u1y * flex_(0);
        double dtheta0 = - u1x * flexdot_ (1) + u1y * flexdot_ (0);
        double xi = u1x*x + u1y*y;
        double dxi = u1x*comdot_(0) + u1y*comdot_(1);

        stateObservation::Vector xik (stateObservation::Vector::Zero(4,1));
        xik(0) = xi;
        xik(1) = theta0;
        xik(2) = dxi;
        xik(3) = dtheta0;

        stateObservation::Vector uik ( stateObservation::Vector::Zero(1,1));
        uik[0] = u1x*comddot(0) + u1y*comddot(1);

        stateObservation::Vector xik1 (p.stateDynamics(xik,uik,time));

        //along the contacts line
        p.setElasticity(2*kth_ + 2*kz_*stepLength/2);

        double theta1 = - u2x * flex_ (1) + u2y * flex_ (0);
        double dtheta1 = - u2x * flexdot_ (1) + u2y * flexdot_ (0);
        double yi = u2x*x + u2y*y;
        double dyi = u2x*comdot_(0) + u2y*comdot_(1);

        stateObservation::Vector yik (stateObservation::Vector::Zero(4,1));
        yik(0) = yi;
        yik(1) = theta1;
        yik(2) = dyi;
        yik(3) = dtheta1;

        uik[0] = u2x*comddot(0) + u2y*comddot(1);

        stateObservation::Vector yik1 (p.stateDynamics(yik,uik,time));

        com_(0)     =   xik1(0) * u1x + yik1(0)*u2x + contact(0);
        com_(1)     =   yik1(0) * u1y + yik1(0)*u2y + contact(1);

        flex_(0)    =  (xik1(1) * u1x + yik1(1)*u2x);
        flex_(1)    = -(xik1(1) * u1y + yik1(1)*u2y);

        comdot_(0)  =  (xik1(2) * u1x + yik1(2)*u2x);
        comdot_(1)  =  (xik1(2) * u1y + yik1(2)*u2y);

        flexdot_(0) =  (xik1(3) * u1x + yik1(3)*u2x);
        flexdot_(1) = -(xik1(3) * u1y + yik1(3)*u2y);
    }
    break;
    };

    com=com_;

    comdotSOUT_.setConstant(comdot_);
    comdotSOUT_.setTime(time);

    dynamicgraph::sot::VectorUTheta flexInvUth;
    flexInvUth(0)=-flex_(0);
    flexInvUth(1)=-flex_(1);
    flexInvUth(2)=-flex_(2);
    dynamicgraph::sot::MatrixRotation flexibilityRotInverse;
    flexibilityRotInverse.fromVector(flexInvUth);

    Vector flexInv = -flex_;
    Vector flexdotInv = -flexibilityRotInverse * flexdot_;


    //provide the inverse flexibility orientation
    flexSOUT_.setConstant(flexInv);
    flexSOUT_.setTime(time);

    //provide the invser flexibility velocity
    flexdotSOUT_.setConstant(flexdotInv);
    flexdotSOUT_.setTime(time);

    return com;
}

} // namespace sotStabilizer
