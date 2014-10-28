#include <sot-stabilizer/controllers/state-space-linear-controller.hh>
#include <sot-stabilizer/tools/hrp2.hpp>

#include <iostream>


namespace sotStabilizer
{
namespace controller
{ 
    StateSpaceLinearController::StateSpaceLinearController(unsigned stateSize, unsigned controlSize)
    {
        stateSize_=stateSize;
        controlSize_=controlSize;
    }

    unsigned StateSpaceLinearController::getStateSize() const
    {
        return stateSize_;
    }

    unsigned StateSpaceLinearController::getControlSize() const
    {
        return controlSize_;
    }

    stateObservation::Vector StateSpaceLinearController::getControl(int time)
    {
        if (time==time_)
        {

#ifndef NDEBUG
#endif // NDEBUG

            double m=hrp2::m;// 59.8;
            double ktheta=hrp2::angKe; //600;
            double g=9.81;
            double comz=hrp2::H; //0.807;
            double mu=ktheta/(m*comz)-g;
            stateObservation::Vector4 c;

            double ctheta=m*comz/ktheta*(a_(2)*mu*comz-mu*mu-a_(0)*comz*comz);
            double cksi=m*comz/ktheta*(a_(0)*comz+a_(2)*g-g/comz);
            double cdtheta=m*comz*comz/ktheta*(a_(3)*mu-a_(1)*comz);
            double cdksi=m*comz/ktheta*(a_(3)*g+a_(1)*comz);
            c <<    cksi,
                    ctheta,
                    cdksi,
                    cdtheta;


            checkState_(x_);
            u_ = - c.transpose()*x_;
            computedInput_=true;
            time_ = time_+1;

        }
        else
        {
            BOOST_ASSERT(time_==time-1 &&
             "The requested control time should be current value or next value.");

            BOOST_ASSERT( computedInput_ &&
             "Input is not initalized, try requesting for current time.");
        }

        return u_;
    }

    stateObservation::Vector StateSpaceLinearController::getTheoricalState(stateObservation::Vector &u, const double dt){
        stateObservation::Matrix4 Ac;
        stateObservation::Vector4 Bc;

        Ac <<   0,1,0,0,
                0,0,1,0,
                0,0,0,1,
                -a_(0),-a_(1),-a_(2),-a_(3);
        Bc <<   0,
                0,
                0,
                1;

        stateObservation::Vector xdot;

        xdot=Ac*x_+u(0)*Bc;

        return x_+xdot*dt;
    }

    void StateSpaceLinearController::setCaracteristicPolynomial(const stateObservation::Vector &a){
        a_=a;
    }

    void StateSpaceLinearController::setCaracteristicPolynomialFromPoles(const stateObservation::Vector &p){

        stateObservation::Vector temp;
        temp.resize(4);
        temp << p[0]*p[1]*p[2]*p[3],
              -(p(0)*p(1)*p(2)+p(0)*p(1)*p(3)+p(0)*p(2)*p(3)+p(1)*p(2)*p(3)),
              (p(0)*p(1)+p(0)*p(2)+p(0)*p(3)+p(1)*p(2)+p(1)*p(3)+p(2)*p(3)),
              -(p(0)+p(1)+p(2)+p(3));

        a_=temp;
    }

    stateObservation::Vector StateSpaceLinearController::getCaracteristicPolynomial(){
        return a_;
    }

}
}
