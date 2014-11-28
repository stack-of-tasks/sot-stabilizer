
/*
 * Copyright 2014,
 * Mehdi Benallegue
 *
 * CNRS
 *
 * This file is part of dynamic-graph-tutorial.
 * dynamic-graph-tutorial is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * dynamic-graph-tutorial is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with dynamic-graph-tutorial.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef COMTROLLERBASELAASHH
#define COMTROLLERBASELAASHH

namespace sotStabilizer
{
namespace controller
{
class ControllerBase
{
public:
    ControllerBase()
    {
        time_=0;
    }

    virtual ~ControllerBase(){}

    virtual void setState(const stateObservation::Vector &x, int time)
    {
        checkState_(x);
        x_=x;
        time_=time;
    }

    virtual stateObservation::Vector& getState(){
        return x_;
    }

    virtual int getTime() const
    {
        return time_;
    }

    virtual stateObservation::Vector getControl(int time)=0;

    virtual unsigned getStateSize() const =0;

    virtual unsigned getControlSize() const =0;

protected:
    inline bool checkState_(const stateObservation::Vector & v)
    {
        BOOST_ASSERT(v.size() == getStateSize() && "The state size is incorrect");
        return (v.size() == getStateSize());
    }

    stateObservation::Vector x_;
    unsigned time_;

};

}
}

#endif //COMTROLLERBASELAASHH
