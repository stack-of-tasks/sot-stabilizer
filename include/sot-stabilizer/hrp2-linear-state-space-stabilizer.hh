//
// Copyright (c) 2014,
// Alexis Mifsud
//
// CNRS
//
// This file is part of sot-stabilizer.
// sot-stabilizer is free software: you can redistribute it and/or
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

#ifndef HRP2LINEARSTATESPACESTABILIZER_H
#define HRP2LINEARSTATESPACESTABILIZER_H

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>

#include <sot/core/task-abstract.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/multi-bound.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sot-stabilizer/controllers/state-space-linear-controller.hh>
#include <sot-state-observation/tools/definitions.hh>

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
  using dynamicgraph::sot::MatrixHomogeneous;
  using dynamicgraph::sot::MatrixRotation;
  using dynamicgraph::sot::VectorUTheta;
  using dynamicgraph::sot::VectorRollPitchYaw;


  class HRP2LinearStateSpaceStabilizer : public TaskAbstract
  {
    DYNAMIC_GRAPH_ENTITY_DECL ();
  public:
    static double constm_;
    static double constcomHeight_;

    /// Constructor by name
    HRP2LinearStateSpaceStabilizer(const std::string& inName);
    ~HRP2LinearStateSpaceStabilizer() {}

    /// Documentation of the entity
    virtual std::string getDocString () const
    {
      std::string doc =
        "Dynamic balance humanoid robot stabilizer\n"
        "\n"
        "This task aims at stabilize a humanoid robot modeled with a 2D table cart model.\n"
        "The lateral dynamic is for now negligate";
      return doc;
    }

    /// Start stabilizer
    void start ()
    {
      on_ = true;
    }

    /// Stop stabilizer
    void stop ()
    {
      on_ = false;
    }

    /// @}
    /// \name Sampling time period
    /// @{

    /// \brief Set sampling time period
    void setTimePeriod(const double& inTimePeriod)
    {
      dt_ = inTimePeriod;
    }
    /// \brief Get sampling time period
    double getTimePeriod() const
    {
      return dt_;
    }
    /// @}


  private:

    /// Compute the number of supports
    unsigned int computeNbSupport(const int& time);//const MatrixHomogeneous& leftFootPosition, const MatrixHomogeneous& rightFootPosition, const Vector& forceLf, const Vector& forceRf,const int& time);

    /// Compute the control law
    VectorMultiBound& computeControlFeedback(VectorMultiBound& comdot,
        const int& time);

    Matrix& computeJacobianCom(Matrix& jacobian, const int& time);

    /// Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comSIN_;
    /// Reference position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comRefSIN_;
    /// Jacobian of the task
    SignalPtr < dynamicgraph::Matrix, int> jacobianSIN_;

    /// State of the flexibility
    SignalPtr <MatrixHomogeneous, int> stateFlexSIN_;
    /// Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> stateFlexDotSIN_;

    /// Position of left foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
    /// Position of right foot force sensor in global frame
    SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
    /// Force in left foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceLeftFootSIN_;
    /// Force in right foot sensor
    SignalPtr <dynamicgraph::Vector, int> forceRightFootSIN_;
    /// Contact Position
    SignalTimeDependent <Vector, int> supportPos1SOUT_;
    SignalTimeDependent <Vector, int> supportPos2SOUT_;

    /// Number of support feet
    SignalTimeDependent <unsigned int, int> nbSupportSOUT_;

    ///error output
    SignalTimeDependent <Vector, int> errorSOUT_;

    SignalPtr <double, int> controlGainSIN_;

    // Time sampling period
    double dt_;
    // Whether stabilizer is on
    bool on_;

    // Threshold on normal force above which the foot is considered in contact
    double forceThreshold_;
    // Number of feet in support
    unsigned int nbSupport_;

    unsigned int iterationsSinceLastSupportLf_;
    unsigned int iterationsSinceLastSupportRf_;
    unsigned int supportCandidateLf_;
    unsigned int supportCandidateRf_;

    // Acceleration of the center of mass computed by stabilizer
    Vector d2com_;
    // coordinates of center of mass velocity in moving frame
    Vector dcom_;

    Vector comdotRef_;

    // Deviation of center of mass
    Vector deltaCom_;

    controller::StateSpaceLinearController controller0_;
    controller::StateSpaceLinearController controller1_;


  }; // class Stabilizer
} // namespace sotStabiilizer

#endif // HRP2LinearStateSpaceStabilizer_H
