/*
 * Copyright 2010,
 * Alexis Mifsud
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

#ifndef DYNAMIC_GRAPH_ROTATIONAL_TABLE_CART_DEVICE_HH
#define DYNAMIC_GRAPH_ROTATIONAL_TABLE_CART_DEVICE_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>

#include <state-observation/tools/definitions.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

namespace sotStabilizer {

    using namespace sotStateObservation;
    using namespace stateObservation;

    /// \brief ROtational table cart model
    ///
    /// This class represents the classical table-cart model as used as a
    /// simplified model for a humanoid robot but with a one more degree of
    /// freedom allowding the control of the orientation of a body around the com
    ///
    class LinearizedRotationalTableCartDevice : public dynamicgraph::Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();
    public:
      /**
         \brief Constructor by name
      */
      LinearizedRotationalTableCartDevice(const std::string& inName);

      ~LinearizedRotationalTableCartDevice();

      /// Integrate equation of motion over time step given as input
      void incr(double inTimeStep);

      /// \name Parameters
      /// @{

      /// \brief Set the mass of the cart
      void setCartMass (const double& inMass) {
        cartMass_ = inMass;
      }

      /// \brief Set the mass of the cart
      void setCartHeight (const double& inHeight) {
        cl_ <<   0,
                 0,
                 inHeight;
      }

      void recomputeMatrices();

      /// \brief Get the mass of the cart
      double getCartMass () const {
        return cartMass_;
      }

      void setMomentOfInertia(const dynamicgraph::Matrix& momentOfInertia)
      {
          I_=convertMatrix<stateObservation::Matrix>(momentOfInertia);
      }

      dynamicgraph::Matrix getMomentOfInertia() const
      {
          return convertMatrix<dynamicgraph::Matrix>(I_);
      }

      /// \brief Set the stiffness of the flexibility
      void setStiffness (const dynamicgraph::Matrix& inStiffness) {
          stiffness_ = convertMatrix<stateObservation::Matrix>(inStiffness);
      }

      /// \brief Get the stiffness of the flexibility
      dynamicgraph::Matrix getStiffness () const {
        return convertMatrix<dynamicgraph::Matrix>(stiffness_);
      }

      /// \brief Set the viscosity of the flexibility
      void setViscosity (const dynamicgraph::Matrix& inViscosity) {
        viscosity_ = convertMatrix<stateObservation::Matrix>(inViscosity);
      }

      /// \brief Get the viscosity of the flexibility
      dynamicgraph::Matrix getViscosity () const {
        return convertMatrix<dynamicgraph::Matrix>(viscosity_);
      }

      /**
         @}
      */

      /**
         \brief Compute the evolution of the state of the pendulum
      */
      stateObservation::Vector computeDynamics(const dynamicgraph::Vector& inState,
                             const dynamicgraph::Vector& inControl,
                             const double& inForce,
                             double inTimeStep,
                 stateObservation::Vector & flexddot,
                                 stateObservation::Vector & realcom,
                                 stateObservation::Vector & zmp,
                             stateObservation::Vector & output);

    private:
      /// Perturbation force acting on the table cart
      dynamicgraph::SignalPtr< double, int > forceSIN_;
      /// Control
      dynamicgraph::SignalPtr< dynamicgraph::Vector, int > controlSIN_;
      /// Real position of the center of mass
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> comrealposSOUT_;
      /// Acceleration of com due to acceleration of flexibility
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> flexcomddotSOUT_;
      /// Height of the CoM
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> clSOUT_;
      /// State of the table cart
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> stateSOUT_;
      /// Output: position of the center of mass and momentum at the foot
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> outputSOUT_;
      /// Zmp position
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> zmpSOUT_;

      /// \brief Mass of the cart
      double cartMass_;
      /// \brief Height of the cart
      stateObservation::Vector cl_;
      /// \brief Stiffness of the flexibility
      stateObservation::Matrix stiffness_;
      /// \brief Viscosity coefficient
      stateObservation::Matrix viscosity_;
      /// Moment of inertia around y axis
      stateObservation::Matrix I_;

      stateObservation::Matrix A_;
      stateObservation::Matrix B_;
    };
}
#endif // DYNAMIC_GRAPH_ROTATIONAL_TABLE_CART_DEVICE_HH
