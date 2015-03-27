/*
 * Copyright 2010,
 * Florent Lamiraux
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

#ifndef DYNAMIC_GRAPH_TUTORIAL_TABLE_CART_DEVICE_HH
#define DYNAMIC_GRAPH_TUTORIAL_TABLE_CART_DEVICE_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>

namespace sotStabilizer {
    /// \brief Table cart model
    ///
    /// This class represents the classical table-cart model as used as a
    /// simplified model for a humanoid robot.
    /// The equation of motion is:
    /// \f{eqnarray*}{\dot x  &=& \textbf{u}\f}
    /// where
    /// \li the state \f$x\f$ is the position of the cart on an horizontal axis
    /// represented by signal stateSOUT,
    /// \li the control is a vector of dimension 1 \f$\textbf{u}\f$ reprensented
    /// by signal controlSIN_.
    /// \li \f$m\f$ is the mass of the cart.
    ///
    class LinearizedTableCartDevice : public dynamicgraph::Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();
    public:
      /**
	 \brief Constructor by name
      */
      LinearizedTableCartDevice(const std::string& inName);

      ~LinearizedTableCartDevice();

      /// Integrate equation of motion over time step given as input
      void incr(double inTimeStep);

      /// \name Parameters
      /// @{

      /// \brief Set the mass of the cart
      void setCartMass (const double& inMass) {
	cartMass_ = inMass;
      }

      void recomputeMatrices();

      /// \brief Get the mass of the cart
      double getCartMass () const {
	return cartMass_;
      }

      /// \brief Set the height of the cart
      void setCartHeight (const double& inHeight) {
	cartHeight_ = inHeight;
	dynamicgraph::Vector comheight;
	comheight.resize(1);
    comheight(0) = cartHeight_;
	comHeightSOUT_.setConstant(comheight);
      }



      /// \brief Get the height of the cart
      double getCartHeight () const {
	return cartHeight_;
      }

      /// \brief Set the stiffness of the flexibility
      void setStiffness (const double& inStiffness) {
	stiffness_ = inStiffness;
      }

      /// \brief Get the stiffness of the flexibility
      double getStiffness () const {
	return stiffness_;
      }

      /// \brief Set the viscosity of the flexibility
      void setViscosity (const double& inViscosity) {
	viscosity_ = inViscosity;
      }

      /// \brief Get the viscosity of the flexibility
      double getViscosity () const {
	return viscosity_;
      }

      /**
	 @}
      */

      /**
         \brief Compute the evolution of the state of the pendulum
      */
      dynamicgraph::Vector computeDynamics(const dynamicgraph::Vector& inState,
                             const dynamicgraph::Vector& inControl,
                             const double& inForce,
                             double inTimeStep,
                             dynamicgraph::Vector & flexddot,
                             dynamicgraph::Vector & realcom,
                             dynamicgraph::Vector & zmp,
                             dynamicgraph::Vector& output);

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
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> comHeightSOUT_;
      /// State of the table cart
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> stateSOUT_;
      /// Output: position of the center of mass and momentum at the foot
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> outputSOUT_;
      /// Zmp position
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> zmpSOUT_;

      /// \brief Mass of the cart
      double cartMass_;
      /// \brief Height of the cart
      double cartHeight_;
      /// \brief Stiffness of the flexibility
      double stiffness_;
      /// \brief Viscosity coefficient
      double viscosity_;
      /// Moment of inertia around y axis
      double Iyy_;
//      /**
//	 \brief Compute the evolution of the state of the pendulum
//      */
//      dynamicgraph::Vector computeDynamics(const dynamicgraph::Vector& inState,
//			     const dynamicgraph::Vector& inControl,
//			     const double& inForce,
//			     double inTimeStep,
//                 dynamicgraph::Vector & flexddot,
//				 dynamicgraph::Vector & realcom,
//				 dynamicgraph::Vector & zmp,
//			     dynamicgraph::Vector& output);
      dynamicgraph::Matrix A_;
      dynamicgraph::Matrix B_;
    };
}
#endif // DYNAMIC_GRAPH_TUTORIAL_TABLE_CART_DEVICE_HH
