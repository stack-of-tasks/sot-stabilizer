/*
 *  Copyright 2013 CNRS
 *
 *  Mehdi Benallegue
 */

#ifndef MOVING_FRAME_TRANSFORMATION_HH
#define MOVING_FRAME_TRANSFORMATION_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-homogeneous.hh>

#include <sot-state-observation/tools/definitions.hh>

#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot/core/flags.hh>


namespace sotStabilizer
{
        /**
           \brief
        */
        class VectorPerturbationsGenerator :
            public dynamicgraph::Entity
        {
        public:
            /**
            \brief Constructor by name
            */
            VectorPerturbationsGenerator(const std::string& inName);

            ~VectorPerturbationsGenerator();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    std::string("Transfers the input to the output with a perturbation");
            }

            virtual void setPerturbationMode (const int & k)
            {
                perturbationMode_ = k;
                iterationNumber_ = 0;
                timeSinceLast_=0;
            }

            virtual void setPeriod (const int & k)
            {
                perturbationPeriod_ = k;
            }

            virtual void activate (const bool & b)
            {
                on_ = b;
                if (!b)
                {
                    iterationNumber_=0;
                    timeSinceLast_=0;
                }
            }

            /**
            \name Parameters
            @{
            */
        protected:
            /*
            \brief Class name
            */
            static const std::string CLASS_NAME;

        private:
            /**
            Compute the control law
            */
            ::dynamicgraph::Vector& computeSout
                        (::dynamicgraph::Vector & output, const int& inTime);

            /**
            \brief local to global frame position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> sinSIN;

            /**
            \brief local to global frame position
            */
            dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> perturbationSIN;

            /**
            \brief Estimation of the attitude
            */
            dynamicgraph::Signal < ::dynamicgraph::Vector, int> soutSOUT;

            /**
            \brief selection matrix
            */
            dynamicgraph::SignalPtr < dynamicgraph::sot::Flags, int> selecSIN;

            double perturbationMode_;
            double perturbationPeriod_;

            unsigned timeSinceLast_;
            bool on_;

            ::dynamicgraph::Vector currentOutput_;

            unsigned iterationNumber_;

            unsigned currentTime_;
        };

} // namespace sotStateObservation

#endif //MOVING_FRAME_TRANSFORMATION_HH

