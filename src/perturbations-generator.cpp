#include <sstream>

#include <dynamic-graph/command-setter.h>

#include <dynamic-graph/factory.h>
#include <jrl/mal/matrixabstractlayervector3jrlmath.hh>
#include <sot-stabilizer/perturbations-generator.hh>

#include <sot/core/flags.hh>


namespace sotStabilizer
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( VectorPerturbationsGenerator,
                                "VectorPerturbationsGenerator" );

    VectorPerturbationsGenerator::VectorPerturbationsGenerator
                                        ( const std::string & inName):
        Entity(inName),
        perturbationMode_(0),
        perturbationPeriod_(0),
        timeSinceLast_(0),
        sinSIN(0x0 , "VectorPerturbationsGenerator("+inName+")::input(vector)::sin"),
        perturbationSIN
            (0x0 , "VectorPerturbationsGenerator("+inName+")::input(vector)::perturbation"),
        soutSOUT( "VectorPerturbationsGenerator("+inName+")::output(vector)::sout"),
        selecSIN(0x0 ,
                "VectorPerturbationsGenerator("+inName+")::input(flags)::selec"),
        on_(false),
        currentTime_(0)
    {
        signalRegistration (sinSIN);
        signalRegistration (soutSOUT);
        signalRegistration (perturbationSIN);
        signalRegistration (selecSIN);

        iterationNumber_ = 0;

        soutSOUT.setFunction(boost::bind(&VectorPerturbationsGenerator::computeSout,
				    this, _1, _2));

        std::string docstring;

        docstring =
                "\n"
                "    Set the perturbation Mode"
                "      - 0 (default): dirac perturbation"
                "      - 1 : step perturbation"
                "\n";

        addCommand(std::string("setMode"),
	     new
	     dynamicgraph::command::Setter <VectorPerturbationsGenerator,int>
	     (*this, &VectorPerturbationsGenerator::setPerturbationMode, docstring));


        docstring =
                "\n"
                "    Set the perturbation Period in number of steps"
                "     (0: for punctual perturbation)"
                "\n";

        addCommand(std::string("setPeriod"),
	     new
	     dynamicgraph::command::Setter <VectorPerturbationsGenerator,int>
            (*this, &VectorPerturbationsGenerator::setPeriod, docstring));


        docstring =
                "\n"
                "    - true: Activate the perturbation"
                "    - false: Deactivate the perturbation"
                "\n";

        addCommand(std::string("activate"),
	     new
	     dynamicgraph::command::Setter <VectorPerturbationsGenerator,bool>
            (*this, &VectorPerturbationsGenerator::activate, docstring));

    }

    VectorPerturbationsGenerator::~VectorPerturbationsGenerator()
    {
    }

    dynamicgraph::Vector& VectorPerturbationsGenerator::computeSout
                (dynamicgraph::Vector & output, const int& inTime)
    {

        if (inTime ==currentTime_ && currentTime_!=0)
        {
            return output=currentOutput_;
        }
        else
        {
            currentTime_=inTime;

            const ::dynamicgraph::Vector & sin (sinSIN(inTime));
            if (on_)
            {
                if (perturbationMode_==0) //dirac
                {
                    output = sin;

                    if (timeSinceLast_==perturbationPeriod_)
                    {
                        ++iterationNumber_;
                        timeSinceLast_=0;
                        const dynamicgraph::Vector & perturbation = perturbationSIN(inTime);
                        const dynamicgraph::sot::Flags & selec = selecSIN(inTime);
                        dynamicgraph::Vector impulsion(perturbation.size());
                        for (unsigned i=0; i<perturbation.size(); ++i)
                        {
                            if (selec(i))
                            {
                                impulsion(i) = perturbation(i);
                            }
                            else
                            {
                                impulsion(i) = 0;
                            }
                        }
                        output += impulsion;
                    }
                    if (perturbationPeriod_ == 0)
                        on_ = false;
                    else
                        ++timeSinceLast_;
                }
                else if (perturbationMode_==1) //step
                {
                    output = sin;

                    if (timeSinceLast_==perturbationPeriod_)
                    {
                        timeSinceLast_=0;
                        ++iterationNumber_;
                    }

                    const dynamicgraph::Vector & perturbation = perturbationSIN(inTime);
                    const dynamicgraph::sot::Flags & selec = selecSIN(inTime);
                    dynamicgraph::Vector step(perturbation.size());
                    for (unsigned i=0; i<perturbation.size(); ++i)
                    {
                        if (selec(i))
                        {
                            step(i) = iterationNumber_*perturbation(i);
                        }
                        else
                        {
                            step(i) = 0;
                        }
                    }
                    output += step;

                    ++timeSinceLast_;
                }
            }
            else
            {
                output = sin;
            }

            return currentOutput_=output;
        }
    }
}
