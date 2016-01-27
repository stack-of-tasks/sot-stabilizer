#include <sot-stabilizer/tools/hrp2.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <sot-stabilizer/hrp2-lqr-twoDof-coupled-stabilizer.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>


#include <iostream>

using namespace sotStabilizer;
using namespace sotStateObservation;
using namespace stateObservation;

int testModel()
{
  /// sampling period
    const double dt=5e-3;

  /// Initializations
    // Dimensions
    const unsigned kinit=0;
    const unsigned kmax=1400;
    const unsigned measurementSize=6;
    const unsigned inputSize=54;
    const unsigned stateSize=18;
    unsigned contactNbr = 2;

    // Input initialization
    stateObservation::Vector u0;
    u0.resize(inputSize-6*contactNbr);
    u0 <<  0.0135673,
            0.001536,
            0.80771,
            -2.63605e-06,
            -1.09258e-08,
            5.71759e-08,
            2.71345,
            0.3072,
            161.542,
            48.1348,
            46.9498,
            1.76068,
            -0.0863332,
            -0.594871,
            -0.0402246,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            -0.098,
            -6.23712e-11,
            1.1174,
            1.58984e-22,
            -5.43636e-21,
            3.9598e-22,
            -2.99589e-06,
            -1.24742e-08,
            -4.7647e-18,
            3.17968e-20,
            -1.08727e-18,
            7.91959e-20,
            -0.000299589,
            -1.24742e-06,
            -4.7647e-16;

    /// Definitions of input vectors
      // Measurement
      IndexedMatrixArray y;
      std::cout << "Loading measurements file" << std::endl;
      y.getFromFile("source_measurement.dat",1,measurementSize);
      // Input
      IndexedMatrixArray u;
      std::cout << "Loading input file" << std::endl;
      u.getFromFile("source_input.dat",1,inputSize);
      //state
      IndexedMatrixArray xRef;
      std::cout << "Loading reference state file" << std::endl;
      xRef.getFromFile("source_state.dat",stateSize,1);

    /// Definition of ouptut vectors
      // Angular momenta
      IndexedMatrixArray momenta_output;

      stateObservation::Matrix3 ac;
      stateObservation::Vector3 f;
      f <<  1,
            2,
            3;
      ac=kine::skewSymmetric(f);



    return 0;
}

int main()
{

    return testModel();

}
