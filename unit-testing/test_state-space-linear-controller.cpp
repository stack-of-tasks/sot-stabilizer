#include <sot-stabilizer/tools/hrp2.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <sot-stabilizer/prototyping/linearized-table-cart-device.hh>
#include <sot-stabilizer/controllers/state-space-linear-controller.hh>

#include <iostream>

using namespace sotStabilizer;
using namespace sotStateObservation;
using namespace stateObservation;

int test()
{

    /// Time
//    int t;
//    t = time(NULL);
    unsigned int k, kmax=1000;
    const double dt=5e-3;

  /// System
    const std::string& inName="system";
    LinearizedTableCartDevice system(inName);
    IndexedMatrixArray x; // state

    // Initialization
    system.setCartHeight(hrp2::H);
    system.setCartMass(hrp2::m);
    system.setStiffness(hrp2::angKe);
    system.setViscosity(0);
    system.recomputeMatrices();
    // Initialization of x
    stateObservation::Vector4 xinit;
    xinit <<    0.807,
                1,
                0,
                0;
    x.setValue(xinit,0);


  /// Controller
    controller::StateSpaceLinearController controller(4,1);
    IndexedMatrixArray u;

    // Initialization of a (Caracteristic polynomial)
    const stateObservation::Vector& p=stateObservation::Vector::Constant(4,1,-5);
    controller.setCaracteristicPolynomialFromPoles(p);

    // Useless but here
    double f;
    dynamicgraph::Vector vect, output;

//    X si le système est parfaitement controle : Attention il faut changer de base de la forme compagneà l'autre
//    stateObservation::Vector xtheo;
//    xtheo=controller.getTheoricalState(u[0],dt);

    // Control loop
    for(k=0;k<kmax;k++){
        controller.setState(x[k],controller.getTime());
        u.setValue(controller.getControl(controller.getTime()),k);
        x.setValue(convertVector<stateObservation::Vector>(system.computeDynamics
                                                    (convertVector<dynamicgraph::Vector>(x[k]),
                                                     convertVector<dynamicgraph::Vector>(u[k]),
                                                     f,dt,vect,vect,vect,output)),k+1);
        std::cout << "x=" << x[k+1].transpose() << std::endl;
    }

    x.writeInFile("state.dat");
    u.writeInFile("control.dat");

    return 0;
}

int main()
{

    return test();

}

