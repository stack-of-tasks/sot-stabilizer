sot-stabilizer
===============

This sofware provides 


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The sot dynamic inverse depends on several packages which
have to be available on your machine.

 - Libraries:
   - sot-core (>= 1.0.0)
   - state-observation
   - state-stabilizer
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)

[sot-core]: http://github.com/stack-of-tasks/sot-core
