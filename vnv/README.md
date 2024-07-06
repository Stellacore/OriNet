# OriNet/vnv - Verification and Validation module

The OriNet/vnv subdirectory contains an independent project that can
be used to validate use of OriNet in an external project environment.

## Contents

This module includes validation program:

* SimNetwork - simulates a survey backsighting network.

	Example use of OriNet robust fitting operations to consolidate
	simulated survey station setups into a network expressed in a
	single coordinate frame.

## Build and Run

### Building the VnV module
```
CC=/usr/bin/clang CXX=/usr/bin/clang++ \
\
cmake\
 -DCMAKE_BUILD_TYPE=Release\
 -DCMAKE_PREFIX_PATH=/tmpLocal/\
 -DCMAKE_INSTALL_PREFIX=/tmpLocal/\
 ~/repos/OriNet/vnv/\
&&\
 cmake --build . --target all -j `nproc`
```

### Running VnV programs

SimNetwork can be run from the build directory as e.g.

```
 ./SimNetwork  /tmp/network_all.dot /tmp/network_mst.dot\
&&\
 dot -T png -o /tmp/network_all.png /tmp/network_all.dot\
&&\
 dot -T png -o /tmp/network_mst.png /tmp/network_mst.dot
```

SimNetwork output optionally (if hardcoded showResults variable set true)
shows the simulated station orientation results along with reconstructed
network station orientations.

The /tmp/network\*.png files provide a visual representation of the
full network and the minimum spanning tree network.

## Dependencies

* The [Graaf library](https://github.com/bobluppes/graaf) is used to
by SimNetwork to represent the survey network connectivity and to
traverse the network effectively.



