# OriNet - Computations for rigid body orientation network


## Introduction

MIT License

The OriNet (Orientation Network) package offers capabilities for working
with a network of rigid bodies.  A quintesential use case is representing
the relationship between various 3D coordinate systems via rigid body
transformations.

Example use cases include determination of survey station setups relative
to an existing reference system, representation of a camera location
relative to a scene, positions of an autonomous vehicle, and so on.

### Source Code

OriNet is an open source software project provided under the liberal
MIT license that provides for varied uses including commercial ones.

The code implementation is in C++20. The 20 standard is required
by the graaflib dependency (used for implementation within network.hpp).
The remainder of the OriNet code is compilable with C++17 or later.


## Theory Overview

A rigid body transformation involves displacement and rotation (but
not scale nor deformation). In 3D Euclidean space, the rigid body
transformation may be specified by a 3D offset vector, and a 3D bivector
that describes the attitude. Together, the translation and attitude are
herein called orientation (aka pose).

This OriNet package provides capabilities associated with a collection of
multiple rigid body orientations (e.g. coordinate frames over different
objets and/or over time). Each such orientation item is herein called a
"station orientation".  Each station may be associated with an abstract
coordinate frame or a physical object.

Arbitrary pairs of station orientations are connected by rigid body
transformations which herein are called "relative orientations". A
collection of an arbitrary number of stations along with an arbitrary
number of pairwise orientations connecting them is herein called a
"network".

The network (comprised of station orientations and relative orientations)
can be conceptualized as a graph data structure (a computer science
data structure) which comprises an arbitrary number of "nodes" that are
arbitrarily linked to each other with "edges". In this model, each node
is associated with a geometric station orientation, and each edge is
associated with a relative orientation. Both orientation types (station
and relative) are represented by rigid body transformations.

Use of such an orentation network is common in many engineering
applications such as 3D parametric modeling, surveying, photogrammetry
and computer vision, robot navigation and so on.  These applications
involve measuring or computing relative orientation relationships between
stations and then using those relative orientation relationships to build
a cohesive model of the individual station orientations for every station
in the network.  (The vnv/SimNetwork.cpp program provides a simulation
example of this use case).


## General Capabilities

OriNet provides several particular capabilities. These include:

* Determination of robust orientations from collections of noisy
  orientation measurements. For example a total station setup may
  involve making backsight measurements to multiple reference points
  and it is desirable to predict nominal station orientation

* Compute station orientations for every station in the network given
  relative orientation data between pairs of stations. (i.e. propagation
  pairwise relative orientations, and one station orientation starting
  value in order to obtain a full network of station orientation)


## Demonstration Programs

Validation programs are avialable in the 'vnv' subdirectory and
are structured as an independent CMake project.
Ref [vnv/README.md file](./vnv/README.md)

Demonstration programs are available in the demo/ subdirectory and
described in the [demo/README.md file](./demo/README.md)


## Project Organization

The project has a fairly standard directory organization including:
OriNet implementation is primarily via header files in ./include/OriNet
directory. The ./src/ directory contains definitions for library functions.
Various other directories provide support/demonstration content.

Library components

* ./include/OriNet/ - directory with header files

* ./src/ - directory with definition code for library modules

Project Infrastructure

* ./test/ - unit test programs

* ./demo/ - demonstration/example and development utility programs[

* ./vnv/ - verification and validation example programs to demonstrate
  use of OriNet in an independently build project.

* ./cmake/ - CMake source code functions and include code

Supporting Information

* ./theory/ - directory for documents describing theory

* ./doc/ - files for creating documentation with doxygen


## Software Components (Namespaces)


* Main project

The OriNet code is structured into several modular units. Each of these
units has it's own subnamespace

Main project capabilities

* orinet::network - functions for creating and processing orientation
  network data.

Core supporting capabilities

* orinet::compare - functions for comparing two orientations and
  extracting quantitative measures of the difference between them.

* orinet::align - functions for estimating the alignment between
  data values (e.g. alignment of pairs of directions).

* orinet::robust - functions for robust estimation of central tendency
  orientation data. E.g. computation of a median orientation that is
  insensitive to outlier data (such as bad orientation solutions).

Simulation and testing support

* orinet::random - functions providing pseudo random data generation
  (support for test/ programs).

* orinet::sim - functions for simulation of data sets (often useful
  for development and testing new applications).

Header files

OriNet project header includes the main and supporting headers.  For
simulation and testing, include those headers directly. E.g.

```
// Some C++(20) file
#include <OriNet>  // Sufficient for general use

#include <OriNet/random>  // For (psudo)random data generation
#include <OriNet/sim>  // For network simulation utility functions
```

## Dependencies

Development Utilities

* A C++20 compiler - tested with gcc 12.2.0-14 and clang 14.0.6.

* [CMake](https://cmake.org/) - for generating cross platform build
  systems.

* [doxygen](https://www.doxygen.nl/) - for generating reference
  documentation.

* [lyx](https://www.lyx.org/) - document processor system for generating
  latex code and associated publication formats. Used for documents
  in ./theory/ directory.

The OriNet software project incorporates the following support libraries.

* [Rigibra - RIGId Body AlgreBRA](
  https://github.com/Stellacore/Rigibra)
  Practical C++ rigid body modeling algebra.  Rigibra classes are used
  to represent both the station and relative orientations.

* [Engabra - ENgineering Geometric AlgeBRA](
  https://github.com/Stellacore/Engabra)
  Practical C++ Geometric Algebra Computation for Engineering.  Engabra
  classes constitute the supporting geometric algebra framework and
  provide basic vector algebra framework for interfacing with the library.

* [graaflib](
  https://github.com/bobluppes/graaf)
  A general-purpose lightweight C++ graph library.  Graaflib provides
  the graph data structure used to store orientation network data
  relationships and to perform efficient network traversal operations.


## Build commands

Example build command (including explicit setting of compiler)

```
CC=/usr/bin/clang CXX=/usr/bin/clang++\
 \
 cmake\
 -DCMAKE_BUILD_TYPE=Release\
 -DCMAKE_PREFIX_PATH=/tmpLocal/\
 -DCMAKE_INSTALL_PREFIX=/tmpLocal/\
 ~/repos/OriNet\
 &&\
 cmake --build . --target all -j `nproc`\
 &&\
 ctest
 &&\
 cpack
```

Package install (debian)

`$ [sudo] apt install ./OriNet*`

Package uninstall

`$ [sudo] apt remove orinet


*Copyright (c) 2024 Stellacore Corporation*
