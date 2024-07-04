# OriNet - Computations for rigid body orientation network


## Demonstration Programs

Demonstration programs are available in the demo/ subdirectory and
described in the [demo/README.md file](./demo/README.md)

### Build commands

Example build command (including explicit setting of compiler)

```
CC=/usr/bin/clang CXX=/usr/bin/clang++\
 ;\
 cmake\
 -DCMAKE_BUILD_TYPE=Release\
 -DCMAKE_PREFIX_PATH=/tmpLocal/\
 -DCMAKE_INSTALL_PREFIX=/tmpLocal/\
 ~/repos/OriNet\
 &&\
 cmake --build . --target all -j `nproc`\
 &&\
 ctest
```


