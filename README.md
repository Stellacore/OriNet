# OriNet - Network of rigid body orientations


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


