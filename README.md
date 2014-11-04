#Simulation environment with Bullet physics

##Installation instructions
Build with cmake as you would normally do:
```
mkdir build_bulletsim
cd build_bulletsim
cmake /path/to/bulletsim
make -j
```

To call bulletsim from python, add the following two paths to your PYTHONPATH:
```
/path/to/bulletsim
/path/to/build_bulletsim/lib
```
