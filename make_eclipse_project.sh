rm -rf $BULLETSIM_BUILD_DIR/bulletsim_eclipse
mkdir $BULLETSIM_BUILD_DIR/bulletsim_eclipse
cd $BULLETSIM_BUILD_DIR/bulletsim_eclipse
cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON $BULLETSIM_SOURCE_DIR
