rm -rf ~/build/bulletsim_eclipse
mkdir ~/build/bulletsim_eclipse
cd ~/build/bulletsim_eclipse
cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DBUILD_PERCEPTION=OFF -DJOHNS_ADVENTURES=OFF ~/rll/bulletsim
