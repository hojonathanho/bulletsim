rm -rf ~/build/bulletsim_eclipse
mkdir ~/build/bulletsim_eclipse
cd ~/build/bulletsim_eclipse
cmake -G"Eclipse CDT4 - Unix Makefiles" -DJOHNS_ADVENTURES=ON -DGUROBI_HOME=/home/joschu/Src/gurobi501/linux64 ~/bulletsim
