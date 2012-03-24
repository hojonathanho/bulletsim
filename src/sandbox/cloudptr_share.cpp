#include <iostream> 
#include <fstream> 

#include "clouds/utils_pcl.h"

using namespace std;

int main() {
    ColorCloudPtr cloud = readPCD("/home/joschu/bulletsim/src/sandbox/towel.pcd");
    ColorCloud* a = cloud.get();
    ColorCloudPtr cloud2(a);
    
    
}