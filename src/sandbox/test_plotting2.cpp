#include "simulation/plotting.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include <iostream>
#include <cstdlib>
using namespace std;
using namespace osg;

float randf() {return (float)rand()/(float)RAND_MAX;}

int main(int argc, char* argv[]) {
    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.read(argc, argv);

    Scene s;
    PlotSpheres::Ptr spheres(new PlotSpheres());
    s.env->add(spheres);

    
    ref_ptr<Vec3Array> centers = new Vec3Array();
    ref_ptr<Vec4Array> colors = new Vec4Array();
    vector<float>* sizes = new vector<float>();

    for (int i=0; i < 5; i++) {
        centers->push_back(Vec3(randf()*10, randf()*10, randf()*10));
        colors->push_back(Vec4(randf(), randf(), randf(), randf()));        
        sizes->push_back(randf());
    }
    

    spheres->plot(centers,colors,*sizes);    
    s.startViewer();
    s.idle(true);


    cout << "now showing second batch..." << endl;
    centers->clear();    
    colors->clear();
    sizes->clear();
    for (int i=0; i < 10; i++) {
        centers->push_back(Vec3(randf(), randf(), randf()));
        colors->push_back(Vec4(randf(), randf(), randf(), randf()));        
        sizes->push_back(randf());
    }    
    spheres->plot(centers,colors,*sizes);    

    s.startLoop();

}
