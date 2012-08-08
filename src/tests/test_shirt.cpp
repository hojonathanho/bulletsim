#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include <iostream>
#include <fstream>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include "simulation/softbodies.h"
//#include <boost/filesystem.hpp>

//daniel's code for importing shirt

using namespace std;

const string data_dir = EXPAND(BULLETSIM_DATA_DIR)"/clothing/";
#define DATA(str) (data_dir+str).c_str()

int main(int argc, char* argv[]) {
    //Open files
    ifstream ele_f, face_f, node_f;
    ele_f.open(DATA("shirt.1.ele"));
    face_f.open(DATA("shirt.1.face"));
    node_f.open(DATA("shirt.1.node"));

    //Read them into strings
    string temp;
    string ele, face, node;
    while (ele_f.good()) {
        getline(ele_f, temp);
        ele.insert(ele.length(), temp);
        ele.insert(ele.length(), "\n");
    }
   
    while (face_f.good()) {
        getline(face_f, temp);
        face.insert(face.length(), temp);
        face.insert(face.length(), "\n");
    }
   
    while (node_f.good()) {
        getline(node_f, temp);
        node.insert(node.length(), temp);
        node.insert(node.length(), "\n");
    }
   
    GeneralConfig::scale = 10.;
    SceneConfig::enableRobot=SceneConfig::enableIK=false;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene s;
    //Create your psb
    btSoftBody* psb=btSoftBodyHelpers::CreateFromTetGenData(*s.env->bullet->softBodyWorldInfo,
                                                            ele.c_str(),
                                                            face.c_str(),
                                                            node.c_str(),
                                                            true,true,true);
    //Close the files
    s.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
    ele_f.close(); face_f.close(); node_f.close();
    s.startViewer();
    s.startLoop();
}
