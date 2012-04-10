#include "simulation/simplescene.h"
#include "utils/vector_io.h"
#include "robots/pr2.h"
#include "simulation/config_bullet.h"
#incude <iostream>
using namespace std;

struct LocalConfig : Config {
  static string infile;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("infile", &infile, "input file")); 

  }
};
string LocalConfig::infile = "";

int main(int argc, char* argv[]) {
  SceneConfig::enableIK = false;

  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  if (LocalConfig::infile=="") throw std::runtime_error("must specify output file");

  vector< vector<double> > jointStates = doubleMatFromFile(infile);

 


  Scene scene;

  scene.startViewer();
  scene.setSyncTime(false);
 
  
  vector<Environment::Ptr> envs;
  vector<RaveRobotObject::Ptr> pr2s;
  vector<btRigidBody*> palms;
  
  RaveInstance::Ptr rave(new RaveInstance());
  
  int N = jointStates.size();
  
  for (int i=0; i < N; i++) {
    
    BulletInstance::Ptr bullet(new BulletInstance());
    OSGInstance::Ptr osg(new OSGInstance());    
    Environment::Ptr env(new Environment(bullet, osg));    
    
    const char ROBOT_MODEL_FILE[] = "robots/pr2-beta-sim.dae";
    RaveRobotObject::Ptr pr2(new RaveRobotObject(rave, ROBOT_MODEL_FILE, btTransform::getIdentity(), CONVEX_HULL, true));



    
    envs.push_back(env);
    pr2s.push_back(pr2);
    palms.push_back(pr2->linkMap[pr2->robot->GetLink("l_palm_link")]->rigidBody.get());


  }
    

  for (int t=0; t < 100; t++) {
    
    for (int i = 0; i < N-1; i++) {
      btRigidBody* rb0 = palms[i];
      btRigidBody* rb1 = palms[i+1];
      btVector3 pos0 = rb0->getCenterOfMassPosition();
      btVector3 pos1 = rb1->getCenterOfMassPosition();
      btVector3 vel0 = rb0->getLinearVelocity();
      btVector3 vel1 = rb1->getLinearVelocity();

      float kp = .1;
      float kd = .01;
      btVector3 f01 = kp*(pos1 - pos0) - kd*(vel1 - vel0);
      rb0->applyCentralForce(f01);
      rb1->applyCentralForce(-f01);
    }
    for (int i=0; i < N; i++) envs[i]->step(DT,1,DT);
    scene.viewer->frame();
  }
 
}