#include "simulation/rope.h"
#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "robots/grabbing.h"
#include "simulation/bullet_io.h"

using boost::shared_ptr;
using namespace util;

int main(int argc, char *argv[]) {

  SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  SceneConfig::enableRobot = true;
  GeneralConfig::scale = 1.0;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);



  const float table_height = .765;
  const float rope_radius = .01;
  const float segment_len = .025;
  const float table_thickness = .10;
  int nLinks = 50;

  vector<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(METERS*btVector3(.5+segment_len*i,0,table_height+5*rope_radius));
  }


  shared_ptr<BulletObject> table(new BoxObject(0,METERS*btVector3(.75,.75,table_thickness/2),
              btTransform(btQuaternion(0, 0, 0, 1), METERS*btVector3(1,0,table_height-table_thickness/2))));

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01*METERS));

  Scene s;
  PR2Manager pr2m(s);

  s.env->add(ropePtr);
  s.env->add(table);
  //table->setColor(0,1,0,1);

  vector<BulletObject::Ptr> children =  ropePtr->getChildren();
  for (int j=0; j<children.size(); j++) {
    children[j]->setColor(1,0,0,1);
  }


  vector< vector<double> > joints;
  vector< int > inds;

  read_1d_array(inds, EXPAND(BULLETSIM_DATA_DIR) "/inds.txt");
  read_2d_array(joints, EXPAND(BULLETSIM_DATA_DIR) "/vals.txt");

  int step = 0;

  //  btVector3 v = ropePtr->bodies[0]->getCenterOfMassPosition();
  RobotBase::ManipulatorPtr rarm(pr2m.pr2->robot->GetManipulators()[5]);
  RobotBase::ManipulatorPtr larm(pr2m.pr2->robot->GetManipulators()[7]);

  Grab* g;
  Grab* g2;

  s.startViewer();
  s.setSyncTime(true);
  for (int i=0; i < joints.size() && !s.viewer.done(); i++) {
    cout << i << endl;
    vector<double> joint = joints[i];
    pr2m.pr2->setDOFValues(inds,joint);


    if (i == 160) {
      btVector3 rhpos = util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin();
      cout << rhpos << endl;
      g = new Grab(ropePtr->children[0]->rigidBody.get(),rhpos*METERS,s.env->bullet->dynamicsWorld);
    }
    if (i > 160) {
      g->updatePosition(util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin()*METERS);
    }


    if (i == 330) {
      btVector3 lhpos = util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin()*METERS;
      cout << lhpos << endl;
      g2 = new Grab(ropePtr->children[nLinks-2]->rigidBody.get(),lhpos*METERS,s.env->bullet->dynamicsWorld);
    }
    if (i > 330) {
      g2->updatePosition(util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin()*METERS);
    }

    s.step(1/30.,300,.001);
    usleep(10*1000);
  }

}
