#include "simulation/rope.h"
#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "robots/grabbing.h"
#include "simulation/bullet_io.h"

using boost::shared_ptr;
using boost::static_pointer_cast;
using namespace util;

void singleStep(int i, vector<vector<double> > &joints, vector<int> &inds,
        RaveRobotObject::Ptr pr2, RobotBase::ManipulatorPtr rarm, RobotBase::ManipulatorPtr larm,
        int nLinks, shared_ptr<CapsuleRope> ropePtr,
        Grab *&g, Grab *&g2,
        btDiscreteDynamicsWorld *dynamicsWorld) {

    cout << i << endl;
    vector<double> joint = joints[i];
    pr2->setDOFValues(inds,joint);

    if (i == 160) {
      btVector3 rhpos = util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin();
      cout << rhpos << endl;
      g = new Grab(ropePtr->children[0]->rigidBody.get(),rhpos*METERS,dynamicsWorld);
    }
    if (i > 160) {
      g->updatePosition(util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin()*METERS);
    }


    if (i == 330) {
      btVector3 lhpos = util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin()*METERS;
      cout << lhpos << endl;
      g2 = new Grab(ropePtr->children[nLinks-2]->rigidBody.get(),lhpos*METERS,dynamicsWorld);
    }
    if (i > 330) {
      g2->updatePosition(util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin()*METERS);
    }
}

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
    children[j]->setColor(1,0,0,0.5);
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

  BulletInstance::Ptr bullet2(new BulletInstance);
  OSGInstance::Ptr osg2(new OSGInstance);
  s.osg->root->addChild(osg2->root.get());
  Fork::Ptr fork(new Fork(s.env, bullet2, osg2));
  s.registerFork(fork);
  Grab* fork_g, *fork_g2;
  RaveRobotObject::Ptr fork_pr2 =
      static_pointer_cast<RaveRobotObject>(fork->forkOf(pr2m.pr2));
  if (!fork_pr2) {
      cout << "oh no" << endl;
      return 0;
  }
  RobotBase::ManipulatorPtr fork_rarm(fork_pr2->robot->GetManipulators()[5]);
  RobotBase::ManipulatorPtr fork_larm(fork_pr2->robot->GetManipulators()[7]);
  CapsuleRope::Ptr fork_rope = static_pointer_cast<CapsuleRope>(fork->forkOf(ropePtr));
  if (!fork_rope) {
      cout << ":(" << endl;
      return 0;
  }
  children =  fork_rope->getChildren();
  for (int j=0; j<children.size(); j++) {
    children[j]->setColor(0,0,1,0.5);
  }

  s.startViewer();
  s.setSyncTime(true);

  // play one world at full speed, and play the second at half speed
  for (int i=0; i < joints.size() && !s.viewer.done(); i++) {
    singleStep(i, joints, inds, pr2m.pr2, rarm, larm, nLinks, ropePtr, g, g2, s.env->bullet->dynamicsWorld);
    singleStep(i, joints, inds, fork_pr2, fork_rarm, fork_larm, nLinks, fork_rope, fork_g, fork_g2, bullet2->dynamicsWorld);
    s.step(1/30.,300,.001);
    usleep(10*1000);
  }

  return 0;
}
