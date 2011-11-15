#include "rope.h"
#include "simplescene.h"
#include "unistd.h"
#include "file_reading.h"
#include "grabbing.h"

using boost::shared_ptr;

int main() {

  const float table_height = .765;
  const float rope_radius = .01;

  int nLinks = 30;
  btAlignedObjectArray<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(btVector3(.5+.05*i,0,table_height+rope_radius));
  }


  shared_ptr <btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1,0,table_height-.03))));
  shared_ptr<BulletObject> table(new BoxObject(0,btVector3(.75,.75,.03),ms));

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01));

  Scene s = Scene();
  s.env->add(ropePtr);
  s.env->add(table);

  vector< vector<double> > joints;
  vector< int > inds;
  read_1d_array(inds, "../data/inds.txt");
  read_2d_array(joints,"../data/vals.txt");

  int step = 0;

  //  btVector3 v = ropePtr->bodies[0]->getCenterOfMassPosition();
  RobotBase::ManipulatorPtr rarm(s.pr2->robot->GetManipulators()[5]);
  RobotBase::ManipulatorPtr larm(s.pr2->robot->GetManipulators()[7]);


  Grab g;
  Grab g2;
  for (int i=0; i < joints.size() && !s.viewer.done(); i++) {
    cout << i << endl;
    vector<double> joint = joints[i];
    s.pr2->setDOFValues(inds,joint);

    

    if (i == 160) {
      btVector3 rhpos = util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin();
      g = Grab(ropePtr->bodies[0].get(),rhpos,s.env->bullet->dynamicsWorld);
    }
    if (i > 160) {
      g.updatePosition(util::toBtTransform(rarm->GetEndEffectorTransform()).getOrigin());
    }


    if (i == 330) {
      btVector3 lhpos = util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin();
      g2 = Grab(ropePtr->bodies[nLinks-2].get(),lhpos,s.env->bullet->dynamicsWorld);
    }
    if (i > 330) {
      g2.updatePosition(util::toBtTransform(larm->GetEndEffectorTransform()).getOrigin());
    }



    /*
    btRigidBody* body = ropePtr->bodies[0].get();
    cout << "position at time " << ++step << ": " << 
      body->getCenterOfMassPosition().x() << " " <<
      body->getCenterOfMassPosition().y() << " " <<
      body->getCenterOfMassPosition().z() << " " << endl;
    */

    s.step(1/30.,300,.001);
    // usleep(10*1000);
  }

}
