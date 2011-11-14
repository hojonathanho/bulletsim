#include "rope.h"
#include "simplescene.h"
#include "unistd.h"
using boost::shared_ptr;
int main() {

  int nLinks = 20;
  btAlignedObjectArray<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(btVector3(.05*(i-nLinks/2.)+1,0,2));
  }


  shared_ptr <btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1,0,.765))));
  shared_ptr<BulletObject> table(new BoxObject(0,btVector3(.75,.75,.01),ms));

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01));

  Scene s = Scene();
  s.env->add(ropePtr);
  s.env->add(table);


  int step = 0;
  while (!s.viewer.done()) {
    btRigidBody* body = ropePtr->bodies[0].get();
    cout << "position at time " << ++step << ": " << 
      body->getCenterOfMassPosition().x() << " " <<
      body->getCenterOfMassPosition().y() << " " <<
      body->getCenterOfMassPosition().z() << " " << endl;
        
    s.step(1/30.,300,.001);
    // usleep(10*1000);
  }

}
