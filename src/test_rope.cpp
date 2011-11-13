#include "rope.h"
#include "simplescene.h"
#include "unistd.h"

int main() {

  int nLinks = 20;
  btAlignedObjectArray<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(btVector3(1*(i-nLinks/2.),0,2));
  }


  boost::shared_ptr<CapsuleRope> ropePtr;
  ropePtr.reset(new CapsuleRope(ctrlPts,.25));

  Scene s = Scene();
  s.env->add(ropePtr);



  s.viewer.realize();
  int step = 0;
  while (!s.viewer.done()) {
    btRigidBody* body = ropePtr->bodies[0].get();
    cout << "position at time " << ++step << ": " << 
      body->getCenterOfMassPosition().x() << " " <<
      body->getCenterOfMassPosition().y() << " " <<
      body->getCenterOfMassPosition().z() << " " << endl;
        
    s.step(.01);
    usleep(100*1000);
  }

}
