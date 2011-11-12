#include "rope.h"
#include "simplescene.h"
#include "unistd.h"

int main() {

  int nLinks = 20;
  btAlignedObjectArray<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(btVector3(.25*i,0,10));
  }


  boost::shared_ptr<CapsuleRope> ropePtr;
  ropePtr.reset(new CapsuleRope(ctrlPts,.5));

  Scene s = Scene();
  s.env->add(ropePtr);



  s.viewer.realize();
  int step = 0;
  while (!s.viewer.done()) {
    cout << "height at time " << ++step << ": " << ropePtr->bodies[0]->getCenterOfMassPosition().z() << endl;
    usleep(1000*10);
    s.step();
  }

}
