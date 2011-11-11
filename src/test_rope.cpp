#include "rope.h"
#include "simplescene.h"
#include "unistd.h"

int main() {

  int nLinks = 20;
  btAlignedObjectArray<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(btVector3(.25*i,0,1));
  }


  boost::shared_ptr<CapsuleRope> ropePtr;
  ropePtr.reset(new CapsuleRope(ctrlPts,.5f));

  Scene s = Scene();
  s.env->add(ropePtr);



  s.viewer.realize();
  while (!s.viewer.done()) {
    //cout << "height at time " << i << ": " << ropePtr->bodies[0]->getCenterOfMassPosition().z() << endl;
    //usleep(1000*10);
    s.step();
  }

}
