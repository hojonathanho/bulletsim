#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>
#include <btBulletCollisionCommon.h>
#include <iostream>
using namespace std;

int main() {

	btBoxShape* box0 = new btBoxShape(btVector3(1,1,1));
	btBoxShape* box1 = new btBoxShape(btVector3(1,1,1));
	btTransform tf0(btQuaternion::getIdentity(),btVector3(0,0,0));
	btTransform tf1(btQuaternion::getIdentity(), btVector3(3,0,0));
	btGjkEpaSolver2::sResults results;
	btVector3 comdiff = tf1.getOrigin() - tf0.getOrigin();
	btGjkEpaSolver2::SignedDistance(box0, tf0, box1, tf1, comdiff, results);
	cout << "distance: " << results.distance << endl;
	cout << "normal: " << results.normal.x() << " "
										 << results.normal.y() << " "
										 << results.normal.y() << endl;
}
