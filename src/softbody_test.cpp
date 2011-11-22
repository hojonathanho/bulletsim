#include "simplescene.h"
#include "softbodies.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

void gen(Scene &scene, btScalar s, btScalar z) {
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(scene.env->bullet->dynamicsWorld->getWorldInfo(),
        btVector3(-s,-s,z),
		btVector3(+s,-s,z),
		btVector3(-s,+s,z),
		btVector3(+s,+s,z),
		31,31,
		0/*1+2+4+8*/, true);

	psb->getCollisionShape()->setMargin(0.4);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
//	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2, pm);
	psb->setTotalMass(150);
    psb->setCollisionFlags(0);
    scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
}

int main() {
    Scene scene(false, false);

//    gen(scene, 1, 0.1);
    gen(scene, 2, 1);
    
    scene.viewerLoop();
    return 0;
}
