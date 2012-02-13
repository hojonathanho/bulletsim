#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

void gen(Scene &scene, btScalar s, btScalar z) {
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(scene.env->bullet->softBodyWorldInfo,
        btVector3(-s,-s,z),
		btVector3(+s,-s,z),
		btVector3(-s,+s,z),
		btVector3(+s,+s,z),
		31, 31,
		0/*1+2+4+8*/, true);

	psb->getCollisionShape()->setMargin(0.4);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
//	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2, pm);
	psb->setTotalMass(150);
    scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 10.;    

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene scene;
//    gen(scene, 1, 0.1);
    gen(scene, 5, 30);

    scene.startViewer();
    scene.startLoop();
    return 0;
}
