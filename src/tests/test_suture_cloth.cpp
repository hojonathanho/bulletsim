#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/softBodyHelpers.h"
#include "simulation/basicobjects.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include "CustomScene.h"

/*void createCloth(Scene &scene, btScalar s, btScalar z,
		 unsigned int resx = 50, unsigned int resy =50) {
	btVector3 corner1(-s,-s,z), corner2(+s,-s,z), corner3(-s,+s,z), corner4(+s,+s,z);
	btSoftBody* psb=btSoftBodyHelpers::CreatePatch(*scene.env->bullet->softBodyWorldInfo,
													corner1,
													corner2,
													corner3,
													corner4,
													resx, resy,
													1+2+4+8, true);

	psb->getCollisionShape()->setMargin(0.4);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
    //	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->m_cfg.kSRHR_CL = 1;
	psb->m_cfg.kSSHR_CL = 1;
	psb->generateBendingConstraints(2, pm);
	psb->setTotalMass(150);

	// cut the soft-body
	cutPlane cut (0.05*corner4 + 0.95*corner2,
			      0.05*corner2 + 0.95*corner1,
			      0.05*corner1 + 0.95*corner3,
			      0.05*corner3 + 0.95*corner4,
			      0.25,0.75);
	cutPlaneSoftBody(psb, &cut, 0.001);
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
    createCloth(scene, 10, 0);

    scene.startViewer();
    scene.startLoop();
    return 0;
}*/


int main(int argc, char *argv[]) {
	GeneralConfig::scale = 20.;
	ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
	BulletConfig::dt = 0.01;
	BulletConfig::internalTimeStep = 0.01;
	BulletConfig::maxSubSteps = 0;

	Parser parser;

	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.read(argc, argv);

	CustomScene().run();
	return 0;
}

