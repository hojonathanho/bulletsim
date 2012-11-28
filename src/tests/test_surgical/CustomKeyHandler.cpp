#include "CustomKeyHandler.h"


bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,
		                      osgGA::GUIActionAdapter &) {
	 //std::pair<std::pair<btVector3, btVector3> , std::pair<int, int> > cutInfo;

    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'a':
            scene.leftAction->reset();
            scene.leftAction->toggleAction();
            scene.runAction(scene.leftAction, BulletConfig::dt);
            break;
        case 's':
            scene.rightAction->reset();
            scene.rightAction->toggleAction();
            scene.runAction(scene.rightAction, BulletConfig::dt);
            break;
        case 'f':
            scene.createFork();
            break;
        case 'g':
            scene.swapFork();
            break;

        case 'v':
        	if (!scene.isRaveViewer) {
        		scene.rave_viewer = OpenRAVE::RaveCreateViewer(scene.rave->env, "qtcoin");
        		scene.rave->env->AddViewer(scene.rave_viewer);
        		scene.isRaveViewer = true;
        	}
        	scene.rave_viewer->main(true);
    		break;

        case 't': // generates a kinematic from the cloth and adds to the openrave environment
        	createKinBodyFromBulletSoftObject(scene.sCloth->cloth, scene.rave);
        	createKinBodyFromBulletBoxObject(scene.table, scene.rave);
        	break;

        case 'r': // saves the openrave environment to a file
        	scene.rave->env->Save("/home/ankush/sandbox/rave_suture/suture_env2.xml");
        	break;

        case 'l': // tests Grasping
        	scene.testGrasping();
        	break;

        case 'z': // plots the points on lying on the cut

        	scene.plotcolors.clear();
        	scene.plotpoints.clear();

        	for (int i=0; i < scene.sCloth->cut_nodes1.size(); i += 1) {
        		scene.plotpoints.push_back(scene.sCloth->cloth->softBody->m_nodes[scene.sCloth->cut_nodes1[i]].m_x);
        		scene.plotcolors.push_back(btVector4(2,0,0,1));
        	}

        	for (int i=0; i < scene.sCloth->cut_nodes2.size(); i += 1) {
        		scene.plotpoints.push_back(scene.sCloth->cloth->softBody->m_nodes[scene.sCloth->cut_nodes2[i]].m_x);
        		scene.plotcolors.push_back(btVector4(3,1,0,1));
        	}
        	scene.plot_points->setPoints(scene.plotpoints,scene.plotcolors);
        	break;

        case 'o': // executes an openrave trajectory
        	scene.testTrajectory2();
        	break;

        case 'O': // executes an openrave trajectory
        	scene.testTrajectory3();
        	break;
        case 'T':
        	scene.testCircular();
        	break;
        }
        break;
    }
    return false;
}
