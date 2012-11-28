#include "CustomKeyHandlerPiercing.h"


bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,
		                      osgGA::GUIActionAdapter &) {
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
        case 'v':
        	if (!scene.isRaveViewer) {
        		scene.rave_viewer = OpenRAVE::RaveCreateViewer(scene.rave->env, "qtcoin");
        		scene.rave->env->AddViewer(scene.rave_viewer);
        		scene.isRaveViewer = true;
        	}
        	scene.rave_viewer->main(true);
    		break;

    	// Piercing stuff:
        case 'm': // Enable suturing needle's piercing
        	scene.sNeedle->togglePiercing();
        	std::cout<<"Is needle piercing? "<<scene.sNeedle->s_piercing<<std::endl;
        	break;
        case 'M': // Enable hole 1's piercing
        	scene.holes[0]->togglePiercing();
        	break;
        case 'c': // Cut cloth at needle tip if piercing
        	scene.cutCloth();
        	break;

        // For plotting
        case 'n': // Plot the needle
        	scene.plotNeedle();
        	break;
        case 'N': // Remove the needle plots
            scene.plotNeedle(true);
            break;
        case 'o': // Plot the holes
        	scene.plotHoles();
        	break;
        case 'O': // Remove hole plots
        	scene.plotHoles(true);
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

        case 't': // generates a kinematic from the cloth and adds to the openrave environment
        		  // Doesn't seem to do anything
        	createKinBodyFromBulletSoftObject(scene.sCloth->cloth, scene.rave);
        	createKinBodyFromBulletBoxObject(scene.table, scene.rave);
        	break;

        // Tests:
        case 'q': // tests Grasping
        	scene.testGrasping();
        	break;
        case 'w': // executes an openrave trajectory
        	scene.testTrajectory2();
        	break;
        case 'e': // executes an openrave trajectory
        	scene.testTrajectory3();
        	break;
        case 'r': // tests circular
        	scene.testCircular();
        	break;
        }
        break;
    }
    return false;
}
