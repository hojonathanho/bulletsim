
#include "CustomKeyHandler.h"


bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,
		osgGA::GUIActionAdapter &) {

	vector<double> dofs;

	boost::shared_ptr<CapsuleRope> rope = scene.sPeg->ropePtr;
	btTransform iT = btTransform::getIdentity();
	const int N    = rope->children.size();
	const float rope_radius=.0006, segment_len=0.003;


	switch (ea.getEventType()) {
	case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		case 'A':
			cout <<"N : "<<N<<endl;
			for(int i=0; i< N; i+=1) {
				iT.setOrigin(METERS*btVector3(0, segment_len*(i - N/2.0), 2*rope_radius));
				rope->children[i]->motionState->setWorldTransform(iT);
				cout <<"   isKinematic: "<<rope->children[i]->isKinematic<<endl;
			}
			break;

		case 'a':
			scene.lAction->reset();
			scene.lAction->toggleAction();
			scene.runAction(scene.lAction, BulletConfig::dt);
			break;
		case 's':
			scene.rAction->reset();
			scene.rAction->toggleAction();
			scene.runAction(scene.rAction, BulletConfig::dt);
			break;

		case 'W':
			if (!scene.isRaveViewer) {
				scene.rave_viewer = OpenRAVE::RaveCreateViewer(scene.rave->env, "qtcoin");
				scene.rave->env->AddViewer(scene.rave_viewer);
				scene.isRaveViewer = true;
			}
			scene.rave_viewer->main(true);
			break;

		case 't': // generates a kinematic body from the cloth and adds to the openrave environment
			//createKinBodyFromBulletSoftObject(scene.sCloth->cloth, scene.rave);
			createKinBodyFromBulletBoxObject(scene.table, scene.rave);
			break;

		case 'L': // saves the openrave environment to a file
			scene.rave->env->Save("/home/ankush/sandbox/rave_suture/suture_env2.xml");
			break;

		//case '&': {
		//	btTransform table_tfm;
		//	scene.table->motionState->getWorldTransform(table_tfm);
		//	table_tfm.getOrigin() += btVector3(0,0,1)*METERS;
		//	if (scene.sNeedle->s_needle->getChildren()[0]->isKinematic)
		//		scene.sNeedle->s_needle->getChildren()[0]->motionState->setKinematicPos(table_tfm);
		//	else
		//		scene.sNeedle->s_needle->getChildren()[0]->motionState->setWorldTransform(table_tfm);
			//scene.sNeedle->s_needle->body->SetTransform(util::toRaveTransform(table_tfm, 1.0f/METERS));
		//	break;
		//}

		case 'z':
			dofs = scene.ravens.ravens->getDOFValues();
			for (int c=0; c<dofs.size(); c+=1)
				std::cout<<" "<<dofs[c];
			std::cout<<std::endl;
			break;
			/*
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
			 */
		case 'o': // executes an openrave trajectory
			scene.testTrajectory2();
			break;

		case 'O': // executes an openrave trajectory
			//scene.testTrajectory3();
			break;
		case 'K':
			scene.recordPoints();
			break;
		case 'I':
			scene.ravens.setArmPose("home",'b');
			break;

			// Recording/Playback stuff
		case 'T':
			scene.jRecorder->toggleRecording();
			break;
		case 'l':
			scene.jPlayback->toggleEnabled();
			break;
		//case ']':
		//	scene.plotGrasp();
		//	break;
		case '-':
			scene.plotAllPoints();
			break;
		//case '[':
		//	scene.plotNeedle();
		//	break;
		//case '=':
		//	scene.testNeedle2();
		//	break;
		//case '+':
		//	scene.testGrab();
		//	break;
		case '_':
			scene.plotHoleTfm();
			break;
		case ')':
			scene.plotPeg();
			break;
		case '(':
			scene.reset();
			break;
		case '?':
			scene.jPlayback->reset();
			break;
		case '\\':
			scene.jPlayback->toggleLfd();
			break;

			/******************************* RIGHT LOCAL **********************************/
		case 'e':
			scene.moveEndEffector('f',false,'r');
			break;
		case 'r':
			scene.moveEndEffector('b',false,'r');
			break;
		case 'd':
			scene.moveEndEffector('u',false,'r');
			break;
		case 'f':
			scene.moveEndEffector('d',false,'r');
			break;
		case 'c':
			scene.moveEndEffector('l',false,'r');
			break;
		case 'v':
			scene.moveEndEffector('r',false,'r');
			break;
			/******************************* LEFT LOCAL **********************************/
		case 'y':
			scene.moveEndEffector('f',false,'l');
			break;
		case 'u':
			scene.moveEndEffector('b',false,'l');
			break;
		case 'h':
			scene.moveEndEffector('u',false,'l');
			break;
		case 'j':
			scene.moveEndEffector('d',false,'l');
			break;
		case 'n':
			scene.moveEndEffector('l',false,'l');
			break;
		case 'm':
			scene.moveEndEffector('r',false,'l');
			break;

			/******************************* RIGHT WORLD **********************************/
		case 'E':
			scene.moveEndEffector('f',true,'r');
			break;
		case 'R':
			scene.moveEndEffector('b',true,'r');
			break;
		case 'D':
			scene.moveEndEffector('u',true,'r');
			break;
		case 'F':
			scene.moveEndEffector('d',true,'r');
			break;
		case 'C':
			scene.moveEndEffector('l',true,'r');
			break;
		case 'V':
			scene.moveEndEffector('r',true,'r');
			break;
			/******************************* LEFT WORLD **********************************/
		case 'Y':
			scene.moveEndEffector('f',true,'l');
			break;
		case 'U':
			scene.moveEndEffector('b',true,'l');
			break;
		case 'H':
			scene.moveEndEffector('u',true,'l');
			break;
		case 'J':
			scene.moveEndEffector('d',true,'l');
			break;
		case 'N':
			scene.moveEndEffector('l',true,'l');
			break;
		case 'M':
			scene.moveEndEffector('r',true,'l');
			break;
			/*****************************************************************************/
		}
		break;
	}
	return false;
}
