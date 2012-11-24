#include "CustomScene.h"
#include "CustomKeyHandler.h"


/** Returns the coordinates of the last point below (-z) SOURCE_PT
    on the cloth represented by PSB. */
btVector3 CustomScene::getDownPoint(btVector3 & source_pt,
		                            boost::shared_ptr<btSoftBody> psb,
				                    btScalar radius) {
  int node_idx= -1;
  int min_z = 9999;

  for(int i = 0; i < psb->m_nodes.size(); i += 1) {
    btScalar rad_dist = getXYDistance(source_pt, psb->m_nodes[i].m_x);
    if (rad_dist < radius)
      if (psb->m_nodes[i].m_x.z() < min_z) {
	min_z = psb->m_nodes[i].m_x.z();
	node_idx   = i;
      }
    }
 return psb->m_nodes[node_idx].m_x;
}


/** Returns ||(v1.x, v1.y) - (v2.x, v2.y)||. */
btScalar inline CustomScene::getXYDistance(btVector3 &v1, btVector3 &v2) {
  btVector3 vec(v1.x() - v2.x(), v1.y() - v2.y(), 0.0);
  return vec.length();
}


/** Raycasts from SOURCE to all the nodes of PSB
    and returns a vector of the same size as the nodes of PSB
    depicting whether that node is visible or not. */
std::vector<btVector3> CustomScene::checkNodeVisibility(btVector3 camera_origin,
				      boost::shared_ptr<btSoftBody> psb) {

  plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());

  std::vector<btVector3> plotpoints;
  std::vector<btVector4> color;

  plotpoints.push_back(camera_origin);
  color.push_back(btVector4(3,1,0,1));

  for (int i=0; i < psb->m_nodes.size(); i += 1) {
    btVector3 node_pos = psb->m_nodes[i].m_x;
    btSoftBody::sRayCast sol;
    bool b = psb->rayTest (camera_origin, node_pos, sol);

    if (sol.fraction >= 0.99) { // iff fraction -> 1 : means the ray can hit the node.
      plotpoints.push_back(node_pos);
      color.push_back(btVector4(2,0,0,1));
    }
  }
  plot_points->setPoints(plotpoints,color);
  return plotpoints;
}


/* Creates a square cloth with side length 2s.
   The four coordinates of the cloth are:
   {(s,s,z) (-s,s,z) (-s,-s,z) (s,-s,z)}
   Then, the center of the cloth (initially at (0,0,0)
   is translated to CENTER.*/
BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, btScalar z, btVector3 center,
								  std::vector<int> &cut_nodes1, std::vector<int> &cut_nodes2,
								  bool getCutIndices,
			                      bool shouldCut,
			                      unsigned int resx, unsigned int resy) {

	btVector3 corner1(-s,-s,z), corner2(+s,-s,z), corner3(-s,+s,z), corner4(+s,+s,z);
	btSoftBody* psb=btSoftBodyHelpers::CreatePatch(*(env->bullet->softBodyWorldInfo),
													center + corner1,
													center + corner2,
													center + corner3,
													center + corner4,
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
	if (shouldCut) {
	cutPlane cut (center + 0.05*corner3 + 0.95*corner4,
			      center + 0.05*corner4 + 0.95*corner2,
			      center + 0.05*corner2 + 0.95*corner1,
			      center + 0.05*corner1 + 0.95*corner3,
			      0.25,0.75);

	CutPlaneSoftBody(psb, &cut, 0.001, cut_nodes1, cut_nodes2, getCutIndices);
	}
	return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}



/** Draws the axes at LEFT_GRIPPER1 and LEFT_GRIPPER2. */
void CustomScene::drawAxes() {
  //cut_axes->setup(btT,1);
}


void CustomScene::createFork() {
    bullet2.reset(new BulletInstance);
    bullet2->setDefaultGravity();
    osg2.reset(new OSGInstance);
    osg->root->addChild(osg2->root.get());

    fork.reset(new Fork(env, bullet2, osg2));
    registerFork(fork);

    cout << "forked!" << endl;

    origRobot = pr2m.pr2;
    EnvironmentObject::Ptr p = fork->forkOf(pr2m.pr2);
    if (!p) {
        cout << "failed to get forked version of robot!" << endl;
        return;
    }
    tmpRobot = boost::static_pointer_cast<RaveRobotObject>(p);
    cout << (tmpRobot->getEnvironment() == env.get()) << endl;
    cout << (tmpRobot->getEnvironment() == fork->env.get()) << endl;
}


void CustomScene::swapFork() {
    // swaps the forked robot with the real one
    cout << "swapping!" << endl;
    int leftidx = pr2m.pr2Left->index;
    int rightidx = pr2m.pr2Right->index;
    origRobot.swap(tmpRobot);
    pr2m.pr2 = origRobot;
    pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
    pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);

/*    vector<int> indices; vector<dReal> vals;
    for (int i = 0; i < tmpRobot->robot->GetDOF(); ++i) {
        indices.push_back(i);
        vals.push_back(0);
    }
    tmpRobot->setDOFValues(indices, vals);*/
}


/** small test to test the controller. */
void CustomScene::testTrajectory() {

	pr2m.pr2->robot->SetActiveManipulator("rightarm");

	ModuleBasePtr basemodule = RaveCreateModule(rave->env,"BaseManipulation");
	rave->env->Add(basemodule,true,pr2m.pr2->robot->GetName());

	Transform rightT = pr2m.pr2Right->manip->GetEndEffectorTransform();
	rightT.trans += Vector(0.0,0,-0.35);

	// see if trajectory can be extracted: Yes it can be!
	TrajectoryBasePtr hand_traj;
	hand_traj = RaveCreateTrajectory(rave->env,"");

	stringstream ssout, ssin; ssin << "MoveToHandPosition outputtraj execute 0 poses 1  " << rightT;
	basemodule->SendCommand(ssout,ssin);
	hand_traj->deserialize(ssout);

	printf("TRAJECTORY INFO : DURATION : %f\n",(float) hand_traj->GetDuration());

	RaveTrajectory::Ptr traj(new RaveTrajectory(hand_traj, pr2m.pr2, pr2m.pr2Right->manip->GetArmIndices()));
	pr2m.controller->appendTrajectory(traj);
	pr2m.controller->run();
}


/** small test to test the planner and the controller. */
void CustomScene::testTrajectory2() {
	//WayPointsPlanner planner1(pr2m.pr2, rave, 'r');
	IKInterpolationPlanner ikPlanner(pr2m, rave, 'r');


	pr2m.pr2->robot->SetActiveManipulator("rightarm");
	Transform rightT = pr2m.pr2Right->manip->GetEndEffectorTransform();
	std::vector<Transform> t;

	//visualize
	plotcolors.clear();
	plotpoints.clear();

	for(int i =0; i < 30; i+=1) {
		Transform t1 = rightT;
		t1.trans += Vector(0.0001*i*i,0,-0.005*i);
		t.push_back(t1);

		// visual aid
		plotpoints.push_back(util::toBtVector(GeneralConfig::scale*t1.trans));
		plotcolors.push_back(btVector4(2,0,0,1));
	}
	plot_points->setPoints(plotpoints,plotcolors);


	std::pair<bool, RaveTrajectory::Ptr> res = ikPlanner.plan(t);
	if (res.first) {
		pr2m.controller->appendTrajectory(res.second);
		pr2m.controller->run();
	} else {
		std::cout<<"Plan failed!"<<std::endl;
	}
}



/* Sets up the scene and UI even handlers,
 * initializes various structures.*/
void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = BulletConfig::dt;
    const float table_height = .5;
    const float table_thickness = .05;
    table = BoxObject::Ptr(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
                                         btTransform(btQuaternion(0, 0, 0, 1),
                        		                     GeneralConfig::scale * btVector3(0.85, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(10);

    sCloth.reset(new SutureCloth(*this,GeneralConfig::scale * 0.5, 0, GeneralConfig::scale * btVector3(0.6, 0, table_height+0.01)));

    btSoftBody * const psb = sCloth->cloth->softBody.get();
    pr2m.pr2->ignoreCollisionWith(psb);
    pr2m.setArmPose("side", 'b');
    pr2m.setTorso(1);

    env->add(table);
    env->add(sCloth->cloth);

    // set up the points for plotting
    plot_points.reset(new PlotPoints(5));
    env->add(plot_points);

    std::pair<btVector3, btVector3> cutInfo = sCloth->fitLine(1);
    btTransform cutT = util::getOrthogonalTransform(cutInfo.second);
    cutT.setOrigin( cutInfo.first);

    plot_axes.reset(new PlotAxes());
	plot_axes1.reset(new PlotAxes());
	plot_axes->setup(btTransform(), 2);



    plot_axes->setup(cutT, 2);
    env->add(plot_axes);
    env->add(plot_axes1);

    leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left,
    		                                      "l_gripper_l_finger_tip_link",
    		                                      "l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(sCloth->cloth);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right,
    		                                       "r_gripper_l_finger_tip_link",
    		                                       "r_gripper_r_finger_tip_link", 1));
    rightAction->setTarget(sCloth->cloth);

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);

    startFixedTimestepLoop(dt);
}



/** Constructor for the cloth in the scene. */
CustomScene::SutureCloth::SutureCloth(CustomScene &scene, btScalar side_length, btScalar z, btVector3 center) {
		cloth = scene.createCloth(side_length, z, center, cut_nodes1, cut_nodes2);
}


/** Returns the line of maximum variance of the cut-points
 * Performs a PCA on the points.
 * SIDE_NUM  \in {1, 2} : if 1 : cut-points on the left.
 *                        if 2 : cut-points on the right.
 * The return value is a point on the line found and the direction-cosine of the line. */
std::pair<btVector3, btVector3> CustomScene::SutureCloth::fitLine(int side_num) {

	std::vector<int> &pt_vector = (side_num==1)? cut_nodes1 : cut_nodes2;
	unsigned int N = pt_vector.size();
	Eigen::MatrixXf X(N, 3);

	for (int i=0; i< N; i+=1) {
		btVector3 node = cloth->softBody->m_nodes[pt_vector.at(i)].m_x;
		Eigen::Vector3f pt(node.getX(),node.getY(),node.getZ());
		X.row(i) = pt;
	}

	Eigen::RowVector3f mean = X.colwise().sum()/N;
	Eigen::MatrixXf X_centered = (X.rowwise() - mean);

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(X_centered / sqrt(N), Eigen::ComputeFullV);
	Eigen::Vector3f pca1 = svd.matrixV().col(0);

	btVector3 pt_on_line(mean(0), mean(1), mean(2));
	btVector3 direction_cosine(pca1(0), pca1(1), pca1(2));

	return std::pair<btVector3, btVector3>(pt_on_line, direction_cosine);
}
