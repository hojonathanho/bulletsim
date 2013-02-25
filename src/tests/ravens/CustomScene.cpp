#include "CustomScene.h"
#include "CustomKeyHandler.h"


/** Rotates a transform TFM by AND, around the point along
 * x-axis of the transform at a distance RAD away.*/
btTransform rotateByAngle (btTransform &tfm, const float ang, const float rad) {
	float pi = 3.14159265, r = rad*GeneralConfig::scale;

	btTransform WorldToEndEffectorTransform = tfm;

	btTransform initT;
	initT.setIdentity();
	initT.setOrigin(r*btVector3(-1,0,0));

	OpenRAVE::Transform T = OpenRAVE::geometry::matrixFromAxisAngle(OpenRAVE::Vector(0,0,ang));
	btTransform bT = util::toBtTransform(T);
	bT.setOrigin(r*btVector3(1,0,0));

	return WorldToEndEffectorTransform*bT*initT;
}


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
   Then, the center of the cloth (initially at (0,,0)
   is translated to CENTER.*/
BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, btScalar z, btVector3 center,
								  std::vector<int> &cut_nodes1, std::vector<int> &cut_nodes2,
								  bool getCutIndices,
			                      bool shouldCut,
			                      unsigned int resx, unsigned int resy) {

	z += 0.05*METERS;
	resx = 80;
	resy = 80;
	btVector3 corner1(-s,-s,z), corner2(+s,-s,z), corner3(-s,+s,z), corner4(+s,+s,z);
	btSoftBody* psb=btSoftBodyHelpers::CreatePatch(*(env->bullet->softBodyWorldInfo),
													center + corner1,
													center + corner2,
													center + corner3,
													center + corner4,
													resx, resy,
													1+2+4+8, true);

	btSoftBody::Material* pm=psb->appendMaterial();
	psb->setTotalMass(1000);

	// cut the soft-body
	if (shouldCut) {
		cutPlane cut (center + 0.05*corner3 + 0.95*corner4,
				      center + 0.05*corner4 + 0.95*corner2,
				      center + 0.05*corner2 + 0.95*corner1,
				      center + 0.05*corner1 + 0.95*corner3,
				      0.25,0.75);

		CutPlaneSoftBody(psb, &cut, 0.001, cut_nodes1, cut_nodes2, getCutIndices);
	}

	psb->generateBendingConstraints(2, psb->m_nodes[0].m_material);
	pm->m_kLST	=	1.0;
	psb->generateClusters(1e3);
	psb->getCollisionShape()->setMargin(0.002*METERS);

	psb->m_cfg.collisions	=	0;
	psb->m_cfg.collisions += btSoftBody::fCollision::CL_SELF;
	psb->m_cfg.collisions += btSoftBody::fCollision::SDF_RS;

    psb->m_cfg.kDF = 1;
    psb->m_cfg.piterations = 50;
    psb->m_cfg.viterations = 50;
    psb->m_cfg.citerations = 50;
    psb->m_cfg.diterations = 50;
    psb->randomizeConstraints();

	return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}



/** Draws the axes at LEFT_GRIPPER1 and LEFT_GRIPPER2. */
void CustomScene::drawAxes() {
  //cut_axes->setup(btT,1);
}


/** small test to test the controller. */
void CustomScene::testTrajectory() {

	ravens.ravens->robot->SetActiveManipulator("r_arm");

	ModuleBasePtr basemodule = RaveCreateModule(rave->env,"BaseManipulation");
	rave->env->Add(basemodule,true,ravens.ravens->robot->GetName());

	Transform rightT = ravens.manipR->manip->GetEndEffectorTransform();
	rightT.trans += Vector(0.0,0,-0.02);

	// see if trajectory can be extracted: Yes it can be!
	TrajectoryBasePtr hand_traj;
	hand_traj = RaveCreateTrajectory(rave->env,"");

	stringstream ssout, ssin; ssin << "MoveToHandPosition outputtraj execute 0 poses 1  " << rightT;
	basemodule->SendCommand(ssout,ssin);
	hand_traj->deserialize(ssout);

	printf("TRAJECTORY INFO : DURATION : %f\n",(float) hand_traj->GetDuration());

	RaveTrajectory::Ptr traj(new RaveTrajectory(hand_traj, ravens.ravens, ravens.manipR->manip->GetArmIndices()));
	ravens.controller->appendTrajectory(traj);
	ravens.controller->run();
}


/** small test to test the planner and the controller. */
void CustomScene::testTrajectory2() {
	//WayPointsPlanner planner1(pr2m.pr2, rave, 'r');
	IKInterpolationPlanner ikPlanner(ravens, rave, 'r');


	ravens.ravens->robot->SetActiveManipulator("r_arm");
	Transform rightT = ravens.manipR->manip->GetEndEffectorTransform();

	std::vector<Transform> t;

	//visualize
	plotcolors.clear();
	plotpoints.clear();

	for(int i =0; i < 10; i+=1) {
		Transform t1 = rightT;
		t1.trans += Vector(0.0001*(i+1)*(i+1),0,-0.005*(i+1));

		t.push_back(t1);

		// visual aid
		plotpoints.push_back(util::toBtVector(GeneralConfig::scale*t1.trans));
		plotcolors.push_back(btVector4(2,0,0,1));
	}
	plot_points->setPoints(plotpoints,plotcolors);

	std::pair<bool, RaveTrajectory::Ptr> res = ikPlanner.smoothPlan(t);
	if (res.first) {
		ravens.controller->appendTrajectory(res.second);
		ravens.controller->run();
	} else {
		std::cout<<"Plan failed!"<<std::endl;
	}
}

/** Test circular. */
void CustomScene::testCircular () {}

/** Small test for plotting grasping relating things. */
void CustomScene::plotGrasp (bool remove) {
	plot_points->setPoints(std::vector<btVector3>(),std::vector<btVector4>());

	btVector3 pt1 = leftAction->getVec1(false);
	btVector3 vec = leftAction->getVec2(false);
	btTransform tfm = util::getOrthogonalTransform(vec);
	//tfm.setOrigin(pt1);

	std::vector<btVector3> plotpoints;
	std::vector<btVector4> color;

	if (!remove) {
		plotpoints.push_back(pt1);
		color.push_back(btVector4(1,0,0,1));

		plotpoints.push_back(vec);
		color.push_back(btVector4(1,0,0,1));

		//plotpoints.push_back(pt2);
		//color.push_back(btVector4(0,0,1,1));

		//plotpoints.push_back(pt3);
		//color.push_back(btVector4(1,0,0,1));

		//plotpoints.push_back(pt4);
		//color.push_back(btVector4(0,0,1,1));

		//util::drawAxes(tfm, 2, env);
	}
	else {
		plotpoints.push_back(btVector3(0,0,0));
		color.push_back(btVector4(0,0,0,0));
	}

	plot_points->setPoints(plotpoints,color);

}


/** small test to test the smooth planning of IK planner. */
void CustomScene::moveEndEffector(char dir, bool world, char lr, float step) {
	assert(("unknown direction to move in; should be in {'f','b','u','d','l','r'}",
			(dir=='f'||dir=='b'||dir=='u'||dir=='d'||dir=='r'||dir=='l')));

	IKInterpolationPlanner::Ptr ikPlanner = (lr=='l')? ikPlannerL : ikPlannerR;

	std::pair<bool, RaveTrajectory::Ptr> res = (world)? ikPlanner->goInWorldDirection(dir,step) : ikPlanner->goInDirection(dir,step);
	if (res.first) {
		ravens.controller->appendTrajectory(res.second);
		ravens.controller->run();
	} else {
		std::cout<<"Plan failed!"<<std::endl;
	}
}


/* Sets up the scene and UI event handlers,
 * initializes various structures.*/
void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    BulletConfig::internalTimeStep = 0.001;
    const float dt = BulletConfig::dt;


    // add a table
    const float table_height = 0.15;
    const float table_thickness = .05;
    table = BoxObject::Ptr(new BoxObject(0, GeneralConfig::scale * btVector3(0.23,0.23,table_thickness/2),
                                         btTransform(btQuaternion(0, 0, 0, 1),
                        		                     GeneralConfig::scale * btVector3(0, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(10);
    env->add(table);
    table->setColor(0.62, 0.32, 0.17, 1.0);
    createKinBodyFromBulletBoxObject(table, rave);

    // add a needle
    sNeedle.reset(new SuturingNeedle(this));
    ravens.ravens->ignoreCollisionWith(sNeedle->s_needle->getChildren()[0]->rigidBody.get());
    env->add(sNeedle->s_needle);
    rave->env->AddKinBody(sNeedle->s_needle->body);

    // add a cloth
    sCloth.reset(new SutureCloth(*this,GeneralConfig::scale * 0.05, 0, GeneralConfig::scale * btVector3(0, 0, table_height+0.01)));
    btSoftBody * const psb = sCloth->cloth->softBody.get();
    env->add(sCloth->cloth);
    sCloth->cloth->setColor(0.933,0.807,0.701,1.0);



    // position the ravens
    btTransform T;
    T.setIdentity();
    T.setOrigin(btVector3(0,0,0.05));
    ravens.applyTransform(util::toRaveTransform(T));


    // set up the points for plotting
    plot_points.reset(new PlotPoints(5));
    env->add(plot_points);
    plot_axes1.reset(new PlotAxes());
    env->add(plot_axes1);
    plot_axes2.reset(new PlotAxes());
    env->add(plot_axes2);

    //addPreStepCallback(boost::bind(&jointRecorder::recordCallback, j_recorder.get()));

    leftAction.reset(new SoftBodyGripperAction( ravens.manipL,
                                               	"l_grasper2_L",
                                               	"l_grasper1_L", 1));
    leftAction->setTarget(sCloth->cloth);
    rightAction.reset(new SoftBodyGripperAction( ravens.manipR,
    		   	   	   	   	   	   	   	   	   	 "r_grasper2_L",
    		   	   	   	   	   	   	   	   	   	 "r_grasper1_L", 1));
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


/** Small test to see if the robot can grasp the cloth.*/
void CustomScene::testGrasping() {}




/** Constructor for the cloth in the scene. */
CustomScene::SutureCloth::SutureCloth(CustomScene &scene, btScalar side_length, btScalar z, btVector3 center) {
		cloth = scene.createCloth(side_length, z, center, cut_nodes1, cut_nodes2);
}

/** Returns the line of maximum variance of the cut-points
 * Performs a PCA on the points.
 * SIDE_NUM  \in {1, 2} : if 1 : cut-points on the left.
 *                        if 2 : cut-points on the right.
 * The return value is a point on the line found and the direction-cosine of the line.
 * Further, it returns 2 ints which are the indices of the extremum points of the cut. */
pair<pair<btVector3, btVector3> , pair<int, int> > CustomScene::SutureCloth::fitLine(int side_num) {

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

	/** Find the extremum nodes of the cut : node closest to the pr2,
	 *     and the node farthest from the pr2. */
	Eigen::VectorXf proj = X_centered * pca1;
	Eigen::MatrixXf::Index maxIdx, minIdx;
	proj.maxCoeff(&maxIdx);
	proj.minCoeff(&minIdx);
	//std::cout<<"Max Index: "<<(int) maxIdx<<" Min Index: "<< (int) minIdx<<std::endl;

	// build the return structure
	pair<btVector3, btVector3> lineInfo(pt_on_line, direction_cosine);
	pair<int, int> extremaIndices((int) minIdx, (int) maxIdx);
	return pair<pair<btVector3, btVector3> , pair<int, int> >(lineInfo, extremaIndices);
}





/** See the doc for fitLine.
 *  In addition to fitting a line to the cut-points this function aligns
 *  the direction of the cut with the x-axis of the robot's (PR2's) transform. */
pair<pair<btVector3, btVector3> , pair<int, int> >
CustomScene::SutureCloth::fitLineAligned(int side_num, RaveRobotObject::Ptr robot) {

	// get the robot transform
	btTransform robotT = robot->getLinkTransform(robot->robot->GetLink("base_link"));
	btVector3 robotX = robotT.getBasis().getColumn(0);

	// fit a line
	pair<pair<btVector3, btVector3> , pair<int, int> > res = fitLine(side_num);

	// flip?
	btVector3 dirCosine = res.first.second;
	btVector3 alignedIdx;
	if (dirCosine.dot(robotX) > 0) {
		return res;
	} else {
		// build the return structure
		pair<btVector3, btVector3> lineInfo(res.first.first, -1*res.first.second);
		pair<int, int> extremaIndices( res.second.second, res.second.first);
		return pair<pair<btVector3, btVector3> , pair<int, int> >(lineInfo, extremaIndices);
	}
}





/** Returns a transform for grasping.
 *  @param SIDE_NUM  \in {1, 2} : if 1 : transform for left-cut
 *                                if 2 : transform for right-cut
 *
 *  @param FRAC   : the fraction of the distance b/w the extreme points
 *                  of the cut, where the grasp has to be found.
 *
 *
 *  @param ROBOT  : Ptr to the robot for which the grasp has to be found.
 *                  The robot information is only used to get the right
 *                  sense of direction. So only the transform of the
 *                  "base_link" of the robot is used. No other information
 *                  about the robot is used.  */
btTransform CustomScene::SutureCloth::getCutGraspTransform(int side_num, RaveRobotObject::Ptr robot, float frac) {

	std::pair<std::pair<btVector3, btVector3> , std::pair<int, int> > cutInfo;
	cutInfo = fitLineAligned(1, robot);

	std::vector<int> &pt_vector = (side_num==1)? cut_nodes1 : cut_nodes2;

	btVector3 minNode = cloth->softBody->m_nodes[pt_vector[cutInfo.second.first]].m_x;
	btVector3 maxNode = cloth->softBody->m_nodes[pt_vector[cutInfo.second.second]].m_x;

	btVector3 translationPt = minNode + ((frac * (maxNode - minNode).length())*cutInfo.first.second);
	//find the node on the cut closest to the translation pt
	int closestNodeIdx = -1;
	float dist = numeric_limits<float>::infinity();
	for (int i=0; i < pt_vector.size(); i+=1) {
		btVector3 nodePos = cloth->softBody->m_nodes[pt_vector[i]].m_x;
		if ((translationPt - nodePos).length() < dist) {
			dist = (translationPt - nodePos).length();
			closestNodeIdx = i;
		}
	}

	btVector3 translation = cloth->softBody->m_nodes[pt_vector[closestNodeIdx]].m_x;
	btTransform cutT = util::getOrthogonalTransform(cutInfo.first.second);
	if (side_num != 1) { // then flip the y and the z axis
		btMatrix3x3 rot = cutT.getBasis().transpose();
		rot[1] *= -1;
		rot[2] *= -1;
		cutT.setBasis(rot.transpose());
	}
	cutT.setOrigin(translation);
	return cutT;
}




//////////////////////////////////////////////// Suturing needle ////////////////////////////////////////
/** Constructor for suturing needle. Creates needle from file.*/
CustomScene::SuturingNeedle::SuturingNeedle(CustomScene * _scene, float _rope_radius, float _segment_len, int _nLinks) :
											scene(*_scene), s_needle_radius(0.0112),
											s_needle_mass(50), s_pierce_threshold(0.03),
											s_end_angle(1.257), s_piercing(false), s_grasped(false),
											rope_radius(_rope_radius), segment_len(_segment_len), nLinks(_nLinks) {


	static const char sNeedle_MODEL_FILE[] = EXPAND(BULLETSIM_DATA_DIR) "/xml/needle.xml";
	KinBodyPtr needle_body = scene.rave->env->ReadKinBodyURI(sNeedle_MODEL_FILE);
	btTransform needle_tfm;
	scene.table->motionState->getWorldTransform(needle_tfm);


	needle_tfm.setOrigin((needle_tfm.getOrigin() + btVector3(-0.4*GeneralConfig::scale,0.45*GeneralConfig::scale,0.05*GeneralConfig::scale))/ GeneralConfig::scale);
	needle_body->SetTransform(util::toRaveTransform(needle_tfm));

	s_needle = RaveObject::Ptr(new RaveObject(scene.rave,needle_body,RAW,true));
	vector<BulletObject::Ptr> children = s_needle->getChildren();
	btVector3 inertia(0,0,0);
	children[0]->rigidBody->getCollisionShape()->calculateLocalInertia(s_needle_mass,inertia);
	children[0]->rigidBody->setMassProps(s_needle_mass,inertia);

	s_needle->setColor(0.97,0.09,0.266,1.0); // set the needle's color
    scene.addPreStepCallback(boost::bind(&CustomScene::SuturingNeedle::setGraspingTransformCallback, this));

    //------------------------ initialize the rope -------------------------------
	vector<btVector3> ctrlPts;
    float height = needle_tfm.getOrigin().z();
    for (int i=0; i< nLinks; i++)
    	ctrlPts.push_back(METERS*btVector3(.5+segment_len*i,0,height+5*rope_radius));

    ropePtr.reset(new CapsuleRope(ctrlPts,METERS*rope_radius,0.4,1,0.75,1,1));
    scene.env->add(ropePtr);

    ropePtr->setColor(0,1,0,1);
    /*vector<BulletObject::Ptr> rope_children =  ropePtr->getChildren();
    for (int j=0; j<rope_children.size(); j++)
    	rope_children[j]->setColor(0,1,0,1);*/


    s_needle->ignoreCollisionWith(ropePtr->children[0]->rigidBody.get());
    needle_rope_grab = new Grab(ropePtr->children[0]->rigidBody.get(), getNeedleHandleTransform().getOrigin(),scene.env->bullet->dynamicsWorld);
    scene.addPreStepCallback(boost::bind(&CustomScene::SuturingNeedle::setConnectedRopeTransformCallback, this));
    //scene.drawAxes(getNeedleHandleTransform());
    util::drawAxes(getNeedleHandleTransform(), 1, scene.env);
    //------------------------------------------------------------------------------
}



/** Returns transform of needle's tip.*/
btTransform CustomScene::SuturingNeedle::getNeedleTipTransform () {

	btTransform tfm = s_needle->getIndexTransform(0);
	return rotateByAngle(tfm, -s_end_angle, s_needle_radius);
}




/** Returns transform of needle's handle.*/
btTransform CustomScene::SuturingNeedle::getNeedleHandleTransform () {

	btTransform tfm = s_needle->getIndexTransform(0);
	return rotateByAngle(tfm, s_end_angle, s_needle_radius);

}


void CustomScene::SuturingNeedle::setGraspingTransformCallback() {
	if (!s_grasped || !s_gripperManip) return;

	btTransform nTfm = s_gripperManip->getTransform();
	nTfm.setBasis(nTfm.getBasis() * s_corrRot);

	nTfm = rotateByAngle(nTfm, -s_end_angle, s_needle_radius);

	s_needle->getChildren()[0]->motionState->setKinematicPos(nTfm);
}

void CustomScene::SuturingNeedle::setConnectedRopeTransformCallback() {
	btTransform nTfm = getNeedleHandleTransform ();
	needle_rope_grab->updatePosition(nTfm.getOrigin());
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Plot needle curve-center and tip. */
void CustomScene::plotNeedle (bool remove) {
	plot_needle->setPoints(std::vector<btVector3>(),std::vector<btVector4>());

	btTransform tfm = sNeedle->s_needle->getIndexTransform(0);
	std::vector<btVector3> plotpoints;
	std::vector<btVector4> color;

	if (!remove) {
		// COM of needle
		plotpoints.push_back(tfm.getOrigin());
		color.push_back(btVector4(1,0,0,1));

		// Needle tip
		plotpoints.push_back(sNeedle->getNeedleTipTransform().getOrigin());
		color.push_back(btVector4(0,0,1,1));

		// Needle handle
		plotpoints.push_back(sNeedle->getNeedleHandleTransform().getOrigin());
		color.push_back(btVector4(1,1,0,1));

	}
	else {
		plotpoints.push_back(btVector3(0,0,0));
		color.push_back(btVector4(0,0,0,0));
	}

	plot_needle->setPoints(plotpoints,color);
}