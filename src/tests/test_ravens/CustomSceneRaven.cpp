#include "CustomSceneRaven.h"
#include "CustomKeyHandler.h"


btTransform rotateByAngle (btTransform &tfm, const float ang, const float rad) {

	float pi = 3.14159265, r = rad*GeneralConfig::scale;

	btTransform WorldToEndEffectorTransform = tfm; //util::toBtTransform(pr2m.pr2Right->manip->GetEndEffectorTransform(),GeneralConfig::scale);

	btTransform initT;
	initT.setIdentity();
	initT.setOrigin(r*btVector3(-1,0,0));

	OpenRAVE::Transform T = OpenRAVE::geometry::matrixFromAxisAngle(OpenRAVE::Vector(0,0,ang));
	btTransform bT = util::toBtTransform(T);
	bT.setOrigin(r*btVector3(1,0,0));

	return WorldToEndEffectorTransform*bT*initT;
}


/** Fills in the rcontacts array with contact information between psb and pco */
void getContactPointsWith(	btSoftBody *psb, btCollisionObject *pco,
							btSoftBody::tRContactArray &rcontacts) {
    // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
    struct Custom_CollideSDF_RS : btDbvt::ICollide {
        Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_) :
        	rcontacts(rcontacts_) { }

        void Process(const btDbvtNode* leaf) {
            btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
            DoNode(*node);
        }

        void DoNode(btSoftBody::Node& n) {
            const btScalar m=n.m_im>0?dynmargin:stamargin;
            btSoftBody::RContact c;
            if (!n.m_battach && psb->checkContact(m_colObj1,n.m_x,m,c.m_cti)) {
                const btScalar ima=n.m_im;
                const btScalar imb= m_rigidBody? m_rigidBody->getInvMass():0.f;
                const btScalar ms=ima+imb;
                if(ms>0) {
                    c.m_node        =       &n;
                    rcontacts.push_back(c);
                }
            }
        }
        btSoftBody*             psb;
        btCollisionObject*      m_colObj1;
        btRigidBody*    m_rigidBody;
        btScalar                dynmargin;
        btScalar                stamargin;
        btSoftBody::tRContactArray &rcontacts;
    };

    Custom_CollideSDF_RS  docollide(rcontacts);
    btRigidBody*            prb1=btRigidBody::upcast(pco);
    btTransform     wtr=pco->getWorldTransform();

    const btTransform       ctr=pco->getWorldTransform();
    const btScalar          timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
    const btScalar          basemargin=psb->getCollisionShape()->getMargin();
    btVector3                       mins;
    btVector3                       maxs;
    ATTRIBUTE_ALIGNED16(btDbvtVolume)               volume;
    pco->getCollisionShape()->getAabb(      pco->getWorldTransform(),
            mins,
            maxs);
    volume=btDbvtVolume::FromMM(mins,maxs);
    volume.Expand(btVector3(basemargin,basemargin,basemargin));
    docollide.psb           =       psb;
    docollide.m_colObj1 = pco;
    docollide.m_rigidBody = prb1;

    docollide.dynmargin     =       basemargin+timemargin;
    docollide.stamargin     =       basemargin;
    psb->m_ndbvt.collideTV(psb->m_ndbvt.m_root,volume,docollide);

}

/** Implicit function for cutting cloth. */
struct	ImplicitSphere : btSoftBody::ImplicitFn
{
	btVector3	center;
	btScalar	sqradius;
	ImplicitSphere() {}
	ImplicitSphere(const btVector3& c,btScalar r) : center(c),sqradius(r*r) {}
	btScalar	Eval(const btVector3& x)
	{
		return((x-center).length2()-sqradius);
	}
};

/** Creates a square cloth with side length 2s.
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

////////////////////////////////// Suture Cloth ////////////////////////////////////////
/** Constructor for the cloth in the scene. */
CustomScene::SutureCloth::SutureCloth(CustomScene &scene, btScalar side_length, btScalar z, btVector3 center) {
		cloth = scene.createCloth(side_length, z, center, cut_nodes1, cut_nodes2, true, true, 60,60);
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
	cutInfo = fitLineAligned(side_num, robot);

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

/** Returns a transform for grasping.
 *  @param SIDE_NUM  \in {1, 2} : if 1 : transform for left-cut
 *                                if 2 : transform for right-cut
 *
 *  @param POINT  : The point (of the cut). The transform returned
 *                  is the transform of the cut closest to this point
 *
 *
 *  @param ROBOT  : Ptr to the robot for which the grasp has to be found.
 *                  The robot information is only used to get the right
 *                  sense of direction. So only the transform of the
 *                  "base_link" of the robot is used. No other information
 *                  about the robot is used.  */

btTransform CustomScene::SutureCloth::getCutGraspTransform(int side_num, RaveRobotObject::Ptr robot, btVector3 point) {

	std::pair<std::pair<btVector3, btVector3> , std::pair<int, int> > cutInfo;
	cutInfo = fitLineAligned(side_num, robot);

	std::vector<int> &pt_vector = (side_num==1)? cut_nodes1 : cut_nodes2;

	btVector3 ptOnLine = cutInfo.first.first;
	btVector3 dir      = cutInfo.first.second;

	btVector3 closestPoint = dir.dot(point-ptOnLine)/dir.length2()*dir+ptOnLine;

	//find the node on the cut closest to the translation pt
	int closestNodeIdx = -1;
	float dist = numeric_limits<float>::infinity();
	for (int i=0; i < pt_vector.size(); i+=1) {
		btVector3 nodePos = cloth->softBody->m_nodes[pt_vector[i]].m_x;
		if ((closestPoint - nodePos).length() < dist) {
			dist = (closestPoint - nodePos).length();
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////// Suturing needle ////////////////////////////////////////
/** Constructor for suturing needle. Creates needle from file.*/
CustomScene::SuturingNeedle::SuturingNeedle(CustomScene * _scene) :
												scene(*_scene), s_needle_radius(0.112),
												s_needle_mass(50), s_pierce_threshold(0.03),
												s_end_angle(1.257), s_piercing(false), grasped(false) {




	static const char sNeedle_MODEL_FILE[] = EXPAND(BULLETSIM_DATA_DIR) "/needle/sneedle.dae";
	KinBodyPtr needle_body = scene.rave->env->ReadKinBodyURI(sNeedle_MODEL_FILE);
	btTransform needle_tfm;
	scene.table->motionState->getWorldTransform(needle_tfm);

	///
	//btMatrix3x3 corrRot;
	//corrRot.setValue(-1, 0, 0, 0, 0, 1, 0, 1, 0);
	//corrRot = corrRot * needle_tfm.getBasis();
	//needle_tfm.setBasis(corrRot);
	///

	needle_tfm.setOrigin((needle_tfm.getOrigin() + btVector3(-0.4*GeneralConfig::scale,0.45*GeneralConfig::scale,0.05*GeneralConfig::scale))/ GeneralConfig::scale);
	needle_body->SetTransform(util::toRaveTransform(needle_tfm));

	s_needle = RaveObject::Ptr(new RaveObject(scene.rave,needle_body,RAW,false));
	vector<BulletObject::Ptr> children = s_needle->getChildren();
	btVector3 inertia(0,0,0);
	children[0]->rigidBody->getCollisionShape()->calculateLocalInertia(s_needle_mass,inertia);
	children[0]->rigidBody->setMassProps(s_needle_mass,inertia);

    scene.addPreStepCallback(boost::bind(&CustomScene::SuturingNeedle::setGraspingTransformCallback, this));
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
	if (!grasped || !gripperManip) return;

	btTransform nTfm = gripperManip->getTransform();
	nTfm.setBasis(nTfm.getBasis() * corrRot);

	nTfm = rotateByAngle(nTfm, -s_end_angle, s_needle_radius);

	s_needle->getChildren()[0]->motionState->setKinematicPos(nTfm);
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

//////////////////////////////////////////////// Hole ///////////////////////////////////////////////////

/** Increases the number of nodes around the hole on the softbody to
 *  make needle-cloth interaction realistic around the hole. */
void CustomScene::Hole::refineAroundHole (int iter) {
	calculateCenter();
	std::cout<<h_center.x()<<" "<<h_center.y()<<" "<<h_center.z()<<std::endl;
	/*
	for (int i = 0; i< h_nodes.size(); ++i){
		btVector3 nodeCenter = h_nodes[i]->m_x;
		ImplicitSphere	iSphere(nodeCenter,0.02*GeneralConfig::scale);
		h_scene->sCloth->cloth->refine(&iSphere, 0.0001, false);
	}*/

	for (int i = 1; i <= iter; ++i){
			btVector3 nodeCenter = h_nodes[i]->m_x;
			ImplicitSphere	iSphere(nodeCenter,0.008*i*GeneralConfig::scale);
			h_scene->sCloth->cloth->refine(&iSphere, 0.0001, false);
}
	}

/** Calculates center of hole. */
void CustomScene::Hole::calculateCenter() {
	int nnodes = h_nodes.size(),j;
	h_center = btVector3(0,0,0);
	for (j = 0; j < nnodes; ++j)
		h_center += h_nodes[j]->m_x;
	h_center /= nnodes;
}

/** Callback of the hole to cut when the suturing needle is close.*/
void CustomScene::Hole::holeCutCallback() {
	if (!h_scene->sNeedle->s_piercing || !h_currently_piercing) return;

	btSoftBody::tRContactArray rcnts;
	BulletObject::Ptr child = h_scene->sNeedle->s_needle->getChildren()[0];
	getContactPointsWith(h_scene->sCloth->cloth->softBody.get(),
						 child->rigidBody.get(), rcnts);
	int numContacts = rcnts.size(),nnodes = h_nodes.size(), i,j;

	if (!numContacts) return;

	btVector3 Tip = h_scene->sNeedle->getNeedleTipTransform().getOrigin();
	// This definitely works (the simple approach of cutting on contact)
#if 0
	for (i = 0; i < nnodes; ++i) {
		for (j = 0; j < numContacts; ++j) {
			if (rcnts[j].m_node == h_nodes[i] &&
				 (Tip-rcnts[j].m_node->m_x).length() <
				  h_scene->sNeedle->s_pierce_threshold*GeneralConfig::scale) {
				ImplicitSphere	iSphere(h_center,0.015*GeneralConfig::scale);
				std::cout<<"Piercing hole at: "<<h_center.x();
				std::cout<<","<<h_center.y()<<","<<h_center.z()<<std::endl;
				h_scene->sCloth->cloth->refine(&iSphere, 0.0001, true);
				h_scene->sNeedle->s_piercing = false;
				h_currently_piercing = false;
				h_pierced = true;
				return;
			}
		}
	}


#else // Code for some realism
	if (!h_started_piercing) {
		for (i = 0; i < nnodes; ++i) {
			for (j = 0; j < numContacts; ++j) {
				if (rcnts[j].m_node == h_nodes[i] &&
					(Tip-rcnts[j].m_node->m_x).length() <
					h_scene->sNeedle->s_pierce_threshold*GeneralConfig::scale){

					h_started_piercing = true;
					h_prev_center = h_center;
					std::cout<<"Started piercing!"<<std::endl;
					return;
				}
			}
		}
		return;
	}

	float STRETCH_THRESHOLD = 0.015*GeneralConfig::scale;
	if ((h_center - h_prev_center).length() >= STRETCH_THRESHOLD &&
			(Tip-h_center).length() <
			h_scene->sNeedle->s_pierce_threshold*GeneralConfig::scale) {

		ImplicitSphere	iSphere(h_center,0.015*GeneralConfig::scale);

		std::cout<<"Piercing hole at: "<<h_center.x()<<","<<h_center.y();
		std::cout<<","<<h_center.z()<<std::endl;

		h_scene->sCloth->cloth->refine(&iSphere, 0.0001, true);
		h_scene->sNeedle->s_piercing = false;
		h_currently_piercing = false;
		h_started_piercing = false;
		h_pierced = true;
	}
#endif
}

btVector3 CustomScene::Hole::normal() {
	btVector3 normal(0,0,0);
	int nh=h_nodes.size();
	for (int i=0; i<nh; ++i)
		normal += h_nodes[0]->m_n;
	normal /= nh;
	return normal.normalized();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Adds nodes near point to hole. */
void CustomScene::findNearbyNodes(Hole::Ptr hole, btVector3 holePt) {
	float DIST_THRESHOLD = 0.03*GeneralConfig::scale;
	int i, nnodes = sCloth->cloth->softBody->m_nodes.size();

	for (i = 0; i < nnodes; ++i)
		if ((sCloth->cloth->softBody->m_nodes[i].m_x-holePt).length()
													< DIST_THRESHOLD)
			hole->h_nodes.push_back(&sCloth->cloth->softBody->m_nodes[i]);

	std::cout<<"Added "<<hole->h_nodes.size()<<" nodes to the hole."<<std::endl;
}

/** Calculates centers of all the holes only when necessary. */
void CustomScene::computeHoleCentersCallBack () {
	int hole_size = holes.size(),n_nodes,i;

	if (!hole_size) return;

	for (i = 0; i < hole_size; ++i)
		holes[i]->calculateCenter();
}

/** Plot the holes. */
void CustomScene::plotHoles (bool remove) {
	plot_holes->setPoints(std::vector<btVector3>(),std::vector<btVector4>());

	std::vector<btVector3> plotpoints;
	std::vector<btVector4> color;

	if (!remove) {
		int holes_size = holes.size();
		for (int i = 0; i < holes_size; ++i) {
			for (int j = 0; j < holes[i]->h_nodes.size(); ++j) {
				plotpoints.push_back(holes[i]->h_nodes[j]->m_x);
				color.push_back(btVector4(1,1,0,1));
			}
			if (holes[i]->h_center != btVector3(0,0,0)) {
				plotpoints.push_back(holes[i]->h_center);
				color.push_back(btVector4(0,1,0,1));

				for (float k = 0; k<= 0.1; k+=0.01) {
					plotpoints.push_back(holes[i]->h_center+holes[i]->normal()*GeneralConfig::scale*k);
					color.push_back(btVector4(0,1,0,1));
				}
			}
		}
	}

	plotpoints.push_back(btVector3(0,0,0));
	color.push_back(btVector4(0,0,0,0));

	plot_holes->setPoints(plotpoints,color);
}

/** Sets up the scene and UI even handlers,
 *  initializes various structures.*/
void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = BulletConfig::dt;

    // Setting up the table
    const float table_height = .8;
    const float table_thickness = .05;
    table = BoxObject::Ptr(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
    									 btTransform(btQuaternion(0, 0, 0, 1),
                            		                 GeneralConfig::scale * btVector3(0.85, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(10);

    // Setting up the needle - Make this into constructor
    const float needle_radius = 0.08*1.4;
    const float needle_mass = 50;
    const float pierce_threshold = 0.03;
    sNeedle.reset(new SuturingNeedle(this));

    // Setting up the cloth
    sCloth.reset(new SutureCloth(*this,GeneralConfig::scale * 0.4, 0, GeneralConfig::scale * btVector3(0.6, 0, table_height+0.01)));
    btSoftBody * const psb = sCloth->cloth->softBody.get();

    // Plotting:
    plot_points.reset(new PlotPoints(5));
    plot_needle.reset(new PlotPoints(10));
    plot_holes.reset(new PlotPoints(5));
    plot_axes1.reset(new PlotAxes());
    plot_axes2.reset(new PlotAxes());

    // Adding objects to the environment
    env->add(table);
	//createKinBodyFromBulletBoxObject(table, rave); // add the table to the rave environment
    env->add(sCloth->cloth);
    env->add(sNeedle->s_needle);
    env->add(plot_points);
    env->add(plot_needle);
    env->add(plot_holes);
    env->add(plot_axes1);
    env->add(plot_axes2);

    // Adding prestep callbacks and setting up holes
    addPreStepCallback(boost::bind(&CustomScene::computeHoleCentersCallBack, this));

    btTransform table_tfm;
    table->motionState->getWorldTransform(table_tfm);
    btVector3 hole1Pt = table_tfm.getOrigin() + btVector3(-0.3,0.1,0.05)*GeneralConfig::scale;
    Hole::Ptr h1 (new Hole(this));
    findNearbyNodes (h1,hole1Pt);
    addPreStepCallback(boost::bind(&CustomScene::Hole::holeCutCallback, h1.get()));
    holes.push_back(h1);

    // Reset right and left actions.
    //leftSBAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left,
    		                                      	  //"l_gripper_l_finger_tip_link",
    		                                      //"l_gripper_r_finger_tip_link", 1));
    //leftSBAction->setTarget(sCloth->cloth);
   // rightSBAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right,
    		                                       //"r_gripper_l_finger_tip_link",
    		                                       //"r_gripper_r_finger_tip_link", 1));
    //rightSBAction->setTarget(sCloth->cloth);

    //leftOCAction.reset(new PR2OpenCloseGripperAction (pr2m.pr2Left,1));
    //rightOCAction.reset(new PR2OpenCloseGripperAction (pr2m.pr2Right,1));
    //leftRBAction.reset(new PR2RigidBodyGripperAction(pr2m.pr2Left,
    //												"l_gripper_l_finger_tip_link",
    //												"l_gripper_r_finger_tip_link", 1));
    //leftRBAction->setTarget(sNeedle->s_needle->getChildren()[0]->rigidBody.get());
    //rightRBAction.reset(new PR2RigidBodyGripperAction(pr2m.pr2Right,
    //												"r_gripper_l_finger_tip_link",
    //												"r_gripper_r_finger_tip_link", 1));
    //rightRBAction->setTarget(sNeedle->s_needle->getChildren()[0]->rigidBody.get());

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    // Open grippers
    //leftOCAction->setOpenAction();
    //runAction(leftOCAction, dt);
    //rightOCAction->setOpenAction();
    //runAction(rightOCAction, dt);

    // Start loop
    startFixedTimestepLoop(dt);
}


//////////////////////////// Previously present code ///////////////////////////////
/** Returns the coordinates of the last point below (-z) SOURCE_PT
    on the cloth represented by PSB. */
btVector3 CustomScene::getDownPoint(btVector3 & source_pt,
		                            boost::shared_ptr<btSoftBody> psb,
				                    btScalar radius) {
  int node_idx = -1;
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
btScalar inline CustomScene::getXYDistance
						(btVector3 &v1, btVector3 &v2) {
  btVector3 vec(v1.x() - v2.x(), v1.y() - v2.y(), 0.0);
  return vec.length();
}


/** Raycasts from SOURCE to all the nodes of PSB
    and returns a vector of the same size as the nodes of PSB
    depicting whether that node is visible or not. */
std::vector<btVector3> CustomScene::checkNodeVisibility
								(btVector3 camera_origin,
								 boost::shared_ptr<btSoftBody> psb) {

  plot_points->setPoints(std::vector<btVector3>(),std::vector<btVector4>());

  std::vector<btVector3> plotpoints;
  std::vector<btVector4> color;

  plotpoints.push_back(camera_origin);
  color.push_back(btVector4(3,1,0,1));

  for (int i=0; i < psb->m_nodes.size(); i += 1) {
    btVector3 node_pos = psb->m_nodes[i].m_x;
    btSoftBody::sRayCast sol;
    bool b = psb->rayTest (camera_origin, node_pos, sol);

    // iff fraction -> 1 : means the ray can hit the node.
    if (sol.fraction >= 0.99) {
      plotpoints.push_back(node_pos);
      color.push_back(btVector4(2,0,0,1));
    }
  }
  plot_points->setPoints(plotpoints,color);
  return plotpoints;
}

void CustomScene::createFork() {
    bullet2.reset(new BulletInstance);
    bullet2->setDefaultGravity();
    osg2.reset(new OSGInstance);
    osg->root->addChild(osg2->root.get());

    fork.reset(new Fork(env, bullet2, osg2));
    registerFork(fork);

    cout << "forked!" << endl;

    //origRobot = pr2m.pr2;
    EnvironmentObject::Ptr p;// = fork->forkOf(pr2m.pr2);
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
    int leftidx = 1;//pr2m.pr2Left->index;
    int rightidx = 2;//pr2m.pr2Right->index;
    origRobot.swap(tmpRobot);
    //pr2m.pr2 = origRobot;
    //pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
    //pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);

/*    vector<int> indices; vector<dReal> vals;
    for (int i = 0; i < tmpRobot->robot->GetDOF(); ++i) {
        indices.push_back(i);
        vals.push_back(0);
    }
    tmpRobot->setDOFValues(indices, vals);*/
}

/** Draws the axes at LEFT_GRIPPER1 and LEFT_GRIPPER2. */
void CustomScene::drawAxes() {
  //cut_axes->setup(btT,1);
}

////////////////////////////////////////////////////////////////////////////////////////
