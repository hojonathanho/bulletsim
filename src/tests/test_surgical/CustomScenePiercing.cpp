/**
 * Author: 	Sibi Venkatesan
 * 		   	(code adopted from Ankush Gupta)
 * Attempts to pierce cloth
 */

#include "CustomScenePiercing.h"
#include "CustomKeyHandlerPiercing.h"


// Fills in the rcontacts array with contact information between psb and pco
void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts) {
    // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
    struct Custom_CollideSDF_RS : btDbvt::ICollide {
        Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_) : rcontacts(rcontacts_) { }

        void Process(const btDbvtNode* leaf) {
            btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
            DoNode(*node);
        }

        void DoNode(btSoftBody::Node& n) {
            const btScalar m=n.m_im>0?dynmargin:stamargin;
            btSoftBody::RContact c;
            if (!n.m_battach && psb->checkContact(m_colObj1,n.m_x,m,c.m_cti)) {
                const btScalar  ima=n.m_im;
                const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                const btScalar  ms=ima+imb;
                if(ms>0) {
                    // there's a lot of extra information we don't need to compute
                    // since we just want to find the contact points

#if 0
                    const btTransform&      wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1->getWorldTransform();
                    static const btMatrix3x3        iwiStatic(0,0,0,0,0,0,0,0,0);
                    const btMatrix3x3&      iwi=m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
                    const btVector3         ra=n.m_x-wtr.getOrigin();
                    const btVector3         va=m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra)*psb->m_sst.sdt : btVector3(0,0,0);
                    const btVector3         vb=n.m_x-n.m_q;
                    const btVector3         vr=vb-va;
                    const btScalar          dn=btDot(vr,c.m_cti.m_normal);
                    const btVector3         fv=vr-c.m_cti.m_normal*dn;
                    const btScalar          fc=psb->m_cfg.kDF*m_colObj1->getFriction();
#endif
                    c.m_node        =       &n;
#if 0
                    c.m_c0          =       ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
                    c.m_c1          =       ra;
                    c.m_c2          =       ima*psb->m_sst.sdt;
                    c.m_c3          =       fv.length2()<(btFabs(dn)*fc)?0:1-fc;
                    c.m_c4          =       m_colObj1->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
#endif
                    rcontacts.push_back(c);
#if 0
                    if (m_rigidBody)
                            m_rigidBody->activate();
#endif
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

/* 	Cuts cloth at on needle contact at tip mentioned (need to account for forces later)
 * 	If needle not piercing, print out info */
void CustomScene::cutCloth () {

	//Get contact points
	btSoftBody::tRContactArray rcnts;
	//assuming there is only one child
	BulletObject::Ptr child = sneedle->getChildren()[0];
	getContactPointsWith(cloth->softBody.get(), child->rigidBody.get(), rcnts);
	int numContacts = rcnts.size();

	// If no contact points, return.
	if (!numContacts) {
		std::cout<<"No contact!"<<std::endl;
		return;
	}

	// Number of points/ point for piercing
	int numPts = 0;
	// Base pointer for nodes
	const btSoftBody::Node*			nbase = &cloth->softBody->m_nodes[0];

	btVector3 pointToPierce(0,0,0);

	btVector3 Tip = getNeedleTip();
	btVector3 currPt;

	for (int i = 0; i < numContacts; ++i) {
		 currPt = rcnts[i].m_node->m_x;
		if ((Tip-currPt).length() < cut_threshold*GeneralConfig::scale) {
			pointToPierce += currPt;
			numPts ++;
		}
	}

	if (numPts)
		pointToPierce /= numPts;
	else {
		std::cout<<"No point close to tips!"<<std::endl;
		return;
	}

	if (!piercing) {
		std::cout<<"Stress on different contact points: "<<std::endl;
		for (int i = 0; i < numContacts; ++i) {
			std::cout<<cloth->nodeStress[int(rcnts[i].m_node - nbase)]<<std::endl;
		}
		std::cout<<"Mean stress: "<<cloth->meanStress<<std::endl;
		std::cout<<"Pierce point: "<<pointToPierce[0]<<","<<pointToPierce[1]<<","<<pointToPierce[2]<<std::endl;

		return;
	}

	std::cout<<"Piercing at: "<<pointToPierce[0]<<","<<pointToPierce[1]<<","<<pointToPierce[2]<<std::endl;

	ImplicitSphere	iSphere(pointToPierce,0.01*GeneralConfig::scale);

	cloth->refine(&iSphere, 0.0001, true);
}

// Attempting to add piercing as callback
void CustomScene::piercingCallBack () {

	//std::cout<<std::endl<<"------NEW CALL BACK-----------"<<std::endl;
	if (!piercing || !cloth->meanStress) return;

	btSoftBody::tRContactArray rcnts;
	BulletObject::Ptr child = sneedle->getChildren()[0];
	getContactPointsWith(cloth->softBody.get(), child->rigidBody.get(), rcnts);
	int numContacts = rcnts.size();

	// If no contact points, return.
	//std::cout<<"Checking for contacts"<<std::endl;
	if (!numContacts) return;

	// Node indices for points to pierce
	vector<int> pierceNodes;
	// Base address for nodes to find index
	const btSoftBody::Node*			nbase = &cloth->softBody->m_nodes[0];

	btVector3 pointToPierce(0,0,0);
	btVector3 Tip = getNeedleTip();
	btVector3 currPt;
	int idx;
	float stressThreshold = 1;

	for (int i = 0; i < numContacts; ++i) {
		 currPt = rcnts[i].m_node->m_x;
		 idx = int(rcnts[i].m_node - nbase);
		if ((Tip-currPt).length() < cut_threshold*GeneralConfig::scale && cloth->nodeStress[idx]/cloth->meanStress >= stressThreshold) {
			pointToPierce += currPt;
			pierceNodes.push_back(idx);
		}
	}

	//std::cout<<"Piercing node number: "<<pierceNodes.size()<<std::endl;
	if (pierceNodes.size() >= 2)
		pointToPierce /= pierceNodes.size();
	else
		return;

	//std::cout<<"Going to pierce at: "<<pointToPierce[0]<<","<<pointToPierce[1]<<","<<pointToPierce[2]<<std::endl;
	ImplicitSphere	iSphere(pointToPierce,0.005*GeneralConfig::scale);
	cloth->refine(&iSphere, 0.0001, true);

}


btVector3 CustomScene::getNeedleTip () {
	return sneedle->getIndexTransform(0)*btVector3(sneedle_radius*GeneralConfig::scale,sneedle_radius*1.3*GeneralConfig::scale,0);
}


//Plot needle tip (transform point for now)
void CustomScene::plotNeedle () {
	plot_needle->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());

	btTransform tfm = sneedle->getIndexTransform(0);
	std::vector<btVector3> plotpoints;
	std::vector<btVector4> color;

	// COM of needle
	plotpoints.push_back(tfm.getOrigin());
    color.push_back(btVector4(3,0,0,1));

    plotpoints.push_back(getNeedleTip());
    color.push_back(btVector4(3,0,0,1));

//    plotpoints.push_back(getNeedleTip(false));

//    color.push_back(btVector4(3,0,0,1));
    plot_needle->setPoints(plotpoints,color);
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
   Then, the center of the cloth (initially at (0,0,0)
   is translated to CENTER. */
BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, btScalar z, btVector3 center,
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
	cutPlaneSoftBody(psb, &cut, 0.001);
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

    /*/Needle info

    const float needle_radius = 0.01;
    const float needle_height = 0.35;
    needle = CapsuleObject::Ptr(new CapsuleObject(	needle_mass,
    												GeneralConfig::scale * needle_radius,
    												GeneralConfig::scale * needle_height,
    												btTransform(btQuaternion(0, 0, 0, 1),
    														GeneralConfig::scale *
    															btVector3(0.85, 0, table_height+table_thickness))));*/

    // Adding the suturing needle to the scene
    const float needle_mass = 50;
    static const char SNEEDLE_MODEL_FILE[] = EXPAND(BULLETSIM_DATA_DIR) "/needle/sneedle.dae";
    KinBodyPtr needle_body = rave->env->ReadKinBodyURI(SNEEDLE_MODEL_FILE);
    btTransform needle_tfm;
    table->motionState->getWorldTransform(needle_tfm);
	needle_tfm.setOrigin((needle_tfm.getOrigin() + btVector3(0.5*GeneralConfig::scale,0,0.2*GeneralConfig::scale))/ GeneralConfig::scale);
    needle_body->SetTransform(util::toRaveTransform(needle_tfm));
    sneedle = RaveObject::Ptr(new RaveObject(rave,needle_body,CONVEX_DECOMP));//,CONVEX_DECOMP));

    vector<BulletObject::Ptr> chldrn = sneedle->getChildren();
    btVector3 inertia(0,0,0);
    chldrn[0]->rigidBody->getCollisionShape()->calculateLocalInertia(needle_mass,inertia);
    chldrn[0]->rigidBody->setMassProps(needle_mass,inertia);


    //std::cout<<"Bullet margin: "<<BulletConfig::margin<<" and METERS: "<<METERS<<std::endl;


    cloth = createCloth(GeneralConfig::scale * 0.5, 0, GeneralConfig::scale * btVector3(0.6, 0, table_height+0.01));
    btSoftBody * const psb = cloth->softBody.get();
    pr2m.pr2->ignoreCollisionWith(psb);

    plot_needle.reset(new PlotPoints(10));

    env->add(table);
    env->add(cloth);
    //env->add(needle);
    env->add(sneedle);
    env->add(plot_needle);

    //boost::function<void(void)>
    //addPreStepCallback(boost::bind(&CustomScene::piercingCallBack, this));

    leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left,
    		                                      "l_gripper_l_finger_tip_link",
    		                                      "l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(cloth);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right,
    		                                       "r_gripper_l_finger_tip_link",
    		                                       "r_gripper_r_finger_tip_link", 1));
    rightAction->setTarget(cloth);

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);


    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);

    startFixedTimestepLoop(dt);
}
