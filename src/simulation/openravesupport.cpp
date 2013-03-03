#include <openrave-core.h>
#include "openravesupport.h"
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include "utils/config.h"
#include "bullet_io.h"
#include "utils/logging.h"
#include "config_bullet.h"

#include <set>

using namespace OpenRAVE;
using namespace std;
using boost::shared_ptr;

/*
 * re: margins, see http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=2358
 * there are three places where you can set a margin: 1. convexbuilder, 2. subshape, and 3. compound shape
 * 1. effectively expands the shape outwards
 * 2. has a weird effect, seems to double-count the margin, both expanding the shape and the collision detection
 * 3. has no effect if negative, but some effect if positive?
 * link padding is currently not being used for shapes other than convex hull
 */


RaveInstance::RaveInstance(OpenRAVE::EnvironmentBasePtr env_) {
  env = env_;
  env->StopSimulation();
}

RaveInstance::RaveInstance() {
	isRoot = true;
	RaveInitialize(true);
	env = RaveCreateEnvironment();
	if (GeneralConfig::verbose  <= log4cplus::DEBUG_LOG_LEVEL)
		RaveSetDebugLevel(Level_Debug);
	env->StopSimulation();
}

RaveInstance::RaveInstance(const RaveInstance &o, int cloneOpts) {
	isRoot = false;
	env = o.env->CloneSelf(cloneOpts);
}

RaveInstance::~RaveInstance() {
    env->Destroy();
	if (isRoot)
		RaveDestroy();
}

void LoadFromRave(Environment::Ptr env, RaveInstance::Ptr rave) {

  std::set<string> bodiesAlreadyLoaded;
  BOOST_FOREACH(EnvironmentObject::Ptr obj, env->objects) {
    RaveObject* robj = dynamic_cast<RaveObject*>(obj.get());
    if (robj) bodiesAlreadyLoaded.insert(robj->body->GetName());
  }

  std::vector<boost::shared_ptr<OpenRAVE::KinBody> > bodies;
  rave->env->GetBodies(bodies);
  BOOST_FOREACH(OpenRAVE::KinBodyPtr body, bodies) {
    if (bodiesAlreadyLoaded.find(body->GetName()) == bodiesAlreadyLoaded.end()) {
      if (body->IsRobot()) env->add(RaveRobotObject::Ptr(new RaveRobotObject(
				rave, boost::dynamic_pointer_cast<RobotBase>(body), CONVEX_HULL, BulletConfig::kinematicPolicy <= 1)));
      else {
        LOG_INFO("loading " << body->GetName());
        env->add(RaveObject::Ptr(new RaveObject(rave, body, CONVEX_HULL, BulletConfig::kinematicPolicy == 0)));
      }
    }
  }

}

void Load(Environment::Ptr env, RaveInstance::Ptr rave, const string& filename) {
	bool success = rave->env->Load(filename);
	if (!success)
		throw runtime_error(
				(boost::format("couldn't load %s!\n") % (filename)).str());

	LoadFromRave(env, rave);

}


RaveObject::Ptr getObjectByName(Environment::Ptr env, RaveInstance::Ptr rave, const string& name) {
  BOOST_FOREACH(EnvironmentObject::Ptr obj, env->objects) {
    RaveObject::Ptr maybeRO = boost::dynamic_pointer_cast<RaveObject>(obj);
    if (maybeRO && maybeRO->body->GetName() == name) {
      return maybeRO;
    }
  }
  return RaveObject::Ptr();
}

std::vector<RaveRobotObject::Ptr> getRobots(Environment::Ptr env, RaveInstance::Ptr rave) {
  vector<RaveRobotObject::Ptr> out;
  BOOST_FOREACH(EnvironmentObject::Ptr obj, env->objects) {
    RaveRobotObject::Ptr maybeRO = boost::dynamic_pointer_cast<RaveRobotObject>(obj);
    if (maybeRO) out.push_back(maybeRO);
  }
  return out;
}

RaveRobotObject::Ptr getRobotByName(Environment::Ptr env, RaveInstance::Ptr rave, const string& name) {
  return boost::dynamic_pointer_cast<RaveRobotObject>(getObjectByName(env, rave, name));
}

RaveObject::RaveObject(RaveInstance::Ptr rave_, KinBodyPtr body_, TrimeshMode trimeshMode, bool isKinematic_) {
	initRaveObject(rave_, body_, trimeshMode, isKinematic_);
}

void RaveObject::init() {
	CompoundObject<BulletObject>::init();

	typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
	BOOST_FOREACH( map_t::value_type &joint_cnt, jointMap ) { 	
	  getEnvironment()->addConstraint(joint_cnt.second);
  }
}

void RaveObject::destroy() {
	CompoundObject<BulletObject>::destroy();

	typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
	map_t mmap;
	BOOST_FOREACH( map_t::value_type &joint_cnt, mmap ) getEnvironment()->removeConstraint(joint_cnt.second);

}


static BulletObject::Ptr createFromLink(KinBody::LinkPtr link,
        std::vector<boost::shared_ptr<btCollisionShape> >& subshapes,
        std::vector<boost::shared_ptr<btStridingMeshInterface> >& meshes,
         TrimeshMode trimeshMode,
        bool isKinematic) {

  LOG_DEBUG("creating link from " << link->GetName());

#if OPENRAVE_VERSION_MINOR>6
  const std::vector<boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES> > & geometries=link->GetGeometries();
#else
  const std::list<KinBody::Link::GEOMPROPERTIES> &geometries =link->GetGeometries();
#endif
	// sometimes the OpenRAVE link might not even have any geometry data associated with it
	// (this is the case with the PR2 model). therefore just add an empty BulletObject
	// pointer so we know to skip it in the future
	if (geometries.empty()) {
		return BulletObject::Ptr();
	}

//	bool useCompound = geometries.size() > 1;
	bool useCompound = true;
  bool useGraphicsMesh = false;

	btCompoundShape* compound;
	if (useCompound) {
    compound = new btCompoundShape();
    compound->setMargin(1e-5*METERS); //margin: compound. seems to have no effect when positive but has an effect when negative
	}



#if OPENRAVE_VERSION_MINOR>6
	BOOST_FOREACH(const boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES>& geom, geometries) {
#else
	for (std::list<KinBody::Link::GEOMPROPERTIES>::const_iterator geom = geometries.begin(); geom != geometries.end(); ++geom) {
#endif

	  const KinBody::Link::TRIMESH &mesh = geom->GetCollisionMesh();
		boost::shared_ptr<btCollisionShape> subshape;

		switch (geom->GetType()) {
		case KinBody::Link::GEOMPROPERTIES::GeomBox:
			subshape.reset(new btBoxShape(util::toBtVector(GeneralConfig::scale
					* geom->GetBoxExtents()) + btVector3(1,1,1)*BulletConfig::linkPadding*METERS));
			break;

		case KinBody::Link::GEOMPROPERTIES::GeomSphere:
			subshape.reset(new btSphereShape(GeneralConfig::scale
					* geom->GetSphereRadius() + BulletConfig::linkPadding*METERS));
			break;

		case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
			// cylinder axis aligned to Y
			subshape.reset(new btCylinderShapeZ(btVector3(GeneralConfig::scale
					* geom->GetCylinderRadius(), GeneralConfig::scale
					* geom->GetCylinderRadius(), GeneralConfig::scale
					* geom->GetCylinderHeight() / 2.)));
			break;

		case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
			if (mesh.indices.size() < 3)
				break;
      else {
        btTriangleMesh* ptrimesh = new btTriangleMesh();
        // for some reason adding indices makes everything crash
        /*
         printf("-----------\n");
         for (int z = 0; z < mesh.indices.size(); ++z)
         printf("%d\n", mesh.indices[z]);
         printf("-----------\n");*/


        for (size_t i = 0; i < mesh.indices.size(); i += 3)
          ptrimesh->addTriangle(util::toBtVector(mesh.vertices[i])*METERS,
                      util::toBtVector(mesh.vertices[i+1])*METERS,
                      util::toBtVector(mesh.vertices[i+2])*METERS);
        // store the trimesh somewhere so it doesn't get deallocated by the smart pointer
        meshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));

        if (BulletConfig::graphicsMesh) useGraphicsMesh = true;

        if (trimeshMode == CONVEX_HULL) {
          boost::shared_ptr<btConvexShape> pconvexbuilder(new btConvexTriangleMeshShape(ptrimesh));
          pconvexbuilder->setMargin(BulletConfig::linkPadding*METERS); // margin: hull padding

          //Create a hull shape to approximate Trimesh
          boost::shared_ptr<btShapeHull> hull(new btShapeHull(pconvexbuilder.get()));
          hull->buildHull(-666); // note: margin argument not used

          btConvexHullShape *convexShape = new btConvexHullShape();
          for (int i = 0; i < hull->numVertices(); ++i)
            convexShape->addPoint(hull->getVertexPointer()[i]);

          subshape.reset(convexShape);

        } else { // RAW
          subshape.reset(new btBvhTriangleMeshShape(ptrimesh, true));
        }
      }
			break;

		default:
			break;
		}

		if (!subshape) {
			cout << "did not create geom type %d\n", geom->GetType();
			continue;
		}

		// store the subshape somewhere so it doesn't get deallocated by the smart pointer
		subshapes.push_back(subshape);
//		if (geom->GetType() == KinBody::Link::GEOMPROPERTIES::GeomTrimesh) subshape->setMargin(0);
		subshape->setMargin(BulletConfig::margin*METERS);  //margin: subshape. seems to result in padding convex shape AND increases collision dist on top of that
		btTransform geomTrans = util::toBtTransform(geom->GetTransform(),GeneralConfig::scale);
		if (useCompound) compound->addChildShape(geomTrans, subshape.get());
	}


	float mass = link->GetMass();
	if (mass==0 && !isKinematic) LOG_WARN_FMT("warning: link %s is non-kinematic but mass is zero", link->GetName().c_str());
	BulletObject::Ptr child;
	if (useCompound) {
    btTransform childTrans = util::toBtTransform(link->GetTransform(),GeneralConfig::scale);
	  child.reset(new BulletObject(mass, compound,childTrans,isKinematic));
	}
	else {
	  btTransform geomTrans = util::toBtTransform(link->GetTransform() * link->GetGeometry(0)->GetTransform(),METERS);
    child.reset(new BulletObject(mass, subshapes.back(), geomTrans, isKinematic));
    if (useGraphicsMesh) child->graphicsShape.reset(new btBvhTriangleMeshShape(meshes.back().get(), true));
	}

	return child;

}

BulletConstraint::Ptr createFromJoint(KinBody::JointPtr joint, std::map<KinBody::LinkPtr, BulletObject::Ptr> linkMap) {

	KinBody::LinkPtr joint1 = joint->GetFirstAttached();
	KinBody::LinkPtr joint2 = joint->GetSecondAttached();

	if (!joint1 || !joint2 || !linkMap[joint1] || !linkMap[joint2])
		return BulletConstraint::Ptr();

	btRigidBody* body0 = linkMap[joint->GetFirstAttached()]->rigidBody.get();
	btRigidBody* body1 = linkMap[joint->GetSecondAttached()]->rigidBody.get();

	Transform t0inv = (joint)->GetFirstAttached()->GetTransform().inverse();
	Transform t1inv = (joint)->GetSecondAttached()->GetTransform().inverse();
	btTypedConstraint* cnt;

	switch ((joint)->GetType()) {
	case KinBody::Joint::JointHinge: {
		btVector3 pivotInA = util::toBtVector(t0inv * (joint)->GetAnchor())*METERS;
		btVector3 pivotInB = util::toBtVector(t1inv * (joint)->GetAnchor())*METERS;
		btVector3 axisInA = util::toBtVector(t0inv.rotate((joint)->GetAxis(0)));
		btVector3 axisInB = util::toBtVector(t1inv.rotate((joint)->GetAxis(0)));
		btHingeConstraint* hinge = new btHingeConstraint(*body0, *body1,
				pivotInA, pivotInB, axisInA, axisInB);
		if (!(joint)->IsCircular(0)) {
			vector<dReal> vlower, vupper;
			(joint)->GetLimits(vlower, vupper);
			btScalar orInitialAngle = (joint)->GetValue(0);
			btScalar btInitialAngle = hinge->getHingeAngle();
			btScalar lower_adj, upper_adj;
			btScalar diff = (btInitialAngle + orInitialAngle);
			lower_adj = diff - vupper.at(0);
			upper_adj = diff - vlower.at(0);
			hinge->setLimit(lower_adj, upper_adj);
		}
		cnt = hinge;
    LOG_INFO("hinge joint");
		break;
	}
	case KinBody::Joint::JointSlider: {
		Transform tslider;
		tslider.rot = quatRotateDirection(Vector(1, 0, 0), (joint)->GetAxis(0));
		btTransform frameInA = util::toBtTransform(t0inv * tslider, METERS);
		btTransform frameInB = util::toBtTransform(t1inv * tslider, METERS);
		cnt = new btSliderConstraint(*body0, *body1, frameInA, frameInB, true);
    LOG_INFO("slider joint");
		break;
	}
	case KinBody::Joint::JointUniversal:
		RAVELOG_ERROR ( "universal joint not supported by bullet\n");
		break;
	case KinBody::Joint::JointHinge2:
		RAVELOG_ERROR("hinge2 joint not supported by bullet\n");
		break;
	default:
		LOG_INFO("unknown joint type: " << joint->GetType());
		break;
	}
	return BulletConstraint::Ptr(new BulletConstraint(cnt, true));
}

void RaveObject::initRaveObject(RaveInstance::Ptr rave_, KinBodyPtr body_,
		TrimeshMode trimeshMode, bool isKinematic_) {
	rave = rave_;
	body = body_;
	rave->rave2bulletsim[body] = this;
  rave->bulletsim2rave[this] = body;

	const std::vector<KinBody::LinkPtr> &links = body->GetLinks();

	isKinematic = isKinematic_;

	getChildren().reserve(links.size());
	// iterate through each link in the robot (to be stored in the children vector)
	BOOST_FOREACH(KinBody::LinkPtr link, links) {
		BulletObject::Ptr child = createFromLink(link, subshapes, meshes, trimeshMode, isKinematic);
		if (child) getChildren().push_back(child);

		linkMap[link] = child;
		childPosMap[child] = getChildren().size() - 1;
		if (child) {
			collisionObjMap[child->rigidBody.get()] = link;
		// since the joints are always in contact, we should ignore their collisions
		// when setting joint positions (OpenRAVE should take care of them anyway)
			ignoreCollisionWith(child->rigidBody.get());
		}
	}

	if (!isKinematic) {
		vector<KinBody::JointPtr> vbodyjoints; vbodyjoints.reserve(body->GetJoints().size()+body->GetPassiveJoints().size());
		vbodyjoints.insert(vbodyjoints.end(),body->GetJoints().begin(),body->GetJoints().end());
		vbodyjoints.insert(vbodyjoints.end(),body->GetPassiveJoints().begin(),body->GetPassiveJoints().end());
		BOOST_FOREACH(KinBody::JointPtr joint, vbodyjoints) {
			BulletConstraint::Ptr constraint = createFromJoint(joint, linkMap);
			if (constraint) {
				// todo: put this in init:
				// getEnvironment()->bullet->dynamicsWorld->addConstraint(constraint->cnt, bIgnoreCollision);
				jointMap[joint] = constraint;
			}
		}
	}
}

bool RaveObject::detectCollisions() {
	getEnvironment()->bullet->dynamicsWorld->updateAabbs();

	BulletInstance::CollisionObjectSet objs;
	for (int i = 0; i < getChildren().size(); ++i) {
		BulletObject::Ptr child = getChildren()[i];
		if (!child)
			continue;
		objs.clear();
		getEnvironment()->bullet->contactTest(child->rigidBody.get(), objs,
				&ignoreCollisionObjs);
		if (!objs.empty()) {
			// collision!
			return true;
		}
	}
	return false;
}

void RaveRobotObject::setDOFValues(const vector<int> &indices, const vector<dReal> &vals) {
	robot->SetActiveDOFs(indices);
  robot->SetActiveDOFValues(vals);
	updateBullet();
	typedef map<RaveObject::Ptr, KinBody::LinkPtr>::value_type Targ2GrabberPair;
	BOOST_FOREACH(Targ2GrabberPair& targ_grabber, m_targ2grabber) targ_grabber.first->updateBullet();
}

void RaveObject::prePhysics() {
  CompoundObject<BulletObject>::prePhysics();
}

void RaveObject::updateRave() {
  assert(children.size() ==1);
  body->SetTransform(util::toRaveTransform(children[0]->rigidBody->getCenterOfMassTransform(),1/METERS));
//    children[i]->motionState->setKinematicPos(util::toBtTransform(transforms[linkIndsWithGeometry[i]],GeneralConfig::scale));
}

void RaveObject::updateBullet() {
	// update bullet structures
	// we gave OpenRAVE the DOFs, now ask it for the equivalent transformations
	// which are easy to feed into Bullet
	vector<OpenRAVE::Transform> transforms;
	body->GetLinkTransformations(transforms);
	const vector<KinBody::LinkPtr>& links = body->GetLinks();

	if (linkIndsWithGeometry.size()==0) {
	  for (int i=0; i < links.size(); ++i) if (associatedObj(links[i])) linkIndsWithGeometry.push_back(i);
	}

	for (int i=0; i < children.size(); ++i)
	  children[i]->motionState->setKinematicPos(util::toBtTransform(transforms[linkIndsWithGeometry[i]],GeneralConfig::scale));
}

vector<double> RaveRobotObject::getDOFValues(const vector<int>& indices) {
	robot->SetActiveDOFs(indices);
	vector<double> out;
	robot->GetActiveDOFValues(out);
	return out;
}

vector<double> RaveRobotObject::getDOFValues() {
	vector<double> out;
	robot->GetDOFValues(out);
	return out;
}

void RaveObject::internalCopy(RaveObject::Ptr o, Fork &f) const {
	CompoundObject<BulletObject>::internalCopy(o, f); // copies all children

	if (f.rave) {
	  o->rave = f.rave;
	}
	else {
	  o->rave.reset(new RaveInstance(*rave, OpenRAVE::Clone_Bodies));
	}

	// now we need to set up mappings in the copied robot
	for (std::map<KinBody::LinkPtr, BulletObject::Ptr>::const_iterator i =
			linkMap.begin(); i != linkMap.end(); ++i) {
		const KinBody::LinkPtr raveObj = o->rave->env->GetKinBody(
				i->first->GetParent()->GetName())->GetLink(i->first->GetName());

		const int j = childPosMap.find(i->second)->second;
		const BulletObject::Ptr bulletObj = o->getChildren()[j];

		o->linkMap.insert(std::make_pair(raveObj, bulletObj));
		o->collisionObjMap.insert(std::make_pair(bulletObj->rigidBody.get(),
				raveObj));
	}

	for (std::map<BulletObject::Ptr, int>::const_iterator i =
			childPosMap.begin(); i != childPosMap.end(); ++i) {
		const int j = childPosMap.find(i->first)->second;
		o->childPosMap.insert(std::make_pair(o->getChildren()[j], i->second));
	}

	o->body = o->rave->env->GetKinBody(body->GetName());
}

EnvironmentObject::Ptr RaveObject::copy(Fork &f) const {
	Ptr o(new RaveObject());
  RaveObject::internalCopy(o, f);
	return o;
}

EnvironmentObject::Ptr RaveRobotObject::copy(Fork &f) const {
	Ptr o(new RaveRobotObject());
  RaveObject::internalCopy(o, f);

	o->robot = o->rave->env->GetRobot(robot->GetName());
	o->createdManips.reserve(createdManips.size());
	for (int i = 0; i < createdManips.size(); ++i) {
		o->createdManips.push_back(createdManips[i]->copy(o, f));
		o->createdManips[i]->index = i;
	}
	return o;
}

void RaveObject::postCopy(EnvironmentObject::Ptr copy, Fork &f) const {
	Ptr o = boost::static_pointer_cast<RaveObject>(copy);

	for (BulletInstance::CollisionObjectSet::const_iterator i =
			ignoreCollisionObjs.begin(); i != ignoreCollisionObjs.end(); ++i)
		o->ignoreCollisionObjs.insert((btCollisionObject *) f.copyOf(*i));
}

RaveRobotObject::RaveRobotObject(RaveInstance::Ptr rave_, RobotBasePtr robot_, TrimeshMode trimeshMode, bool isKinematic_) {
	robot = robot_;
	initRaveObject(rave_, robot_, trimeshMode, isKinematic_);
}

RaveRobotObject::RaveRobotObject(RaveInstance::Ptr rave_, const std::string &uri, TrimeshMode trimeshMode, bool isKinematic_) {
	robot = rave_->env->ReadRobotURI(uri);
	initRaveObject(rave_, robot, trimeshMode, isKinematic_);
	rave->env->AddRobot(robot);
}

void RaveRobotObject::grab(RaveObject::Ptr targ, KinBody::LinkPtr link) {
  assert(targ->children.size()==1);
  targ->updateRave();
  robot->Grab(targ->body, link);
	targ->children[0]->setKinematic(true);
  m_targ2grabber[targ] = link;
	m_grabber2targ[link] = targ;
}

void RaveRobotObject::release(RaveObject::Ptr targ) {
  robot->Release(targ->body);
  targ->children[0]->setKinematic(false);
  KinBody::LinkPtr link = m_targ2grabber[targ];
  m_targ2grabber.erase(targ);
  m_grabber2targ.erase(link);
}

KinBody::LinkPtr RaveRobotObject::getGrabberLink(RaveObject::Ptr target) {
  return m_targ2grabber[target];
}


RobotManipulatorPtr RaveRobotObject::getManipByName(const std::string& name) {
  BOOST_FOREACH(RobotManipulatorPtr manip, createdManips) {
    if (manip->manip->GetName() == name) return manip;
  }
  return Manipulator::Ptr();
}

RaveRobotObject::Manipulator::Ptr RaveRobotObject::createManipulator(
		const std::string &manipName) {
	if (getManipByName(manipName)) return getManipByName(manipName);
  RaveRobotObject::Manipulator::Ptr m(new Manipulator(this));
	// initialize the ik module
  robot->SetActiveManipulator(manipName);
  m->manip = m->origManip = robot->GetActiveManipulator();
#if 0
	m->ikmodule = RaveCreateModule(rave->env, "ikfast");
	rave->env->AddModule(m->ikmodule, "");
#endif

	// stringstream ssin, ssout;
	// ssin << "LoadIKFastSolver " << robot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
	// if (!m->ikmodule->SendCommand(ssout, ssin)) {
	//   cout << "failed to load iksolver\n";
	//     return Manipulator::Ptr(); // null
	// }

	m->index = createdManips.size();
	createdManips.push_back(m);
	return m;
}

bool RaveRobotObject::Manipulator::solveIKUnscaled(
		const OpenRAVE::Transform &targetTrans, vector<dReal> &vsolution) {

	vsolution.clear();
	// TODO: lock environment?!?!
	// notice: we use origManip, which is the original manipulator (after cloning)
	// this way we don't have to clone the iksolver, which is attached to the manipulator
	if (!origManip->FindIKSolution(IkParameterization(targetTrans), vsolution,
      IKFO_CheckEnvCollisions |
			IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions)) {
		cout << "ik failed on " << manip->GetName() << endl;
		return false;
	}
	return true;
}

bool RobotManipulator::solveAllIKUnscaled(
		const OpenRAVE::Transform &targetTrans,
		vector<vector<dReal> > &vsolutions) {

	vsolutions.clear();
	// see comments for solveIKUnscaled
	if (!origManip->FindIKSolutions(IkParameterization(targetTrans),
			vsolutions,
      IKFO_CheckEnvCollisions |
      IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions)) {
		std::cout << "ik  failed on " << manip->GetName() << endl;
		return false;
	}
	return true;
}

bool RobotManipulator::moveByIKUnscaled(
		const OpenRAVE::Transform &targetTrans, bool checkCollisions,
		bool revertOnCollision) {

	vector<dReal> vsolution;
	if (!solveIKUnscaled(targetTrans, vsolution))
		return false;

	// save old manip pose if we might want to revert
	vector<dReal> oldVals(0);
	if (checkCollisions && revertOnCollision) {
		robot->robot->SetActiveDOFs(manip->GetArmIndices());
		robot->robot->GetActiveDOFValues(oldVals);
	}

	// move the arm
	robot->setDOFValues(manip->GetArmIndices(), vsolution);
	cout << "dof values: " << vsolution << endl;
	if (checkCollisions && robot->detectCollisions()) {
		if (revertOnCollision)
			robot->setDOFValues(manip->GetArmIndices(), oldVals);
		return false;
	}

	return true;
}

float RobotManipulator::getGripperAngle() {
	vector<int> inds = manip->GetGripperIndices();
	vector<double> vals = robot->getDOFValues(inds);
	return vals[0];
}

void RobotManipulator::setGripperAngle(float x) {
	vector<int> inds = manip->GetGripperIndices();
	vector<double> vals(inds.size(),x);
	robot->setDOFValues(inds, vals);
}

vector<double> RobotManipulator::getDOFValues() {
	return robot->getDOFValues(manip->GetArmIndices());
}

void RobotManipulator::setDOFValues(const vector<double>& vals) {
	robot->setDOFValues(manip->GetArmIndices(), vals);
}


RobotManipulator::Ptr RaveRobotObject::Manipulator::copy(
		RaveRobotObject::Ptr newRobot, Fork &f) {
	OpenRAVE::EnvironmentMutex::scoped_lock lock(
			newRobot->rave->env->GetMutex());

	RobotManipulator::Ptr o(new RobotManipulator(newRobot.get()));

	o->ikmodule = ikmodule;
	o->origManip = origManip;

	newRobot->robot->SetActiveManipulator(manip->GetName());
	o->manip = newRobot->robot->GetActiveManipulator();

	return o;
}

void RaveRobotObject::destroyManipulator(RaveRobotObject::Manipulator::Ptr m) {
	std::vector<Manipulator::Ptr>::iterator i = std::find(
			createdManips.begin(), createdManips.end(), m);
	if (i == createdManips.end())
		return;
	(*i).reset();
	createdManips.erase(i);
}
