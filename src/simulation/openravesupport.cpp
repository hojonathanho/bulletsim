#include "openravesupport.h"
#include <openrave-core.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include "convexdecomp.h"
#include "utils/config.h"
#include "bullet_io.h"
#include "utils/logging.h"
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


btVector3 computeCentroid(const KinBody::Link::TRIMESH& mesh) {
	btVector3 sum(0, 0, 0);
	BOOST_FOREACH(const RaveVector<double>& v, mesh.vertices) {
		sum += util::toBtVector(v);
	}
	return sum / mesh.vertices.size();
}

RaveInstance::RaveInstance() {
	isRoot = true;
	RaveInitialize(true);
	env = RaveCreateEnvironment();
	if (GeneralConfig::verbose  <= log4cplus::DEBUG_LOG_LEVEL)
		RaveSetDebugLevel(Level_Debug);
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

void Load(Environment::Ptr env, RaveInstance::Ptr rave, const string& filename,
		bool dynamicRobots) {
	bool success = rave->env->Load(filename);
	if (!success)
		throw runtime_error(
				(boost::format("couldn't load %s!\n") % (filename)).str());

	std::set<string> bodiesAlreadyLoaded;
	BOOST_FOREACH(EnvironmentObject::Ptr obj, env->objects) {
		RaveObject* robj = dynamic_cast<RaveObject*>(obj.get());
		if (robj) bodiesAlreadyLoaded.insert(robj->body->GetName());
	}

	std::vector<boost::shared_ptr<OpenRAVE::KinBody> > bodies;
	rave->env->GetBodies(bodies);
	BOOST_FOREACH(OpenRAVE::KinBodyPtr body, bodies) {
		if (bodiesAlreadyLoaded.find(body->GetName()) == bodiesAlreadyLoaded.end()) {
			if (body->IsRobot()) env->add(RaveRobotObject::Ptr(new RaveRobotObject(rave, boost::dynamic_pointer_cast<RobotBase>(body), CONVEX_HULL, dynamicRobots)));
			else {
				cout << "loading " << body->GetName() << endl;;
				env->add(RaveObject::Ptr(new RaveObject(rave, body, CONVEX_HULL, true)));
			}
		}
	}

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

RaveRobotObject::Ptr getRobotByName(Environment::Ptr env, RaveInstance::Ptr rave, const string& name) {
  return boost::dynamic_pointer_cast<RaveRobotObject>(getObjectByName(env, rave, name));
}

RaveObject::RaveObject(RaveInstance::Ptr rave_, KinBodyPtr body_, TrimeshMode trimeshMode, bool isDynamic) {
	initRaveObject(rave_, body_, trimeshMode, BulletConfig::margin * METERS, isDynamic);
}

void RaveObject::init() {
	CompoundObject<BulletObject>::init();

	typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
	BOOST_FOREACH( map_t::value_type &joint_cnt, jointMap ) { 	
	  getEnvironment()->addConstraint(joint_cnt.second);
  }
}

void RaveObject::destroy() {
	CompoundObject<BulletObject>::init();

	typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
	map_t mmap;
	BOOST_FOREACH( map_t::value_type &joint_cnt, mmap ) getEnvironment()->removeConstraint(joint_cnt.second);
}


btCompoundShape* shiftTransform(btCompoundShape* boxCompound,btScalar mass,btTransform& shift)
{
	btCompoundShape* newBoxCompound;
	btTransform principal;
	btVector3 principalInertia;
	btScalar* masses = new btScalar[boxCompound->getNumChildShapes()];
	for (int j=0;j<boxCompound->getNumChildShapes();j++)
	{
		//evenly distribute mass
		masses[j]=mass/boxCompound->getNumChildShapes();
	}


	boxCompound->calculatePrincipalAxisTransform(masses,principal,principalInertia);


	///create a new compound with world transform/center of mass properly aligned with the principal axis

	///non-recursive compound shapes perform better

#ifdef USE_RECURSIVE_COMPOUND

	btCompoundShape* newCompound = new btCompoundShape();
	newCompound->addChildShape(principal.inverse(),boxCompound);
	newBoxCompound = newCompound;
	//m_collisionShapes.push_back(newCompound);

	//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,newCompound,principalInertia);

#else
#ifdef CHANGE_COMPOUND_INPLACE
	newBoxCompound = boxCompound;
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		boxCompound->updateChildTransform(i,newChildTransform);
	}
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		boxCompound->calculateLocalInertia(mass,localInertia);

#else
	///creation is faster using a new compound to store the shifted children
	newBoxCompound = new btCompoundShape();
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		newBoxCompound->addChildShape(newChildTransform,boxCompound->getChildShape(i));
	}



#endif

#endif//USE_RECURSIVE_COMPOUND

	shift = principal;
	return newBoxCompound;
}

static BulletObject::Ptr createFromLink(KinBody::LinkPtr link,
        std::vector<boost::shared_ptr<btCollisionShape> >& subshapes,
        std::vector<boost::shared_ptr<btStridingMeshInterface> >& meshes,
         TrimeshMode trimeshMode,
        float fmargin, bool isDynamic) {

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

	// each link is a compound of several btCollisionShapes
	btCompoundShape *compound = new btCompoundShape();
	compound->setMargin(0); //margin: compound. seems to have no effect when positive but has an effect when negative

	float volumeAccumulator(0);
	btVector3 firstMomentAccumulator(0,0,0);

#if OPENRAVE_VERSION_MINOR>6
	BOOST_FOREACH(const boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES>& geom, geometries) {
#else
        for (std::list<KinBody::Link::GEOMPROPERTIES>::const_iterator geom = geometries.begin(); geom != geometries.end(); ++geom) {
#endif
		btVector3 offset(0, 0, 0);

		boost::shared_ptr<btCollisionShape> subshape;
		const KinBody::Link::TRIMESH &mesh = geom->GetCollisionMesh();

		switch (geom->GetType()) {
		case KinBody::Link::GEOMPROPERTIES::GeomBox:
			subshape.reset(new btBoxShape(util::toBtVector(GeneralConfig::scale
					* geom->GetBoxExtents())));
			break;

		case KinBody::Link::GEOMPROPERTIES::GeomSphere:
			subshape.reset(new btSphereShape(GeneralConfig::scale
					* geom->GetSphereRadius()));
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
			if (trimeshMode == CONVEX_DECOMP) {
				printf("running convex decomposition\n");
				ConvexDecomp decomp(fmargin);
				for (size_t i = 0; i < mesh.vertices.size(); ++i)
					decomp.addPoint(util::toBtVector(mesh.vertices[i]));
				for (size_t i = 0; i < mesh.indices.size(); i += 3)
					decomp.addTriangle(mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]);
				subshape = decomp.run(subshapes); // use subshapes to just store smart pointer

			} else {


				btTriangleMesh* ptrimesh = new btTriangleMesh();
				// for some reason adding indices makes everything crash
				/*
				 printf("-----------\n");
				 for (int z = 0; z < mesh.indices.size(); ++z)
				 printf("%d\n", mesh.indices[z]);
				 printf("-----------\n");*/

				offset = computeCentroid(mesh)*METERS*0;

				for (size_t i = 0; i < mesh.indices.size(); i += 3)
					ptrimesh->addTriangle(util::toBtVector(mesh.vertices[i])*METERS - offset,
										  util::toBtVector(mesh.vertices[i+1])*METERS - offset,
										  util::toBtVector(mesh.vertices[i+2])*METERS - offset);
				// store the trimesh somewhere so it doesn't get deallocated by the smart pointer
				meshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));

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
		subshape->setMargin(0); //margin: subshape. seems to result in padding convex shape AND increases collision dist on top of that
		btTransform transform = util::toBtTransform(geom->GetTransform(),GeneralConfig::scale);
		transform.setOrigin(transform.getOrigin() + offset);
		compound->addChildShape(transform, subshape.get());
	}

	btTransform childTrans = util::toBtTransform(link->GetTransform(),GeneralConfig::scale);

	float mass = isDynamic ? link->GetMass() : 0;
	BulletObject::Ptr child(new BulletObject(mass, compound, childTrans,!isDynamic));
//	child->drawingOn=false;
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
		TrimeshMode trimeshMode, float fmargin, bool isDynamic) {
	rave = rave_;
	body = body_;

	const std::vector<KinBody::LinkPtr> &links = body->GetLinks();
	getChildren().reserve(links.size());
	// iterate through each link in the robot (to be stored in the children vector)
	BOOST_FOREACH(KinBody::LinkPtr link, links) {
		BulletObject::Ptr child = createFromLink(link, subshapes, meshes, trimeshMode, fmargin, isDynamic && !link->IsStatic());
		getChildren().push_back(child);

		linkMap[link] = child;
		childPosMap[child] = getChildren().size() - 1;
		if (child) {
			collisionObjMap[child->rigidBody.get()] = link;
		// since the joints are always in contact, we should ignore their collisions
		// when setting joint positions (OpenRAVE should take care of them anyway)
			ignoreCollisionWith(child->rigidBody.get());
	}
}

if (isDynamic) {
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

void RaveRobotObject::setDOFValues(const vector<int> &indices, const vector<
		dReal> &vals) {
	// update openrave structure
	{
		EnvironmentMutex::scoped_lock lock(rave->env->GetMutex());
		robot->SetActiveDOFs(indices);
		robot->SetActiveDOFValues(vals);
		rave->env->UpdatePublishedBodies();
	}
	updateBullet();
}

void RaveObject::updateBullet() {
	// update bullet structures
	// we gave OpenRAVE the DOFs, now ask it for the equivalent transformations
	// which are easy to feed into Bullet
	vector<OpenRAVE::Transform> transforms;
	body->GetLinkTransformations(transforms);
	BOOST_ASSERT(transforms.size() == getChildren().size());
	for (int i = 0; i < getChildren().size(); ++i) {
		BulletObject::Ptr c = getChildren()[i];
		if (!c)
			continue;
		c->motionState->setKinematicPos(util::toBtTransform(transforms[i],
															GeneralConfig::scale));
	}

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

EnvironmentObject::Ptr RaveObject::copy(Fork &f) const {
	Ptr o(new RaveObject());

	internalCopy(o, f); // copies all children

	o->rave.reset(new RaveInstance(*rave, OpenRAVE::Clone_Bodies));

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

	return o;
}

EnvironmentObject::Ptr RaveRobotObject::copy(Fork &f) const {

	Ptr o(new RaveRobotObject());

	/////////////////////// duplicated from RaveObject::copy //////////////////
	internalCopy(o, f); // copies all children

	o->rave.reset(new RaveInstance(*rave, OpenRAVE::Clone_Bodies));

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
	/////////////////// end duplicated portion //////////////////


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

RaveRobotObject::RaveRobotObject(RaveInstance::Ptr rave_, RobotBasePtr robot_, TrimeshMode trimeshMode, bool isDynamic) {
	robot = robot_;
	initRaveObject(rave_, robot_, trimeshMode, BulletConfig::margin * METERS, isDynamic);
}

RaveRobotObject::RaveRobotObject(RaveInstance::Ptr rave_, const std::string &uri, TrimeshMode trimeshMode, bool isDynamic) {
	robot = rave_->env->ReadRobotURI(uri);
	initRaveObject(rave_, robot, trimeshMode, BulletConfig::margin * METERS, isDynamic);
	rave->env->AddRobot(robot);
}

RaveRobotObject::Manipulator::Ptr RaveRobotObject::createManipulator(
		const std::string &manipName, bool useFakeGrabber) {
	RaveRobotObject::Manipulator::Ptr m(new Manipulator(this));
	// initialize the ik module
	robot->SetActiveManipulator(manipName);
	m->manip = m->origManip = robot->GetActiveManipulator();
	m->ikmodule = RaveCreateModule(rave->env, "ikfast");
	rave->env->AddModule(m->ikmodule, "");
	// stringstream ssin, ssout;
	// ssin << "LoadIKFastSolver " << robot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
	// if (!m->ikmodule->SendCommand(ssout, ssin)) {
	//   cout << "failed to load iksolver\n";
	//     return Manipulator::Ptr(); // null
	// }

	m->useFakeGrabber = useFakeGrabber;
	if (useFakeGrabber) {
		m->grabber.reset(new GrabberKinematicObject(0.02, 0.05));
		m->updateGrabberPos();
		ignoreCollisionWith(m->grabber->rigidBody.get());
	}

	m->index = createdManips.size();
	createdManips.push_back(m);
	return m;
}

void RaveRobotObject::Manipulator::updateGrabberPos() {
	// set the grabber right on top of the end effector
	if (useFakeGrabber)
		grabber->motionState->setKinematicPos(getTransform());
}

bool RaveRobotObject::Manipulator::solveIKUnscaled(
		const OpenRAVE::Transform &targetTrans, vector<dReal> &vsolution) {

	vsolution.clear();
	// TODO: lock environment?!?!
	// notice: we use origManip, which is the original manipulator (after cloning)
	// this way we don't have to clone the iksolver, which is attached to the manipulator
	if (!origManip->FindIKSolution(IkParameterization(targetTrans), vsolution,
			IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions)) {
		cout << "ik failed on " << manip->GetName() << endl;
		return false;
	}
	return true;
}

bool RaveRobotObject::Manipulator::solveAllIKUnscaled(
		const OpenRAVE::Transform &targetTrans,
		vector<vector<dReal> > &vsolutions) {

	vsolutions.clear();
	// see comments for solveIKUnscaled
	if (!origManip->FindIKSolutions(IkParameterization(targetTrans),
			vsolutions, true)) {
		std::cout << "ik  failed on " << manip->GetName() << endl;
		return false;
	}
	return true;
}

bool RaveRobotObject::Manipulator::moveByIKUnscaled(
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
	if (checkCollisions && robot->detectCollisions()) {
		if (revertOnCollision)
			robot->setDOFValues(manip->GetArmIndices(), oldVals);
		return false;
	}

	if (useFakeGrabber)
		updateGrabberPos();

	return true;
}

float RaveRobotObject::Manipulator::getGripperAngle() {
	vector<int> inds = manip->GetGripperIndices();
	assert(inds.size() ==1 );
	vector<double> vals = robot->getDOFValues(inds);
	return vals[0];
}

void RaveRobotObject::Manipulator::setGripperAngle(float x) {
	vector<int> inds = manip->GetGripperIndices();
	assert(inds.size() ==1 );
	vector<double> vals;
	vals.push_back(x);
	robot->setDOFValues(inds, vals);
}

vector<double> RaveRobotObject::Manipulator::getDOFValues() {
	return robot->getDOFValues(manip->GetArmIndices());
}

void RaveRobotObject::Manipulator::setDOFValues(const vector<double>& vals) {
	robot->setDOFValues(manip->GetArmIndices(), vals);
}


RaveRobotObject::Manipulator::Ptr RaveRobotObject::Manipulator::copy(
		RaveRobotObject::Ptr newRobot, Fork &f) {
	OpenRAVE::EnvironmentMutex::scoped_lock lock(
			newRobot->rave->env->GetMutex());

	Manipulator::Ptr o(new Manipulator(newRobot.get()));

	o->ikmodule = ikmodule;
	o->origManip = origManip;

	newRobot->robot->SetActiveManipulator(manip->GetName());
	o->manip = newRobot->robot->GetActiveManipulator();

	o->useFakeGrabber = useFakeGrabber;
	if (useFakeGrabber)
		o->grabber = boost::static_pointer_cast<GrabberKinematicObject>(
				grabber->copy(f));

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


/* Adds the a trimesh built from the current state of the softbody SB to the openrave
 * environment RAVE. Returns a pointer to the kinematicbody created and added to the environment.*/
OpenRAVE::KinBodyPtr createKinBodyFromBulletSoftObject(BulletSoftObject::Ptr sb,
													   RaveInstance::Ptr rave,
													   std::string name) {
	size_t i;
	btSoftBody* psb = sb->softBody.get();
	const btSoftBody::Node*	nbase = &(psb->m_nodes[0]);

	// --> Create the Trimesh
    OpenRAVE::KinBody::Link::TRIMESH raveMesh;

    // fill in the vertices of the mesh
	 raveMesh.vertices.resize(psb->m_nodes.size());
     for(i=0;i<psb->m_nodes.size();++i)	 {
		btVector3 node_pos = psb->m_nodes[i].m_x / GeneralConfig::scale;
		OpenRAVE::Vector vec = util::toRaveVector(node_pos);
	    raveMesh.vertices[i] = vec;
	}

	// fill the indices for the faces of the trimesh
	raveMesh.indices.resize(3 * (psb->m_faces.size()),0);
	for(i=0;i<psb->m_faces.size();i++)	 {
		const btSoftBody::Face&	feat=psb->m_faces[i];
		const int	idx[]={	int(feat.m_n[0]-nbase),
             				int(feat.m_n[1]-nbase),
			             	int(feat.m_n[2]-nbase)};
		raveMesh.indices[3*i]   = idx[0];
		raveMesh.indices[3*i+1] = idx[1];
		raveMesh.indices[3*i+2] = idx[2];
	}

    // Get a random name for the body.
	if (name=="") {
		std::stringstream ss; ss << "TrimeshBody" << (int) clock();
		name = ss.str();
	}

	// create a kinbody from mesh and add to the environment
    OpenRAVE::KinBodyPtr raveBodyPtr;
    { // lock the rave env
    	OpenRAVE::EnvironmentMutex::scoped_lock lockenv(rave->env->GetMutex());
    	raveBodyPtr = OpenRAVE::RaveCreateKinBody(rave->env);
    	raveBodyPtr->InitFromTrimesh(raveMesh, true);
    	raveBodyPtr->SetName(name);
    	rave->env->AddKinBody(raveBodyPtr);
    } // rave env is unlocked
    return raveBodyPtr;
}


/* Adds an openrave box kinematic-body to the openrave environment RAVE, based on the
 * the bullet box object BOX.
 * Returns a pointer to the kinematic-body created and added to the environment. */
OpenRAVE::KinBodyPtr createKinBodyFromBulletBoxObject(BoxObject::Ptr box,
		 	 	 	 	 	 	 	 	 	 	 	  RaveInstance::Ptr rave,
		 	 	 	 	 	 	 	 	 	 	 	  std::string name) {
	// get the box orientation and size
	btTransform boxTransform;
	box->motionState->getWorldTransform(boxTransform);
	boxTransform.setOrigin(boxTransform.getOrigin() / GeneralConfig::scale);
	btVector3 halfExtents = box->getHalfExtents() / GeneralConfig::scale;

	OpenRAVE::RaveVector<OpenRAVE::dReal> pos(0,0,0);
	OpenRAVE::RaveVector<OpenRAVE::dReal> extents(halfExtents.getX(), halfExtents.getY(), halfExtents.getZ());
	std::vector<OpenRAVE::AABB> boxInfo;
	OpenRAVE::AABB ibox(pos, extents);
	boxInfo.push_back(ibox);

	// Get a random name for the body.
	if (name=="") {
		std::stringstream ss; ss << "BoxBody" << (int) clock();
		name = ss.str();
	}

	// create a kinbody from mesh and add to the environment
    OpenRAVE::KinBodyPtr raveBoxPtr;
    { // lock the rave env
    	OpenRAVE::EnvironmentMutex::scoped_lock lockenv(rave->env->GetMutex());
    	raveBoxPtr = OpenRAVE::RaveCreateKinBody(rave->env);
    	raveBoxPtr->InitFromBoxes(boxInfo, true);
    	raveBoxPtr->SetName(name);
    	raveBoxPtr->SetTransform(util::toRaveTransform(boxTransform));
    	rave->env->AddKinBody(raveBoxPtr);
    } // rave env is unlocked
    return raveBoxPtr;
}
