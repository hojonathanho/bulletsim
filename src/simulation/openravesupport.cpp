#include "openravesupport.h"
#include <openrave-core.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include "convexdecomp.h"
#include "utils/config.h"

using namespace OpenRAVE;
using boost::shared_ptr;

RaveInstance::RaveInstance() {
  isRoot = true;
  RaveInitialize(true);
  env = RaveCreateEnvironment();
  if (GeneralConfig::verbose)
    RaveSetDebugLevel(Level_Debug);
}

RaveInstance::RaveInstance(const RaveInstance &o, int cloneOpts) {
  isRoot = false;
  env = o.env->CloneSelf(cloneOpts);
}

RaveInstance::~RaveInstance() {
  if (isRoot)
    RaveDestroy();
}



RaveRobotObject::RaveRobotObject(
				 RaveInstance::Ptr rave_,
				 const std::string &uri,
				 const btTransform &initialTransform_,
				 TrimeshMode trimeshMode,
				 bool isDynamic) :
  rave(rave_),
  initialTransform(initialTransform_),
  scale(GeneralConfig::scale) {
  robot = rave->env->ReadRobotURI(uri);
  rave->env->AddRobot(robot);
  initRobot(initialTransform, trimeshMode, .0005 * METERS, isDynamic);
  }


RaveRobotObject::RaveRobotObject(
				 RaveInstance::Ptr rave_, 
				 RobotBasePtr robot_, 
				 TrimeshMode trimeshMode,
				 bool isDynamic) : 
  rave(rave_),
  scale(GeneralConfig::scale),
  robot(robot_),
  initialTransform(btTransform::getIdentity()) 
{
  initRobot(initialTransform, trimeshMode, .0005*METERS, isDynamic);
}


void RaveRobotObject::init() {

  CompoundObject<BulletObject>::init();

  typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
  map_t mmap;  
  BOOST_FOREACH( map_t::value_type &joint_cnt, mmap ) {
    getEnvironment()->addConstraint(joint_cnt.second);
  }
}

void RaveRobotObject::destroy() {
  CompoundObject<BulletObject>::init();

  typedef std::map<KinBody::JointPtr, BulletConstraint::Ptr> map_t;
  map_t mmap;  
  BOOST_FOREACH( map_t::value_type &joint_cnt, mmap ) {
    getEnvironment()->removeConstraint(joint_cnt.second);
  }
  
}


BulletObject::Ptr createFromLink(KinBody::LinkPtr link, 
   std::vector<boost::shared_ptr<btCollisionShape> >& subshapes,     
   std::vector<boost::shared_ptr<btStridingMeshInterface> >& meshes,  
   float scale, TrimeshMode trimeshMode, float fmargin, bool isDynamic) {
  
  const std::list<KinBody::Link::GEOMPROPERTIES> &geometries = link->GetGeometries();
  // sometimes the OpenRAVE link might not even have any geometry data associated with it
  // (this is the case with the PR2 model). therefore just add an empty BulletObject
  // pointer so we know to skip it in the future
  if (geometries.empty()) {
    return BulletObject::Ptr();
  }

  // each link is a compound of several btCollisionShapes
  btCompoundShape *compound = new btCompoundShape();
  compound->setMargin(fmargin);

  for (std::list<KinBody::Link::GEOMPROPERTIES>::const_iterator geom = geometries.begin();
       geom != geometries.end(); ++geom) {
    boost::shared_ptr<btCollisionShape> subshape;
    const KinBody::Link::TRIMESH &mesh = geom->GetCollisionMesh();

    switch (geom->GetType()) {
    case KinBody::Link::GEOMPROPERTIES::GeomBox:
      subshape.reset(new btBoxShape(util::toBtVector(GeneralConfig::scale * geom->GetBoxExtents())));
      break;

    case KinBody::Link::GEOMPROPERTIES::GeomSphere:
      subshape.reset(new btSphereShape(scale * geom->GetSphereRadius()));
      break;

    case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
      // cylinder axis aligned to Y
      subshape.reset(new btCylinderShapeZ(btVector3(scale * geom->GetCylinderRadius(),
						    scale * geom->GetCylinderRadius(),
						    scale * geom->GetCylinderHeight()/2.)));
      break;

    case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
      if (mesh.indices.size() < 3) break;
      if (trimeshMode == CONVEX_DECOMP) {
	printf("running convex decomposition\n");
	ConvexDecomp decomp(fmargin);              
	for (size_t i = 0; i < mesh.vertices.size(); ++i)
	  decomp.addPoint(util::toBtVector(mesh.vertices[i]));
	for (size_t i = 0; i < mesh.indices.size(); i += 3)
	  decomp.addTriangle(mesh.indices[i], mesh.indices[i+1], mesh.indices[i+2]);
	subshape = decomp.run(subshapes); // use subshapes to just store smart pointer

      }
      else {
	btTriangleMesh* ptrimesh = new btTriangleMesh();
	// for some reason adding indices makes everything crash
	/*
	  printf("-----------\n");
	  for (int z = 0; z < mesh.indices.size(); ++z)
	  printf("%d\n", mesh.indices[z]);
	  printf("-----------\n");*/

	for (size_t i = 0; i < mesh.indices.size(); i += 3)
	  ptrimesh->addTriangle(util::toBtVector(scale * mesh.vertices[i]),
				util::toBtVector(scale * mesh.vertices[i+1]),
				util::toBtVector(scale * mesh.vertices[i+2]));
	// store the trimesh somewhere so it doesn't get deallocated by the smart pointer
	meshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));

	if (trimeshMode == CONVEX_HULL) {
	  boost::shared_ptr<btConvexShape> pconvexbuilder(new btConvexTriangleMeshShape(ptrimesh));
	  pconvexbuilder->setMargin(fmargin);

	  //Create a hull shape to approximate Trimesh
	  boost::shared_ptr<btShapeHull> hull(new btShapeHull(pconvexbuilder.get()));
	  hull->buildHull(fmargin);

	  btConvexHullShape *convexShape = new btConvexHullShape();
	  for (int i = 0; i < hull->numVertices(); ++i)
	    convexShape->addPoint(hull->getVertexPointer()[i]);

	  subshape.reset(convexShape);

	} 
	else { // RAW
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
    subshape->setMargin(fmargin);
    compound->addChildShape(util::toBtTransform(geom->GetTransform(), scale), subshape.get());
  }  
  
  
  btTransform childTrans = util::toBtTransform(link->GetTransform(), scale);
  BulletObject::Ptr child(new BulletObject(0, compound, childTrans, true));


  if (isDynamic) {
    btVector3 inertia(0, 0, 0);
    btRigidBody* rb = child->rigidBody.get();
    float mass = link->GetMass();
    rb->getCollisionShape()->calculateLocalInertia(mass, inertia);
    rb->setMassProps(mass, inertia);
  }
  
  return child;

}


BulletConstraint::Ptr createFromJoint(KinBody::JointPtr joint, std::map<KinBody::LinkPtr, BulletObject::Ptr> linkMap) {
  btRigidBody* body0 = NULL, *body1 = NULL;
  if( !!(joint)->GetFirstAttached() && !(joint)->GetFirstAttached()->IsStatic() ) {
    body0 = linkMap[joint->GetFirstAttached()]->rigidBody.get();
  }
  if( !!(joint)->GetSecondAttached() && !(joint)->GetSecondAttached()->IsStatic()) {
    body1 = linkMap[joint->GetSecondAttached()]->rigidBody.get();
  }
  if( !body0 || !body1 ) {
    RAVELOG_ERROR(str(boost::format("joint %s needs to be attached to two bodies!\n")%(joint)->GetName()));
    return BulletConstraint::Ptr();
  }

  Transform t0inv = (joint)->GetFirstAttached()->GetTransform().inverse();
  Transform t1inv = (joint)->GetSecondAttached()->GetTransform().inverse();
  btTypedConstraint* cnt;

  switch((joint)->GetType()) {
  case KinBody::Joint::JointHinge: {
    btVector3 pivotInA = util::toBtVector(t0inv * (joint)->GetAnchor());
    btVector3 pivotInB = util::toBtVector(t1inv * (joint)->GetAnchor());
    btVector3 axisInA = util::toBtVector(t0inv.rotate((joint)->GetAxis(0)));
    btVector3 axisInB = util::toBtVector(t1inv.rotate((joint)->GetAxis(0)));
    btHingeConstraint* hinge = new btHingeConstraint(*body0, *body1, pivotInA, pivotInB, axisInA, axisInB);
    if( !(joint)->IsCircular(0) ) {
      vector<dReal> vlower, vupper;
      (joint)->GetLimits(vlower,vupper);
      btScalar orInitialAngle = (joint)->GetValue(0);
      btScalar btInitialAngle = hinge->getHingeAngle();
      btScalar lower_adj, upper_adj;
      btScalar diff = (btInitialAngle + orInitialAngle);
      lower_adj = diff - vupper.at(0);
      upper_adj = diff - vlower.at(0);
      hinge->setLimit(lower_adj,upper_adj);
    }
    cnt = hinge;
    break;
  }
  case KinBody::Joint::JointSlider: {
    Transform tslider; tslider.rot = quatRotateDirection(Vector(1,0,0),(joint)->GetAxis(0));
    btTransform frameInA = util::toBtTransform(t0inv*tslider);
    btTransform frameInB = util::toBtTransform(t1inv*tslider);
    cnt = new btSliderConstraint(*body0, *body1, frameInA, frameInB, true);
    break;
  }
  case KinBody::Joint::JointUniversal:
    RAVELOG_ERROR("universal joint not supported by bullet\n");
    break;
  case KinBody::Joint::JointHinge2:
    RAVELOG_ERROR("hinge2 joint not supported by bullet\n");
    break;
  default:
    RAVELOG_ERROR("unknown joint type %d\n", joint->GetType());
    break;
  }
  return BulletConstraint::Ptr(new BulletConstraint(cnt));
}


void RaveRobotObject::initRobot(const btTransform &initialTransform, TrimeshMode trimeshMode, float fmargin, bool isDynamic) {
  const std::vector<KinBody::LinkPtr> &links = robot->GetLinks();
  getChildren().reserve(links.size());
  // iterate through each link in the robot (to be stored in the children vector)
  BOOST_FOREACH(KinBody::LinkPtr link, links) {
      
    BulletObject::Ptr child = createFromLink(link, subshapes, meshes, scale, trimeshMode, fmargin, isDynamic);
                
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
    vector<KinBody::JointPtr> vbodyjoints; vbodyjoints.reserve(robot->GetJoints().size()+robot->GetPassiveJoints().size());
    vbodyjoints.insert(vbodyjoints.end(),robot->GetJoints().begin(),robot->GetJoints().end());
    vbodyjoints.insert(vbodyjoints.end(),robot->GetPassiveJoints().begin(),robot->GetPassiveJoints().end());
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



bool RaveRobotObject::detectCollisions() {
  getEnvironment()->bullet->dynamicsWorld->updateAabbs();

  BulletInstance::CollisionObjectSet objs;
  for (int i = 0; i < getChildren().size(); ++i) {
    BulletObject::Ptr child = getChildren()[i];
    if (!child) continue;
    objs.clear();
    getEnvironment()->bullet->contactTest(child->rigidBody.get(),
					  objs, &ignoreCollisionObjs);
    if (!objs.empty()) {
      // collision!
      return true;
    }
  }
  return false;
}

void RaveRobotObject::setDOFValues(const vector<int> &indices, const vector<dReal> &vals) {
  // update openrave structure
  {
    EnvironmentMutex::scoped_lock lock(rave->env->GetMutex());
    robot->SetActiveDOFs(indices);
    robot->SetActiveDOFValues(vals);
    rave->env->UpdatePublishedBodies();
  }
  updateBullet();
}

void RaveRobotObject::updateBullet() {
  // update bullet structures
  // we gave OpenRAVE the DOFs, now ask it for the equivalent transformations
  // which are easy to feed into Bullet
  vector<OpenRAVE::Transform> transforms;
  robot->GetLinkTransformations(transforms);
  BOOST_ASSERT(transforms.size() == getChildren().size());
  for (int i = 0; i < getChildren().size(); ++i) {
    BulletObject::Ptr c = getChildren()[i];
    if (!c) continue;
    c->motionState->setKinematicPos(initialTransform * util::toBtTransform(transforms[i], scale));
  }

}

vector<double> RaveRobotObject::getDOFValues(const vector<int>& indices) {
  robot->SetActiveDOFs(indices);
  vector<double> out;
  robot->GetActiveDOFValues(out);
  return out;
}

EnvironmentObject::Ptr RaveRobotObject::copy(Fork &f) const {
  Ptr o(new RaveRobotObject());

  internalCopy(o, f); // copies all children

  o->initialTransform = initialTransform;
  o->rave.reset(new RaveInstance(*rave, OpenRAVE::Clone_Bodies));

  // now we need to set up mappings in the copied robot
  for (std::map<KinBody::LinkPtr, BulletObject::Ptr>::const_iterator i = linkMap.begin();
       i != linkMap.end(); ++i) {
    const KinBody::LinkPtr raveObj = o->rave->env->GetKinBody(i->first->GetParent()->GetName())->GetLink(i->first->GetName());

    const int j = childPosMap.find(i->second)->second;
    const BulletObject::Ptr bulletObj = o->getChildren()[j];

    o->linkMap.insert(std::make_pair(raveObj, bulletObj));
    o->collisionObjMap.insert(std::make_pair(bulletObj->rigidBody.get(), raveObj));
  }

  for (std::map<BulletObject::Ptr, int>::const_iterator i = childPosMap.begin();
       i != childPosMap.end(); ++i) {
    const int j = childPosMap.find(i->first)->second;
    o->childPosMap.insert(std::make_pair(o->getChildren()[j], i->second));
  }

  o->robot = o->rave->env->GetRobot(robot->GetName());

  o->createdManips.reserve(createdManips.size());
  for (int i = 0; i < createdManips.size(); ++i) {
    o->createdManips.push_back(createdManips[i]->copy(o, f));
    o->createdManips[i]->index = i;
  }

  return o;
}

void RaveRobotObject::postCopy(EnvironmentObject::Ptr copy, Fork &f) const {
  Ptr o = boost::static_pointer_cast<RaveRobotObject>(copy);

  for (BulletInstance::CollisionObjectSet::const_iterator i = ignoreCollisionObjs.begin();
       i != ignoreCollisionObjs.end(); ++i)
    o->ignoreCollisionObjs.insert((btCollisionObject *) f.copyOf(*i));
}

RaveRobotObject::Manipulator::Ptr
RaveRobotObject::createManipulator(const std::string &manipName, bool useFakeGrabber) {
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
						   const OpenRAVE::Transform &targetTrans,
						   vector<dReal> &vsolution) {

  vsolution.clear();
  // TODO: lock environment?!?!
  // notice: we use origManip, which is the original manipulator (after cloning)
  // this way we don't have to clone the iksolver, which is attached to the manipulator
  if (!origManip->FindIKSolution(IkParameterization(targetTrans), vsolution, IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions)) {
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
  if (!origManip->FindIKSolutions(IkParameterization(targetTrans), vsolutions, true)) {
    std::cout << "ik  failed on " << manip->GetName() << endl;
    return false;
  }
  return true;
}

bool RaveRobotObject::Manipulator::moveByIKUnscaled(
						    const OpenRAVE::Transform &targetTrans,
						    bool checkCollisions, bool revertOnCollision) {

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

RaveRobotObject::Manipulator::Ptr
RaveRobotObject::Manipulator::copy(RaveRobotObject::Ptr newRobot, Fork &f) {
  OpenRAVE::EnvironmentMutex::scoped_lock lock(newRobot->rave->env->GetMutex());

  Manipulator::Ptr o(new Manipulator(newRobot.get()));

  o->ikmodule = ikmodule;
  o->origManip = origManip;

  newRobot->robot->SetActiveManipulator(manip->GetName());
  o->manip = newRobot->robot->GetActiveManipulator();

  o->useFakeGrabber = useFakeGrabber;
  if (useFakeGrabber)
    o->grabber = boost::static_pointer_cast<GrabberKinematicObject>(grabber->copy(f));

  return o;
}

void RaveRobotObject::destroyManipulator(RaveRobotObject::Manipulator::Ptr m) {
  std::vector<Manipulator::Ptr>::iterator i =
    std::find(createdManips.begin(), createdManips.end(), m);
  if (i == createdManips.end()) return;
  (*i).reset();
  createdManips.erase(i);
}
