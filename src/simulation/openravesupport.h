#ifndef _OPENRAVESUPPORT_H_
#define _OPENRAVESUPPORT_H_

#include <openrave/openrave.h>
using namespace OpenRAVE;

#include <btBulletDynamicsCommon.h>

#include <vector>
using namespace std;

#include "environment.h"
#include "basicobjects.h"
#include "util.h"
#include "config_bullet.h"

class RaveObject;

struct RaveInstance {
  typedef boost::shared_ptr<RaveInstance> Ptr;

  bool isRoot;
  EnvironmentBasePtr env;
  std::map<KinBodyPtr, RaveObject*> rave2bulletsim;
  std::map<RaveObject*, KinBodyPtr> bulletsim2rave;

  RaveInstance();
  RaveInstance(OpenRAVE::EnvironmentBasePtr);
  RaveInstance(const RaveInstance &o, int cloneOpts);
  ~RaveInstance();
};

void LoadFromRave(Environment::Ptr env, RaveInstance::Ptr rave);
void Load(Environment::Ptr env, RaveInstance::Ptr rave, const string& name);
// copy constructor. will never call RaveInitialize or RaveDestroy

enum TrimeshMode {
  CONVEX_DECOMP, // use HACD convex decomposition
  CONVEX_HULL, // use btShapeHull
  RAW, // use btBvhTriangleMeshShape (not recommended, makes simulation very slow)
};

class RaveObject : public CompoundObject<BulletObject> {
protected:

  // these two containers just keep track of the smart pointers
  // so that the objects get deallocated on destruction
  std::vector<boost::shared_ptr<btStridingMeshInterface> > meshes;
  std::vector<boost::shared_ptr<btCollisionShape> > subshapes;

  // for looking up the associated Bullet object for an OpenRAVE link
  std::map<KinBody::LinkPtr, BulletObject::Ptr> linkMap;
  std::map<KinBody::JointPtr, BulletConstraint::Ptr> jointMap;
  std::map<btCollisionObject *, KinBody::LinkPtr> collisionObjMap;

  // maps a child to a position in the children array. used for copying
  std::map<BulletObject::Ptr, int> childPosMap;

  // maps from child index to link index. only used in updateBullet
  std::vector<int> linkIndsWithGeometry;

  // vector of objects to ignore collision with
  BulletInstance::CollisionObjectSet ignoreCollisionObjs;

  // for the loaded robot, this will create BulletObjects
  // and place them into the children vector
  void initRaveObject(RaveInstance::Ptr rave_, KinBodyPtr body_, TrimeshMode trimeshMode, bool isKinematic);
  RaveObject() {} // for manual copying
  bool isKinematic;

public:
  typedef boost::shared_ptr<RaveObject> Ptr;
  RaveInstance::Ptr rave;
  KinBodyPtr body;

  RaveObject(RaveInstance::Ptr rave_, KinBodyPtr body, TrimeshMode trimeshMode = CONVEX_HULL, bool isKinematic=true);
  // This constructor assumes the robot is already in openrave. Use this if you're loading a bunch of stuff from an
  // xml file, and you want to put everything in bullet

  void init();
  void destroy();
  void prePhysics();

  // forking
  EnvironmentObject::Ptr copy(Fork &f) const;
  void postCopy(EnvironmentObject::Ptr copy, Fork &f) const;

  // Gets equivalent rigid bodies in OpenRAVE and in Bullet
  BulletObject::Ptr associatedObj(KinBody::LinkPtr link) const {
    std::map<KinBody::LinkPtr, BulletObject::Ptr>::const_iterator i = linkMap.find(link);
    return i == linkMap.end() ? BulletObject::Ptr() : i->second;
  }
  KinBody::LinkPtr associatedObj(btCollisionObject *obj) const {
    std::map<btCollisionObject *, KinBody::LinkPtr>::const_iterator i = collisionObjMap.find(obj);
    return i == collisionObjMap.end() ? KinBody::LinkPtr() : i->second;
  }

  btTransform toRaveFrame(const btTransform &t) const { return util::scaleTransform(t, 1./GeneralConfig::scale); }
  btTransform toWorldFrame(const btTransform &t) const { return util::scaleTransform(t, GeneralConfig::scale); }
  // When getting transforms of links, you must remember to scale!
  // or just get the transforms directly from the equivalent Bullet rigid bodies
  btTransform getLinkTransform(KinBody::LinkPtr link) const {
    return toWorldFrame(util::toBtTransform(link->GetTransform()));
  }

  void ignoreCollisionWith(const btCollisionObject *obj) { ignoreCollisionObjs.insert(obj); }
  // Returns true if the robot's current pose collides with anything in the environment
  // (this will call updateAabbs() on the dynamicsWorld)
  bool detectCollisions();

  // Positions the robot according to DOF values in the OpenRAVE model
  // and copy link positions to the Bullet rigid bodies.
  void updateBullet();
  // update's openrave stuff based on bullet
  void updateRave();

};

class RaveRobotObject : public RaveObject {
public:
  typedef boost::shared_ptr<RaveRobotObject> Ptr;
  RobotBasePtr robot;

  map<RaveObject::Ptr, KinBody::LinkPtr> m_targ2grabber;
  map<KinBody::LinkPtr, RaveObject::Ptr> m_grabber2targ;


  RaveRobotObject(RaveInstance::Ptr rave_, RobotBasePtr robot, TrimeshMode trimeshMode = CONVEX_HULL, bool isStatic=true);
  RaveRobotObject(RaveInstance::Ptr rave_, const std::string &uri, TrimeshMode trimeshMode = CONVEX_HULL, bool isStatic=true);

  EnvironmentObject::Ptr copy(Fork &f) const;

  void setDOFValues(const vector<int> &indices, const vector<dReal> &vals);
  vector<double> getDOFValues(const vector<int> &indices);
  vector<double> getDOFValues();
  void setTransform(const btTransform& trans) {
    robot->SetTransform(util::toRaveTransform(util::scaleTransform(trans,1/METERS)));
    updateBullet();
  }
  void setTransform(float x, float y, float a) {
    setTransform(btTransform(btQuaternion(0,0,a), btVector3(x,y,0)));
  }
  btTransform getTransform() {
    return util::toBtTransform(robot->GetTransform(), METERS);
  }
	void grab(RaveObject::Ptr target, KinBody::LinkPtr link);
  void release(RaveObject::Ptr target);
	KinBody::LinkPtr getGrabberLink(RaveObject::Ptr target);


  struct Manipulator {
    RaveRobotObject *robot;
    ModuleBasePtr ikmodule;
    RobotBase::ManipulatorPtr manip, origManip;
    int index; // id for this manipulator in this robot instance

    bool useFakeGrabber;
    GrabberKinematicObject::Ptr grabber;
    void updateGrabberPos();

    typedef boost::shared_ptr<Manipulator> Ptr;
    Manipulator(RaveRobotObject *robot_) : robot(robot_) { }

    btTransform getTransform() const {
      return robot->toWorldFrame(util::toBtTransform(manip->GetTransform()));
    }
    // Gets one IK solution closest to the current position in joint space
    bool solveIKUnscaled(const OpenRAVE::Transform &targetTrans,
			 vector<dReal> &vsolution);
    bool solveIK(const btTransform &targetTrans, vector<dReal> &vsolution) {
      return solveIKUnscaled(
              util::toRaveTransform(robot->toRaveFrame(targetTrans)),
              vsolution);
    }

    // Gets all IK solutions
    bool solveAllIKUnscaled(const OpenRAVE::Transform &targetTrans,
			    vector<vector<dReal> > &vsolutions);
    bool solveAllIK(const btTransform &targetTrans,
		    vector<vector<dReal> > &vsolutions) {
      return solveAllIKUnscaled(
              util::toRaveTransform(robot->toRaveFrame(targetTrans)),
              vsolutions);
    }
    vector<double> getDOFValues();
    void setDOFValues(const vector<double>& vals);

    // Moves the manipulator with IK to targetTrans in unscaled coordinates
    // Returns false if IK cannot find a solution
    // If checkCollisions is true, then this will return false if the new
    // robot pose collides with anything in the environment (true otherwise).
    // The robot will revert is position to the pre-collision state if
    // revertOnCollision is set to true.
    bool moveByIKUnscaled(const OpenRAVE::Transform &targetTrans, bool checkCollisions = false, bool revertOnCollision=true);
    // Moves the manipulator in scaled coordinates
    bool moveByIK(const btTransform &targetTrans, bool checkCollisions = false, bool revertOnCollision=true) {
      return moveByIKUnscaled(util::toRaveTransform(robot->toRaveFrame(targetTrans)), checkCollisions, revertOnCollision);
    }

    float getGripperAngle();
    void setGripperAngle(float);

    Manipulator::Ptr copy(RaveRobotObject::Ptr newRobot, Fork &f);
  };

  // If useFakeGrabber is true, the manipulator will use a GrabberKinematicObject
  // which can "grab" objects by simply setting a point constraint with the nearest
  // object in front of the manipulator. Pass in false for realistic grasping.
  Manipulator::Ptr createManipulator(const std::string &manipName, bool useFakeGrabber         = false);
  void destroyManipulator(Manipulator::Ptr m); // not necessary to call this on destruction
  Manipulator::Ptr getManipByIndex(int i) const { return createdManips[i]; }
  Manipulator::Ptr getManipByName(const std::string& name);
  int numCreatedManips() const { return createdManips.size(); }
protected:
  std::vector<Manipulator::Ptr> createdManips;
  RaveRobotObject() {}
};

std::vector<RaveRobotObject::Ptr> getRobots(Environment::Ptr env, RaveInstance::Ptr rave);
RaveObject::Ptr getObjectByName(Environment::Ptr env, RaveInstance::Ptr rave, const string& name);
RaveRobotObject::Ptr getRobotByName(Environment::Ptr env, RaveInstance::Ptr rave, const string& name);

class ScopedRobotSave {
  std::vector<double> m_dofvals;
  OpenRAVE::Transform m_tf;
  OpenRAVE::RobotBasePtr m_robot;
public:
  ScopedRobotSave(OpenRAVE::RobotBasePtr robot) : m_robot(robot) {
    robot->GetDOFValues(m_dofvals);
    m_tf = robot->GetTransform();
  }
  ~ScopedRobotSave() {
    m_robot->SetDOFValues(m_dofvals);
    m_robot->SetTransform(m_tf);
  }
};






#endif // _OPENRAVESUPPORT_H_
