#ifndef _OPENRAVESUPPORT_H_
#define _OPENRAVESUPPORT_H_

#include <openrave/openrave.h>
using namespace OpenRAVE;

#include <btBulletDynamicsCommon.h>

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>
using namespace std;

#include "environment.h"
#include "basicobjects.h"

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

struct RaveInstance {
    typedef boost::shared_ptr<RaveInstance> Ptr;

    EnvironmentBasePtr env;

    RaveInstance();
    ~RaveInstance();
};

class RaveRobotKinematicObject : public EnvironmentObject {
private:
    // utility functions
    static inline btTransform GetBtTransform(const Transform& t) {
        return btTransform(btQuaternion(t.rot.y,t.rot.z,t.rot.w,t.rot.x),GetBtVector(t.trans));
    }

    static inline OpenRAVE::Transform GetRaveTransform(const btTransform &t) {
        OpenRAVE::Transform s;
        s.rot = OpenRAVE::Vector(t.getRotation().w(), t.getRotation().x(), t.getRotation().y(), t.getRotation().z());
        s.trans = OpenRAVE::Vector(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
        return s;
    }

    static inline btVector3 GetBtVector(const Vector& v) {
        return btVector3(v.x,v.y,v.z);
    }

    RaveInstance::Ptr rave;
    RobotBasePtr robot;
    btTransform initialTransform;
    std::vector<BulletKinematicObject::Ptr> children;

    // these two containers just keep track of the smart pointers
    // so that the objects get deallocated on destruction
    std::vector<boost::shared_ptr<btStridingMeshInterface> > meshes;
    std::vector<boost::shared_ptr<btCollisionShape> > subshapes;

    // for the loaded robot, this will create BulletKinematicObjects
    // and place them into the children vector
    void initRobotWithoutDynamics(const btTransform &initialTransform, float fmargin=0.0005);

public:
    typedef boost::shared_ptr<RaveRobotKinematicObject> Ptr;

    // this class is actually a collection of BulletKinematicObjects,
    // each of which represents a link of the robot
    RaveRobotKinematicObject(RaveInstance::Ptr rave_, const std::string &uri, const btTransform &initialTransform_);

    // EnvironmentObject methods
    // these act on each sub-object
    void init();
    void prePhysics();
    void preDraw();

    // position the robot according to DOF values in the OpenRAVE model
    // and copy link positions to the Bullet rigid bodies
    void setDOFValues(const vector<int> &indices, const vector<dReal> &vals);

    // IK support
    struct Manipulator {
        RaveRobotKinematicObject *robot;
        ModuleBasePtr ikmodule;
        RobotBase::ManipulatorPtr manip;
        GrabberKinematicObject::Ptr grabber;
        void updateGrabberPos();

        typedef boost::shared_ptr<Manipulator> Ptr;
        Manipulator(RaveRobotKinematicObject *robot_) : robot(robot_) { }
        void moveByIK(const OpenRAVE::Transform &targetTrans);
        void moveByIK(const btTransform &targetTrans) { moveByIK(GetRaveTransform(targetTrans)); }
    };
    Manipulator::Ptr createManipulator(const std::string &manipName);
};

class OpenRAVESupport {
public:
    
    class KinematicMotionState : public btMotionState {
    private:
        btTransform trans;

    public:
        KinematicMotionState(const btTransform &trans_) : trans(trans_) { }
        void getWorldTransform(btTransform &worldTrans) const { worldTrans = trans; }
        void setWorldTransform(const btTransform &) { }
        void setKinematicPos(const btTransform &pos) { trans = pos; }
    };

    // information about the kinematics of the body
    class KinBodyInfo : public UserData
    {
    public:
        struct LINK : public btMotionState
        {
            virtual ~LINK() { }
            virtual void getWorldTransform(btTransform& centerOfMassWorldTrans ) const {
                centerOfMassWorldTrans = GetBtTransform(plink->GetTransform());
            }
            virtual void setWorldTransform(const btTransform& centerOfMassWorldTrans) {
                Vector quat(centerOfMassWorldTrans.getRotation().w(), centerOfMassWorldTrans.getRotation().x(), centerOfMassWorldTrans.getRotation().y(), centerOfMassWorldTrans.getRotation().z());
                Vector trans(centerOfMassWorldTrans.getOrigin().x(),centerOfMassWorldTrans.getOrigin().y(),centerOfMassWorldTrans.getOrigin().z());
                plink->SetTransform(Transform(quat,trans));
            }

            boost::shared_ptr<btCollisionObject> obj;
            boost::shared_ptr<btRigidBody> _rigidbody;
            boost::shared_ptr<btCollisionShape> shape;
            list<boost::shared_ptr<btCollisionShape> > listchildren;
            list<boost::shared_ptr<btStridingMeshInterface> > listmeshes;

            KinBody::LinkPtr plink;
            Transform tlocal;     /// local offset transform to account for inertias not aligned to axes
        };

        KinBodyInfo(boost::shared_ptr<btCollisionWorld> world, bool bPhysics) : _world(world) {
            nLastStamp = 0;
            _worlddynamics = boost::dynamic_pointer_cast<btDiscreteDynamicsWorld>(_world);
        }
        virtual ~KinBodyInfo() { Reset(); }
        void Reset() {
#if 0
            // added code: remove all associated constraints, or we'll get an assertion error from bullet
            for (MAPJOINTS::iterator itmapjoint = _mapjoints.begin();
                    itmapjoint != _mapjoints.end(); ++itmapjoint) {
                _worlddynamics->removeConstraint(itmapjoint->second.get());
            }
#endif

            FOREACH(itlink, vlinks) {
                _worlddynamics->removeRigidBody((*itlink)->_rigidbody.get());
            }
            vlinks.resize(0);
            _mapjoints.clear();
            _geometrycallback.reset();
        }

        void setDOFValues(const vector<int> &indices, const vector<dReal> &vals);
        void moveActiveManipByIk(const Transform &targetEndEffectorTrans);

        EnvironmentBasePtr env;
        RobotBasePtr probot;
        KinBodyPtr pbody;     ///< body associated with this structure
        int nLastStamp;

        btTransform initialTransform;
        std::vector<boost::shared_ptr<KinematicMotionState> > motionstates;
        std::vector<boost::shared_ptr<LINK> > vlinks;     ///< if body is disabled, then geom is static (it can't be connected to a joint!)
        ///< the pointer to this Link is the userdata
        typedef std::map< KinBody::JointConstPtr, boost::shared_ptr<btTypedConstraint> > MAPJOINTS;
        MAPJOINTS _mapjoints;
        boost::shared_ptr<void> _geometrycallback;
    //    boost::weak_ptr<BulletSpace> _bulletspace;

    private:
        boost::shared_ptr<btCollisionWorld> _world;
        boost::shared_ptr<btDiscreteDynamicsWorld> _worlddynamics;
    };
    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;


    OpenRAVESupport(boost::shared_ptr<btDiscreteDynamicsWorld> btWorld_) : btWorld(btWorld_) { }
    void initRAVE();
    ~OpenRAVESupport() { RaveDestroy(); }
    KinBodyInfoPtr loadURIIntoBullet(const char *uri, const btTransform &offsetTrans);

private:
    EnvironmentBasePtr env;
    boost::shared_ptr<btDiscreteDynamicsWorld> btWorld;
    vector<KinBodyInfoPtr> loadedBodies;

    static inline btTransform GetBtTransform(const Transform& t) {
        return btTransform(btQuaternion(t.rot.y,t.rot.z,t.rot.w,t.rot.x),GetBtVector(t.trans));
    }

    static inline btVector3 GetBtVector(const Vector& v) {
        return btVector3(v.x,v.y,v.z);
    }

#if 0
    void _Synchronize(KinBodyInfoPtr pinfo) {
        vector<Transform> vtrans;
        pinfo->pbody->GetLinkTransformations(vtrans);
        pinfo->nLastStamp = pinfo->pbody->GetUpdateStamp();
        BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
        for(size_t i = 0; i < vtrans.size(); ++i) {
            pinfo->vlinks[i]->obj->getWorldTransform() = GetBtTransform(vtrans[i]*pinfo->vlinks[i]->tlocal);
        }
    }
#endif

    // takes an OpenRAVE KinBody and converts it to Bullet structures and places it into the given btWorld
//    KinBodyInfoPtr InitKinBody(KinBodyPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr(), btScalar fmargin=0.0005);
    KinBodyInfoPtr InitRobotWithoutDynamics(RobotBasePtr probot, const btTransform &transform, KinBodyInfoPtr pinfo = KinBodyInfoPtr(), btScalar fmargin = 0.0005);

};

#endif // _OPENRAVESUPPORT_H_
