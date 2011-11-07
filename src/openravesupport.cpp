#include "openravesupport.h"
#include <openrave-core.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
using namespace OpenRAVE;

RaveInstance::RaveInstance() {
    RaveInitialize(true);
    env = RaveCreateEnvironment();
    RaveSetDebugLevel(Level_Debug);
}

RaveInstance::~RaveInstance() {
    RaveDestroy();
}

RaveRobotKinematicObject::RaveRobotKinematicObject(
        RaveInstance::Ptr rave_, const std::string &uri,
        const btTransform &initialTransform_) : rave(rave_), initialTransform(initialTransform_) {
    robot = rave->env->ReadRobotURI(uri);
    rave->env->AddRobot(robot);
    initRobotWithoutDynamics(initialTransform);
}

void RaveRobotKinematicObject::initRobotWithoutDynamics(const btTransform &initialTransform, float fmargin) {
    const std::vector<KinBody::LinkPtr> &links = robot->GetLinks();
    children.reserve(links.size());
    // iterate through each link in the robot (to be stored in the children vector)
    for (std::vector<KinBody::LinkPtr>::const_iterator link = links.begin(); link != links.end(); ++link) {
        const std::list<KinBody::Link::GEOMPROPERTIES> &geometries = (*link)->GetGeometries();
        // sometimes the OpenRAVE link might not even have any geometry data associated with it
        // (this is the case with the PR2 model). therefore just add an empty BulletKinematicObject
        // pointer so we know to skip it in the future
        if (geometries.empty()) {
            children.push_back(BulletKinematicObject::Ptr());
            continue;
        }

        // each link is a compound of several btCollisionShapes
        boost::shared_ptr<btCompoundShape> compound(new btCompoundShape());
        compound->setMargin(fmargin);

        for (std::list<KinBody::Link::GEOMPROPERTIES>::const_iterator geom = geometries.begin();
                geom != geometries.end(); ++geom) {
            boost::shared_ptr<btCollisionShape> subshape;

            switch (geom->GetType()) {
            case KinBody::Link::GEOMPROPERTIES::GeomBox:
                subshape.reset(new btBoxShape(GetBtVector(geom->GetBoxExtents())));
                break;

            case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                subshape.reset(new btSphereShape(geom->GetSphereRadius()));
                break;

            case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                // cylinder axis aligned to Y
                subshape.reset(new btCylinderShapeZ(btVector3(geom->GetCylinderRadius(), geom->GetCylinderRadius(), geom->GetCylinderHeight()/2.)));
                break;

            case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
                if (geom->GetCollisionMesh().indices.size() >= 3) {
                    // TODO: not sure if leaking memory here...
                    btTriangleMesh* ptrimesh = new btTriangleMesh();

                    // for some reason adding indices makes everything crash
                    for(size_t i = 0; i < geom->GetCollisionMesh().indices.size(); i += 3)
                        ptrimesh->addTriangle(GetBtVector(geom->GetCollisionMesh().vertices[i]),
                                              GetBtVector(geom->GetCollisionMesh().vertices[i+1]),
                                              GetBtVector(geom->GetCollisionMesh().vertices[i+2]));

                    RAVELOG_DEBUG("converting triangle mesh to convex hull\n");
                    boost::shared_ptr<btConvexShape> pconvexbuilder(new btConvexTriangleMeshShape(ptrimesh));
                    pconvexbuilder->setMargin(fmargin);

                    //Create a hull shape to approximate Trimesh
                    boost::shared_ptr<btShapeHull> hull(new btShapeHull(pconvexbuilder.get()));
                    hull->buildHull(fmargin);

                    btConvexHullShape *convexShape = new btConvexHullShape();
                    for (int i = 0; i < hull->numVertices(); ++i)
                        convexShape->addPoint(hull->getVertexPointer()[i]);

                    subshape.reset(convexShape);
                    // store the trimesh somewhere so it doesn't get deallocated by the smart pointer
                    meshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));
                }
                break;

            default:
                break;
            }

            if (!subshape) {
                RAVELOG_WARN("did not create geom type %d\n", geom->GetType());
                continue;
            }

            // store the subshape somewhere so it doesn't get deallocated by the smart pointer
            subshapes.push_back(subshape);
            subshape->setMargin(fmargin);
            compound->addChildShape(GetBtTransform(geom->GetTransform()), subshape.get());
        }

        btTransform childTrans = initialTransform * GetBtTransform((*link)->GetTransform());
        BulletKinematicObject::Ptr child(new BulletKinematicObject(compound, childTrans));
        children.push_back(child);
    }
}

void RaveRobotKinematicObject::init() {
    std::vector<BulletKinematicObject::Ptr>::iterator i;
    for (i = children.begin(); i != children.end(); ++i) {
        if (*i) {
            (*i)->setEnvironment(getEnvironment());
            (*i)->init();
        }
    }
}

void RaveRobotKinematicObject::prePhysics() {
    std::vector<BulletKinematicObject::Ptr>::iterator i;
    for (i = children.begin(); i != children.end(); ++i)
        if (*i)
            (*i)->prePhysics();
}

void RaveRobotKinematicObject::preDraw() {
    std::vector<BulletKinematicObject::Ptr>::iterator i;
    for (i = children.begin(); i != children.end(); ++i)
        if (*i)
            (*i)->preDraw();
}

void RaveRobotKinematicObject::setDOFValues(const vector<int> &indices, const vector<dReal> &vals) {
    // update openrave structure
    {
        EnvironmentMutex::scoped_lock lock(rave->env->GetMutex());
        robot->SetActiveDOFs(indices);
        robot->SetActiveDOFValues(vals);
        rave->env->UpdatePublishedBodies();
    }

    // update bullet structures
    // we gave OpenRAVE the DOFs, now ask it for the equivalent transformations
    // which are easy to feed into Bullet
    vector<OpenRAVE::Transform> transforms;
    robot->GetLinkTransformations(transforms);

    // iterate through the transforms vector and the vlinks at the same time
    BOOST_ASSERT(transforms.size() == children.size());
    for (int i = 0; i < transforms.size(); ++i)
        if (children[i])
            children[i]->getKinematicMotionState().setKinematicPos(
                initialTransform * GetBtTransform(transforms[i]));
}

RaveRobotKinematicObject::ManipulatorIKModel::Ptr
RaveRobotKinematicObject::createManipulatorIKModel(const std::string &manipName) {
    RaveRobotKinematicObject::ManipulatorIKModel::Ptr m(new ManipulatorIKModel);
    robot->SetActiveManipulator(manipName);
    m->ikmodule = RaveCreateModule(rave->env, "ikfast");
    rave->env->AddModule(m->ikmodule, "");
    stringstream ssin, ssout;
    ssin << "LoadIKFastSolver " << robot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
    if (!m->ikmodule->SendCommand(ssout, ssin)) {
        RAVELOG_ERROR("failed to load iksolver\n");
        return ManipulatorIKModel::Ptr(); // null
    }
    return m;
}

void RaveRobotKinematicObject::moveByIK(ManipulatorIKModel::Ptr ikmodel, const OpenRAVE::Transform &targetTrans) {
    if (!ikmodel) return;

    vector<dReal> vsolution;
    // TODO: lock environment?!?!
    if (!ikmodel->manip->FindIKSolution(IkParameterization(targetTrans), vsolution, true)) {
        stringstream ss;
        ss << "failed to get solution for target transform for end effector: " << targetTrans << endl;
        RAVELOG_INFO(ss.str());
        return;
    }
    setDOFValues(ikmodel->manip->GetArmIndices(), vsolution);
}

#if 0
void OpenRAVESupport::initRAVE() {
    RaveInitialize(true);
    env = RaveCreateEnvironment();
    RaveSetDebugLevel(Level_Debug);
}

OpenRAVESupport::KinBodyInfoPtr OpenRAVESupport::loadURIIntoBullet(const char *uri, const btTransform &offsetTrans) {
/*    KinBodyPtr kinbody = env->ReadKinBodyURI(uri);
    env->AddKinBody(kinbody);
    printf("active dofs (1): %d\n", kinbody->GetDOF());
    KinBodyInfoPtr info = InitKinBody(kinbody);*/

    RobotBasePtr robot = env->ReadRobotURI(uri);
    env->AddRobot(robot);
    robot->SetActiveManipulator("rightarm");

    // load IK
    ModuleBasePtr pikfast = RaveCreateModule(env, "ikfast");
    env->AddModule(pikfast, "");
    stringstream ssin, ssout;
    ssin << "LoadIKFastSolver " << robot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
    if (!pikfast->SendCommand(ssout, ssin)) {
        RAVELOG_ERROR("failed to load iksolver\n");
    }

    KinBodyInfoPtr info = InitRobotWithoutDynamics(robot, offsetTrans);
    loadedBodies.push_back(info);
    return info;
}

void OpenRAVESupport::KinBodyInfo::setDOFValues(const vector<int> &indices, const vector<dReal> &vals) {
    // update openrave structure (pbody)
    probot->SetActiveDOFs(indices);
    probot->SetActiveDOFValues(vals);

    // update bullet structures
    // we gave OpenRAVE the DOFs, now ask it for the equivalent transformations
    // which are easy to feed into Bullet
    vector<Transform> transforms;
    pbody->GetLinkTransformations(transforms);

    // iterate through the transforms vector and the vlinks at the same time
    BOOST_ASSERT(transforms.size() == vlinks.size());
    for (int i = 0; i < transforms.size(); ++i) {
        if (vlinks[i] == boost::shared_ptr<KinBodyInfo::LINK>()) continue;
        ((KinematicMotionState *)vlinks[i]->_rigidbody->getMotionState())->setKinematicPos(initialTransform * GetBtTransform(transforms[i]));
    }
}

void OpenRAVESupport::KinBodyInfo::moveActiveManipByIk(const Transform &targetEndEffectorTrans) {
    vector<dReal> vsolution;
    EnvironmentMutex::scoped_lock lock(env->GetMutex());
    if (!probot->GetActiveManipulator()->FindIKSolution(IkParameterization(targetEndEffectorTrans), vsolution, true)) {
        stringstream ss; ss << "failed to get solution for target transform for end effector: " << targetEndEffectorTrans << endl;
        RAVELOG_INFO(ss.str());
        return;
    }
    setDOFValues(probot->GetActiveManipulator()->GetArmIndices(), vsolution);
    env->UpdatePublishedBodies();
}


OpenRAVESupport::KinBodyInfoPtr OpenRAVESupport::InitRobotWithoutDynamics(RobotBasePtr probot, const btTransform &transform, KinBodyInfoPtr pinfo, btScalar fmargin) {
    KinBodyPtr pbody = probot;
    if( !pinfo ) {
        pinfo.reset(new KinBodyInfo(btWorld, true));
    }
    pinfo->Reset();
    pinfo->env = env;
    pinfo->pbody = pbody;
    pinfo->probot = probot;
    pinfo->vlinks.reserve(pbody->GetLinks().size());
    RAVELOG_DEBUG("creating %d links\n", pbody->GetLinks().size());

    FOREACHC(itlink, pbody->GetLinks()) {
        if ((*itlink)->GetGeometries().size() == 0) {
            pinfo->vlinks.push_back(boost::shared_ptr<KinBodyInfo::LINK>());
            continue;
        }

        boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK());

        btCompoundShape* pshapeparent = new btCompoundShape();
        link->shape.reset(pshapeparent);
        RAVELOG_DEBUG("link has %d geometries\n", (*itlink)->GetGeometries().size());
        pshapeparent->setMargin(fmargin);     // need to set margin very small (we're not simulating anyway)

        // add all the correct geometry objects
        FOREACHC(itgeom, (*itlink)->GetGeometries()) {
            boost::shared_ptr<btCollisionShape> child;
            switch(itgeom->GetType()) {
            case KinBody::Link::GEOMPROPERTIES::GeomBox:
                child.reset(new btBoxShape(GetBtVector(itgeom->GetBoxExtents())));
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                child.reset(new btSphereShape(itgeom->GetSphereRadius()));
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                // cylinder axis aligned to Y
                child.reset(new btCylinderShapeZ(btVector3(itgeom->GetCylinderRadius(),itgeom->GetCylinderRadius(),itgeom->GetCylinderHeight()*0.5f)));
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomTrimesh: {
                if( itgeom->GetCollisionMesh().indices.size() >= 3 ) {
                    btTriangleMesh* ptrimesh = new btTriangleMesh();

                    // for some reason adding indices makes everything crash
                    for(size_t i = 0; i < itgeom->GetCollisionMesh().indices.size(); i += 3) {
                        ptrimesh->addTriangle(GetBtVector(itgeom->GetCollisionMesh().vertices[i]), GetBtVector(itgeom->GetCollisionMesh().vertices[i+1]), GetBtVector(itgeom->GetCollisionMesh().vertices[i+2]));
                    }
                    RAVELOG_DEBUG("converting triangle mesh to convex hull\n");
                    boost::shared_ptr<btConvexShape> pconvexbuilder(new btConvexTriangleMeshShape(ptrimesh));
                    pconvexbuilder->setMargin(fmargin);

                    //Create a hull shape to approximate Trimesh
                    boost::shared_ptr<btShapeHull> hull(new btShapeHull(pconvexbuilder.get()));
                    hull->buildHull(fmargin);

                    btConvexHullShape* convexShape = new btConvexHullShape();
                    for (int i=0; i< (hull->numVertices()); i++) {
                        convexShape->addPoint(hull->getVertexPointer()[i]);
                    }
                    child.reset(convexShape);
                    link->listmeshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));
                }
                break;
            }
            default:
                break;
            }

            if( !child ) {
                RAVELOG_WARN("did not create geom type %d\n", itgeom->GetType());
                continue;
            }

            link->listchildren.push_back(child);
            child->setMargin(fmargin);     // need to set margin very small (we're not simulating anyway)
            pshapeparent->addChildShape(GetBtTransform(itgeom->GetTransform()), child.get());
        }
        pinfo->initialTransform = transform;
        btTransform totalTrans = GetBtTransform((*itlink)->GetTransform() /* * tinertiaframe*/);
  //      totalTrans.setOrigin(transform.getOrigin() + totalTrans.getOrigin());
  //      totalTrans.setRotation(transform.getRotation() * totalTrans.getRotation());
        totalTrans = transform * totalTrans;
        boost::shared_ptr<KinematicMotionState> motionstate(new KinematicMotionState(totalTrans));
        pinfo->motionstates.push_back(motionstate);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(0., motionstate.get(), pshapeparent, /*localInertia*/btVector3(0., 0., 0.));
        link->_rigidbody.reset(new btRigidBody(rbInfo));
        link->obj = link->_rigidbody;

        link->plink = *itlink;
        link->obj->setUserPointer(&link->plink);
        link->obj->setCollisionFlags(link->obj->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        link->obj->setActivationState(DISABLE_DEACTIVATION);

        if( !(*itlink)->IsStatic() ) {
            RAVELOG_DEBUG("adding rigid body\n");
            btWorld->addRigidBody(link->_rigidbody.get());
        }
        else {
            RAVELOG_DEBUG("adding collision object (static)\n");
            btWorld->addCollisionObject(link->obj.get());
        }
        link->obj->activate(true);
        pinfo->vlinks.push_back(link);
    }

    return pinfo;
}
#endif
