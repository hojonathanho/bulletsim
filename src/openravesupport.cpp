#include "openravesupport.h"
#include <openrave-core.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include "userconfig.h"

using namespace OpenRAVE;

RaveInstance::RaveInstance() {
    RaveInitialize(true);
    env = RaveCreateEnvironment();
    if (CFG.verbose)
        RaveSetDebugLevel(Level_Debug);
}

RaveInstance::~RaveInstance() {
    RaveDestroy();
}

RaveRobotKinematicObject::RaveRobotKinematicObject(
        RaveInstance::Ptr rave_, const std::string &uri,
        const btTransform &initialTransform_, btScalar scale_) :
            rave(rave_),
            initialTransform(initialTransform_),
            scale(scale_) {
    robot = rave->env->ReadRobotURI(uri);
    rave->env->AddRobot(robot);
    initRobotWithoutDynamics(initialTransform);
}

void RaveRobotKinematicObject::initRobotWithoutDynamics(const btTransform &initialTransform, bool useConvexHull, float fmargin) {
    const std::vector<KinBody::LinkPtr> &links = robot->GetLinks();
    getChildren().reserve(links.size());
    // iterate through each link in the robot (to be stored in the children vector)
    for (std::vector<KinBody::LinkPtr>::const_iterator link = links.begin(); link != links.end(); ++link) {
        const std::list<KinBody::Link::GEOMPROPERTIES> &geometries = (*link)->GetGeometries();
        // sometimes the OpenRAVE link might not even have any geometry data associated with it
        // (this is the case with the PR2 model). therefore just add an empty BulletKinematicObject
        // pointer so we know to skip it in the future
        if (geometries.empty()) {
            getChildren().push_back(BulletKinematicObject::Ptr());
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
                subshape.reset(new btBoxShape(util::toBtVector(scale * geom->GetBoxExtents())));
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
                if (geom->GetCollisionMesh().indices.size() >= 3) {
                    btTriangleMesh* ptrimesh = new btTriangleMesh();
                    // for some reason adding indices makes everything crash
                    for(size_t i = 0; i < geom->GetCollisionMesh().indices.size(); i += 3)
                        ptrimesh->addTriangle(util::toBtVector(scale * geom->GetCollisionMesh().vertices[i]),
                                              util::toBtVector(scale * geom->GetCollisionMesh().vertices[i+1]),
                                              util::toBtVector(scale * geom->GetCollisionMesh().vertices[i+2]));
                    // store the trimesh somewhere so it doesn't get deallocated by the smart pointer
                    meshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));

                    if (useConvexHull) {
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
                    else {
                        subshape.reset(new btBvhTriangleMeshShape(ptrimesh, true));
                    }
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
            compound->addChildShape(util::toBtTransform(geom->GetTransform(), scale), subshape.get());
        }

        btTransform childTrans = initialTransform * util::toBtTransform((*link)->GetTransform(), scale);
        BulletKinematicObject::Ptr child(new BulletKinematicObject(compound, childTrans));
        getChildren().push_back(child);

        // since the joints are always in contact, we should ignore their collisions
        // when setting joint positions (OpenRAVE should take care of them anyway)
        ignoreCollisionWith(child->rigidBody.get());
    }
}

bool RaveRobotKinematicObject::detectCollisions() {
    getEnvironment()->bullet->dynamicsWorld->updateAabbs();

    BulletInstance::CollisionObjectSet objs;
    for (int i = 0; i < getChildren().size(); ++i) {
        BulletKinematicObject::Ptr child = getChildren()[i];
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
    BOOST_ASSERT(transforms.size() == getChildren().size());
    for (int i = 0; i < getChildren().size(); ++i)
        if (getChildren()[i])
            getChildren()[i]->getKinematicMotionState().setKinematicPos(
                initialTransform * util::toBtTransform(transforms[i], scale));
}

RaveRobotKinematicObject::Manipulator::Ptr
RaveRobotKinematicObject::createManipulator(const std::string &manipName) {
    RaveRobotKinematicObject::Manipulator::Ptr m(new Manipulator(this));
    // initialize the ik module
    robot->SetActiveManipulator(manipName);
    m->manip = robot->GetActiveManipulator();
    m->ikmodule = RaveCreateModule(rave->env, "ikfast");
    rave->env->AddModule(m->ikmodule, "");
    stringstream ssin, ssout;
    ssin << "LoadIKFastSolver " << robot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
    if (!m->ikmodule->SendCommand(ssout, ssin)) {
        RAVELOG_ERROR("failed to load iksolver\n");
        return Manipulator::Ptr(); // null
    }

    // initialize the grabber
    m->grabber.reset(new GrabberKinematicObject(0.02, 0.05));
    m->updateGrabberPos();
    ignoreCollisionWith(m->grabber->rigidBody.get());
    return m;
}

void RaveRobotKinematicObject::Manipulator::updateGrabberPos() {
    // set the grabber right on top of the end effector
    grabber->getKinematicMotionState().setKinematicPos(util::toBtTransform(manip->GetTransform(), robot->scale));
}

bool RaveRobotKinematicObject::Manipulator::moveByIKUnscaled(
        const OpenRAVE::Transform &targetTrans,
        bool checkCollisions, bool revertOnCollision) {

    vector<dReal> vsolution;
    // TODO: lock environment?!?!
    if (!manip->FindIKSolution(IkParameterization(targetTrans), vsolution, true)) {
        stringstream ss;
        ss << "failed to get solution for target transform for end effector: " << targetTrans << endl;
        RAVELOG_DEBUG(ss.str());
        return false;
    }

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
    updateGrabberPos();
    return true;
}
