#include "openravesupport.h"
#include <openrave-core.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include "convexdecomp.h"
#include "utils/config.h"

using namespace OpenRAVE;

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

RaveRobotKinematicObject::RaveRobotKinematicObject(
        RaveInstance::Ptr rave_,
        const std::string &uri,
        const btTransform &initialTransform_,
        btScalar scale_,
        TrimeshMode trimeshMode) :
            rave(rave_),
            initialTransform(initialTransform_),
            scale(scale_) {
    robot = rave->env->ReadRobotURI(uri);
    rave->env->AddRobot(robot);
    initRobotWithoutDynamics(initialTransform, trimeshMode);
}

void RaveRobotKinematicObject::initRobotWithoutDynamics(const btTransform &initialTransform, TrimeshMode trimeshMode, float fmargin) {
    ConvexDecomp decomp(fmargin);
    const std::vector<KinBody::LinkPtr> &links = robot->GetLinks();
    getChildren().reserve(links.size());
    // iterate through each link in the robot (to be stored in the children vector)
    for (std::vector<KinBody::LinkPtr>::const_iterator link = links.begin(); link != links.end(); ++link) {
        const std::list<KinBody::Link::GEOMPROPERTIES> &geometries = (*link)->GetGeometries();
        // sometimes the OpenRAVE link might not even have any geometry data associated with it
        // (this is the case with the PR2 model). therefore just add an empty BulletObject
        // pointer so we know to skip it in the future
        if (geometries.empty()) {
            getChildren().push_back(BulletObject::Ptr());
            continue;
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
                if (mesh.indices.size() < 3) break;
                if (trimeshMode == CONVEX_DECOMP) {
                    printf("running convex decomposition\n");
                    decomp.reset();
                    for (size_t i = 0; i < mesh.vertices.size(); ++i)
                        decomp.addPoint(util::toBtVector(mesh.vertices[i]));
                    for (size_t i = 0; i < mesh.indices.size(); i += 3)
                        decomp.addTriangle(mesh.indices[i], mesh.indices[i+1], mesh.indices[i+2]);
                    subshape = decomp.run(subshapes); // use subshapes to just store smart pointer

                } else {
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

                    } else { // RAW
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
        BulletObject::Ptr child(new BulletObject(0, compound, childTrans, true));
        getChildren().push_back(child);
        linkMap[*link] = child;
        collisionObjMap[child->rigidBody.get()] = *link;
        childPosMap[child] = getChildren().size() - 1;

        // since the joints are always in contact, we should ignore their collisions
        // when setting joint positions (OpenRAVE should take care of them anyway)
        ignoreCollisionWith(child->rigidBody.get());
    }
}

bool RaveRobotKinematicObject::detectCollisions() {
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
            getChildren()[i]->motionState->setKinematicPos(
                initialTransform * util::toBtTransform(transforms[i], scale));
}

EnvironmentObject::Ptr RaveRobotKinematicObject::copy(Fork &f) const {
    Ptr o(new RaveRobotKinematicObject(scale));

    internalCopy(o, f);

    o->rave.reset(new RaveInstance(*rave.get(), OpenRAVE::Clone_Bodies));

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

void RaveRobotKinematicObject::postCopy(EnvironmentObject::Ptr copy, Fork &f) const {
    Ptr o = boost::static_pointer_cast<RaveRobotKinematicObject>(copy);

    for (BulletInstance::CollisionObjectSet::const_iterator i = ignoreCollisionObjs.begin();
            i != ignoreCollisionObjs.end(); ++i)
        o->ignoreCollisionObjs.insert((btCollisionObject *) f.copyOf(*i));
}

RaveRobotKinematicObject::Manipulator::Ptr
RaveRobotKinematicObject::createManipulator(const std::string &manipName, bool useFakeGrabber) {
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

void RaveRobotKinematicObject::Manipulator::updateGrabberPos() {
    // set the grabber right on top of the end effector
    if (useFakeGrabber)
        grabber->motionState->setKinematicPos(getTransform());
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

    if (useFakeGrabber)
        updateGrabberPos();

    return true;
}

RaveRobotKinematicObject::Manipulator::Ptr
RaveRobotKinematicObject::Manipulator::copy(RaveRobotKinematicObject::Ptr newRobot, Fork &f) {
    OpenRAVE::EnvironmentMutex::scoped_lock lock(newRobot->rave->env->GetMutex());

    Manipulator::Ptr o(new Manipulator(newRobot.get()));
    o->ikmodule = ikmodule; // use same ik module

    newRobot->robot->SetActiveManipulator(manip->GetName());
    o->manip = newRobot->robot->GetActiveManipulator();

    o->useFakeGrabber = useFakeGrabber;
    if (useFakeGrabber)
        o->grabber = boost::static_pointer_cast<GrabberKinematicObject>(grabber->copy(f));

    return o;
}

void RaveRobotKinematicObject::destroyManipulator(RaveRobotKinematicObject::Manipulator::Ptr m) {
    std::vector<Manipulator::Ptr>::iterator i =
        std::find(createdManips.begin(), createdManips.end(), m);
    if (i == createdManips.end()) return;
    (*i).reset();
    createdManips.erase(i);
}
