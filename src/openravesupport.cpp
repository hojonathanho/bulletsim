#include "openravesupport.h"
#include <openrave-core.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

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

#if 0
OpenRAVESupport::KinBodyInfoPtr OpenRAVESupport::InitKinBody(KinBodyPtr pbody, KinBodyInfoPtr pinfo, btScalar fmargin) {
    // create all ode bodies and joints
    if( !pinfo ) {
        pinfo.reset(new KinBodyInfo(btWorld, true));
    }
    pinfo->Reset();
    pinfo->pbody = pbody;
//        pinfo->_bulletspace = weak_space();
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
                    //child.reset(new btBvhTriangleMeshShape(ptrimesh, true, true)); // doesn't do tri-tri collisions!
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
                    //convexShape->updateBound();
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

        // set the mass and inertia and extract the eigenvectors of the tensor
        TransformMatrix inertiatensor = (*itlink)->GetInertia();
        double fCovariance[9] = { inertiatensor.m[0],inertiatensor.m[1],inertiatensor.m[2],inertiatensor.m[4],inertiatensor.m[5],inertiatensor.m[6],inertiatensor.m[8],inertiatensor.m[9],inertiatensor.m[10]};
        double eigenvalues[3], eigenvectors[9];
        mathextra::EigenSymmetric3(fCovariance,eigenvalues,eigenvectors);
        TransformMatrix tinertiaframe;
        tinertiaframe.trans = inertiatensor.trans;
        for(int j = 0; j < 3; ++j) {
            tinertiaframe.m[4*0+j] = eigenvectors[3*j];
            tinertiaframe.m[4*1+j] = eigenvectors[3*j+1];
            tinertiaframe.m[4*2+j] = eigenvectors[3*j+2];
        }
        btVector3 localInertia(eigenvalues[0],eigenvalues[1],eigenvalues[2]);
        btRigidBody::btRigidBodyConstructionInfo rbInfo((*itlink)->GetMass(),/*link.get()*/ NULL,pshapeparent,localInertia);
        link->tlocal = tinertiaframe;
        rbInfo.m_startWorldTransform = GetBtTransform((*itlink)->GetTransform()*link->tlocal);
        link->_rigidbody.reset(new btRigidBody(rbInfo));
        link->obj = link->_rigidbody;

        link->plink = *itlink;
        link->obj->setWorldTransform(GetBtTransform((*itlink)->GetTransform()*link->tlocal));
        link->obj->setUserPointer(&link->plink);
        link->obj->setCollisionFlags((*itlink)->IsStatic() ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);

        if( !(*itlink)->IsStatic() ) {
            RAVELOG_DEBUG("adding rigid body\n");
            btWorld->addRigidBody(link->_rigidbody.get());
        }
        else {
            RAVELOG_DEBUG("adding collision object (static)\n");
            btWorld->addCollisionObject(link->obj.get());
        }

        //Activates all kinematic objects added to btDiscreteDynamicsWorld
        //link->body->setActivationState(DISABLE_DEACTIVATION);

        link->obj->activate(true);
        pinfo->vlinks.push_back(link);
    }

    vector<KinBody::JointPtr> vbodyjoints; vbodyjoints.reserve(pbody->GetJoints().size()+pbody->GetPassiveJoints().size());
    vbodyjoints.insert(vbodyjoints.end(),pbody->GetJoints().begin(),pbody->GetJoints().end());
    vbodyjoints.insert(vbodyjoints.end(),pbody->GetPassiveJoints().begin(),pbody->GetPassiveJoints().end());
    RAVELOG_DEBUG("creating %d joints (%d passive)\n", vbodyjoints.size(), pbody->GetPassiveJoints().size());
    FOREACH(itjoint, vbodyjoints) {
        btRigidBody* body0 = NULL, *body1 = NULL;
        RAVELOG_DEBUG("joint %s first attached (%s) is %s\n", (*itjoint)->GetName().c_str(), (*itjoint)->GetFirstAttached()->GetName().c_str(), (*itjoint)->GetFirstAttached()->IsStatic() ? "static" : "non-static");
        RAVELOG_DEBUG("joint %s second attached (%s) is %s\n", (*itjoint)->GetName().c_str(), (*itjoint)->GetSecondAttached()->GetName().c_str(), (*itjoint)->GetSecondAttached()->IsStatic() ? "static" : "non-static");
        if( !!(*itjoint)->GetFirstAttached() && !(*itjoint)->GetFirstAttached()->IsStatic() ) {
//        if( !!(*itjoint)->GetFirstAttached()) {
            boost::shared_ptr<KinBodyInfo::LINK> &pl = pinfo->vlinks[(*itjoint)->GetFirstAttached()->GetIndex()];
            if (pl != boost::shared_ptr<KinBodyInfo::LINK>())
                body0 = (btRigidBody *) pl->obj.get();
        }
        if( !!(*itjoint)->GetSecondAttached() && !(*itjoint)->GetSecondAttached()->IsStatic()) {
//        if( !!(*itjoint)->GetSecondAttached()) {
            boost::shared_ptr<KinBodyInfo::LINK> &pl = pinfo->vlinks[(*itjoint)->GetSecondAttached()->GetIndex()];
            if (pl != boost::shared_ptr<KinBodyInfo::LINK>())
                body1 = (btRigidBody *) pl->obj.get();
        }
        if( !body0 || !body1 ) {
            RAVELOG_ERROR(str(boost::format("joint %s needs to be attached to two bodies!\n")%(*itjoint)->GetName()));
            continue;
        }

        Transform t0inv = (*itjoint)->GetFirstAttached()->GetTransform().inverse();
        Transform t1inv = (*itjoint)->GetSecondAttached()->GetTransform().inverse();
        boost::shared_ptr<btTypedConstraint> joint;

        switch((*itjoint)->GetType()) {
        case KinBody::Joint::JointHinge: {
            btVector3 pivotInA = GetBtVector(t0inv * (*itjoint)->GetAnchor());
            btVector3 pivotInB = GetBtVector(t1inv * (*itjoint)->GetAnchor());
            btVector3 axisInA = GetBtVector(t0inv.rotate((*itjoint)->GetAxis(0)));
            btVector3 axisInB = GetBtVector(t1inv.rotate((*itjoint)->GetAxis(0)));
            boost::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*body0, *body1, pivotInA, pivotInB, axisInA, axisInB));
            if( !(*itjoint)->IsCircular(0) ) {
                vector<dReal> vlower, vupper;
                (*itjoint)->GetLimits(vlower,vupper);
                btScalar orInitialAngle = (*itjoint)->GetValue(0);
                btScalar btInitialAngle = hinge->getHingeAngle();
                btScalar lower_adj, upper_adj;
                btScalar diff = (btInitialAngle + orInitialAngle);
                lower_adj = diff - vupper.at(0);
                upper_adj = diff - vlower.at(0);
                hinge->setLimit(lower_adj,upper_adj);
            }
            joint = hinge;
            break;
        }
        case KinBody::Joint::JointSlider: {
            Transform tslider; tslider.rot = quatRotateDirection(Vector(1,0,0),(*itjoint)->GetAxis(0));
            btTransform frameInA = GetBtTransform(t0inv*tslider);
            btTransform frameInB = GetBtTransform(t1inv*tslider);
            joint.reset(new btSliderConstraint(*body0, *body1, frameInA, frameInB, true));
            break;
        }
        case KinBody::Joint::JointUniversal:
            RAVELOG_ERROR("universal joint not supported by bullet\n");
            break;
        case KinBody::Joint::JointHinge2:
            RAVELOG_ERROR("hinge2 joint not supported by bullet\n");
            break;
        default:
            RAVELOG_ERROR("unknown joint type %d\n", (*itjoint)->GetType());
            break;
        }

        if( !!joint ) {
            KinBody::LinkPtr plink0 = (*itjoint)->GetFirstAttached(), plink1 = (*itjoint)->GetSecondAttached();
            int minindex = min(plink0->GetIndex(), plink1->GetIndex());
            int maxindex = max(plink0->GetIndex(), plink1->GetIndex());

            bool bIgnoreCollision = pbody->GetAdjacentLinks().find(minindex|(maxindex<<16)) != pbody->GetAdjacentLinks().end() || plink0->IsRigidlyAttached(plink0);
            btWorld->addConstraint(joint.get(), bIgnoreCollision);
            pinfo->_mapjoints[*itjoint] = joint;
        }
    }

    //pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&BulletSpace::GeometryChangedCallback,boost::bind(&sptr_from<BulletSpace>, weak_space()),KinBodyWeakPtr(pbody)));
    //_Synchronize(pinfo);

/*
    FOREACH(itlink, pinfo->vlinks) {
        (*itlink)->_rigidbody->setFriction(0.4);
        (*itlink)->_rigidbody->setRestitution(0.2);
    }
*/
    return pinfo;
}
#endif

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
