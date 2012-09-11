#include "basicobjects.h"
#include "config_bullet.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <osg/BlendFunc>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
//#include <osgDB/WriteFile>
#include <osgwTools/Shapes.h>
#include <osgbCollision/CollisionShapes.h>
#include <Serialize/BulletFileLoader/btBulletFile.h>
#include <boost/scoped_array.hpp>
#include "set_colors_visitor.h"
#include <opencv2/imgproc/imgproc.hpp>

/*
 * todo: it would make more sense to create node at construction time
 * so we don't have to do setColorAfterInit stuff
 */


#define MAX_RAYCAST_DISTANCE 100.0

BulletObject::MotionState::Ptr BulletObject::MotionState::clone(BulletObject &newObj) {
    btTransform t; getWorldTransform(t);
    return Ptr(new MotionState(newObj, t));
}

BulletObject::BulletObject(CI ci, const btTransform &initTrans, bool isKinematic_) : isKinematic(isKinematic_), enable_texture(false), m_color(1,1,1,1) {
    BOOST_ASSERT(ci.m_motionState == NULL);
    if (isKinematic) {
        ci.m_mass = 0;
        ci.m_localInertia = btVector3(0, 0, 0);
    }
    motionState.reset(new MotionState(*this, initTrans));
    ci.m_motionState = motionState.get();
    collisionShape.reset(ci.m_collisionShape);
    rigidBody.reset(new btRigidBody(ci));
    if (isKinematic) {
        rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    }
	rigidBody->setActivationState(DISABLE_DEACTIVATION);

}

BulletObject::BulletObject(btScalar mass, btCollisionShape *cs, const btTransform &initTrans, bool isKinematic_) : isKinematic(isKinematic_), enable_texture(false), m_color(1,1,1,1) {
    motionState.reset(new MotionState(*this, initTrans));
    collisionShape.reset(cs);

    btVector3 fallInertia(0, 0, 0);
    if (!isKinematic)
        collisionShape->calculateLocalInertia(mass, fallInertia);
    CI ci(mass, cs, fallInertia);
    ci.m_motionState = motionState.get();

    rigidBody.reset(new btRigidBody(ci));

    if (isKinematic) {
        rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    }
	rigidBody->setActivationState(DISABLE_DEACTIVATION);
	rigidBody->setFriction(BulletConfig::friction);

}

void BulletObject::init() {
    getEnvironment()->bullet->dynamicsWorld->addRigidBody(rigidBody.get());
    node = createOSGNode();
    transform = new osg::MatrixTransform;
    transform->addChild(node.get());
    getEnvironment()->osg->root->addChild(transform.get());

    osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
    blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    osg::StateSet *ss = node->getOrCreateStateSet();
    ss->setAttributeAndModes(blendFunc);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
//    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    if (enable_texture)
    	setTextureAfterInit();
    else
    	setColorAfterInit();
}

osg::ref_ptr<osg::Node> BulletObject::createOSGNode() {
    return osg::ref_ptr<osg::Node>(osgbCollision::osgNodeFromBtCollisionShape(collisionShape.get()));
}

btScalar IDENTITY[] = {1,0,0,0,
		       0,1,0,0,
		       0,0,1,0,
		       0,0,0,1};

btScalar* identityIfBad(btScalar m[16]) { //doesn't fix segfaults, unfortunately.
  for (int i=0; i<16; i++) {
    if (m[i] > 1000 || m[i] < -1000 || !isfinite(m[i])) {
      cout << "warning: bad values detected in transformation matrix. rendering at origin" << endl;
      return IDENTITY;
    }
  }
  return m;
}

void BulletObject::preDraw() {
    // before drawing, we must copy the orientation/position
    // of the object from Bullet to OSG
    btTransform btTrans;
    rigidBody->getMotionState()->getWorldTransform(btTrans);

    btScalar m[16];
   	btTrans.getOpenGLMatrix(m);
		//transform->setMatrix(osg::Matrix(identityIfBad(m)));
    transform->setMatrix(osg::Matrix(m));
}

void BulletObject::destroy() {
    getEnvironment()->bullet->dynamicsWorld->removeRigidBody(rigidBody.get());
    getEnvironment()->osg->root->removeChild(transform.get());
}

BulletObject::BulletObject(const BulletObject &o) : isKinematic(o.isKinematic) {
    // we need to access lots of private members of btRigidBody and etc
    // the easiest way to do this is to use serialization

    // first copy over the collisionShape. This isn't a real deep copy,
    // but we can share collisionShapes so this should be fine
    collisionShape = o.collisionShape;

    // then copy the motionstate
    motionState = o.motionState->clone(*this);

    // then serialize the rigid body
    boost::shared_ptr<btDefaultSerializer> serializer(new btDefaultSerializer());
    int len; btChunk *chunk; const char *structType;
    // http://www.bulletphysics.com/Bullet/BulletFull/btDiscreteDynamicsWorld_8cpp_source.html#l01147
    serializer->startSerialization();
        // rigid body
        len = o.rigidBody->calculateSerializeBufferSize();
        chunk = serializer->allocate(len, 1);
        structType = o.rigidBody->serialize(chunk->m_oldPtr, serializer.get());
        serializer->finalizeChunk(chunk, structType, BT_RIGIDBODY_CODE, o.rigidBody.get());
        // collision shape
//        len = collisionShape->calculateSerializeBufferSize();
//        chunk = serializer->allocate(len, 1);
//        structType = collisionShape->serialize(chunk->m_oldPtr, serializer.get());
//        serializer->finalizeChunk(chunk, structType, BT_SHAPE_CODE, collisionShape);
        // TODO: constraints?
    serializer->finishSerialization();

    // read the data that the serializer just wrote
    int bufSize = serializer->getCurrentBufferSize();
    boost::scoped_array<char> buf(new char[bufSize]);
    memcpy(buf.get(), serializer->getBufferPointer(), bufSize);
    boost::shared_ptr<bParse::btBulletFile> bulletFile(
        new bParse::btBulletFile(buf.get(), bufSize));
    bulletFile->parse(false);
    // create a new rigidBody with the data
    BOOST_ASSERT(bulletFile->m_rigidBodies.size() == 1);
    if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION) {
        // double precision not supported
        BOOST_ASSERT(false);
    } else {
        // single precision
        btRigidBodyFloatData *data = reinterpret_cast<btRigidBodyFloatData *> (bulletFile->m_rigidBodies[0]);
        btScalar mass = btScalar(data->m_inverseMass? 1.f/data->m_inverseMass : 0.f);
        btVector3 localInertia; localInertia.setZero();
        btTransform startTransform;
        startTransform.deSerializeFloat(data->m_collisionObjectData.m_worldTransform);
        //	startTransform.setBasis(btMatrix3x3::getIdentity());
        if (collisionShape->isNonMoving())
            mass = 0.f;
        if (mass)
            collisionShape->calculateLocalInertia(mass, localInertia);

        // fill in btRigidBody params
        btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(), collisionShape.get(), localInertia);
        ci.m_linearDamping = data->m_linearDamping;
        ci.m_angularDamping = data->m_angularDamping;
        ci.m_additionalDampingFactor = data->m_additionalDampingFactor;
        ci.m_additionalLinearDampingThresholdSqr = data->m_additionalLinearDampingThresholdSqr;
        ci.m_additionalAngularDampingThresholdSqr = data->m_additionalAngularDampingThresholdSqr;
        ci.m_additionalAngularDampingFactor = data->m_additionalAngularDampingFactor;
        ci.m_linearSleepingThreshold = data->m_linearSleepingThreshold;
        ci.m_angularSleepingThreshold = data->m_angularSleepingThreshold;
        ci.m_additionalDamping = data->m_additionalDamping;
        rigidBody.reset(new btRigidBody(ci));

        // extra "active" params
        rigidBody->setLinearVelocity(o.rigidBody->getLinearVelocity());
        rigidBody->setAngularVelocity(o.rigidBody->getAngularVelocity());
        rigidBody->applyCentralForce(o.rigidBody->getTotalForce());
        rigidBody->applyTorque(o.rigidBody->getTotalTorque());

        // fill in btCollisionObject params for the rigid body
        btCollisionObject *colObj = rigidBody.get();
        btCollisionObjectFloatData &colObjData = data->m_collisionObjectData;
        btVector3 temp;
        temp.deSerializeFloat(colObjData.m_anisotropicFriction);
        colObj->setAnisotropicFriction(temp);
        colObj->setContactProcessingThreshold(colObjData.m_contactProcessingThreshold);
        colObj->setFriction(colObjData.m_friction);
        colObj->setRestitution(colObjData.m_restitution);
        colObj->setCollisionFlags(colObjData.m_collisionFlags);
        colObj->setHitFraction(colObjData.m_hitFraction);
        colObj->setActivationState(colObjData.m_activationState1);
    }
}

void BulletObject::MoveAction::step(float dt) {
    if (done()) return;
    stepTime(dt);
    const float a = fracElapsed();
    // linear interpolation of pos
    btVector3 newpos = (1-a)*start.getOrigin() + a*end.getOrigin();
    btQuaternion newrot = start.getRotation().slerp(end.getRotation(), a);
    btTransform newtrans(newrot, newpos);
    if (obj->isKinematic)
        obj->motionState->setKinematicPos(newtrans);
    else
        obj->motionState->setWorldTransform(newtrans);
}

void BulletObject::setColor(float r, float g, float b, float a) {
		m_color = osg::Vec4f(r,g,b,a);
		if (node) setColorAfterInit();
		enable_texture = false;
}

void BulletObject::setColorAfterInit() {
		//clear out texture mapping information
  	osg::StateSet *ss = node->getOrCreateStateSet();
		ss->getTextureAttributeList().clear();
		ss->getTextureModeList().clear();

  	if (m_color.a() != 1.0f) {
  		osg::StateSet *ss = node->getOrCreateStateSet();
  		ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }
    SetColorsVisitor visitor(m_color.r(),m_color.g(),m_color.b(),m_color.a());
    node->accept(visitor);
}

void BulletObject::setTexture(const cv::Mat& image) {
	m_cvimage.reset(new cv::Mat);
	image.copyTo(*m_cvimage);

	//hack to convert cv::Mat images to osg::Image images
	cv::imwrite("/tmp/image.jpg", image);
	m_image = osgDB::readImageFile("/tmp/image.jpg");

	if (node) setTextureAfterInit();
	enable_texture = true;
}

void BulletObject::setTextureAfterInit() {
	if (m_image) {
		// clear out color information
		m_color = osg::Vec4f(1,1,1,m_color.a());
		setColorAfterInit();

		osg::Texture2D* texture = new osg::Texture2D;
		// protect from being optimized away as static state:
		texture->setDataVariance(osg::Object::DYNAMIC);
		// Assign the texture to the image we read from file:
		texture->setImage(m_image.get());
		// Create a new StateSet with default settings:
		//osg::StateSet* stateOne = new osg::StateSet();
		osg::StateSet* state = node->getOrCreateStateSet();
		// Assign texture unit 0 of our new StateSet to the texture
		// we just created and enable the texture.
		state->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
	}
}

void BulletObject::adjustTransparency(float increment) {
	m_color.a() += increment;
	if (m_color.a() > 1.0f) m_color.a() = 1.0f;
	if (m_color.a() < 0.0f) m_color.a() = 0.0f;
	if (enable_texture)
		setTextureAfterInit();
	else
		setColorAfterInit();
}

void BulletConstraint::init() {
    getEnvironment()->bullet->dynamicsWorld->addConstraint(cnt.get(), disableCollisionsBetweenLinkedBodies);
}

void BulletConstraint::destroy() {
    getEnvironment()->bullet->dynamicsWorld->removeConstraint(cnt.get());
}

EnvironmentObject::Ptr BulletConstraint::copy(Fork &f) const {
    // serialize the current constraint
    boost::shared_ptr<btDefaultSerializer> serializer(new btDefaultSerializer());
    serializer->startSerialization();
    int len = cnt->calculateSerializeBufferSize();
    btChunk *chunk = serializer->allocate(len, 1);
    const char *structType = cnt->serialize(chunk->m_oldPtr, serializer.get());
    serializer->finalizeChunk(chunk, structType, BT_CONSTRAINT_CODE, cnt.get());
    serializer->finishSerialization();

    // read the data that the serializer just wrote
    int bufSize = serializer->getCurrentBufferSize();
    boost::scoped_array<char> buf(new char[bufSize]);
    memcpy(buf.get(), serializer->getBufferPointer(), bufSize);
    boost::shared_ptr<bParse::btBulletFile> bulletFile(
        new bParse::btBulletFile(buf.get(), bufSize));
    bulletFile->parse(false);

    BOOST_ASSERT(bulletFile->m_constraints.size() == 1);
    if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION) {
        // double precision not supported
        BOOST_ASSERT(false);
        return Ptr();
    }

    // get equivalents of rigid bodies in the new cloned world
    btRigidBody *rbA = (btRigidBody *) f.copyOf(&cnt->getRigidBodyA());
    btRigidBody *rbB = (btRigidBody *) f.copyOf(&cnt->getRigidBodyB());

    // create new constraint
    boost::shared_ptr<btTypedConstraint> newcnt;
    btTypedConstraintData *constraintData = (btTypedConstraintData*)bulletFile->m_constraints[0];
    if (constraintData->m_objectType == POINT2POINT_CONSTRAINT_TYPE) {
        btPoint2PointConstraintFloatData* p2pData = (btPoint2PointConstraintFloatData*)constraintData;
        if (rbA && rbB) {
            btVector3 pivotInA,pivotInB;
            pivotInA.deSerializeFloat(p2pData->m_pivotInA);
            pivotInB.deSerializeFloat(p2pData->m_pivotInB);
            newcnt.reset(new btPoint2PointConstraint(*rbA, *rbB, pivotInA, pivotInB));
        } else {
            btVector3 pivotInA;
            pivotInA.deSerializeFloat(p2pData->m_pivotInA);
            newcnt.reset(new btPoint2PointConstraint(*rbA, pivotInA));
        }

    } else if (constraintData->m_objectType == HINGE_CONSTRAINT_TYPE) {
        btHingeConstraintFloatData* hingeData = (btHingeConstraintFloatData*)constraintData;
        btHingeConstraint *hinge = 0;
        if (rbA && rbB) {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
            rbBFrame.deSerializeFloat(hingeData->m_rbBFrame);
            hinge = new btHingeConstraint(*rbA,*rbB,rbAFrame,rbBFrame,hingeData->m_useReferenceFrameA!=0);
        } else {
            btTransform rbAFrame;
            rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
            hinge = new btHingeConstraint(*rbA,rbAFrame,hingeData->m_useReferenceFrameA!=0);
        }
        if (hingeData->m_enableAngularMotor)
            hinge->enableAngularMotor(true,hingeData->m_motorTargetVelocity,hingeData->m_maxMotorImpulse);
        hinge->setAngularOnly(hingeData->m_angularOnly!=0);
        hinge->setLimit(btScalar(hingeData->m_lowerLimit),btScalar(hingeData->m_upperLimit),btScalar(hingeData->m_limitSoftness),btScalar(hingeData->m_biasFactor),btScalar(hingeData->m_relaxationFactor));
        newcnt.reset(hinge);

    } else if (constraintData->m_objectType == CONETWIST_CONSTRAINT_TYPE) {
        btConeTwistConstraintData* coneData = (btConeTwistConstraintData*)constraintData;
        btConeTwistConstraint* coneTwist = 0;
        if (rbA&& rbB) {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
            rbBFrame.deSerializeFloat(coneData->m_rbBFrame);
            coneTwist = new btConeTwistConstraint(*rbA,*rbB,rbAFrame,rbBFrame);
        } else {
            btTransform rbAFrame;
            rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
            coneTwist = new btConeTwistConstraint(*rbA,rbAFrame);
        }
        coneTwist->setLimit(coneData->m_swingSpan1,coneData->m_swingSpan2,coneData->m_twistSpan,coneData->m_limitSoftness,coneData->m_biasFactor,coneData->m_relaxationFactor);
        coneTwist->setDamping(coneData->m_damping);
        newcnt.reset(coneTwist);

    } else if (constraintData->m_objectType == D6_SPRING_CONSTRAINT_TYPE) {
        btGeneric6DofSpringConstraintData* dofData = (btGeneric6DofSpringConstraintData*)constraintData;
        btGeneric6DofSpringConstraint* dof = 0;
        if (rbA && rbB) {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(dofData->m_6dofData.m_rbAFrame);
            rbBFrame.deSerializeFloat(dofData->m_6dofData.m_rbBFrame);
            dof = new btGeneric6DofSpringConstraint(*rbA,*rbB,rbAFrame,rbBFrame,dofData->m_6dofData.m_useLinearReferenceFrameA!=0);
        } else {
            printf("Error in btWorldImporter::createGeneric6DofSpringConstraint: requires rbA && rbB\n");
            BOOST_ASSERT(false);
        }
        if (dof) {
            btVector3 angLowerLimit,angUpperLimit, linLowerLimit,linUpperlimit;
            angLowerLimit.deSerializeFloat(dofData->m_6dofData.m_angularLowerLimit);
            angUpperLimit.deSerializeFloat(dofData->m_6dofData.m_angularUpperLimit);
            linLowerLimit.deSerializeFloat(dofData->m_6dofData.m_linearLowerLimit);
            linUpperlimit.deSerializeFloat(dofData->m_6dofData.m_linearUpperLimit);

            dof->setAngularLowerLimit(angLowerLimit);
            dof->setAngularUpperLimit(angUpperLimit);
            dof->setLinearLowerLimit(linLowerLimit);
            dof->setLinearUpperLimit(linUpperlimit);

            for (int i=0;i<6;i++) {
                dof->setStiffness(i,dofData->m_springStiffness[i]);
                dof->setEquilibriumPoint(i,dofData->m_equilibriumPoint[i]);
                dof->enableSpring(i,dofData->m_springEnabled[i]!=0);
                dof->setDamping(i,dofData->m_springDamping[i]);
            }
        }
        newcnt.reset(dof);

    } else if (constraintData->m_objectType == D6_CONSTRAINT_TYPE) {
        btGeneric6DofConstraintData* dofData = (btGeneric6DofConstraintData*)constraintData;
        btGeneric6DofConstraint* dof = 0;
        if (rbA&& rbB) {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(dofData->m_rbAFrame);
            rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
            dof = new btGeneric6DofConstraint(*rbA,*rbB,rbAFrame,rbBFrame,dofData->m_useLinearReferenceFrameA!=0);
        } else {
            if (rbB) {
                btTransform rbBFrame;
                rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
                dof = new btGeneric6DofConstraint(*rbB,rbBFrame,dofData->m_useLinearReferenceFrameA!=0);
            } else {
                printf("Error in btWorldImporter::createGeneric6DofConstraint: missing rbB\n");
                BOOST_ASSERT(false);
            }
        }
        if (dof) {
            btVector3 angLowerLimit,angUpperLimit, linLowerLimit,linUpperlimit;
            angLowerLimit.deSerializeFloat(dofData->m_angularLowerLimit);
            angUpperLimit.deSerializeFloat(dofData->m_angularUpperLimit);
            linLowerLimit.deSerializeFloat(dofData->m_linearLowerLimit);
            linUpperlimit.deSerializeFloat(dofData->m_linearUpperLimit);

            dof->setAngularLowerLimit(angLowerLimit);
            dof->setAngularUpperLimit(angUpperLimit);
            dof->setLinearLowerLimit(linLowerLimit);
            dof->setLinearUpperLimit(linUpperlimit);
        }
        newcnt.reset(dof);

    } else if (constraintData->m_objectType == SLIDER_CONSTRAINT_TYPE) {
        btSliderConstraintData* sliderData = (btSliderConstraintData*)constraintData;
        btSliderConstraint* slider = 0;
        if (rbA&& rbB) {
            btTransform rbAFrame,rbBFrame;
            rbAFrame.deSerializeFloat(sliderData->m_rbAFrame);
            rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
            slider = new btSliderConstraint(*rbA,*rbB,rbAFrame,rbBFrame,sliderData->m_useLinearReferenceFrameA!=0);
        } else {
            btTransform rbBFrame;
            rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
            slider = new btSliderConstraint(*rbB,rbBFrame,sliderData->m_useLinearReferenceFrameA!=0);
        }
        slider->setLowerLinLimit(sliderData->m_linearLowerLimit);
        slider->setUpperLinLimit(sliderData->m_linearUpperLimit);
        slider->setLowerAngLimit(sliderData->m_angularLowerLimit);
        slider->setUpperAngLimit(sliderData->m_angularUpperLimit);
        slider->setUseFrameOffset(sliderData->m_useOffsetForConstraintFrame!=0);
        newcnt.reset(slider);

    } else {
        printf("unknown constraint type\n");
    }

    if (newcnt)
        f.registerCopy(cnt.get(), newcnt.get());

    return Ptr(new BulletConstraint(newcnt, disableCollisionsBetweenLinkedBodies));
}

GrabberKinematicObject::GrabberKinematicObject(float radius_, float height_) :
    radius(radius_), height(height_),
    constraintPivot(0, 0, height_), // this is where objects will attach to
    BulletObject(0, new btConeShapeZ(radius_, height_),
            btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)), true) {
}

osg::ref_ptr<osg::Node> GrabberKinematicObject::createOSGNode() {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Cone> cone = new osg::Cone(osg::Vec3(), radius, height);
    cone->setCenter(osg::Vec3(0, 0, -cone->getBaseOffset()));
    // FIXME: this cone seems a bit taller than the bullet cone
    geode->addDrawable(new osg::ShapeDrawable(cone));
    return geode;
}

void GrabberKinematicObject::grabNearestObjectAhead() {
    // first, try to find the object ahead.
    // trace a ray in the direction of the end affector and get the first object hit
    btTransform trans; motionState->getWorldTransform(trans);
    btVector3 rayFrom = trans(btVector3(0, 0, 0)); // some point in the middle of the stick
    btVector3 rayTo = trans(constraintPivot); // the end affector
    rayTo = (rayTo - rayFrom).normalize()*MAX_RAYCAST_DISTANCE + rayTo;

    printf("from: %f %f %f\nto:%f %f %f\n", rayFrom.x(), rayFrom.y(), rayFrom.z(), rayTo.x(), rayTo.y(), rayTo.z());
    // send the ray
    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);
    getEnvironment()->bullet->dynamicsWorld->rayTest(rayFrom, rayTo, rayCallback);
    printf("tracing!\n");
    if (rayCallback.hasHit()) {
        printf("hit!\n");
        btRigidBody *hitBody = btRigidBody::upcast(rayCallback.m_collisionObject);
        if (hitBody && !hitBody->isStaticObject() && !hitBody->isKinematicObject()) {
            hitBody->setActivationState(DISABLE_DEACTIVATION);

            releaseConstraint();

            // the constraint in the grabber's coordinate system
            btTransform grabberFrame;
            grabberFrame.setIdentity();
            grabberFrame.setOrigin(constraintPivot);
            grabberFrame.setRotation(trans.inverse().getRotation());

            // the constraint in the target's coordinate system
            const btTransform &hitBodyTransInverse = hitBody->getCenterOfMassTransform().inverse();
            btVector3 localHitPoint = hitBodyTransInverse * rayCallback.m_hitPointWorld;
            btTransform hitFrame;
            hitFrame.setIdentity();
            hitFrame.setOrigin(localHitPoint);
            hitFrame.setRotation(hitBodyTransInverse.getRotation());

            btGeneric6DofConstraint *cnt = new btGeneric6DofConstraint(*rigidBody, *hitBody, grabberFrame, hitFrame, false);
            // make the constraint completely rigid
            cnt->setLinearLowerLimit(btVector3(0., 0., 0.));
            cnt->setLinearUpperLimit(btVector3(0., 0., 0.));
            cnt->setAngularLowerLimit(btVector3(0., 0., 0.));
            cnt->setAngularUpperLimit(btVector3(0., 0., 0.));
            constraint.reset(new BulletConstraint(cnt));
            getEnvironment()->addConstraint(constraint);
        }
    }
}

void GrabberKinematicObject::releaseConstraint() {
    if (!constraint) return;
    getEnvironment()->removeConstraint(constraint);
    constraint.reset();
}

PlaneStaticObject::PlaneStaticObject(const btVector3 &planeNormal_, btScalar planeConstant_, const btTransform &initTrans, btScalar drawHalfExtents_) :
    planeNormal(planeNormal_), planeConstant(planeConstant_), drawHalfExtents(drawHalfExtents_),
    BulletObject(0, new btStaticPlaneShape(planeNormal_, planeConstant_), initTrans) {
}

osg::ref_ptr<osg::Node> PlaneStaticObject::createOSGNode() {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-drawHalfExtents, -drawHalfExtents, 0.));
    vertices->push_back(osg::Vec3(drawHalfExtents, -drawHalfExtents, 0.));
    vertices->push_back(osg::Vec3(drawHalfExtents, drawHalfExtents, 0.));
    vertices->push_back(osg::Vec3(-drawHalfExtents, drawHalfExtents, 0.));

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0., 0., 1.));

    osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
    quad->setVertexArray(vertices.get());
    quad->setNormalArray(normals.get());
    quad->setNormalBinding(osg::Geometry::BIND_OVERALL);
    quad->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(quad.get());
    return geode;
}

CylinderStaticObject::CylinderStaticObject(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans) :
    mass(mass_), radius(radius_), height(height_),
    BulletObject(mass_, new btCylinderShapeZ(btVector3(radius_, radius_, height_/2.)), initTrans) {
}

SphereObject::SphereObject(btScalar mass_, btScalar radius_, const btTransform &initTrans, bool isKinematic) :
    mass(mass_), radius(radius_),
    BulletObject(mass_, new btSphereShape(radius_), initTrans, isKinematic) {
}

BoxObject::BoxObject(btScalar mass_, const btVector3 &halfExtents_, const btTransform &initTrans) :
    mass(mass_), halfExtents(halfExtents_),
    BulletObject(mass_, new btBoxShape(halfExtents_), initTrans) {
}

CapsuleObject::CapsuleObject(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans) :
    mass(mass_), radius(radius_), height(height_),
    BulletObject(mass_, new btCapsuleShapeX(radius_, height_), initTrans) {
}

osg::ref_ptr<osg::Node> CapsuleObject::createOSGNode() {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Capsule> capsule = new osg::Capsule(osg::Vec3(0, 0, 0), radius, height);
    capsule->setRotation(osg::Quat(osg::PI_2, osg::Vec3(0, 1, 0)));
    geode->addDrawable(new osg::ShapeDrawable(capsule));
    return geode;
}

CapsuleObjectY::CapsuleObjectY(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans) :
    mass(mass_), radius(radius_), height(height_),
    BulletObject(mass_, new btCapsuleShape(radius_, height_), initTrans) {
}

osg::ref_ptr<osg::Node> CapsuleObjectY::createOSGNode() {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Capsule> capsule = new osg::Capsule(osg::Vec3(0, 0, 0), radius, height);
    capsule->setRotation(osg::Quat(osg::PI_2, osg::Vec3(1, 0, 0)));
    geode->addDrawable(new osg::ShapeDrawable(capsule));
    return geode;
}
