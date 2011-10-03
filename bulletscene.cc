#include <string>
#include <GL/glut.h>
#include <btBulletDynamicsCommon.h>
#include "GLDebugDrawer.h"

// SCALING FACTOR: centimeter --> meter
#define SCALE(n) (100.0*(n))
#define SCALE2(n) (100.0*100.0*(n))

static const int WINDOW_WIDTH = 800, WINDOW_HEIGHT = 800;
static const bool DEBUG_DRAWING_ENABLED = true;
static const btScalar GRAVITY = SCALE(9.81);
static const btScalar CYL_HEIGHT = SCALE(0.1), CYL_RADIUS = SCALE(.005), CYL_SPACING = SCALE(0.05);
static const int NUM_CYLINDERS = 7;
static const btScalar TORUS_INNER_RADIUS = SCALE(0.003), TORUS_OUTER_RADIUS = SCALE(0.02), TORUS_SUBDIVISIONS = 32;
static const btScalar SPHERE_RADIUS = SCALE(0.01);
static const btScalar PLANE_SIZE = SCALE(1.0);
static const btScalar GRABBER_LENGTH = SCALE(0.1), GRABBER_RADIUS = SCALE(0.003), GRABBER_FINGER_LENGTH = SCALE(0.005),
    GRABBER_FINGER_WIDTH = SCALE(0.002), GRABBER_FINGER_THICKNESS = SCALE(0.001);
static const btScalar MAX_RAYCAST_DISTANCE = SCALE(1.0);

class SimulationObject {
public:
    btDiscreteDynamicsWorld *dynamicsWorld;

    btRigidBody *rigidBody;
    btMotionState *motionState;
    btCollisionShape *collisionShape;
    GLint displayList;

public:
    SimulationObject(btDiscreteDynamicsWorld *dynamicsWorld_, btMotionState *motionState_) :
        dynamicsWorld(dynamicsWorld_), rigidBody(NULL), motionState(motionState_), collisionShape(NULL) { }
    virtual ~SimulationObject();

    virtual void initGraphics() = 0;
    virtual void initPhysics() = 0;
    void init() { initPhysics(); initGraphics(); }

    virtual void drawGL();
    virtual void drawLocal() { glCallList(displayList); }
};

SimulationObject::~SimulationObject() {
    if (rigidBody) {
        dynamicsWorld->removeRigidBody(rigidBody);
        if (rigidBody->getMotionState())
            delete rigidBody->getMotionState();
        delete rigidBody;
    }
    if (collisionShape)
        delete collisionShape;

    glDeleteLists(displayList, 1);
}

void SimulationObject::drawGL() {
    // get the opengl transformation matrix to orient the object
    btTransform trans;
    rigidBody->getMotionState()->getWorldTransform(trans);
    btScalar m[16];
    trans.getOpenGLMatrix(m);

    // draw
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMultMatrixf(m);
    drawLocal();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

class GroundPlaneObject : public SimulationObject {
private:
    btScalar size;

public:
    GroundPlaneObject(btDiscreteDynamicsWorld *dynamicsWorld, btScalar size_) :
        SimulationObject(dynamicsWorld, new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),
                                                                 btVector3(0, 0, 0)))),
        size(size_) { }

    virtual void initGraphics();
    virtual void initPhysics();
};

void GroundPlaneObject::initGraphics() {
    displayList = glGenLists(1);
    glNewList(displayList, GL_COMPILE);
    glBegin(GL_QUADS);
        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(size, 0.0, size);
        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(size, 0.0, -size);
        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(-size, 0.0, -size);
        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(-size, 0.0, size);
    glEnd();
    glEndList();
}

void GroundPlaneObject::initPhysics() {
    collisionShape = new btStaticPlaneShape(btVector3(0.0, 1.0, 0.0), 0.0);
    btRigidBody::btRigidBodyConstructionInfo ci(0, motionState, collisionShape, btVector3(0, 0, 0));
    rigidBody = new btRigidBody(ci);
    dynamicsWorld->addRigidBody(rigidBody);
}


class SphereObject : public SimulationObject {
private:
    btScalar mass, radius;

public:
    SphereObject(btDiscreteDynamicsWorld *dynamicsWorld, btScalar mass_, btScalar radius_, btMotionState *motionState_) :
        SimulationObject(dynamicsWorld, motionState_), mass(mass_), radius(radius_) { }

    virtual void initGraphics();
    virtual void initPhysics();
};

void SphereObject::initGraphics() {
    displayList = glGenLists(1);
    glNewList(displayList, GL_COMPILE);
    glutSolidSphere(radius, 20, 16);
    glEndList();
}

void SphereObject::initPhysics() {
    collisionShape = new btSphereShape(radius);
    btVector3 fallInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, collisionShape, fallInertia);
    rigidBody = new btRigidBody(ci);
    dynamicsWorld->addRigidBody(rigidBody);
}


class TorusObject : public SimulationObject {
private:
    btScalar mass, innerRadius, outerRadius;

public:
    TorusObject(btDiscreteDynamicsWorld *dynamicsWorld, btScalar mass_, btScalar innerRadius_, btScalar outerRadius_, btMotionState *motionState_) :
        SimulationObject(dynamicsWorld, motionState_), mass(mass_), innerRadius(innerRadius_), outerRadius(outerRadius_) { }
    virtual ~TorusObject() { delete static_cast<btCompoundShape *> (collisionShape)->getChildShape(0); }
    virtual void initGraphics();
    virtual void initPhysics();
};

void TorusObject::initGraphics() {
    displayList = glGenLists(1);
    glNewList(displayList, GL_COMPILE);
    glMatrixMode(GL_MODELVIEW);
    glRotatef(-90, 1, 0, 0);
    glutSolidTorus(innerRadius, outerRadius, 32, 32);
    glEndList();
}

void TorusObject::initPhysics() {
    const double subdivisions = TORUS_SUBDIVISIONS;
    btVector3 forward(btScalar(0.0),btScalar(1.0),btScalar(0.0));
    btVector3 side(btScalar(outerRadius),btScalar(0.0),btScalar(0.0));
    double gap = sqrt(2.0*innerRadius*innerRadius
                    - 2.0*innerRadius*innerRadius*cos((2.0*SIMD_PI)/subdivisions));
    btCylinderShapeZ *shape = new btCylinderShapeZ(btVector3(btScalar(innerRadius),
                                                             btScalar(innerRadius),
                                                             btScalar((SIMD_PI/subdivisions) + 2*gap)));
    btTransform t;
    btCompoundShape *torusShape = new btCompoundShape();
    for (int x = 0; x < (int)subdivisions; x++) {
        btScalar angle = btScalar((x*2.0*SIMD_PI)/subdivisions);
        btVector3 position = side.rotate(forward, angle);
        btQuaternion q(forward, angle);
        t.setIdentity();
        t.setOrigin(position);
        t.setRotation(q);
        torusShape->addChildShape(t, shape);
    }
    collisionShape = torusShape;

    btVector3 fallInertia(0, 0, 0);
    torusShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, torusShape, fallInertia);
    rigidBody = new btRigidBody(ci);
    dynamicsWorld->addRigidBody(rigidBody);
}


class CylinderObject : public SimulationObject {
private:
    btScalar mass, radius, height;

public:
    CylinderObject(btDiscreteDynamicsWorld *dynamicsWorld, btScalar mass_, btScalar radius_, btScalar height_, btMotionState *motionState_) :
        SimulationObject(dynamicsWorld, motionState_), mass(mass_), radius(radius_), height(height_) { }

    virtual void initGraphics();
    virtual void initPhysics();
};

void CylinderObject::initGraphics() {
    GLUquadricObj *qobj = gluNewQuadric();
    displayList = glGenLists(1);
    gluQuadricDrawStyle(qobj, GLU_FILL);
    gluQuadricNormals(qobj, GLU_SMOOTH); 
    glNewList(displayList, GL_COMPILE);
    glMatrixMode(GL_MODELVIEW);
    glTranslatef(0, -height/2., 0);
    glRotatef(-90, 1, 0, 0);
    gluCylinder(qobj, radius, radius, height, 32, 32);
    glEndList();
    gluDeleteQuadric(qobj);
}

void CylinderObject::initPhysics() {
    collisionShape = new btCylinderShape(btVector3(radius, height/2., radius));

    btVector3 fallInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, collisionShape, fallInertia);
    rigidBody = new btRigidBody(ci);
    dynamicsWorld->addRigidBody(rigidBody);
}

class KinematicMotionState : public btMotionState {
private:
    btTransform trans;

public:
    KinematicMotionState(const btTransform &trans_) : trans(trans_) { }
    void getWorldTransform(btTransform &worldTrans) const { worldTrans = trans; }
    void setWorldTransform(const btTransform &) { }
    void setKinematicPos(const btTransform &pos) { trans = pos; }
};

class P2PGrabberKinematicObject : public SimulationObject {
private:
    btScalar radius;
    btScalar height;
    bool grabState; // true => grabbing, false => nothing
    const btVector3 constraintPivot;
    btGeneric6DofConstraint *constraint;

public:
    P2PGrabberKinematicObject(btDiscreteDynamicsWorld *dynamicsWorld, btScalar radius_, btScalar height_, const btTransform &initialTransform) :
        SimulationObject(dynamicsWorld, new KinematicMotionState(initialTransform)),
        radius(radius_), height(height_), grabState(false),
        constraintPivot(0, -height/2., 0),
        constraint(NULL) { }

    virtual ~P2PGrabberKinematicObject() { if (constraint) delete constraint; }

    virtual void initGraphics();
    virtual void initPhysics();

    void setTransform(const btTransform &trans) { static_cast<KinematicMotionState *> (motionState)->setKinematicPos(trans); }
    void toggleGrabber() { if (grabState) releaseConstraint(); else grabNearestObjectAhead(); grabState = !grabState; }
    void grabNearestObjectAhead();
    void releaseConstraint();
};

void P2PGrabberKinematicObject::initGraphics() {
    // draw a cylinder with a sphere on top of it at the end effector location
    // the default orientation is this: end effector at (0, 0, 0), and the stick going
    // up in the (0, 1, 0) direction
    GLUquadricObj *qobj = gluNewQuadric();
    displayList = glGenLists(1);
    gluQuadricDrawStyle(qobj, GLU_FILL);
    gluQuadricNormals(qobj, GLU_SMOOTH); 

    glNewList(displayList, GL_COMPILE);
    glMatrixMode(GL_MODELVIEW);
    glTranslatef(0, -height/2., 0);
    glRotatef(-90, 1, 0, 0);
    gluCylinder(qobj, radius, radius, height, 32, 32);
    glTranslatef(0, 0, radius);
    glutSolidSphere(radius * 1.5, 32, 32);
    glEndList();

    gluDeleteQuadric(qobj);
}

void P2PGrabberKinematicObject::initPhysics() {
    // we only model the cylinder/stick part. the sphere at the end effector is just cosmetic
    collisionShape = new btCylinderShape(btVector3(radius, height / 2.0, radius));

    btRigidBody::btRigidBodyConstructionInfo ci(0.0, motionState, collisionShape, btVector3(0.0, 0.0, 0.0));
    rigidBody = new btRigidBody(ci);
    // special flags for kinematic objects
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    rigidBody->setActivationState(DISABLE_DEACTIVATION);

    dynamicsWorld->addRigidBody(rigidBody);
}

void P2PGrabberKinematicObject::grabNearestObjectAhead() {
    // first, try to find the object ahead.
    // trace a ray in the direction of the end affector and get the first object hit
    btTransform trans; motionState->getWorldTransform(trans);
    btVector3 rayFrom = trans(btVector3(0, 0, 0)); // some point in the middle of the stick
    btVector3 rayTo = trans(constraintPivot); // the end affector
    rayTo = (rayTo - rayFrom).normalize()*MAX_RAYCAST_DISTANCE + rayTo;

    printf("from: %f %f %f\nto:%f %f %f\n", rayFrom.x(), rayFrom.y(), rayFrom.z(), rayTo.x(), rayTo.y(), rayTo.z());
    // send the ray
    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);
    dynamicsWorld->rayTest(rayFrom, rayTo, rayCallback);
    printf("tracing!\n");
    if (rayCallback.hasHit()) {
        printf("hit!\n");
        btRigidBody *hitBody = btRigidBody::upcast(rayCallback.m_collisionObject);
        if (hitBody && !hitBody->isStaticObject() && !hitBody->isKinematicObject()) {
            hitBody->setActivationState(DISABLE_DEACTIVATION);
            // if it hit, get the point where it hit in the coordinate system of the object hit
            btVector3 localHitPoint = hitBody->getCenterOfMassTransform().inverse() * rayCallback.m_hitPointWorld;
            // now implement the constraint
            releaseConstraint();
            //constraint = new btPoint2PointConstraint(*rigidBody, *hitBody, constraintPivot, localHitPoint);

            btTransform grabberFrame; grabberFrame.setIdentity(); grabberFrame.setOrigin(constraintPivot);
            btTransform hitFrame; hitFrame.setIdentity(); hitFrame.setOrigin(localHitPoint);

			constraint = new btGeneric6DofConstraint(*rigidBody, *hitBody, grabberFrame, hitFrame, false);
/*			constraint->setLinearLowerLimit(btVector3(0,0,0));
			constraint->setLinearUpperLimit(btVector3(0,0,0));
			constraint->setAngularLowerLimit(btVector3(0,0,0));
			constraint->setAngularUpperLimit(btVector3(0,0,0));*/
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.8,0);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.8,1);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.8,2);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.8,3);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.8,4);
			constraint->setParam(BT_CONSTRAINT_STOP_CFM,0.8,5);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.1,0);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.1,1);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.1,2);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.1,3);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.1,4);
			constraint->setParam(BT_CONSTRAINT_STOP_ERP,0.1,5);


            for (int i = 0; i < 3; ++i) {
                constraint->getTranslationalLimitMotor()->m_enableMotor[i] = true;
                constraint->getTranslationalLimitMotor()->m_damping = 1.f;
                //constraint->getTranslationalLimitMotor()->m_targetVelocity[i] = -5.0f;
                //constraint->getTranslationalLimitMotor()->m_maxMotorForce[i] = 0.1f;

                constraint->getRotationalLimitMotor(i)->m_damping = 1.f;
            }

			dynamicsWorld->addConstraint(constraint);


//            constraint->m_setting.m_impulseClamp = 30.;
//            constraint->m_setting.m_tau = 0.001f;
//            dynamicsWorld->addConstraint(constraint);
        }
    }
}

void P2PGrabberKinematicObject::releaseConstraint() {
    if (constraint) {
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
        constraint = NULL;
    }
}

class GrabberKinematicObject : public SimulationObject {
private:
    btScalar radius;
    btScalar height;
    btScalar fingerLength, fingerWidth, fingerThickness;
    const btScalar fingerBaseSeparation;
    bool grabState; // true => grabbing, false => nothing
    const btVector3 finger1Pivot, finger2Pivot;
    btHingeConstraint *finger1Constraint, *finger2Constraint;
    btCompoundShape *compoundShape;
    btCollisionShape *rodShape, *fingerShape;

    GLint rodDisplayList;

public:
    GrabberKinematicObject(btDiscreteDynamicsWorld *dynamicsWorld, btScalar radius_, btScalar height_, btScalar fingerLength_, btScalar fingerWidth_, btScalar fingerThickness_, const btTransform &initialTransform) :
        SimulationObject(dynamicsWorld, new KinematicMotionState(initialTransform)),
        radius(radius_), height(height_), grabState(false),
        fingerLength(fingerLength_), fingerWidth(fingerWidth_), fingerThickness(fingerThickness_),
        fingerBaseSeparation(radius_),
        finger1Pivot(-fingerBaseSeparation/2., -height/2., 0.), finger2Pivot(fingerBaseSeparation/2., -height/2., 0.),
        finger1Constraint(NULL), finger2Constraint(NULL) { }

    virtual ~GrabberKinematicObject();

    virtual void initGraphics();
    virtual void initPhysics();
    virtual void drawLocal();

    void setTransform(const btTransform &trans) { static_cast<KinematicMotionState *> (motionState)->setKinematicPos(trans); }
/*
    void toggleGrabber() { if (grabState) releaseConstraint(); else grabNearestObjectAhead(); grabState = !grabState; }
    void grabNearestObjectAhead();
    void releaseConstraint();*/
};

GrabberKinematicObject::~GrabberKinematicObject() {
    // delete all child shapes of the compound shape
    delete rodShape;
    delete fingerShape;

    if (finger1Constraint) {
        dynamicsWorld->removeConstraint(finger1Constraint);
        delete finger1Constraint;
    }

    if (finger2Constraint) {
        dynamicsWorld->removeConstraint(finger2Constraint);
        delete finger2Constraint;
    }

    glDeleteLists(rodDisplayList, 1);
}

void GrabberKinematicObject::initGraphics() {
    GLUquadricObj *qobj = gluNewQuadric();
    rodDisplayList = glGenLists(1);
    gluQuadricDrawStyle(qobj, GLU_FILL);
    gluQuadricNormals(qobj, GLU_SMOOTH); 
    glNewList(rodDisplayList, GL_COMPILE);
    glMatrixMode(GL_MODELVIEW);
    glRotatef(-90., 1., 0., 0.);
    gluCylinder(qobj, radius, radius, height, 32, 32);
    glEndList();
    gluDeleteQuadric(qobj);
}

void GrabberKinematicObject::drawLocal() {
    // up in the (0, 1, 0) direction
    btScalar m[16];

    glMatrixMode(GL_MODELVIEW);

    // draw the left finger
    compoundShape->getChildTransform(1).getOpenGLMatrix(m);
    glPushMatrix();
    glMultMatrixf(m);
    glScalef(fingerThickness, fingerLength, fingerWidth);
    glutSolidCube(1.0);
    glPopMatrix();

    // draw the right finger
    compoundShape->getChildTransform(2).getOpenGLMatrix(m);
    glPushMatrix();
    glMultMatrixf(m);
    glScalef(fingerThickness, fingerLength, fingerWidth);
    glutSolidCube(1.0);
    glPopMatrix();

    // draw the rod
    glPushMatrix();
    glCallList(rodDisplayList);
    glPopMatrix();
}

void GrabberKinematicObject::initPhysics() {
    // the model has its origin at the base of the rod where the fingers meet

    compoundShape = new btCompoundShape();

    // first add in the handle/rod
    rodShape = new btCylinderShape(btVector3(radius, height/2., radius));
    compoundShape->addChildShape(btTransform(btQuaternion(0., 0., 0., 1.), btVector3(0., height/2., 0.)),
                                 rodShape);

    // add in the left finger
    fingerShape = new btBoxShape(btVector3(fingerThickness/2., fingerLength/2., fingerWidth/2.));
    compoundShape->addChildShape(btTransform(btQuaternion(0., 0., 0., 1.),
                                             btVector3(-fingerThickness/2. - radius, -fingerLength/2., 0.)),
                                 fingerShape);
    
    // add the right finger
    compoundShape->addChildShape(btTransform(btQuaternion(0., 0., 0., 1.),
                                             btVector3(fingerThickness/2. + radius, -fingerLength/2., 0.)),
                                 fingerShape);


    collisionShape = compoundShape;

    btRigidBody::btRigidBodyConstructionInfo ci(0.0, motionState, collisionShape, btVector3(0.0, 0.0, 0.0));
    rigidBody = new btRigidBody(ci);
    // special flags for kinematic objects
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    rigidBody->setActivationState(DISABLE_DEACTIVATION);

    dynamicsWorld->addRigidBody(rigidBody);
}


// GLOBALS ////////////////////////////////
static btBroadphaseInterface *broadphase;
static btDefaultCollisionConfiguration *collisionConfiguration;
static btCollisionDispatcher *dispatcher;
static btSequentialImpulseConstraintSolver *solver;
static btDiscreteDynamicsWorld *dynamicsWorld;
static SimulationObject *groundObject, *sphereObject, *torusObject;
static SimulationObject *cylinderObject[NUM_CYLINDERS];
static GrabberKinematicObject *grabberObject;
static GLDebugDrawer *debugDrawer;

static bool debugDrawingOn = false;
static bool standardDrawingOn = true;

GLfloat viewAzimuth = 0.5, viewAltitude = 1.414/2 /* 45 degrees */, viewDistance = SCALE(0.5);
static GLfloat cameraPos[] = { 0.0, 0.0, 0.0 };
static GLfloat cameraTarget[] = { 0.0, 0.0, 0.0 };
static GLfloat cameraUp[] = { 0.0, 1.0, 0.0 };
///////////////////////////////////////////


void initGraphics() 
{
    static const GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    static const GLfloat mat_shininess[] = { 50.0 };

    static const GLfloat pos[4] =
        {1.0, 1.0, 1.0, 0.0};
    static const GLfloat red[4] =
        {0.8, 0.1, 0.0, 1.0};
    static const GLfloat green[4] =
        {0.0, 0.8, 0.2, 1.0};
    static const GLfloat blue[4] =
        {0.2, 0.2, 1.0, 1.0};

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);

    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
}

void initPhysics() {
    broadphase = new btDbvtBroadphase();
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -GRAVITY, 0));

    if (DEBUG_DRAWING_ENABLED) {
        debugDrawer = new GLDebugDrawer;
        debugDrawer->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE/* | btIDebugDraw::DBG_DrawAabb*/);
        dynamicsWorld->setDebugDrawer(debugDrawer);
    }

    groundObject = new GroundPlaneObject(dynamicsWorld, PLANE_SIZE);
    groundObject->init();

    sphereObject = new SphereObject(dynamicsWorld, 1.0, SPHERE_RADIUS,
        new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.3, 10, 0))));
    sphereObject->init();

    for (int i = 0; i < NUM_CYLINDERS; ++i) {
        cylinderObject[i] = new CylinderObject(dynamicsWorld, 0.0, CYL_RADIUS, CYL_HEIGHT, 
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3((i - NUM_CYLINDERS/2)*CYL_SPACING, CYL_HEIGHT/2., 0))));
        cylinderObject[i]->init();
    }

    torusObject = new TorusObject(dynamicsWorld, 1.0, TORUS_INNER_RADIUS, TORUS_OUTER_RADIUS,
        new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 15+TORUS_INNER_RADIUS, 0))));
    torusObject->init();

    grabberObject = new GrabberKinematicObject(dynamicsWorld, GRABBER_RADIUS, GRABBER_LENGTH, GRABBER_FINGER_LENGTH,
                                               GRABBER_FINGER_WIDTH, GRABBER_FINGER_THICKNESS,
                                               btTransform(btQuaternion(0, 0, 0, 1), btVector3(5.0, 5.0, 0)));
    grabberObject->init();
}

void destroy() {
    delete groundObject;
    delete sphereObject;
    for (int i = 0; i < NUM_CYLINDERS; ++i)
        delete cylinderObject[i];
    delete torusObject;
    delete grabberObject;

    if (DEBUG_DRAWING_ENABLED)
        delete debugDrawer;

    delete dynamicsWorld;
    delete solver;
    delete collisionConfiguration;
    delete dispatcher;
    delete broadphase;
}


static void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 0.0);

    if (standardDrawingOn) {
        groundObject->drawGL();
        for (int i = 0; i < NUM_CYLINDERS; ++i)
            cylinderObject[i]->drawGL();
        sphereObject->drawGL();
        torusObject->drawGL();
        grabberObject->drawGL();
    }

    if (DEBUG_DRAWING_ENABLED && debugDrawingOn)
        dynamicsWorld->debugDrawWorld();

    glutSwapBuffers();
}

static void setCamera() {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // spherical coordinate transformation
    cameraPos[0] = viewDistance * sinf(viewAzimuth) * cosf(viewAltitude);
    cameraPos[1] = viewDistance * sinf(viewAltitude);
    cameraPos[2] = viewDistance * cosf(viewAzimuth) * cosf(viewAltitude);

    cameraUp[0] = 0.0;
    cameraUp[1] = cosf(viewAltitude) > 0.0 ? 1.0 : -1.0;
    cameraUp[2] = 0.0;

    gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2],
              cameraTarget[0], cameraTarget[1], cameraTarget[2],
              cameraUp[0], cameraUp[1], cameraUp[2]);
}

static void reshape(int w, int h) {
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();

    /*float znear = 0.1, zfar = 100;
    float ymax, xmax;
    float temp, temp2, temp3, temp4;
    ymax = znear * tanf(45 * M_PI / 360.0);
    xmax = ymax * w/h;
    glFrustum(-xmax, xmax, -ymax, ymax, znear, zfar);*/
    gluPerspective(45, (float)w/h, 0.1, 100);
    //glFrustum(-1.0, 1.0, -1.0, 1.0, 0.1, 100);
//    glOrtho(-w/2, w/2, -h/2, h/2, 0, 10);

    setCamera();
}

static void idle() {
    static int prevDrawTime = glutGet(GLUT_ELAPSED_TIME);
    int currTime = glutGet(GLUT_ELAPSED_TIME);
    int elapsedTime = currTime - prevDrawTime;
    prevDrawTime = currTime;

    dynamicsWorld->stepSimulation((float)elapsedTime / 1000.0, 500, 1./500.);
    glutPostRedisplay();
}

void visible(int vis) {
    glutIdleFunc(vis == GLUT_VISIBLE ? idle : NULL);
}

static bool rightButtonDown = false, leftButtonDown = false;
static bool holdingZ = false;
static int lastX = -1, lastY = -1;
static void mouse(int button, int state, int x, int y) {
    switch (button) {
    case GLUT_RIGHT_BUTTON:
        if (state == GLUT_DOWN) {
            rightButtonDown = true;
            lastX = -1;
        } else
            rightButtonDown = false;
        break;

    case GLUT_LEFT_BUTTON:
        if (state == GLUT_DOWN) {
            leftButtonDown = true;
            lastX = -1;
        } else
            leftButtonDown = false;
        break;

    case 3: // wheel up
        if (holdingZ) {
            btTransform trans; grabberObject->motionState->getWorldTransform(trans);
            btQuaternion rot = trans.getRotation() * btQuaternion(M_PI/180., 0., 0.);
            trans.setRotation(rot);
            grabberObject->setTransform(trans);
            break;
        }

        if (viewDistance <= 0) break;
        viewDistance -= 1;
        setCamera();
        glutPostRedisplay();
        break;

    case 4: // wheel down
        if (holdingZ) {
            btTransform trans; grabberObject->motionState->getWorldTransform(trans);
            btQuaternion rot = trans.getRotation() * btQuaternion(-M_PI/180., 0., 0.);
            trans.setRotation(rot);
            grabberObject->setTransform(trans);
            break;
        }

        viewDistance += 1;
        setCamera();
        glutPostRedisplay();
        break;
    }
}
static void motion(int x, int y) {
    if (lastX == -1) {
        lastX = x; lastY = y;
        return;
    }
    int dx = x - lastX, dy = y - lastY;
    lastX = x; lastY = y;

    if (rightButtonDown) {
        // right-dragging tilts the camera
        viewAzimuth += -dx*M_PI/180.; // (one degree per pixel of mouse movement)
        viewAltitude += dy*M_PI/180.;
        setCamera();
        glutPostRedisplay();
    } else if (leftButtonDown) {
        if (holdingZ) {
            // if holding z, left-dragging changes pitch/yaw
            btTransform trans; grabberObject->motionState->getWorldTransform(trans);
            btQuaternion rot = btQuaternion(-dx*M_PI/180., dy*M_PI/180., 0.) * trans.getRotation();
            trans.setRotation(rot);
            grabberObject->setTransform(trans);
        } else {
            // else left-dragging will drag the grabber in the plane of view
            btVector3 from(cameraPos[0], cameraPos[1], cameraPos[2]);
            btVector3 to(cameraTarget[0], cameraTarget[1], cameraTarget[2]);
            btVector3 up(cameraUp[0], cameraUp[1], cameraUp[2]); up.normalize();

            // compute basis vectors for the plane of view
            // (the plane normal to the ray from the camera to the center of the scene)
            btVector3 normal = to - from; normal.normalize();
            up = (up.dot(-normal))*normal + up; up.normalize();
            btVector3 xDisplacement = normal.cross(up) * dx / 100.;
            btVector3 yDisplacement = -up * dy / 100.;

            btTransform origTrans; grabberObject->motionState->getWorldTransform(origTrans);
            btTransform newTrans(origTrans);
            newTrans.setOrigin(newTrans.getOrigin() + xDisplacement + yDisplacement);
            grabberObject->setTransform(newTrans);
        }
    }
}

static void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 'q':
        case 27: // esc
            destroy();
            exit(0);
            break;

        case ' ':
            //grabberObject->toggleGrabber();
            break;

        case 'z': case 'Z':
            holdingZ = true;
            break;
    }
}

static void keyboardUp(unsigned char key, int x, int y) {
    switch (key) {
        case 'z': case 'Z':
            holdingZ = false;
            break;
    }
}

static void special(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_F1:
            standardDrawingOn = true;
            debugDrawingOn = false;
            break;

        case GLUT_KEY_F2:
            if (!DEBUG_DRAWING_ENABLED) break;
            standardDrawingOn = true;
            debugDrawingOn = true;
            break;

        case GLUT_KEY_F3:
            if (!DEBUG_DRAWING_ENABLED) break;
            standardDrawingOn = false;
            debugDrawingOn = true;
            break;
    }
}

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);

    glutInitWindowPosition(0, 0);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("hello");
    
    glutDisplayFunc(draw);
    glutReshapeFunc(reshape);
    glutVisibilityFunc(visible);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(special);

    initGraphics();
    initPhysics();

    glutMainLoop();

    return 0;
}
