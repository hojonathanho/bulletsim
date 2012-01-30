#include "softbodies.h"
#include "util.h"

#include <osg/LightModel>
#include <osg/BlendFunc>
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyData.h>
#include <osgbCollision/Utils.h>

#define COPY_ARRAY(a, b) { (a).resize((b).size()); for (int z = 0; z < (b).size(); ++z) (a)[z] = (b)[z]; }

// there is no btSoftBody::appendFace that takes Node pointers, so here it is
static void btSoftBody_appendFace(btSoftBody *psb, btSoftBody::Node *node0, btSoftBody::Node *node1,
        btSoftBody::Node *node2, btSoftBody::Material *mat=0) {
    btAssert(node0!=node1);
    btAssert(node1!=node2);
    btAssert(node2!=node0);
    if (node0==node1)
            return;
    if (node1==node2)
            return;
    if (node2==node0)
            return;

    psb->appendFace(-1, mat);
    btSoftBody::Face &f = psb->m_faces[psb->m_faces.size()-1];
    f.m_n[0] = node0;
    f.m_n[1] = node1;
    f.m_n[2] = node2;
    f.m_ra = AreaOf(f.m_n[0]->m_x, f.m_n[1]->m_x, f.m_n[2]->m_x);
    psb->m_bUpdateRtCst = true;
}

void BulletSoftObject::init() {
    getEnvironment()->bullet->dynamicsWorld->addSoftBody(softBody.get());

    vertices = new osg::Vec3Array;
    normals = new osg::Vec3Array;
    geom = new osg::Geometry;
    geom->setDataVariance(osg::Object::DYNAMIC);
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);
    geom->setVertexArray(vertices.get());
    geom->setNormalArray(normals.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    geode = new osg::Geode;
    geode->addDrawable(geom);

    osg::ref_ptr<osg::LightModel> lightModel = new osg::LightModel;
    lightModel->setTwoSided(true);
    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(lightModel.get(), osg::StateAttribute::ON);

    transform = new osg::MatrixTransform;
    transform->addChild(geode);
    getEnvironment()->osg->root->addChild(transform);
}

void BulletSoftObject::setColor(float r, float g, float b, float a) {
  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(r,g,b,a));
  geom->setColorArray(colors);
  geom->setColorBinding(osg::Geometry::BIND_OVERALL);

  if (a != 1.0f) { // precision problems?
    osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
    blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    osg::StateSet *ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(blendFunc);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  }
}


void BulletSoftObject::preDraw() {
    transform->setMatrix(osgbCollision::asOsgMatrix(softBody->getWorldTransform()));

    if (geom->getNumPrimitiveSets() > 0)
        geom->removePrimitiveSet(0); // there should only be one
    const btSoftBody::tFaceArray &faces = softBody->m_faces;
    vertices->clear();
    normals->clear();

    for (int i = 0; i < faces.size(); ++i) {
        vertices->push_back(util::toOSGVector(faces[i].m_n[0]->m_x));
        normals->push_back(util::toOSGVector(faces[i].m_n[0]->m_n));

        vertices->push_back(util::toOSGVector(faces[i].m_n[1]->m_x));
        normals->push_back(util::toOSGVector(faces[i].m_n[1]->m_n));

        vertices->push_back(util::toOSGVector(faces[i].m_n[2]->m_x));
        normals->push_back(util::toOSGVector(faces[i].m_n[2]->m_n));
    }
    vertices->dirty();
    normals->dirty();
    geom->dirtyBound();
    geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertices->size()));
}

void BulletSoftObject::destroy() {
    getEnvironment()->bullet->dynamicsWorld->removeSoftBody(softBody.get());
}

// adapted from bullet's SerializeDemo.cpp
EnvironmentObject::Ptr BulletSoftObject::copy(Fork &f) const {
    const btSoftBody * const orig = softBody.get();
    int i, j;

    // create a new softBody with the data
    btSoftBody * const psb = new btSoftBody(&f.env->bullet->softBodyWorldInfo);
    f.registerCopy(orig, psb);

    // materials
    psb->m_materials.reserve(orig->m_materials.size());
    for (i=0;i<orig->m_materials.size();i++)
    {
        const btSoftBody::Material *mat = orig->m_materials[i];
        btSoftBody::Material *newMat = psb->appendMaterial();
        newMat->m_flags = mat->m_flags;
        newMat->m_kAST = mat->m_kAST;
        newMat->m_kLST = mat->m_kLST;
        newMat->m_kVST = mat->m_kVST;
        f.registerCopy(mat, newMat);
    }

    // nodes
    psb->m_nodes.reserve(orig->m_nodes.size());
    for (i=0;i<orig->m_nodes.size();i++) {
        const btSoftBody::Node *node = &orig->m_nodes[i];
        psb->appendNode(node->m_x, node->m_im ? 1./node->m_im : 0.);
        btSoftBody::Node *newNode = &psb->m_nodes[psb->m_nodes.size()-1];
        newNode->m_area = node->m_area;
        newNode->m_battach = node->m_battach;
        newNode->m_f = node->m_f;
        newNode->m_im = node->m_im;
        newNode->m_n = node->m_n;
        newNode->m_q = node->m_q;
        newNode->m_v = node->m_v;

        newNode->m_material = (btSoftBody::Material *) f.copyOf(node->m_material);
        BOOST_ASSERT(newNode->m_material);

        f.registerCopy(node, newNode);
        BOOST_ASSERT(f.copyOf(node) == newNode);
        BOOST_ASSERT( ((btSoftBody::Node*)f.copyOf(node))->m_x == node->m_x );
    }

    // links
    psb->m_links.reserve(orig->m_links.size());
    for (i=0;i<orig->m_links.size();i++) {
        const btSoftBody::Link *link = &orig->m_links[i];
        btSoftBody::Material *mat = (btSoftBody::Material *) f.copyOf(link->m_material);
        btSoftBody::Node *n0 = (btSoftBody::Node *) f.copyOf(link->m_n[0]);
        btSoftBody::Node *n1 = (btSoftBody::Node *) f.copyOf(link->m_n[1]);
        BOOST_ASSERT(n0 && n1);
        BOOST_ASSERT( ((btSoftBody::Node*)f.copyOf(link->m_n[0]))->m_x == link->m_n[0]->m_x );
        BOOST_ASSERT( ((btSoftBody::Node*)f.copyOf(link->m_n[1]))->m_x == link->m_n[1]->m_x );
        psb->appendLink(n0, n1, mat);

        btSoftBody::Link *newLink = &psb->m_links[psb->m_links.size() - 1];
        newLink->m_bbending = link->m_bbending;
        newLink->m_rl = link->m_rl;
    }

    // faces
    psb->m_faces.reserve(orig->m_faces.size());
    for (i=0;i<orig->m_faces.size();i++) {
        const btSoftBody::Face *face = &orig->m_faces[i];
        btSoftBody::Material *mat = (btSoftBody::Material *) f.copyOf(face->m_material);
        btSoftBody::Node *n0 = (btSoftBody::Node *) f.copyOf(face->m_n[0]);
        btSoftBody::Node *n1 = (btSoftBody::Node *) f.copyOf(face->m_n[1]);
        btSoftBody::Node *n2 = (btSoftBody::Node *) f.copyOf(face->m_n[2]);
        BOOST_ASSERT(n0 && n1 && n2);

        btSoftBody_appendFace(psb, n0, n1, n2, mat);

        btSoftBody::Face *newFace = &psb->m_faces[psb->m_faces.size()-1];
        newFace->m_normal = face->m_normal;
        newFace->m_ra = face->m_ra;
    }

    // pose
    psb->m_pose.m_bvolume = orig->m_pose.m_bvolume;
    psb->m_pose.m_bframe = orig->m_pose.m_bframe;
    psb->m_pose.m_volume = orig->m_pose.m_volume;
    COPY_ARRAY(psb->m_pose.m_pos, orig->m_pose.m_pos);
    COPY_ARRAY(psb->m_pose.m_wgh, orig->m_pose.m_wgh);
    psb->m_pose.m_com = orig->m_pose.m_com;
    psb->m_pose.m_rot = orig->m_pose.m_rot;
    psb->m_pose.m_scl = orig->m_pose.m_scl;
    psb->m_pose.m_aqq = orig->m_pose.m_aqq;

    // config
    psb->m_cfg.aeromodel = orig->m_cfg.aeromodel;
    psb->m_cfg.kVCF = orig->m_cfg.kVCF;
    psb->m_cfg.kDP = orig->m_cfg.kDP;
    psb->m_cfg.kDG = orig->m_cfg.kDG;
    psb->m_cfg.kLF = orig->m_cfg.kLF;
    psb->m_cfg.kPR = orig->m_cfg.kPR;
    psb->m_cfg.kVC = orig->m_cfg.kVC;
    psb->m_cfg.kDF = orig->m_cfg.kDF;
    psb->m_cfg.kMT = orig->m_cfg.kMT;
    psb->m_cfg.kCHR = orig->m_cfg.kCHR;
    psb->m_cfg.kKHR = orig->m_cfg.kKHR;
    psb->m_cfg.kSHR = orig->m_cfg.kSHR;
    psb->m_cfg.kAHR = orig->m_cfg.kAHR;
    psb->m_cfg.kSRHR_CL = orig->m_cfg.kSRHR_CL;
    psb->m_cfg.kSKHR_CL = orig->m_cfg.kSKHR_CL;
    psb->m_cfg.kSSHR_CL = orig->m_cfg.kSSHR_CL;
    psb->m_cfg.kSR_SPLT_CL = orig->m_cfg.kSR_SPLT_CL;
    psb->m_cfg.kSK_SPLT_CL = orig->m_cfg.kSK_SPLT_CL;
    psb->m_cfg.kSS_SPLT_CL = orig->m_cfg.kSS_SPLT_CL;
    psb->m_cfg.maxvolume = orig->m_cfg.maxvolume;
    psb->m_cfg.timescale = orig->m_cfg.timescale;
    psb->m_cfg.viterations = orig->m_cfg.viterations;
    psb->m_cfg.piterations = orig->m_cfg.piterations;
    psb->m_cfg.diterations = orig->m_cfg.diterations;
    psb->m_cfg.citerations = orig->m_cfg.citerations;
    psb->m_cfg.collisions = orig->m_cfg.collisions;
    COPY_ARRAY(psb->m_cfg.m_vsequence, orig->m_cfg.m_vsequence);
    COPY_ARRAY(psb->m_cfg.m_psequence, orig->m_cfg.m_psequence);
    COPY_ARRAY(psb->m_cfg.m_dsequence, orig->m_cfg.m_dsequence);
    psb->getCollisionShape()->setMargin(orig->getCollisionShape()->getMargin());

    //clusters
    psb->generateClusters(orig->m_clusters.size());

    psb->updateConstants();
    return Ptr(new BulletSoftObject(psb));
}

void BulletSoftObject::postCopy(EnvironmentObject::Ptr copy, Fork &f) const {
    const btSoftBody *orig = softBody.get();
    btSoftBody *psb = boost::static_pointer_cast<BulletSoftObject>(copy)->softBody.get();

    // copy the anchors
    psb->m_anchors.reserve(orig->m_anchors.size());
    for (int i=0;i<orig->m_anchors.size();i++) {
        const btSoftBody::Anchor &anchor = orig->m_anchors[i];
        btRigidBody *body = btRigidBody::upcast((btCollisionObject *) f.copyOf(anchor.m_body));
        BOOST_ASSERT(body);
        if (!body) continue;

        btSoftBody::Anchor newAnchor = anchor;
        newAnchor.m_body = body;

        newAnchor.m_node = (btSoftBody::Node *) f.copyOf(anchor.m_node);
        BOOST_ASSERT(newAnchor.m_node);

        psb->m_anchors.push_back(newAnchor);

        // TODO: disableCollisionBetweenLinkedBodies
    }

    // copy the joints
    // TODO: THIS IS NOT TESTED
#if 0
    psb->m_joints.reserve(orig->m_joints.size());
    for (int i=0;i<orig->m_joints.size();i++) {
        btSoftBody::Joint *joint = orig->m_joints[i];

        const btTransform &transA = psb->m_clusters[0]->m_framexform;

        btSoftBody::Body bdyB;
        const btSoftBody::Body &origBodyB = joint->m_bodies[1];
        if (origBodyB.m_soft)
            bdyB.m_soft = (btSoftBody::Cluster *) f.copyOf(origBodyB.m_soft);
        if (origBodyB.m_rigid)
            bdyB.m_rigid = (btRigidBody *) f.copyOf(origBodyB.m_rigid);
        if (origBodyB.m_collisionObject)
            bdyB.m_collisionObject = (btCollisionObject *) f.copyOf(origBodyB.m_collisionObject);

        /*
        btCollisionObject *colB = f.copyOf(joint->m_bodies[1]);
        switch (colB->getInternalType()) {
        case CO_COLLISION_OBJECT:
            bdyB = colB; break;
        case CO_RIGID_BODY:
            bdyB = btRigidBody::upcast(colB); break;
        case CO_SOFT_BODY:
            bdyB = ((btSoftBody *) colB)->m_clusters[0]; break;
        }*/

        if (joint->Type() == btSoftBody::Joint::eType::Linear) {
            btSoftBody::LJoint *ljoint = (btSoftBody::LJoint *) joint, newljoint;
            btSoftBody::LJoint::Specs specs;
            specs.cfm = ljoint->m_cfm;
            specs.erp = ljoint->m_erp;
            specs.split = ljoint->m_split;
            specs.position = transA * ljoint->m_refs[0];
            psb->appendLinearJoint(specs, psb->m_clusters[0], bdyB);
        } else if (joint->Type() == btSoftBody::Joint::eType::Angular) {
            btSoftBody::AJoint *ajoint = (btSoftBody::AJoint *) joint;
            btSoftBody::AJoint::Specs specs;
            specs.cfm = ajoint->m_cfm;
            specs.erp = ajoint->m_erp;
            specs.split = ajoint->m_split;
            specs.axis = transA.getBasis() * ajoint->m_refs[0];
            psb->appendAngularJoint(specs, psb->m_clusters[0], bdyB);
        } else if (joint->Type() == btSoftBody::Joint::eType::Contact) {
            btSoftBody::CJoint *cjoint = (btSoftBody::CJoint *) joint;
            cout << "error: contact joints not implemented!\n";
        } else {
            cout << "error: unknown joint type " << joint->Type() << '\n';
        }
    }
#endif
}
