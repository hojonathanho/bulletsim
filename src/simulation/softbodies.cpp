#include "softbodies.h"
#include "util.h"
#include "bullet_io.h"
#include <fstream>

#include <osg/LightModel>
#include <osg/BlendFunc>
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyData.h>
#include <osgbCollision/Utils.h>
#include <osgUtil/SmoothingVisitor>

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

    trivertices = new osg::Vec3Array;
    trinormals = new osg::Vec3Array;
    trigeom = new osg::Geometry;
    trigeom->setDataVariance(osg::Object::DYNAMIC);
    trigeom->setUseDisplayList(false);
    trigeom->setUseVertexBufferObjects(true);
    trigeom->setVertexArray(trivertices.get());
    trigeom->setNormalArray(trinormals.get());
    trigeom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    quadvertices = new osg::Vec3Array;
    quadnormals = new osg::Vec3Array;
    quadgeom = new osg::Geometry;
    quadgeom->setDataVariance(osg::Object::DYNAMIC);
    quadgeom->setUseDisplayList(false);
    quadgeom->setUseVertexBufferObjects(true);
    quadgeom->setVertexArray(quadvertices.get());
    quadgeom->setNormalArray(quadnormals.get());
    quadgeom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    geode = new osg::Geode;
    geode->addDrawable(trigeom);
    geode->addDrawable(quadgeom);

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
  trigeom->setColorArray(colors);
  trigeom->setColorBinding(osg::Geometry::BIND_OVERALL);
  quadgeom->setColorArray(colors);
  quadgeom->setColorBinding(osg::Geometry::BIND_OVERALL);

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

    if (trigeom->getNumPrimitiveSets() > 0)
        trigeom->removePrimitiveSet(0); // there should only be one
    trivertices->clear();
    trinormals->clear();
    const btSoftBody::tFaceArray &faces = softBody->m_faces;
    for (int i = 0; i < faces.size(); ++i) {
        trivertices->push_back(util::toOSGVector(faces[i].m_n[0]->m_x));
        trivertices->push_back(util::toOSGVector(faces[i].m_n[1]->m_x));
        trivertices->push_back(util::toOSGVector(faces[i].m_n[2]->m_x));

        trinormals->push_back(util::toOSGVector(faces[i].m_n[0]->m_n));
        trinormals->push_back(util::toOSGVector(faces[i].m_n[1]->m_n));
        trinormals->push_back(util::toOSGVector(faces[i].m_n[2]->m_n));
    }
    trivertices->dirty();
    trinormals->dirty();
    trigeom->dirtyBound();
    trigeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, trivertices->size()));

    if (quadgeom->getNumPrimitiveSets() > 0)
        quadgeom->removePrimitiveSet(0);
    quadvertices->clear();
    quadnormals->clear();
    const btSoftBody::tTetraArray &tetras = softBody->m_tetras;
    for (int i = 0; i < tetras.size(); ++i) {
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[0]->m_x));
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[1]->m_x));
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[2]->m_x));
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[3]->m_x));
/*
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[0]->m_n));
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[1]->m_n));
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[2]->m_n));
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[3]->m_n));*/
    }
    quadvertices->dirty();
    quadnormals->dirty();
    quadgeom->dirtyBound();
    quadgeom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, quadvertices->size()));

/*    osgUtil::SmoothingVisitor sv;
    sv.apply(*geode);*/
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
    for (i=0;i<orig->m_materials.size();i++) {
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

    // solver state
    psb->m_sst = orig->m_sst;

    // clusters
    psb->m_clusters.resize(orig->m_clusters.size());
    for (i=0;i<orig->m_clusters.size();i++) {
        btSoftBody::Cluster *cl = orig->m_clusters[i];
        btSoftBody::Cluster *newcl = psb->m_clusters[i] =
            new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();

        newcl->m_nodes.resize(cl->m_nodes.size());
        for (j = 0; j < cl->m_nodes.size(); ++j)
            newcl->m_nodes[j] = (btSoftBody::Node *) f.copyOf(cl->m_nodes[j]);
        COPY_ARRAY(newcl->m_masses, cl->m_masses);
        COPY_ARRAY(newcl->m_framerefs, cl->m_framerefs);
        newcl->m_framexform = cl->m_framexform;
        newcl->m_idmass = cl->m_idmass;
        newcl->m_imass = cl->m_imass;
        newcl->m_locii = cl->m_locii;
        newcl->m_invwi = cl->m_invwi;
        newcl->m_com = cl->m_com;
        newcl->m_vimpulses[0] = cl->m_vimpulses[0];
        newcl->m_vimpulses[1] = cl->m_vimpulses[1];
        newcl->m_dimpulses[0] = cl->m_dimpulses[0];
        newcl->m_dimpulses[1] = cl->m_dimpulses[1];
        newcl->m_nvimpulses = cl->m_nvimpulses;
        newcl->m_ndimpulses = cl->m_ndimpulses;
        newcl->m_lv = cl->m_lv;
        newcl->m_av = cl->m_av;
        newcl->m_leaf = 0; // soft body code will set this automatically
        newcl->m_ndamping = cl->m_ndamping;
        newcl->m_ldamping = cl->m_ldamping;
        newcl->m_adamping = cl->m_adamping;
        newcl->m_matching = cl->m_matching;
        newcl->m_maxSelfCollisionImpulse = cl->m_maxSelfCollisionImpulse;
        newcl->m_selfCollisionImpulseFactor = cl->m_selfCollisionImpulseFactor;
        newcl->m_containsAnchor = cl->m_containsAnchor;
        newcl->m_collide = cl->m_collide;
        newcl->m_clusterIndex = cl->m_clusterIndex;
    }

    // cluster connectivity
    COPY_ARRAY(psb->m_clusterConnectivity, orig->m_clusterConnectivity);

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

void saveSoftBody(const btSoftBody* orig, const char* fileName) {  
  int i, j;
  ofstream saveFile;
  saveFile.open(fileName);
  
  // materials
  map<const btSoftBody::Material*, int> matMap;
  saveFile << orig->m_materials.size() << endl;
  for (i = 0;i < orig->m_materials.size(); i++) {
    const btSoftBody::Material* mat = orig->m_materials[i];
    matMap[mat] = i;
    saveFile << mat->m_flags << " ";
    saveFile << mat->m_kAST << " ";
    saveFile << mat->m_kLST << " ";
    saveFile << mat->m_kVST << endl;
  }

  // nodes
  map<const btSoftBody::Node*, int> nodeMap;
  saveFile << orig->m_nodes.size() << endl;
  for (i = 0; i < orig->m_nodes.size(); i++) {
    const btSoftBody::Node* node = &orig->m_nodes[i];
    nodeMap[node] = i;
    saveFile << node->m_x << " ";
    saveFile << node->m_im << " ";
    saveFile << node->m_area << " ";
    saveFile << node->m_battach << " ";
    saveFile << node->m_f << " ";
    saveFile << node->m_n << " ";
    saveFile << node->m_q << " ";
    saveFile << node->m_v << " ";
    saveFile << matMap[node->m_material] << endl;
  }
  
  // links
  saveFile << orig->m_links.size() << endl;
  for (i = 0; i < orig->m_links.size(); i++) {
    const btSoftBody::Link *link = &orig->m_links[i];
    saveFile << matMap[link->m_material] << " ";
    saveFile << nodeMap[link->m_n[0]] << " ";
    saveFile << nodeMap[link->m_n[1]] << " ";
    saveFile << link->m_bbending << " ";
    saveFile << link->m_rl << endl;
  }
  
  // faces
  saveFile << orig->m_faces.size() << endl;
  for (i = 0; i < orig->m_faces.size(); i++) {
    const btSoftBody::Face *face = &orig->m_faces[i];
    saveFile << matMap[face->m_material] << " ";
    saveFile << nodeMap[face->m_n[0]] << " ";
    saveFile << nodeMap[face->m_n[1]] << " ";
    saveFile << nodeMap[face->m_n[2]] << " ";
    saveFile << face->m_normal << " ";
    saveFile << face->m_ra << endl;
  }
  
  // pose
  saveFile << orig->m_pose.m_bvolume << " ";
  saveFile << orig->m_pose.m_bframe << " ";
  saveFile << orig->m_pose.m_volume << endl;
  saveFile << orig->m_pose.m_pos.size() << endl;
  for (i = 0; i < orig->m_pose.m_pos.size(); i++) {
    saveFile << orig->m_pose.m_pos[i] << endl;
  }
  saveFile << orig->m_pose.m_wgh.size() << endl;
  for (i = 0; i < orig->m_pose.m_wgh.size(); i++) {
    saveFile << orig->m_pose.m_wgh[i] << endl;
  }
  saveFile << orig->m_pose.m_com << " ";
  saveFile << orig->m_pose.m_rot << " ";
  saveFile << orig->m_pose.m_scl << " ";
  saveFile << orig->m_pose.m_aqq << endl;

  // config
  saveFile << orig->m_cfg.aeromodel << " ";
  saveFile << orig->m_cfg.kVCF << " ";
  saveFile << orig->m_cfg.kDP << " ";
  saveFile << orig->m_cfg.kDG << " ";
  saveFile << orig->m_cfg.kLF << " ";
  saveFile << orig->m_cfg.kPR << " ";
  saveFile << orig->m_cfg.kVC << " ";
  saveFile << orig->m_cfg.kDF << " ";
  saveFile << orig->m_cfg.kMT << " ";
  saveFile << orig->m_cfg.kCHR << " ";
  saveFile << orig->m_cfg.kKHR << " ";
  saveFile << orig->m_cfg.kSHR << " ";
  saveFile << orig->m_cfg.kAHR << " ";
  saveFile << orig->m_cfg.kSRHR_CL << " ";
  saveFile << orig->m_cfg.kSKHR_CL << " ";
  saveFile << orig->m_cfg.kSSHR_CL << " ";
  saveFile << orig->m_cfg.kSR_SPLT_CL << " ";
  saveFile << orig->m_cfg.kSK_SPLT_CL << " ";
  saveFile << orig->m_cfg.kSS_SPLT_CL << " ";
  saveFile << orig->m_cfg.maxvolume << " ";
  saveFile << orig->m_cfg.timescale << " ";
  saveFile << orig->m_cfg.viterations << " ";
  saveFile << orig->m_cfg.piterations << " ";
  saveFile << orig->m_cfg.diterations << " ";
  saveFile << orig->m_cfg.citerations << " ";
  saveFile << orig->m_cfg.collisions << endl;
  saveFile << orig->m_cfg.m_vsequence.size() << endl;
  for (i = 0; i < orig->m_cfg.m_vsequence.size(); i++) {
    saveFile << orig->m_cfg.m_vsequence[i] << endl;
  }
  saveFile << orig->m_cfg.m_psequence.size() << endl;
  for (i = 0; i < orig->m_cfg.m_psequence.size(); i++) {
    saveFile << orig->m_cfg.m_psequence[i] << endl;
  }
  saveFile << orig->m_cfg.m_dsequence.size() << endl;
  for (i = 0; i < orig->m_cfg.m_dsequence.size(); i++) {
    saveFile << orig->m_cfg.m_dsequence[i] << endl;
  }
  saveFile << orig->getCollisionShape()->getMargin() << endl;

  // solver state
  saveFile << orig->m_sst.isdt << " ";
  saveFile << orig->m_sst.radmrg << " ";
  saveFile << orig->m_sst.sdt << " ";
  saveFile << orig->m_sst.updmrg << " ";
  saveFile << orig->m_sst.velmrg << endl;
  
  // clusters
  saveFile << orig->m_clusters.size() << endl;
  for (i = 0; i < orig->m_clusters.size(); i++) {
    btSoftBody::Cluster *cl = orig->m_clusters[i];
    saveFile << cl->m_nodes.size() << endl;
    for (j = 0; j < cl->m_nodes.size(); j++)
      saveFile << nodeMap[cl->m_nodes[j]] << endl;
    saveFile << cl->m_masses.size() << endl;
    for (j = 0; j < cl->m_masses.size(); j++)
      saveFile << cl->m_masses[j] << endl;
    saveFile << cl->m_framerefs.size() << endl;
    for (j = 0; j < cl->m_framerefs.size(); j++)
      saveFile << cl->m_framerefs[j] << endl;
    saveFile << cl->m_framexform << " ";
    saveFile << cl->m_idmass << " ";
    saveFile << cl->m_imass << " ";
    saveFile << cl->m_locii << " ";
    saveFile << cl->m_invwi << " ";
    saveFile << cl->m_com << " ";
    saveFile << cl->m_vimpulses[0] << " ";
    saveFile << cl->m_vimpulses[1] << " ";
    saveFile << cl->m_dimpulses[0] << " ";
    saveFile << cl->m_dimpulses[1] << " ";
    saveFile << cl->m_nvimpulses << " ";
    saveFile << cl->m_ndimpulses << " ";
    saveFile << cl->m_lv << " ";
    saveFile << cl->m_av << " ";
    saveFile << cl->m_ndamping << " ";
    saveFile << cl->m_ldamping << " ";
    saveFile << cl->m_adamping << " ";
    saveFile << cl->m_matching << " ";
    saveFile << cl->m_maxSelfCollisionImpulse << " ";
    saveFile << cl->m_selfCollisionImpulseFactor << " ";
    saveFile << cl->m_containsAnchor << " ";
    saveFile << cl->m_collide << " ";
    saveFile << cl->m_clusterIndex << endl;
  }

  // cluster connectivity
  saveFile << orig->m_clusterConnectivity.size() << endl;
  for (i = 0; i < orig->m_clusterConnectivity.size(); i++) {
    saveFile << orig->m_clusterConnectivity[i] << " ";
  }
  
  saveFile.close();
}

btSoftBody* loadSoftBody(btSoftBodyWorldInfo& worldInfo,
			 const char* fileName) {
  int i, j, size;
  ifstream loadFile;
  loadFile.open(fileName);

  btSoftBody * const psb = new btSoftBody(&worldInfo);

  // materials
  loadFile >> size;
  psb->m_materials.reserve(size);
  for (i = 0; i < size; i++) {
    btSoftBody::Material *newMat = psb->appendMaterial();
    loadFile >> newMat->m_flags;
    loadFile >> newMat->m_kAST;
    loadFile >> newMat->m_kLST;
    loadFile >> newMat->m_kVST;
  }

  // nodes
  loadFile >> size;
  psb->m_nodes.reserve(size);
  for (i = 0; i < size; i++) {
    btVector3 m_x;
    float m_im;
    loadFile >> m_x >> m_im;
    psb->appendNode(m_x, m_im ? 1./m_im : 0.);
    btSoftBody::Node *newNode = &psb->m_nodes[psb->m_nodes.size()-1];
    newNode->m_im = m_im;
    loadFile >> newNode->m_area;
    int b;
    loadFile >> b;
    newNode->m_battach = b;
    loadFile >> newNode->m_f;
    loadFile >> newNode->m_n;
    loadFile >> newNode->m_q;
    loadFile >> newNode->m_v;
    int m;
    loadFile >> m;
    newNode->m_material = psb->m_materials[m];
    BOOST_ASSERT(newNode->m_material);
  }

  // links
  loadFile >> size;
  psb->m_links.reserve(size);
  for (i = 0; i < size; i++) {
    int m, n0, n1;
    loadFile >> m >> n0 >> n1;
    btSoftBody::Material* mat = psb->m_materials[m];
    btSoftBody::Node* node0 = &psb->m_nodes[n0];
    btSoftBody::Node* node1 = &psb->m_nodes[n1];
    BOOST_ASSERT(mat && node0 && node1);
    psb->appendLink(node0, node1, mat);

    btSoftBody::Link *newLink = &psb->m_links[psb->m_links.size() - 1];
    int b;
    loadFile >> b;
    newLink->m_bbending = b;
    loadFile >> newLink->m_rl;
  }

  // faces
  loadFile >> size;
  psb->m_faces.reserve(size);
  for (i = 0; i < size; i++) {
    int m, n0, n1, n2;
    loadFile >> m >> n0 >> n1 >> n2;
    btSoftBody::Material* mat = psb->m_materials[m];
    btSoftBody::Node* node0 = &psb->m_nodes[n0];
    btSoftBody::Node* node1 = &psb->m_nodes[n1];
    btSoftBody::Node* node2 = &psb->m_nodes[n2];
    BOOST_ASSERT(mat && node0 && node1 && node2);
    btAssert(node0!=node1);
    btAssert(node1!=node2);
    btAssert(node2!=node0);
    psb->appendFace(-1, mat);

    btSoftBody::Face &newFace = psb->m_faces[psb->m_faces.size()-1];
    newFace.m_n[0] = node0;
    newFace.m_n[1] = node1;
    newFace.m_n[2] = node2;
    psb->m_bUpdateRtCst = true;
    loadFile >> newFace.m_normal;
    loadFile >> newFace.m_ra;
  }

  // pose
  loadFile >> psb->m_pose.m_bvolume;
  loadFile >> psb->m_pose.m_bframe;
  loadFile >> psb->m_pose.m_volume;
  loadFile >> size;
  psb->m_pose.m_pos.resize(size);
  for (i = 0; i < size; i++) {
    loadFile >> psb->m_pose.m_pos[i];
  }
  loadFile >> size;
  psb->m_pose.m_wgh.resize(size);
  for (i = 0; i < size; i++) {
    loadFile >> psb->m_pose.m_wgh[i];
  }
  loadFile >> psb->m_pose.m_com;
  loadFile >> psb->m_pose.m_rot;
  loadFile >> psb->m_pose.m_scl;
  loadFile >> psb->m_pose.m_aqq;

  // config
  int a;
  loadFile >> a;
  psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_) a;
  loadFile >> psb->m_cfg.kVCF;
  loadFile >> psb->m_cfg.kDP;
  loadFile >> psb->m_cfg.kDG;
  loadFile >> psb->m_cfg.kLF;
  loadFile >> psb->m_cfg.kPR;
  loadFile >> psb->m_cfg.kVC;
  loadFile >> psb->m_cfg.kDF;
  loadFile >> psb->m_cfg.kMT;
  loadFile >> psb->m_cfg.kCHR;
  loadFile >> psb->m_cfg.kKHR;
  loadFile >> psb->m_cfg.kSHR;
  loadFile >> psb->m_cfg.kAHR;
  loadFile >> psb->m_cfg.kSRHR_CL;
  loadFile >> psb->m_cfg.kSKHR_CL;
  loadFile >> psb->m_cfg.kSSHR_CL;
  loadFile >> psb->m_cfg.kSR_SPLT_CL;
  loadFile >> psb->m_cfg.kSK_SPLT_CL;
  loadFile >> psb->m_cfg.kSS_SPLT_CL;
  loadFile >> psb->m_cfg.maxvolume;
  loadFile >> psb->m_cfg.timescale;
  loadFile >> psb->m_cfg.viterations;
  loadFile >> psb->m_cfg.piterations;
  loadFile >> psb->m_cfg.diterations;
  loadFile >> psb->m_cfg.citerations;
  loadFile >> psb->m_cfg.collisions;
  loadFile >> size;
  psb->m_cfg.m_vsequence.resize(size);
  for (i = 0; i < size; i++) {
    int v;
    loadFile >> v;
    psb->m_cfg.m_vsequence[i] = (btSoftBody::eVSolver::_) v;
  }
  loadFile >> size;
  psb->m_cfg.m_psequence.resize(size);
  for (i = 0; i < size; i++) {
    int p;
    loadFile >> p;
    psb->m_cfg.m_psequence[i] = (btSoftBody::ePSolver::_) p;
  }
  loadFile >> size;
  psb->m_cfg.m_dsequence.resize(size);
  for (i = 0; i < size; i++) {
    int p;
    loadFile >> p;
    psb->m_cfg.m_dsequence[i] = (btSoftBody::ePSolver::_) p;
  }
  float m;
  loadFile >> m;
  psb->getCollisionShape()->setMargin(m);

  // solver state
  loadFile >> psb->m_sst.isdt;
  loadFile >> psb->m_sst.radmrg;
  loadFile >> psb->m_sst.sdt;
  loadFile >> psb->m_sst.updmrg;
  loadFile >> psb->m_sst.velmrg;

  // clusters
  loadFile >> size;
  psb->m_clusters.resize(size);
  for (i = 0; i < size; i++) {
    btSoftBody::Cluster *newcl = psb->m_clusters[i] =
      new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();
    
    int size2;
    loadFile >> size2;
    newcl->m_nodes.resize(size2);
    for (j = 0; j < size2; j++) {
      int n;
      loadFile >> n;
      newcl->m_nodes[j] = &psb->m_nodes[n];
    }
    loadFile >> size2;
    newcl->m_masses.resize(size2);
    for (j = 0; j < size2; j++) {
      loadFile >> newcl->m_masses[j];
    }
    loadFile >> size2;
    newcl->m_framerefs.resize(size2);
    for (j = 0; j < size2; j++) {
      loadFile >> newcl->m_framerefs[j];
    }
    loadFile >> newcl->m_framexform;
    loadFile >> newcl->m_idmass;
    loadFile >> newcl->m_imass;
    loadFile >> newcl->m_locii;
    loadFile >> newcl->m_invwi;
    loadFile >> newcl->m_com;
    loadFile >> newcl->m_vimpulses[0];
    loadFile >> newcl->m_vimpulses[1];
    loadFile >> newcl->m_dimpulses[0];
    loadFile >> newcl->m_dimpulses[1];
    loadFile >> newcl->m_nvimpulses;
    loadFile >> newcl->m_ndimpulses;
    loadFile >> newcl->m_lv;
    loadFile >> newcl->m_av;
    newcl->m_leaf = 0; // soft body code will set this automatically
    loadFile >> newcl->m_ndamping;
    loadFile >> newcl->m_ldamping;
    loadFile >> newcl->m_adamping;
    loadFile >> newcl->m_matching;
    loadFile >> newcl->m_maxSelfCollisionImpulse;
    loadFile >> newcl->m_selfCollisionImpulseFactor;
    loadFile >> newcl->m_containsAnchor;
    loadFile >> newcl->m_collide;
    loadFile >> newcl->m_clusterIndex;
  }

  // cluster connectivity
  loadFile >> size;
  psb->m_clusterConnectivity.resize(size);
  for (i = 0; i < size; i++) {
    loadFile >> psb->m_clusterConnectivity[i];
  }

  loadFile.close();
  return psb;
}
