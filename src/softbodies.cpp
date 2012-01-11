#include "softbodies.h"
#include "util.h"

#include <osg/LightModel>
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyData.h>
#include <osgbCollision/Utils.h>

// there is no btSoftBody::appendFace that takes Node pointers, so here it is
static void btSoftBody_appendFace(btSoftBody *psb, btSoftBody::Node *node0, btSoftBody::Node *node1,
        btSoftBody::Node *node2, btSoftBody::Material *mat=0) {
    if (node0==node1)
            return;
    if (node1==node2)
            return;
    if (node2==node0)
            return;

    psb->appendFace(-1, mat);
    btSoftBody::Face &f = psb->m_faces[psb->m_faces.size()-1];
    btAssert(node0!=node1);
    btAssert(node1!=node2);
    btAssert(node2!=node0);
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

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);

    osg::ref_ptr<osg::LightModel> lightModel = new osg::LightModel;
    lightModel->setTwoSided(true);
    geode->getOrCreateStateSet()->setAttributeAndModes(lightModel.get(), osg::StateAttribute::ON);

    transform = new osg::MatrixTransform;
    transform->addChild(geode);
    getEnvironment()->osg->root->addChild(transform);
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
EnvironmentObject::Ptr BulletSoftObject::copy(Fork &f) {
    //TODO: rewrite to do this directly without serialization
    // use postCopy to assign anchors and joints
#if 0
    // to copy, we just serialize and deserialize (just like BulletObject)
    boost::shared_ptr<btDefaultSerializer> serializer(new btDefaultSerializer());
    int len; btChunk *chunk; const char *structType;
    serializer->startSerialization();
        len = o.softBody->calculateSerializeBufferSize();
        chunk = serializer->allocate(len, 1);
        structType = o.softBody->serialize(chunk->m_oldPtr, serializer.get());
        serializer->finalizeChunk(chunk, structType, BT_SOFTBODY_CODE, o.softBody.get());
    serializer->finishSerialization();

    // read the data that the serializer just wrote
    const int bufSize = serializer->getCurrentBufferSize();
    boost::scoped_array<char> buf(new char[bufSize]);
    memcpy(buf.get(), serializer->getBufferPointer(), bufSize);
    boost::shared_ptr<bParse::btBulletFile> bulletFile(
        new bParse::btBulletFile(buf.get(), bufSize));
    bulletFile->parse(false);
#endif

    const btSoftBody &orig = *softBody.get();
    int i, j;

    // create a new softBody with the data
    btSoftBody *psb = new btSoftBody(&f.env->bullet->softBodyWorldInfo);
    f.registerCopy(&orig, psb);

    //materials
 //   typedef std::map<const btSoftBody::Material *, btSoftBody::Material *> MaterialMap; // maps old mats to new ones
 //   MaterialMap materialMap;
    for (i=0;i<orig.m_materials.size();i++)
    {
        const btSoftBody::Material *mat = orig.m_materials[i];
        btSoftBody::Material *newMat = psb->appendMaterial();
        newMat->m_flags = mat->m_flags;
        newMat->m_kAST = mat->m_kAST;
        newMat->m_kLST = mat->m_kLST;
        newMat->m_kVST = mat->m_kVST;
        f.registerCopy(mat, newMat);
//        materialMap[mat] = newMat;
/*
            SoftBodyMaterialData* matData = softBodyData->m_materials[i];
            btSoftBody::Material** matPtr = materialMap.find(matData);
            btSoftBody::Material* mat = 0;
            if (matPtr&& *matPtr)
            {
                    mat = *matPtr;
            } else
            {
                    mat = psb->appendMaterial();
                    mat->m_flags = matData->m_flags;
                    mat->m_kAST = matData->m_angularStiffness;
                    mat->m_kLST = matData->m_linearStiffness;
                    mat->m_kVST = matData->m_volumeStiffness;
                    materialMap.insert(matData,mat);
            }
            */
    }

//    typedef std::map<const btSoftBody::Node *, btSoftBody::Node *> NodeMap; // maps old nodes to new ones
  //  NodeMap nodeMap;
    for (i=0;i<orig.m_nodes.size();i++)
    {
        const btSoftBody::Node &node = orig.m_nodes[i];
        btScalar mass = node.m_im ? 1./node.m_im : 0.f;
        psb->appendNode(node.m_x, mass);
        btSoftBody::Node &newNode = psb->m_nodes[psb->m_nodes.size()-1];
        newNode.m_area = node.m_area;
        newNode.m_battach = node.m_battach;
        newNode.m_f = node.m_f;
        newNode.m_im = node.m_im;
        newNode.m_n = node.m_n;
        newNode.m_q = node.m_q; /*newNode.m_x;*/
        newNode.m_v = node.m_v;

/*        MaterialMap::const_iterator it = materialMap.find(node.m_material);
        if (it != materialMap.end())
            newNode.m_material = it->second;*/
        newNode.m_material = (btSoftBody::Material *) f.forkOf(node.m_material);

        f.registerCopy(&node, &newNode);
        //nodeMap[&node] = &newNode;
/*
            SoftBodyNodeData& nodeData = softBodyData->m_nodes[i];
            btVector3 position;
            position.deSerializeFloat(nodeData.m_position);
            btScalar mass = nodeData.m_inverseMass? 1./nodeData.m_inverseMass : 0.f;
            psb->appendNode(position,mass);
            btSoftBody::Node* node = &psb->m_nodes[psb->m_nodes.size()-1];
            node->m_area = nodeData.m_area;
            node->m_battach = nodeData.m_attach;
            node->m_f.deSerializeFloat(nodeData.m_accumulatedForce);
            node->m_im = nodeData.m_inverseMass;

            btSoftBody::Material** matPtr = materialMap.find(nodeData.m_material);
            if (matPtr && *matPtr)
            {
                    node->m_material = *matPtr;
            } else
            {
                    printf("no mat?\n");
            }
            
            node->m_n.deSerializeFloat(nodeData.m_normal);
            node->m_q = node->m_x;
            node->m_v.deSerializeFloat(nodeData.m_velocity);
 */           
    }

    for (i=0;i<orig.m_links.size();i++)
    {
        const btSoftBody::Link &link = orig.m_links[i];
        btSoftBody::Material *mat = (btSoftBody::Material *) f.forkOf((const void *) link.m_material);
//        MaterialMap::const_iterator it = materialMap.find(link.m_material);
        psb->appendLink(link.m_n[0], link.m_n[1], mat);

        btSoftBody::Link &newLink = psb->m_links[psb->m_links.size() - 1];
        newLink.m_bbending = link.m_bbending;
        newLink.m_rl = link.m_rl;
/*
            SoftBodyLinkData& linkData = softBodyData->m_links[i];
            btSoftBody::Material** matPtr = materialMap.find(linkData.m_material);
            if (matPtr && *matPtr)
            {
                    psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1],*matPtr);
            } else
            {
                    psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1]);
            }
            btSoftBody::Link* link = &psb->m_links[psb->m_links.size()-1];
            link->m_bbending = linkData.m_bbending;
            link->m_rl = linkData.m_restLength;
            */
    }

    for (i=0;i<orig.m_faces.size();i++)
    {
        const btSoftBody::Face &face = orig.m_faces[i];
/*        MaterialMap::const_iterator it = materialMap.find(face.m_material);
        if (it != materialMap.end())
            btSoftBody_appendFace(psb, face.m_n[0], face.m_n[1], face.m_n[2], it->second);
        else
            btSoftBody_appendFace(psb, face.m_n[0], face.m_n[1], face.m_n[2]);*/
        btSoftBody::Material *mat = (btSoftBody::Material *) f.forkOf(face.m_material);
        btSoftBody_appendFace(psb, face.m_n[0], face.m_n[1], face.m_n[2], mat);

        btSoftBody::Face &newFace = psb->m_faces[psb->m_faces.size()-1];
        newFace.m_normal = face.m_normal;
        newFace.m_ra = face.m_ra;
/*
            SoftBodyFaceData& faceData = softBodyData->m_faces[i];
            btSoftBody::Material** matPtr = materialMap.find(faceData.m_material);
            if (matPtr && *matPtr)
            {
                    psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2],*matPtr);
            } else
            {
                    psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2]);
            }
            btSoftBody::Face* face = &psb->m_faces[psb->m_faces.size()-1];
            face->m_normal.deSerializeFloat(faceData.m_normal);
            face->m_ra = faceData.m_restArea;
            */
    }

    

    psb->m_pose = orig.m_pose;
    /*
    if (softBodyData->m_pose)
    {
            psb->m_pose.m_aqq.deSerializeFloat(  softBodyData->m_pose->m_aqq);
            psb->m_pose.m_bframe = (softBodyData->m_pose->m_bframe!=0);
            psb->m_pose.m_bvolume = (softBodyData->m_pose->m_bvolume!=0);
            psb->m_pose.m_com.deSerializeFloat(softBodyData->m_pose->m_com);
            
            psb->m_pose.m_pos.resize(softBodyData->m_pose->m_numPositions);
            for (i=0;i<softBodyData->m_pose->m_numPositions;i++)
            {
                    psb->m_pose.m_pos[i].deSerializeFloat(softBodyData->m_pose->m_positions[i]);
            }
            psb->m_pose.m_rot.deSerializeFloat(softBodyData->m_pose->m_rot);
            psb->m_pose.m_scl.deSerializeFloat(softBodyData->m_pose->m_scale);
            psb->m_pose.m_wgh.resize(softBodyData->m_pose->m_numWeigts);
            for (i=0;i<softBodyData->m_pose->m_numWeigts;i++)
            {
                    psb->m_pose.m_wgh[i] = softBodyData->m_pose->m_weights[i];
            }
            psb->m_pose.m_volume = softBodyData->m_pose->m_restVolume;
    }*/

    psb->m_cfg = orig.m_cfg;
/*

    psb->m_cfg.piterations=softBodyData->m_config.m_positionIterations;
    psb->m_cfg.diterations=softBodyData->m_config.m_driftIterations;
    psb->m_cfg.citerations=softBodyData->m_config.m_clusterIterations;
    psb->m_cfg.viterations=softBodyData->m_config.m_velocityIterations;

    //psb->setTotalMass(0.1);
    psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_)softBodyData->m_config.m_aeroModel;
    psb->m_cfg.kLF = softBodyData->m_config.m_lift;
    psb->m_cfg.kDG = softBodyData->m_config.m_drag;
    psb->m_cfg.kMT = softBodyData->m_config.m_poseMatch;
    psb->m_cfg.collisions = softBodyData->m_config.m_collisionFlags;
    psb->m_cfg.kDF = softBodyData->m_config.m_dynamicFriction;
    psb->m_cfg.kDP = softBodyData->m_config.m_damping;
    psb->m_cfg.kPR = softBodyData->m_config.m_pressure;
    psb->m_cfg.kVC = softBodyData->m_config.m_volume;
    psb->m_cfg.kAHR = softBodyData->m_config.m_anchorHardness;
    psb->m_cfg.kKHR = softBodyData->m_config.m_kineticContactHardness;
    psb->m_cfg.kSHR = softBodyData->m_config.m_softContactHardness;
    psb->m_cfg.kSRHR_CL = softBodyData->m_config.m_softRigidClusterHardness;
    psb->m_cfg.kSKHR_CL = softBodyData->m_config.m_softKineticClusterHardness;
    psb->m_cfg.kSSHR_CL = softBodyData->m_config.m_softSoftClusterHardness;
*/
//      pm->m_kLST                              =       1;

    //clusters
    psb->m_clusters.resize(orig.m_clusters.size());
    for (i=0;i<orig.m_clusters.size();i++) {
        const btSoftBody::Cluster *cluster = orig.m_clusters[i];
        btSoftBody::Cluster *newCluster = psb->m_clusters[i] =
            new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();

        *newCluster = *cluster;

        // not sure if this part is necessary
        newCluster->m_framerefs.resize(cluster->m_framerefs.size());
        for (j=0;j<cluster->m_framerefs.size();j++)
            newCluster->m_framerefs[j] = cluster->m_framerefs[j];

        newCluster->m_nodes.resize(cluster->m_nodes.size());
        for (j=0;j<cluster->m_nodes.size();j++) {
/*            NodeMap::const_iterator it = nodeMap.find(cluster->m_nodes[j]);
            if (it != nodeMap.end())
                newCluster->m_nodes[j] = it->second;*/
            newCluster->m_nodes[j] = (btSoftBody::Node *) f.forkOf(cluster->m_nodes[j]);
        }

        // not sure if this part is necessary
        newCluster->m_masses.resize(cluster->m_masses.size());
        for (j=0;j<cluster->m_masses.size();j++)
            newCluster->m_masses[j] = cluster->m_masses[j];
    }
/*

            m_clusterBodyMap.insert(softBodyData->m_clusters,psb);
            int j;
            psb->m_clusters.resize(softBodyData->m_numClusters);
            for (i=0;i<softBodyData->m_numClusters;i++)
            {
                    psb->m_clusters[i] = new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();
                    psb->m_clusters[i]->m_adamping = softBodyData->m_clusters[i].m_adamping;
                    psb->m_clusters[i]->m_av.deSerializeFloat(softBodyData->m_clusters[i].m_av);
                    psb->m_clusters[i]->m_clusterIndex = softBodyData->m_clusters[i].m_clusterIndex;
                    psb->m_clusters[i]->m_collide = (softBodyData->m_clusters[i].m_collide!=0);
                    psb->m_clusters[i]->m_com.deSerializeFloat(softBodyData->m_clusters[i].m_com);
                    psb->m_clusters[i]->m_containsAnchor = (softBodyData->m_clusters[i].m_containsAnchor!=0);
                    psb->m_clusters[i]->m_dimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[0]);
                    psb->m_clusters[i]->m_dimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[1]);

                    psb->m_clusters[i]->m_framerefs.resize(softBodyData->m_clusters[i].m_numFrameRefs);
                    for (j=0;j<softBodyData->m_clusters[i].m_numFrameRefs;j++)
                    {
                            psb->m_clusters[i]->m_framerefs[j].deSerializeFloat(softBodyData->m_clusters[i].m_framerefs[j]);
                    }
                    psb->m_clusters[i]->m_nodes.resize(softBodyData->m_clusters[i].m_numNodes);
                    for (j=0;j<softBodyData->m_clusters[i].m_numNodes;j++)
                    {
                            int nodeIndex = softBodyData->m_clusters[i].m_nodeIndices[j];
                            psb->m_clusters[i]->m_nodes[j] = &psb->m_nodes[nodeIndex];
                    }

                    psb->m_clusters[i]->m_masses.resize(softBodyData->m_clusters[i].m_numMasses);
                    for (j=0;j<softBodyData->m_clusters[i].m_numMasses;j++)
                    {
                            psb->m_clusters[i]->m_masses[j] = softBodyData->m_clusters[i].m_masses[j];
                    }
                    psb->m_clusters[i]->m_framexform.deSerializeFloat(softBodyData->m_clusters[i].m_framexform);
                    psb->m_clusters[i]->m_idmass = softBodyData->m_clusters[i].m_idmass;
                    psb->m_clusters[i]->m_imass = softBodyData->m_clusters[i].m_imass;
                    psb->m_clusters[i]->m_invwi.deSerializeFloat(softBodyData->m_clusters[i].m_invwi);
                    psb->m_clusters[i]->m_ldamping = softBodyData->m_clusters[i].m_ldamping;
                    psb->m_clusters[i]->m_locii.deSerializeFloat(softBodyData->m_clusters[i].m_locii);
                    psb->m_clusters[i]->m_lv.deSerializeFloat(softBodyData->m_clusters[i].m_lv);
                    psb->m_clusters[i]->m_matching = softBodyData->m_clusters[i].m_matching;
                    psb->m_clusters[i]->m_maxSelfCollisionImpulse = 0;//softBodyData->m_clusters[i].m_maxSelfCollisionImpulse;
                    psb->m_clusters[i]->m_ndamping = softBodyData->m_clusters[i].m_ndamping;
                    psb->m_clusters[i]->m_ndimpulses = softBodyData->m_clusters[i].m_ndimpulses;
                    psb->m_clusters[i]->m_nvimpulses = softBodyData->m_clusters[i].m_nvimpulses;
                    psb->m_clusters[i]->m_selfCollisionImpulseFactor = softBodyData->m_clusters[i].m_selfCollisionImpulseFactor;
                    psb->m_clusters[i]->m_vimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[0]);
                    psb->m_clusters[i]->m_vimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[1]);
                    
            }
            //psb->initializeClusters();
            //psb->updateClusters();

    }
    psb->updateConstants();
    }*/

    psb->updateConstants();

    return Ptr(new BulletSoftObject(psb));
}

void BulletSoftObject::postCopy(EnvironmentObject::Ptr copy, Fork &f) {
    const btSoftBody *orig = softBody.get();
    btSoftBody *psb = ((BulletSoftObject &) copy).softBody.get();
    // copy the anchors
    for (int i=0;i<orig->m_anchors.size();i++) {
        const btSoftBody::Anchor &anchor = orig->m_anchors[i];
        btRigidBody *body = btRigidBody::upcast((btCollisionObject *) f.forkOf(anchor.m_body));
        if (!body) continue;

        btSoftBody::Anchor newAnchor = anchor;
        newAnchor.m_body = body;

/*        NodeMap::const_iterator it2 = nodeMap.find(anchor.m_node);
        if (it2 != nodeMap.end())
            newAnchor->m_node = it2->second;*/

        newAnchor.m_node = (btSoftBody::Node *) f.forkOf(anchor.m_node);

        psb->m_anchors.push_back(newAnchor);

        // TODO: disableCollisionBetweenLinkedBodies
    }

    // copy the joints
    // TODO: THIS IS NOT TESTED

    for (int i=0;i<orig->m_joints.size();i++) {
        btSoftBody::Joint *joint = orig->m_joints[i];

        const btTransform &transA = psb->m_clusters[0]->m_framexform;

        btSoftBody::Body bdyB;
        const btSoftBody::Body &origBodyB = joint->m_bodies[1];
        if (origBodyB.m_soft)
            bdyB.m_soft = (btSoftBody::Cluster *) f.forkOf(origBodyB.m_soft);
        if (origBodyB.m_rigid)
            bdyB.m_rigid = (btRigidBody *) f.forkOf(origBodyB.m_rigid);
        if (origBodyB.m_collisionObject)
            bdyB.m_collisionObject = (btCollisionObject *) f.forkOf(origBodyB.m_collisionObject);

        /*
        btCollisionObject *colB = f.forkOf(joint->m_bodies[1]);
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
}
