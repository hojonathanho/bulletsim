#include "softbodies.h"
#include "util.h"

#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <osgbCollision/Utils.h>

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
    transform = new osg::MatrixTransform;
    transform->addChild(geode);
    getEnvironment()->osg->root->addChild(transform);
}

void BulletSoftObject::preDraw() {
#if 0
    btSoftBodyHelpers::DrawFrame(softBody.get(), getEnvironment()->osg->softBodyDrawer.get());
    btSoftBodyHelpers::Draw(softBody.get(), getEnvironment()->osg->softBodyDrawer.get(),
                            getEnvironment()->bullet->dynamicsWorld->getDrawFlags());
#endif
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
