#ifndef _SOFTBODIES_H_
#define _SOFTBODIES_H_

#include "environment.h"
#include "basicobjects.h"

class BulletSoftObject : public EnvironmentObject {
protected:
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec3Array> vertices, normals;
    osg::ref_ptr<osg::MatrixTransform> transform;

public:
    typedef boost::shared_ptr<BulletSoftObject> Ptr;

    boost::shared_ptr<btSoftBody> softBody;

    BulletSoftObject(boost::shared_ptr<btSoftBody> softBody_) : softBody(softBody_) { }
    BulletSoftObject(btSoftBody *softBody_) : softBody(softBody_) { }
    virtual ~BulletSoftObject() { }

    EnvironmentObject::Ptr copy(Fork &f) const;
    void postCopy(EnvironmentObject::Ptr copy, Fork &f) const;

    // called by Environment
    void init();
    void preDraw();
    void destroy();
};

#endif // _SOFTBODIES_H_
