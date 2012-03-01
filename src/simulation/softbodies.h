#ifndef _SOFTBODIES_H_
#define _SOFTBODIES_H_

#include "environment.h"
#include "basicobjects.h"

class BulletSoftObject : public EnvironmentObject {
protected:
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::MatrixTransform> transform;

    osg::ref_ptr<osg::Geometry> trigeom;
    osg::ref_ptr<osg::Vec3Array> trivertices, trinormals;

    osg::ref_ptr<osg::Geometry> quadgeom;
    osg::ref_ptr<osg::Vec3Array> quadvertices, quadnormals;

public:
    typedef boost::shared_ptr<BulletSoftObject> Ptr;

    boost::shared_ptr<btSoftBody> softBody;

    BulletSoftObject(boost::shared_ptr<btSoftBody> softBody_) : softBody(softBody_) { }
    BulletSoftObject(btSoftBody *softBody_) : softBody(softBody_) { }
    virtual ~BulletSoftObject() { }

    void setColor(float,float,float,float);

    EnvironmentObject::Ptr copy(Fork &f) const;
    void postCopy(EnvironmentObject::Ptr copy, Fork &f) const;

    // called by Environment
    void init();
    void preDraw();
    void destroy();

};

// for saving and loading softbodies
void saveSoftBody(const btSoftBody* orig, const char* fileName);
btSoftBody* loadSoftBody(btSoftBodyWorldInfo& worldInfo, const char* fileName);

#endif // _SOFTBODIES_H_
