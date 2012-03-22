#ifndef __FL_NODEPICKER_H__
#define __FL_NODEPICKER_H__

#include "simulation/simplescene.h"
#include <osg/Camera>

class SoftBodyNodePicker {
    Scene &scene;
    osg::Camera *cam;
    btSoftBody *psb;

    typedef boost::function<void(btSoftBody::Node *)> PickCallback;
    PickCallback pickCb;

    bool pickCallback(const osgGA::GUIEventAdapter &);

public:
    typedef boost::shared_ptr<SoftBodyNodePicker> Ptr;
    SoftBodyNodePicker(Scene &, osg::Camera *, btSoftBody *);
    void setPickCallback(PickCallback cb) { pickCb = cb; }
};

#endif // __FL_NODEPICKER_H__
