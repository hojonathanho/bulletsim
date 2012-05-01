#ifndef __FL_FACEPICKER_H__
#define __FL_FACEPICKER_H__

#include "simulation/simplescene.h"
#include <osg/Camera>

class SoftBodyFacePicker {
    Scene &scene;
    osg::Camera *cam;
    btSoftBody *psb;

    typedef boost::function<void(int)> PickCallback;
    PickCallback pickCb;

    bool pickCallback(const osgGA::GUIEventAdapter &);

public:
    typedef boost::shared_ptr<SoftBodyFacePicker> Ptr;
    SoftBodyFacePicker(Scene &, osg::Camera *, btSoftBody *);
    void setPickCallback(PickCallback cb) { pickCb = cb; }
};

#endif // __FL_FACEPICKER_H__
