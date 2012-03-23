#include "facepicker.h"

SoftBodyFacePicker::SoftBodyFacePicker(Scene &scene_, osg::Camera *cam_, btSoftBody *psb_) : scene(scene_), cam(cam_), psb(psb_) {
    scene.addCallback(osgGA::GUIEventAdapter::RELEASE,
            boost::bind(&SoftBodyFacePicker::pickCallback, this, _1));
}

static const btScalar RAYCAST_DIST = 10000;
bool SoftBodyFacePicker::pickCallback(const osgGA::GUIEventAdapter &ea) {
    if (ea.getEventType() != osgGA::GUIEventAdapter::RELEASE
            || ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON
            || !(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL))
        return false;

    if (!pickCb) return false;

    osg::Matrix view = cam->getViewMatrix();
    osg::Vec3 look, at, up;
    view.getLookAt(look, at, up);
    osg::Matrix proj = cam->getProjectionMatrix();
    double fovy, aspect, zNear, zFar;
    proj.getPerspective(fovy, aspect, zNear, zFar);
    view.invert(view);
    proj.invert(proj);
    osg::Vec4 clip(ea.getXnormalized() * zFar, ea.getYnormalized() * zFar, zFar, zFar);
    osg::Vec4 wc = clip * proj * view;
    btVector3 from = util::toBtVector(look);
    btVector3 dir = (btVector3(wc[0], wc[1], wc[2]) - from).normalized();

    btSoftBody::sRayCast res = { 0 };
    bool b = psb->rayTest(from, from + RAYCAST_DIST*dir, res);
    cout << "raytest result: "
        << b <<  ' ' << res.feature << ' ' << res.index << endl;
    if (res.feature == btSoftBody::eFeature::Face)
        pickCb(res.index);

    return false;
}
