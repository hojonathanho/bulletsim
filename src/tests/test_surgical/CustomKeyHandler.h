#ifndef _CUSTOM_KEY_HANDLER_
#define _CUSTOM_KEY_HANDLER_



#include "CustomScene.h"

#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/openravesupport.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#define PI 3.14159265

class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

#endif
