#ifndef _CONFIG_VIEWER_H_
#define _CONFIG_VIEWER_H_

#include "utils/config.h"
#include <btBulletDynamicsCommon.h>

struct ViewerConfig : Config {
    static btVector3 cameraHomePosition;
    static btVector3 cameraHomeCenter;
    static btVector3 cameraHomeUp;
    static int windowWidth;
    static int windowHeight;
    static float zoomFactor;

  ViewerConfig() : Config() {
    params.push_back(new Parameter<int>("windowWidth", &windowWidth, "viewer window width"));
    params.push_back(new Parameter<int>("windowHeight", &windowHeight, "viewer window height"));
    params.push_back(new Parameter<float>("zoomFactor", &zoomFactor, "scroll wheel zoom factor"));
  }
};

#endif // _CONFIG_VIEWER_H_
