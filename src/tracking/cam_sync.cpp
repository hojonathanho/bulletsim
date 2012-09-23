#include "cam_sync.h"
#include "simulation/simplescene.h"
#include "utils/logging.h"

CamSync::CamSync(Scene &s) : scene(s) {
  scene.addPreDrawCallback(boost::bind(&CamSync::cb, this));
}

CamSync::~CamSync() {
  file.close();
}

void CamSync::enable(Mode mode_, const string &path) {
  mode = mode_;
  if (mode == PLAYBACK) {
    file.open(path.c_str(), fstream::in);
  } else {
    file.open(path.c_str(), fstream::out);
  }
}

void CamSync::cb() {
  if (mode == DISABLED) {
    return;
  }
  if (!file.is_open()) {
    LOG_WARN("CamSync file not open, but scene is running");
    return;
  }
  switch (mode) {
  case RECORD: {
    osg::Matrixd m = scene.viewer.getCameraManipulator()->getMatrix();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        file << m(i, j) << ' ';
      }
    }
    file << '\n';
    break;
  }
  case PLAYBACK: {
    osg::Matrixd m;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        file >> m(i, j);
      }
    }
    scene.viewer.getCameraManipulator()->setByMatrix(m);
    break;
  }
  }
}
