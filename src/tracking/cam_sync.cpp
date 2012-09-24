#include "cam_sync.h"
#include "simulation/simplescene.h"
#include "utils/logging.h"

static const double PLAYBACK_MAX_OFFSET = 0.1;

CamSync::CamSync(Scene &s) : scene(s), mode(DISABLED) {
  scene.addPreDrawCallback(boost::bind(&CamSync::cb, this));
  first_cb = true;
  last_playback_pos = 0;
}

CamSync::~CamSync() {
  file.close();
}

void CamSync::enable(Mode mode_, const string &path) {
  mode = mode_;
  if (mode == PLAYBACK) {
    file.open(path.c_str(), fstream::in);
    while (file.good()) {
      double t; file >> t;
      osg::Matrixd m;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          file >> m(i, j);
        }
      }
      playback_matrices.push_back(std::make_pair(t, m));
    }
    file.close();

  } else if (mode == RECORD) {
    file.open(path.c_str(), fstream::out);
  }
}

void CamSync::cb() {
  if (mode == DISABLED) {
    return;
  }
  if (first_cb) {
    timer.restart();
    first_cb = false;
  }
  if (mode == RECORD) {
    if (!file.is_open()) {
      LOG_WARN("CamSync file not open, but scene is running");
      return;
    }
    file.precision(10);
    file << timer.elapsed() << '\t';
    osg::Matrixd m = scene.viewer.getCameraManipulator()->getMatrix();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        file << m(i, j) << ' ';
      }
    }
    file << std::endl;

  } else if (mode == PLAYBACK) {
    double curr_time = timer.elapsed();
    for (int k = last_playback_pos; k < playback_matrices.size(); ++k) {
      if (abs(playback_matrices[k].first - curr_time) < PLAYBACK_MAX_OFFSET) {
        scene.viewer.getCameraManipulator()->setByMatrix(playback_matrices[k].second);
        last_playback_pos = k;
        break;
      }
    }
  }
}
