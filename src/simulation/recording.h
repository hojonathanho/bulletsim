#pragma once
#include "utils/config.h"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>


enum RecordingMode {
  DONT_RECORD=0,
  EVERY_ITERATION=1,
  FINAL_ITERATION=2
};

struct RecordingConfig : Config {
  static int record;
  RecordingConfig() : Config() {
    params.push_back(new Parameter<int>("record", &record, "0: no recording. 1: record every iterations. 2: final iteration"));
  }
};

class ScreenRecorder {
public:
  osgViewer::ScreenCaptureHandler* m_captureHandler;
  osgViewer::Viewer& m_viewer;
  int frameCount;
  ScreenRecorder(osgViewer::Viewer& viewer);
  void snapshot(); //call this BEFORE scene's step() method
};
