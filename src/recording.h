#pragma once
#include "config.h"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>


struct RecordingConfig : Config {
  static int record;
  RecordingConfig() : Config() {
    params.push_back(new Parameter<int>("record", &record, "record every n frames (default 0 means record nothing)"));
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
