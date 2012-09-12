#pragma once
#include "utils/config.h"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <opencv2/core/core.hpp>

enum RecordingMode {
  DONT_RECORD=0,
  EVERY_ITERATION=1,
  FINAL_ITERATION=2
};

struct RecordingConfig : Config {
  static int record;
  static std::string dir;
  RecordingConfig() : Config() {
    params.push_back(new Parameter<int>("record", &record, "0: no recording. 1: record every iterations. 2: final iteration"));
    params.push_back(new Parameter<std::string>("dir", &dir, "directory to save files in"));
  }
};

struct MyWriteToFile : public osgViewer::ScreenCaptureHandler::CaptureOperation {
  int n;
  std::string m_outPath;
  MyWriteToFile(std::string outPath) : m_outPath(outPath), n(0) {}
  void operator () (const osg::Image& image, const unsigned int context_id);
};



class ScreenRecorder {
public:
  osg::ref_ptr<osgViewer::ScreenCaptureHandler> m_captureHandler;
  osgViewer::Viewer& m_viewer;
  int frameCount;
  ScreenRecorder(osgViewer::Viewer& viewer);
  void snapshot(); //call this BEFORE scene's step() method
};


struct ConsecutiveImageWriter {
  int n;
  std::string m_outPath;
  ConsecutiveImageWriter(std::string outPath);
  void write(cv::Mat mat);
};
