#pragma once
#include "utils/config.h"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread.hpp>

enum RecordingMode {
  DONT_RECORD=0,
  EVERY_ITERATION=1,
  FINAL_ITERATION=2,
  RECORD_RENDER_ONLY=3,
  RECORD_RENDER_AND_TOPIC=4
};

struct RecordingConfig : Config {
  static int record;
  static std::string dir;
  static std::string video_file;
  static float frame_rate;
  static float speed_up;

  RecordingConfig() : Config() {
    params.push_back(new Parameter<int>("record", &record, "0: no recording. 1: record every iterations. 2: final iteration"));
    params.push_back(new Parameter<std::string>("dir", &dir, "directory to save files in"));
    params.push_back(new Parameter<std::string>("video_file", &video_file, "file name of the video file saved (this doesn't include the path, i.e. dir)"));
    params.push_back(new Parameter<float>("frame_rate", &frame_rate, "frame rate of the output video"));
    params.push_back(new Parameter<float>("speed_up", &speed_up, "the video is speed up by this factor"));
  }
};

struct MyWriteToFile : public osgViewer::ScreenCaptureHandler::CaptureOperation {
  int n;
  std::string m_outPath;
  MyWriteToFile(std::string outPath) : m_outPath(outPath), n(0) {}
  void operator () (const osg::Image& image, const unsigned int context_id);
};

struct TempWriteToFile : public osgViewer::ScreenCaptureHandler::CaptureOperation {
	cv::Mat m_image;
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

class ScreenCapture {
public:
  osg::ref_ptr<osgViewer::ScreenCaptureHandler> m_captureHandler;
  osgViewer::Viewer& m_viewer;
  TempWriteToFile* m_capture_op;
  ScreenCapture(osgViewer::Viewer& viewer);
  cv::Mat snapshot();
};

class ScreenThreadRecorder {
	cv::VideoWriter m_video_writer;
	ScreenCapture m_capture;
	boost::thread m_thread;
	bool m_exit_loop;
	void recordLoop();

public:
	ScreenThreadRecorder(osgViewer::Viewer& viewer);
	~ScreenThreadRecorder();
};


struct ConsecutiveImageWriter {
  int n;
  std::string m_outPath;
  ConsecutiveImageWriter(std::string outPath);
  void write(cv::Mat mat);
};
