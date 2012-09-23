#include "recording.h"
#include "config_viewer.h"
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "utils/my_assert.h"
#include <opencv2/highgui/highgui.hpp>
#include "utils/logging.cpp"
#include <osgDB/WriteFile>
namespace fs = boost::filesystem;
using namespace std;

int RecordingConfig::record = 0;
string RecordingConfig::dir = "/tmp/snapshots";
string RecordingConfig::video_file = "video";
float RecordingConfig::frame_rate = 30;
float RecordingConfig::speed_up = 1;

bool yesOrNo(char message[]) {
  while (true) {
    cout << message << " (y/n): ";
    char yn;
    cin >> yn;
    if (yn == 'y') return true;
    else if (yn == 'n') return false;
  }
}

void askToResetDir(fs::path p) {
  if (fs::exists(p)) {
    char buffer[150];
    sprintf(buffer, "%s already exists. Delete it?", p.string().c_str());
    bool consent = yesOrNo(buffer);
    if (consent) {
      cout << "deleting " << p.string() << endl;
      fs::remove_all(p);
    }
    else throw IOError();
  }
  ENSURE(fs::create_directory(p));
}

void MyWriteToFile::operator () (const osg::Image& image, const unsigned int context_id) {
  char fname[30];
  sprintf(fname,"image%.4i.jpg", n);
  fs::path savePath = fs::path(m_outPath) / fname;
  osgDB::writeImageFile(image, savePath.string());
  ++n;
  LOG_INFO_FMT("writing %s", savePath.string().c_str());
}


ScreenRecorder::ScreenRecorder(osgViewer::Viewer& viewer) : frameCount(0), m_viewer(viewer) {
  askToResetDir(RecordingConfig::dir);
  m_captureHandler = new osgViewer::ScreenCaptureHandler(new MyWriteToFile(RecordingConfig::dir));
//    m_viewer.addEventHandler(m_captureHandler);
}

void ScreenRecorder::snapshot() {
  LOG_INFO("capturing next frame...");
  m_captureHandler->captureNextFrame(m_viewer);
  m_captureHandler->setFramesToCapture(1);
}


void TempWriteToFile::operator () (const osg::Image& image, const unsigned int context_id) {
  osgDB::writeImageFile(image, "/tmp/image_temp.jpg");
  m_image = cv::imread("/tmp/image_temp.jpg");
}

ScreenCapture::ScreenCapture(osgViewer::Viewer& viewer) : m_viewer(viewer) {
	m_capture_op = new TempWriteToFile();
  m_captureHandler = new osgViewer::ScreenCaptureHandler(m_capture_op);
}

cv::Mat ScreenCapture::snapshot() {
  m_captureHandler->captureNextFrame(m_viewer);
  m_captureHandler->setFramesToCapture(1);
  return m_capture_op->m_image;
}


ScreenThreadRecorder::ScreenThreadRecorder(osgViewer::Viewer& viewer) :
		m_capture(ScreenCapture(viewer)),
		m_exit_loop(false)
{
	string full_filename = RecordingConfig::dir + "/" +  RecordingConfig::video_file + ".avi";
	m_video_writer.open(full_filename,  CV_FOURCC('P','I','M','1'), RecordingConfig::frame_rate, cv::Size(ViewerConfig::windowWidth, ViewerConfig::windowHeight));

	if (!m_video_writer.isOpened()) {
		runtime_error("Video destination file " + full_filename + " doesn't exits.");
	}

	m_thread = boost::thread(boost::bind(&ScreenThreadRecorder::recordLoop, this));
}

ScreenThreadRecorder::ScreenThreadRecorder(osgViewer::Viewer& viewer, std::string full_filename) :
		m_capture(ScreenCapture(viewer)),
		m_exit_loop(false)
{
	m_video_writer.open(full_filename,  CV_FOURCC('P','I','M','1'), RecordingConfig::frame_rate, cv::Size(ViewerConfig::windowWidth, ViewerConfig::windowHeight));

	if (!m_video_writer.isOpened()) {
		runtime_error("Video destination file " + full_filename + " doesn't exits.");
	}

	m_thread = boost::thread(boost::bind(&ScreenThreadRecorder::recordLoop, this));
}

ScreenThreadRecorder::~ScreenThreadRecorder() {
	m_exit_loop = true;
	m_thread.join();
}

void ScreenThreadRecorder::recordLoop() {
	int cycle_us = 1000000.0 * RecordingConfig::speed_up/RecordingConfig::frame_rate;
	boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	while(!m_exit_loop) {
		m_video_writer << m_capture.snapshot();

		int time_diff = (boost::posix_time::microsec_clock::local_time() - time).total_microseconds();
		if (time_diff < cycle_us) usleep(cycle_us-time_diff);
		time = boost::posix_time::microsec_clock::local_time();
	}
}


ConsecutiveImageWriter::ConsecutiveImageWriter(std::string outPath) :
  m_outPath(outPath),
  n(0)
{
  askToResetDir(fs::path(outPath));
}

void ConsecutiveImageWriter::write(cv::Mat mat) {
  char fname[30];
  sprintf(fname,"image%.4i.jpg", n);
  fs::path savePath = fs::path(m_outPath) / fname;
  cv::imwrite(savePath.string(), mat);
  LOG_INFO_FMT("writing %s", savePath.string().c_str());
  ++n;
}
