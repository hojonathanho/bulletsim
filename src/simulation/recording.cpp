#include "recording.h"
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
