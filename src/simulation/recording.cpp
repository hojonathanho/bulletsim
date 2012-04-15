#include "recording.h"
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "utils/my_assert.h"
namespace fs = boost::filesystem;
using namespace std;

int RecordingConfig::record = 0;

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


ScreenRecorder::ScreenRecorder(osgViewer::Viewer& viewer) : frameCount(0), m_viewer(viewer) {
  askToResetDir("screenshots");
  m_captureHandler = new osgViewer::ScreenCaptureHandler(new osgViewer::ScreenCaptureHandler::WriteToFile("screenshots/img", "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER));
  //  m_viewer.addEventHandler(m_captureHandler);
}

void ScreenRecorder::snapshot() {
  cout << "taking snapshot!" << endl;
  m_captureHandler->captureNextFrame(m_viewer);
}
