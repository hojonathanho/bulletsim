#include "recording.h"

int RecordingConfig::record = 0;

ScreenRecorder::ScreenRecorder(osgViewer::Viewer& viewer) : frameCount(0), m_viewer(viewer) {
  m_captureHandler = new osgViewer::ScreenCaptureHandler(new osgViewer::ScreenCaptureHandler::WriteToFile("screenshots/img", "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER));
  m_viewer.addEventHandler(m_captureHandler);
}

void ScreenRecorder::snapshot() {
  m_captureHandler->captureNextFrame(m_viewer);
}
