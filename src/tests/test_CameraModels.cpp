#include "CameraModels.h"
#include "simplescene.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.cpp>

}

void main(int argc, char* argv[]) {

  Scene scene;

  Kinect kinect;

  cv::Mat rgb = kinect.getRGB(scene.env);
  cv::Mat depth = kinect.getDepth(scene.env);

  // assertions about depth of some points
  // show images in highgui

}
