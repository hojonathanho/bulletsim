#include "simulation/recording.h"
#include "simulation/simplescene.h"
#include <opencv2/core/core.hpp>
int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  cv::Mat_<cv::Scalar> mat(640,480, 1);

  Scene s;
  s.startViewer();
  ScreenRecorder sr(s.viewer);
  ConsecutiveImageWriter cir("/tmp/images");
  for (int i=0; true; ++i) {
    if ((i%100) == 0){
      cir.write(mat);
      sr.snapshot();
    }
    s.step(.01);
  }

}
