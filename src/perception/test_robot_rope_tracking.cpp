#include "perception/multihyp_vision.h"
#include "perception/config_perception.h"
#include "simulation/config_bullet.h"
#include "simulation/recording.h"

int main(int argc, char* argv[]) {

  Eigen::internal::setNbThreads(2);

  Parser parser;
  GeneralConfig::scale = 10;
  parser.addGroup(TrackingConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(GeneralConfig());

  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  initComm();
  SingleHypRobotAndRopeVision visionSystem;

  extern bool LIVE;
  visionSystem.runOffline();


}
