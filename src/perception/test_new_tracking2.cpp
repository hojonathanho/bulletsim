#include "perception/rope_tracking.h"
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
  SingleHypRopeTracker TrackerSystem;

  extern bool LIVE;
  if (false) TrackerSystem.runOnline();
  else TrackerSystem.runOffline();


}
