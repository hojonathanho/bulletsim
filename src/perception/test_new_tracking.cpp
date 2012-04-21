#include "tracking.h"
#include "perception/config_perception.h"
#include "simulation/config_bullet.h"

int main(int argc, char* argv[]) {

  Parser parser;
  GeneralConfig::scale = 10;
  parser.addGroup(TrackingConfig());
  parser.addGroup(RecordingConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(GeneralConfig());

  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  Tracker* TrackerSystem;
  if (TrackingConfig::objType=="towel")
    TrackerSystem = new TowelTracker();
  else if (TrackingConfig::objType=="rope")
    TrackerSystem = new RopeTracker();
  else throw std::runtime_error("invalid objType");
  extern bool LIVE;
  if (LIVE) TrackerSystem->runOnline();
  else TrackerSystem->runOffline();
}
