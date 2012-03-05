#include "vision.h"
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

  initComm();
  TowelVision2 tv;
  tv.runOffline();

  // extern bool LIVE;
  // if (LIVE) visionSystem.runOnline();
  // else visionSystem.runOffline();


}
