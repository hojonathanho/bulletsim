#include "sensor_msgs/Image.h"
#include "hydra_msgs/Calib.h"
#include <iostream>
#include <ros/ros.h>
#include "robots/hydra_pr2_teleop.h"
#include "robots/pr2.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
using namespace std;


void callback(const hydra_msgs::Calib& msg) {
  cout << msg << endl;
}

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);
  Scene scene;
  PR2Manager pr2m(scene);
  HydraPR2Teleop teleop(pr2m.pr2, &scene);
  scene.startViewer();
  cout << "startloop" << endl;
  scene.startLoop();

  return 0;
}
