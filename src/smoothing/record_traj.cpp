#include "simulation/simplescene.h"
#include "smoothing/smoothing.h"
#incude <iostream>
using namespace std;

bool recording = false;
void toggleRecording() {
  recording = !recording;
  cout << "recording " << (recording ? "ON" : "OFF") << endl;
}
bool wantsExit = false;
void setWantsExit() {
  wantsExit = true;
}

struct LocalConfig : Config {
  static string outfile;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("outfile", &outfile, "output file")); 

  }
};
string LocalConfig::outfile = "";


int main(int argc, char* argv[]) {

  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);
  if (LocalConfig::outfile=="") throw std::runtime_error("must specify output file");

  Scene scene;
  PR2Manager pr2m(scene);
  // todo: make table
  scene.startViewer();
  scene.setSyncTime(true);
  
  scene.addVoidKeyCallback('r',&toggleRecording);
  scene.addVoidKeyCallback('q',&setWantsExit);

  ofstream jointsFile( (SMOOTHING_DATA  / LocalConfig::outfile ).string().c_str(), std::ios_base::out | std::ios_base::trunc);

  while (!wantsExit) {
    if (recording) {
      vector<double> values;
      pr2m.pr2->robot->GetDOFValues(values);
      jointsFile << values << endl;
    }
    scene.step(DT);
  }

  jointsFile.close();

  
}