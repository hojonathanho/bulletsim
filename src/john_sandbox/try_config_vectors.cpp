#include "utils/config.h"
#include "utils/utils_vector.h"
#include <vector>
#include <iostream>
using namespace std;

struct LocalConfig : Config {
  static vector<int> ints;
  LocalConfig() {
    params.push_back(new ParameterVec< int >("ints", &ints, "scale factor applied to distances that are assumed to be in meters"));
  }
};

vector<int> LocalConfig::ints;

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);
  cout << LocalConfig::ints << endl;
}
