#include <json/json.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;

int main() {
  std::stringstream buffer;
  std::ifstream infile("/home/joschu/Junk/test.json");
  buffer << infile.rdbuf();
  Json::Reader reader;
  Json::Value root;
  bool parsedSuccess = reader.parse(buffer.str(), root, false);
  cout << "success: " << parsedSuccess << endl;
  Json::Value back = root["back"];
  float x = back[0u].asDouble();
  float y = back[1u].asDouble();
  float z = back[2u].asDouble();

  cout << x << " " << y << " " << z << " " << endl;
}
