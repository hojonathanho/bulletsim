#include "userconfig.h"
#include <iostream>
#include <string>
using namespace std;

struct MyConfigData : public ConfigData {
  struct  {
    string myopt;
  } mygroup;
  static void use() {
    instancePtr = new MyConfigData();
  }
  MyConfigData() : ConfigData() {
    opts.add_options()
      OPT(mygroup.myopt, string, "hi", "what to say");
  }
};
#define CFG2 static_cast<MyConfigData*>(MyConfigData::Inst())

int main(int argc, char* argv[]) {
  MyConfigData::use();
  CFG2->read(argc,argv);

  cout << CFG2->mygroup.myopt << endl;
}
