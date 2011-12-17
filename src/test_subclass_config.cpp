#include "userconfig.h"
#include <iostream>
#include <string>
using namespace std;

struct MyConfigData : public ConfigData {
  struct  {
    string myopt;
  } mygroup;

  MyConfigData()  {
    opts.add_options()
      OPT(mygroup.myopt, string, "hi", "what to say");
  }
};

static MyConfigData* mcfg;
MyConfigData* getMyConfigData() {
  if (mcfg==NULL) mcfg = new MyConfigData;
  return mcfg;
}
#define CFG2 getMyConfigData()

int main(int argc, char* argv[]) {
  MyConfigData* mcd1 = getMyConfigData();
  MyConfigData* mcd2 = getMyConfigData();
 
   cout << "mcd1 "<< mcd1 << endl;
   cout << "mcd2 "<< mcd2 << endl;

  setConfigData(mcd1);
  ConfigData* cd1 = getConfigData();
  ConfigData* cd2 = getConfigData();
  cout << "cd1 "<< cd1 << endl;
  cout << "cd2 "<< cd2 << endl;


  // CFG2.read(argc,argv);
  // cout << CFG2.mygroup.myopt << endl;
   CFG2->scene.enableIK = true;
   cout << CFG2->scene.enableIK << endl;
   cout << CFG->scene.enableIK << endl;
   cout << CFG << endl;
  //cout << & CFG2 << endl;
}

