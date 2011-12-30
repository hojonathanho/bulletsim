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

static MyConfigData* myConfigData = new MyConfigData;
#define CFG2 myConfigData

int main(int argc, char* argv[]) {

    string a = "ewrewffef";
    //CFG2->setDefault("mygroup.myopt",a);

  CFG2->read(argc,argv);
  CFG2->scene.enableIK = true;
  setConfigData(myConfigData);
 

   cout << CFG2->scene.enableIK << endl;
   cout << CFG->scene.enableIK << endl;

   cout <<"size: " << sizeof(CFG2->mygroup.myopt) << endl;
   cout << CFG2->mygroup.myopt << endl;

}

