#include "preprocessing.h"

int main(int argc, char* argv[]) {
  initComm();
  TowelGrabAndProc tgap(10); 
  tgap.run();
}
