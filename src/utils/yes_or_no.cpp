#include <iostream>
#include <cstdlib>
using namespace std;
bool yesOrNo(char message[]) {
  cout << message << endl;
  while (true) {
    char yn;
    cin >> yn;
    if (yn == 'y') return true;
    else if (yn == 'n') return false;
  }
}
