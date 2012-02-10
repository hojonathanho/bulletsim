#include <iostream>
#include <cstdlib>
using namespace std;
bool yesOrNo(char message[]) {
  cout << message << endl;
  while (true) {
    char yn;
    cin >> yn;
    if (yn == 'y') break;
    else if (yn == 'n') exit(0);
  }
}
