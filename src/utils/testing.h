#include <iostream>
using namespace std;
#define TEST_FUNC(func)\
  cout << "function: " << #func << endl;		       \
  func();						       \
  cout << "success!" << endl
