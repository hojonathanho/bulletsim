#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
using namespace std;
using namespace Eigen;

template <class T>
void read_2d_array(vector< vector<T> >& arr, string fname) {

  ifstream infile(fname.c_str());

  string line;
  arr.clear();
  while (getline(infile,line)) {
    stringstream ss (stringstream::in | stringstream::out);
    ss << line;
    vector<T> v;
    v.clear();
    while (ss) {
      T f;
      ss >> f;
      v.push_back(f);
    }
    arr.push_back(v);
  }
}



int main() {
  stringstream s;
  Matrix3f m = Matrix3f::Identity();
  s << 
    s << m;
  cout << s.str();
  MatrixXf m1;
}
