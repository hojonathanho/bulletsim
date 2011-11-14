#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

template <class T>
void read_2d_array(vector< vector<T> >& arr, string fname) {

  ifstream infile(fname.c_str());
  string line;
  
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

template <class T>
void read_1d_array(vector<T>& arr, string fname) {
  ifstream infile(fname.c_str());
  T i;
  while (infile) {
    infile >>i;
    arr.push_back(i);
  }
}

/*
int main() {
  vector<int> inds;
  read_int_array(inds, "/home/joschu/Data/knot_kinect1/inds.txt");
  cout << "number of inds: " << inds.size() << endl;
  vector< vector<float> > joints;
  read_2d_array(joints,"/home/joschu/Data/knot_kinect1/vals.txt");
  cout << "number of joint positions: " << joints.size() << endl;
}
*/
