#include "simplescene.h"
#include "unistd.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

int main() {

  int i;
  ifstream indsfile("/home/joschu/Data/knot_kinect1/inds.txt");

  vector<int>inds;
  cout << "indices: ";
  while (indsfile) {
    indsfile >>i;
    inds.push_back(i);
    cout << i << " ";
  }
  cout << endl;


  vector< vector<float> > joints;
  float f;
  ifstream jointsfile("/home/joschu/Data/knot_kinect1/vals.txt");

  string line;
  while (getline(jointsfile,line)) {
    stringstream ss (stringstream::in | stringstream::out);
    ss << line;
    vector<float> v;
    v.clear();
    for (int i=0; i<inds.size(); i++) {
      float f;
      ss >> f;
      v.push_back(f);
    }
    joints.push_back(v);

  }

  cout<< "joints: " << endl;
  for (int i=0; i<joints.size(); i++)
    {
      vector<float> v = joints[i];
      for (int j=0; j<v.size(); j++) cout << v[j] << " ";
      cout << endl;
    }

  Scene s = Scene();
  for (int i = 0; i < joints.size(); i++) {
    vector<double> joint(joints[i].begin(),joints[i].end());
    s.pr2->setDOFValues(inds,joint);
    s.step(.01, 0, 0.01);
    usleep(1000*100);

  }

}
