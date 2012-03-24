#pragma once
#include "utils_python.h"
#include <boost/foreach.hpp>
#include <utility>
#include <algorithm>
#include "utils/interpolation.h"
#include <btBulletDynamicsCommon.h>
#include <openrave/openrave.h>

using namespace std;
using namespace Eigen;

template <typename T>
void extend(vector<T>& head, const vector<T>& tail) {
  BOOST_FOREACH(T& x, head) head.push_back(x);
  return head;
}

template <typename A, typename B>
vector< pair<B,vector<A> > > splitby(vector<A> as, vector<B> bs) {

  vector< pair<B, vector<A> > > out(1);
  int iCur = 0;
  out[iCur].first = bs[0];

  for (int i=0; i < as.size(); i++) {
    if (bs[i] != out[iCur].first) {
      iCur++;
      out.push_back(pair<B, vector<A> >(bs[i], vector<A>()));
    }
    out[iCur].second.push_back(as[i]);
  }
  return out;
}


vector<btTransform> findBasePoses(const vector<btTransform>& leftPoses, const vector<double>& curJoints);

template <typename T>
vector<T> decimate(vector<T> in, int n) {
  vector<T> out;
  for (int i=0; i < in.size(); i+=n) out.push_back(in[i]);
  return out;
}

vector<btTransform> findBasePoses1(OpenRAVE::RobotBase::ManipulatorPtr manip, const vector<btTransform>& leftPoses);