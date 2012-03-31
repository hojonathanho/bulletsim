#pragma once
#include "utils_python.h"
#include <boost/foreach.hpp>
#include <utility>
#include <algorithm>
#include "utils/interpolation.h"
#include <btBulletDynamicsCommon.h>
#include <openrave/openrave.h>
#include "knots.h"


template <typename T>
void extend(std::vector<T>& head, const std::vector<T>& tail) {
  BOOST_FOREACH(T& x, head) head.push_back(x);
  return head;
}

template <typename A, typename B>
std::vector< pair<B,std::vector<A> > > splitby(std::vector<A> as, std::vector<B> bs) {

  std::vector< pair<B, std::vector<A> > > out(1);
  int iCur = 0;
  out[iCur].first = bs[0];

  for (int i=0; i < as.size(); i++) {
    if (bs[i] != out[iCur].first) {
      iCur++;
      out.push_back(pair<B, std::vector<A> >(bs[i], std::vector<A>()));
    }
    out[iCur].second.push_back(as[i]);
  }
  return out;
}


std::vector<btTransform> findBasePoses(const std::vector<btTransform>& leftPoses, const std::vector<double>& curJoints);

template <typename T>
std::vector<T> decimate(std::vector<T> in, int n) {
  std::vector<T> out;
  for (int i=0; i < in.size(); i+=n) out.push_back(in[i]);
  return out;
}

std::vector<btTransform> findBasePoses1(OpenRAVE::RobotBase::ManipulatorPtr manip, const std::vector<btTransform>& leftPoses);

std::vector<btTransform> findBasePoses2(const std::vector<RobotAndRopeState>& states, std::vector<double> curJoints);
