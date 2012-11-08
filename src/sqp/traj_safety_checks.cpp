#include "traj_safety_checks.h"
#include <boost/foreach.hpp>
#include "utils_sqp.h"
#include "utils/logging.h"
#include "config_sqp.h"
using namespace std;
/**
 * Returns true if d is within one of the given intervals
 * @param d the floating point number to test
 * @param intervals a list of intervals to test
 * @return true if d is within one of the given intervals
 */
bool inSomeInterval(double d, const std::vector<paird>& intervals) {
  bool included = false;
  BOOST_FOREACH(const paird& interval, intervals) {
    if (d >= interval.first && d <= interval.second) included=true;
  }
  return included;
}
/**
 * Removes numbers from in which fall within the specified intervals
 * @param in input vector of doubles to be filtered
 * @param exclude vector of intervals to be excluded from the output
 * @return the filtered vector of doubles
 */
vector<double> filterOutIntervals(const std::vector<double>& in, const std::vector<paird>& exclude) {
  vector<double> out;
  BOOST_FOREACH(const double d, in) if (!inSomeInterval(d, exclude)) out.push_back(d);
  return out;
}
/**
 * @param lc a LinkCollision corresponding to a link in collision
 * @param time the time the collision occurs
 * @param allowedCollisions mapping of link indices to allowed time intervals
 *  for collision
 * @return true if the given LinkCollision is allowed
 */
bool linkInAllowedCollision(const LinkCollision lc, double time,
    const map<int, vector<paird> >& allowedCollisions){
  map<int, vector<paird> >::const_iterator intervals = allowedCollisions.find(lc.linkInd);
  if(intervals != allowedCollisions.end()){
    if(inSomeInterval(time, intervals->second)){
      return true;
    }
  }
  return false;
}
/**
 * Returns whether the given trajectory is safe according to the given minimum
 * distance from collisions, optionally ignoring collisions in specified
 * time intervals of the trajectory
 *
 * @param cci Collision information for the trajectory
 * @param distSafe minimum distance from potential collisions
 * @param times a vector of time values for the trajectory
 * @param allowedCollisions time intervals in which collisions are allowed
 */
   bool isDiscreteSafe(const TrajCartCollInfo& cci, double distSafe, const Eigen::VectorXd& times,
		const map<int, vector<paird> >& allowedCollisions) {
//  countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDist, int& nNear, int& nUnsafe, int& nColl)
  assert(cci.size() == times.size());
  vector<double> discreteUnsafeTimes;
  for (int i=0; i < cci.size(); ++i) {
	  BOOST_FOREACH(const LinkCollision& lc, cci[i]) {
	    bool linkInAllowedCollisions = linkInAllowedCollision(lc, times(i), allowedCollisions);
		  if(!linkInAllowedCollisions && lc.dist < distSafe){
		    discreteUnsafeTimes.push_back(times(i));
		  }
	  }
  }
  if (discreteUnsafeTimes.size() >0) {
    LOG_INFO("steps with a discrete collision: " << discreteUnsafeTimes);
    stringstream ss;
//    BOOST_FOREACH(paird p, allowedCollisionIntervals) ss << "(" << p.first << "," << p.second << ") ";
//    LOG_INFO("allowed collision intervals:" << ss.str());
  }
  return discreteUnsafeTimes.size()==0;
}

/**
 * Given a trajectory with continuous collisions, this function decides when to
 * add points to the trajectory to form a safe trajectory
 * @param cci collision information for a trajectory
 * @param times time information for a trajectory
 * @return a list of new times to be sampled in a new trajectory
 */
vector<double> getSubdivisionTimes(const TrajCartCollInfo& cci, const Eigen::VectorXd& times,
    const AllowedCollisions& allowedCollisions) {
  int nContColl=0;
  vector<double> insertTimes;
  for (int i = 0; i < cci.size() - 1; ++i) {
    nContColl += cci[i].size();
    if (cci[i].size() > 0){
      BOOST_FOREACH(const LinkCollision& lc, cci[i]) {
        // Check if this link is allowed to collide
        // (the distance is meaningless for continuous collisions)
        bool linkInAllowedCollisions = linkInAllowedCollision(lc, times(i), allowedCollisions);
        if(!linkInAllowedCollisions){
          insertTimes.push_back((times(i) + times(i+1)) / 2);
          break;
        }
      }
    }
  }
  return insertTimes;
}
