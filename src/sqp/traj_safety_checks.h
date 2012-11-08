#include <vector>
#include <map>
#include "sqp_fwd.h"
#include "utils_sqp.h"
#include "collisions.h"
using std::map;
using std::pair;

typedef pair<double, double> paird;
typedef vector<paird> IntervalList;
typedef map<int, IntervalList > AllowedCollisions;

/**
 * Returns true if d is within one of the given intervals
 * @param d the floating point number to test
 * @param intervals a list of intervals to test
 * @return true if d is within one of the given intervals
 */
bool inSomeInterval(double d, const IntervalList& intervals);

/**
 * Removes numbers from in which fall within the specified intervals
 * @param in input vector of doubles to be filtered
 * @param exclude vector of intervals to be excluded from the output
 * @return the filtered vector of doubles
 */
vector<double> filterOutIntervals(const std::vector<double>& in, const IntervalList& exclude);
 
/**
 * @param lc a LinkCollision corresponding to a link in collision
 * @param time the time the collision occurs
 * @param allowedCollisions mapping of link indices to allowed time intervals
 *  for collision
 * @return true if the given LinkCollision is allowed
 */
bool linkInAllowedCollision(const LinkCollision lc, double time, const AllowedCollisions& allowedCollisions);
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
    const AllowedCollisions& allowedCollisions);

/**
 * Given a trajectory with continuous collisions, this function decides when to
 * add points to the trajectory to form a safe trajectory
 * @param cci collision information for a trajectory
 * @param times time information for a trajectory
 * @return a list of new times to be sampled in a new trajectory
 */
vector<double> getSubdivisionTimes(const TrajCartCollInfo& cci, const Eigen::VectorXd& times,
    const AllowedCollisions& allowedCollisions);
