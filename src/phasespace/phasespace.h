#ifndef PHASESPACE_H_
#define PHASESPACE_H_

#include <Eigen/Dense>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <stdint.h>
#include <owl.h>
#include "phasespace_utils.h"
#include <ros/ros.h>
#include "utils/config.h"
#include "utils/conversions.h"
#include "simulation/environment.h"
#include "simulation/plotting.h"
#include "config_phasespace.h"

#include <boost/thread.hpp>

#define PHASESPACE_SERVER_NAME "169.229.222.231"
#define PHASESPACE_INIT_FLAGS 0
//#define PHASESPACE_INIT_FLAGS OWL_POSTPROCESS

typedef int16_t ledid_t;

class MarkerBody {
public:
  typedef boost::shared_ptr<MarkerBody> Ptr;
  enum marker_body_t { RIGID, POINT, SOFT, POINT_COLLECTION };
  marker_body_t m_body_type;
  std::vector<ledid_t> m_led_ids;
  std::vector<int> m_last_valid_frame;
  std::vector<int> m_last_frame;
  std::vector<Eigen::Vector3f> m_positions;
  std::vector<std::deque<Eigen::Vector3f> > m_past_positions;

  // for plotting
  Environment::Ptr m_env;

  MarkerBody(marker_body_t body_type, std::vector<ledid_t> led_ids, Environment::Ptr env=Environment::Ptr()) :
  	m_body_type(body_type),
  	m_led_ids(led_ids),
    m_last_valid_frame(m_led_ids.size(), 0),
    m_last_frame(m_led_ids.size(), PhasespaceConfig::validityNumFrames),
    m_positions(m_led_ids.size(), Eigen::Vector3f::Zero()),
    m_past_positions(m_led_ids.size(), std::deque<Eigen::Vector3f>(0)),
  	m_env(env)
  {
  	if (m_env) {
  		m_plotSpheres = PlotSpheres::Ptr(new PlotSpheres());
  		m_env->add(m_plotSpheres);
  	}
  }

  inline ledid_t getLedId(int ind) { return m_led_ids[ind]; }
	int getInd(ledid_t led_id) {
		for (int ind=0; ind<m_led_ids.size(); ind++)
			if (m_led_ids[ind] == led_id)	return ind;
		return -1;
	}

  inline bool isValid(int ind) { return (m_last_frame[ind] - m_last_valid_frame[ind]) < PhasespaceConfig::validityNumFrames; }
  bool isValid(ledid_t led_id) { return isValid(getInd(led_id)); }
  // returns true if all the markers are valid
  virtual bool isValid() {
		for (int ind=0; ind<m_led_ids.size(); ind++)
			if (!isValid(ind)) return false;
		return true;
  }

  inline Eigen::Vector3f getPosition(int ind) { return m_positions[ind]; }
  inline Eigen::Vector3f getPosition(ledid_t led_id) { getPosition(getInd(led_id)); }

  Eigen::Vector3f getPositionMean(int ind);
  inline Eigen::Vector3f getPositionMean(ledid_t led_id) { getPositionMean(getInd(led_id)); }
  float getPositionVariance(int ind);
  inline float getPositionVariance(ledid_t led_id) { getPositionVariance(getInd(led_id)); }

  // updates the member variables that changes when phasespace is queried
  virtual void updateMarkers(OWLMarker markers[], int markers_count) {
  	for (int ind=0; ind<m_led_ids.size(); ind++) {
  		m_last_frame[ind]++;
  		if(markers[getLedId(ind)].cond > PhasespaceConfig::minConfidence) {
				m_last_valid_frame[ind] = m_last_frame[ind];
				m_positions[ind] = Eigen::Vector3f(markers[getLedId(ind)].x, markers[getLedId(ind)].y, markers[getLedId(ind)].z) * METERS/1000.0;

				if (m_past_positions[ind].size() > PhasespaceConfig::maxPosHistory) m_past_positions[ind].pop_front();
				m_past_positions[ind].push_back(m_positions[ind]);
			}
//  		if (getLedId(ind)) cout << "LED 35 cond " << markers[getLedId(ind)].cond << " " << isValid(ind) << " " << getPosition(ind).transpose() << endl;
		}
  }

  // for plotting
  PlotSpheres::Ptr m_plotSpheres;
  // if ind is outside the range [0, m_led_ids.size()], then all the leds are plotted
  virtual void plot(int ind=-1);
};

class MarkerRigid : public MarkerBody {
public:
  typedef boost::shared_ptr<MarkerRigid> Ptr;
  int m_last_valid_rigid_frame;
  int m_last_rigid_frame;
  std::vector<Eigen::Vector3f> m_marker_positions;
  Eigen::Affine3f m_transform;

	MarkerRigid(std::vector<ledid_t> led_ids, std::vector<Eigen::Vector3f> marker_positions, Environment::Ptr env=Environment::Ptr());
	virtual void updateMarkers(OWLMarker markers[], int markers_count);

	using MarkerBody::isValid;
	inline bool isValid() { return (m_last_rigid_frame - m_last_valid_rigid_frame) < PhasespaceConfig::validityNumFrames; }
	inline Eigen::Affine3f getTransform() { return m_transform; }

	// for plotting
  PlotAxes::Ptr m_plotAxes;
	void plot(int ind=-1);
};

class MarkerRigidStatic : public MarkerRigid {
public:
  typedef boost::shared_ptr<MarkerRigidStatic> Ptr;
  MarkerRigidStatic(std::vector<ledid_t> led_ids, std::vector<Eigen::Vector3f> marker_positions, Environment::Ptr env=Environment::Ptr()) : MarkerRigid(led_ids, marker_positions, env) {}
  void updateMarkers(OWLMarker markers[], int markers_count);
};

class MarkerPoint : public MarkerBody {
public:
  typedef boost::shared_ptr<MarkerPoint> Ptr;

  MarkerPoint(ledid_t led_id, Environment::Ptr env=Environment::Ptr()) : MarkerBody(POINT, vector<ledid_t>(1, led_id), env) {}

  using MarkerBody::getLedId;
  using MarkerBody::isValid;
  using MarkerBody::getPosition;
  inline ledid_t getLedId() { return getLedId(0); }
  inline bool isValid() { return isValid(0); }
  inline Eigen::Vector3f getPosition() { return getPosition(0); }
};

class MarkerPointCollection : public MarkerBody {
public:
  typedef boost::shared_ptr<MarkerPointCollection> Ptr;

  MarkerPointCollection(std::vector<ledid_t> led_ids, Environment::Ptr env=Environment::Ptr()) : MarkerBody(POINT_COLLECTION, led_ids, env) {}
};

class MarkerSoft : public MarkerBody {
public:
	typedef boost::shared_ptr<MarkerSoft> Ptr;

	EnvironmentObject::Ptr m_sim;

	std::vector<int> m_ind2simInd;
	// Defines a transform with respect to a specific (indexed) part of an EnvironmentObject. This is particularly useful for non-rigid bodies since their transform is not trivially defined.
	std::vector<Eigen::Affine3f> m_sim2ind_transforms;

	MarkerSoft(std::vector<ledid_t> led_ids, std::vector<Eigen::Vector3f> marker_positions, EnvironmentObject::Ptr sim, Environment::Ptr env=Environment::Ptr());

	inline int getSimInd(int ind) { return m_ind2simInd[ind]; }
	inline int getSimInd(ledid_t led_id) { return getSimInd(getInd(led_id)); }

	inline Eigen::Affine3f getSimTransform(int ind) { return toEigenTransform(m_sim->getIndexTransform(getSimInd(ind))) * m_sim2ind_transforms[ind]; }
	inline Eigen::Affine3f getSimTransform(ledid_t led_id) { return getSimTransform(getInd(led_id)); }
	inline Eigen::Vector3f getSimPosition(int ind) { return getSimTransform(ind).translation(); }
	inline Eigen::Vector3f getSimPosition(ledid_t led_id) { return getSimPosition(getInd(led_id)); }

	float evaluateError();
};

class MarkerSystemBase {
public:
  typedef boost::shared_ptr<MarkerSystemBase> Ptr;

  bool isAllValid();
  void plot();
  void blockUntilAllValid();
  void updateMarkerBodies(OWLMarker markers[]);

  virtual ~MarkerSystemBase() {}

protected:
  MarkerSystemBase() {}

  std::vector<MarkerBody::Ptr> m_marker_bodies;

public:
  static const int m_marker_count;

  void add(MarkerBody::Ptr marker_body) { m_marker_bodies.push_back(marker_body); }
  template<class T>
  void add(std::vector<boost::shared_ptr<T> > marker_bodies) { m_marker_bodies.insert(m_marker_bodies.end(), marker_bodies.begin(), marker_bodies.end()); }
};

class MarkerSystem : public MarkerSystemBase {
public:
  typedef boost::shared_ptr<MarkerSystem> Ptr;

  MarkerSystem();
  ~MarkerSystem();

	virtual void updateIteration();

protected:
	void listenOWLServer(OWLMarker markers[]);

public:
	void startUpdateLoopThread();
	void stopUpdateLoopThread();

private:
	void owl_print_error(const char *s, int n);

  bool m_exit_loop;
  boost::shared_ptr<boost::thread> m_update_loop_thread;
	void startUpdateLoop();
};

class MarkerSystemPublisher : public MarkerSystem {
public:
	typedef boost::shared_ptr<MarkerSystemPublisher> Ptr;
	MarkerSystemPublisher(boost::shared_ptr<ros::Publisher> publisher) : m_publisher(publisher) {}

private:
	boost::shared_ptr<ros::Publisher> m_publisher;
	void updateIteration();
	void publishPhasespaceMsg(OWLMarker markers[]);
};

class MarkerSystemPhasespaceMsg : public MarkerSystemBase {
public:
  typedef boost::shared_ptr<MarkerSystemPhasespaceMsg> Ptr;

	void updateIteration(bulletsim_msgs::OWLPhasespace phasespace_msg);
};

MarkerRigid::Ptr createMarkerRigid(std::string rigid_info_filename, Environment::Ptr env=Environment::Ptr());
Eigen::Affine3f waitForRigidBodyTransform(std::string rigid_info_filename, int iter=1);
vector<Eigen::Vector3f> waitForMarkerPositions(std::vector<ledid_t> led_ids, Eigen::Affine3f marker_system_transform);
MarkerSoft::Ptr createMarkerSoftCloth(std::vector<ledid_t> led_ids_front, std::vector<ledid_t> led_ids_back, Eigen::Vector3f frontToBackVector, EnvironmentObject::Ptr sim, Eigen::Affine3f marker_system_transform, Environment::Ptr env=Environment::Ptr());
bool savePhasespaceRigid(const std::string& filename, const std::vector<ledid_t>& led_ids, const std::vector<Eigen::Vector3f>& positions);
bool loadPhasespaceRigid(const std::string& filename, std::vector<ledid_t>& led_ids, std::vector<Eigen::Vector3f>& positions);

#endif /* PHASESPACE_H_ */
