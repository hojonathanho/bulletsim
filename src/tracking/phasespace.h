#ifndef PHASESPACE_H_
#define PHASESPACE_H_

#include <Eigen/Dense>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <stdint.h>
#include <owl.h>
#include "utils/config.h"
#include "utils/conversions.h"
#include "simulation/environment.h"
#include "simulation/plotting.h"

//#include <boost/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/shared_mutex.hpp>
//#include <boost/thread/locks.hpp>

#define PHASESPACE_SERVER_NAME "169.229.222.231"
#define PHASESPACE_INIT_FLAGS 0
//#define PHASESPACE_INIT_FLAGS OWL_POSTPROCESS

typedef int16_t ledid_t;

class MarkerBody {
public:
  typedef boost::shared_ptr<MarkerBody> Ptr;
  enum marker_body_t { RIGID, POINT, SOFT };
  marker_body_t m_body_type;
  std::vector<ledid_t> m_led_ids;
  std::vector<int> m_last_valid_frame;
  std::vector<int> m_last_frame;
  static const int m_validity_num_frames;
  std::vector<Eigen::Vector3f> m_positions;
  static const float m_min_confidence;

  // for plotting
  Environment::Ptr m_env;

  MarkerBody(marker_body_t body_type, std::vector<ledid_t> led_ids, Environment::Ptr env=Environment::Ptr()) :
  	m_body_type(body_type),
  	m_led_ids(led_ids),
    m_last_valid_frame(m_led_ids.size(), 0),
    m_last_frame(m_led_ids.size(), m_validity_num_frames),
    m_positions(m_led_ids.size(), Eigen::Vector3f::Zero()),
  	m_env(env)
  {
  	if (m_env) {
  		m_plotSpheres = PlotSpheres::Ptr(new PlotSpheres());
  		m_env->add(m_plotSpheres);
  	}
  }

  int getMarkerInd(OWLMarker markers[], int markers_count, ledid_t led_id) {
  	for (int iMarker=0; iMarker<markers_count; iMarker++)
  		if (INDEX(markers[iMarker].id) == led_id)	return iMarker;
  	return -1;
  }
  inline ledid_t getLedId(int ind) { return m_led_ids[ind]; }
	int getInd(ledid_t led_id) {
		for (int ind=0; ind<m_led_ids.size(); ind++)
			if (m_led_ids[ind] == led_id)	return ind;
		return -1;
	}

  inline bool isValid(int ind) { return (m_last_frame[ind] - m_last_valid_frame[ind]) < m_validity_num_frames; }
  bool isValid(ledid_t led_id) { return isValid(getInd(led_id)); }
  // returns true if all the markers are valid
  virtual bool isValid() {
		for (int ind=0; ind<m_led_ids.size(); ind++)
			if (!isValid(ind)) return false;
		return true;
  }

  inline Eigen::Vector3f getPosition(int ind) { return m_positions[ind]; }
  inline Eigen::Vector3f getPosition(ledid_t led_id) { getPosition(getInd(led_id)); }

  // creates tracker, set markers and activates tracker
  virtual void init(int tracker);

  // updates the member variables that changes when phasespace is queried
  virtual void updateMarkers(OWLMarker markers[], int markers_count, OWLRigid rigids[], int rigids_count) {
  	for (int ind=0; ind<m_led_ids.size(); ind++) {
  		m_last_frame[ind]++;
  		int iMarker = getMarkerInd(markers, markers_count, getLedId(ind));
			if(iMarker!=-1 && markers[iMarker].cond > m_min_confidence) {
				m_last_valid_frame[ind] = m_last_frame[ind];
				m_positions[ind] = Eigen::Vector3f(markers[iMarker].x, markers[iMarker].y, markers[iMarker].z) * METERS/1000.0;
			}
		}
  }

  // for plotting
  PlotSpheres::Ptr m_plotSpheres;
  virtual void plot();
};

class MarkerRigid : public MarkerBody {
public:
  typedef boost::shared_ptr<MarkerRigid> Ptr;
  int m_last_valid_rigid_frame;
  int m_last_rigid_frame;
  std::vector<Eigen::Vector3f> m_marker_positions;
  Eigen::Affine3f m_transform;

	MarkerRigid(std::vector<ledid_t> led_ids, std::vector<Eigen::Vector3f> marker_positions, Environment::Ptr env=Environment::Ptr());
	void init(int tracker);
	void updateMarkers(OWLMarker markers[], int markers_count, OWLRigid rigids[], int rigids_count);

	using MarkerBody::isValid;
	inline bool isValid() { return (m_last_rigid_frame - m_last_valid_rigid_frame) < m_validity_num_frames; }
	inline Eigen::Affine3f getTransform() { return m_transform; }

	// for plotting
  PlotAxes::Ptr m_plotAxes;
	void plot();
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

class MarkerSystem {
public:
  typedef boost::shared_ptr<MarkerSystem> Ptr;

  std::vector<MarkerBody::Ptr> m_marker_bodies;
  int m_rigid_count;
  int m_marker_count;

  //boost::shared_mutex marker_mutex;

  MarkerSystem(std::vector<MarkerBody::Ptr> marker_bodies, Eigen::Affine3f transform=Eigen::Affine3f::Identity());
  ~MarkerSystem();

	void updateMarkers();
	void updateMarkers(Eigen::Affine3f transform);
	void blockUntilAllValid();
	void toOwlPose(const Eigen::Affine3f& transform, float* pose);
  void owl_print_error(const char *s, int n);
};

bool savePhasespaceRigid(const std::string& filename, const std::vector<ledid_t>& led_ids, const std::vector<Eigen::Vector3f>& positions);
bool loadPhasespaceRigid(const std::string& filename, std::vector<ledid_t>& led_ids, std::vector<Eigen::Vector3f>& positions);


#endif /* PHASESPACE_H_ */


