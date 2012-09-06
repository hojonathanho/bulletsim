#include "phasespace.h"
#include "utils/conversions.h"
#include <algorithm>
#include <boost/foreach.hpp>
#include <fstream>

using namespace std;
using namespace Eigen;

const int MarkerBody::m_validity_num_frames = 100;
const float MarkerBody::m_min_confidence = 1.0;


void MarkerBody::init(int tracker)
{
	// create point tracker
	owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);

	// set markers
	for (int ind=0; ind<m_led_ids.size(); ind++)
		owlMarkeri(MARKER(tracker, getLedId(ind)), OWL_SET_LED, getLedId(ind));

	// activate tracker
	owlTracker(tracker, OWL_ENABLE);
}

void MarkerBody::plot()
{
	if (m_plotSpheres) {
		osg::ref_ptr<osg::Vec3Array> centers = new osg::Vec3Array();
		osg::ref_ptr<osg::Vec4Array> rgba = new osg::Vec4Array();
		vector<float> sizes;
		for (int ind=0; ind<m_positions.size(); ind++) {
			if (isValid(ind)) {
				centers->push_back(osg::Vec3(getPosition(ind)(0), getPosition(ind)(1), getPosition(ind)(2)));
				rgba->push_back(osg::Vec4(1,1,1,1));
				sizes.push_back(0.005*METERS);
			}
		}
		if (!centers->empty())  m_plotSpheres->plot(centers, rgba, sizes);
		else                    m_plotSpheres->clear();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MarkerRigid::MarkerRigid(vector<ledid_t> led_ids, vector<Eigen::Vector3f> marker_positions, Environment::Ptr env) :
	MarkerBody(RIGID, led_ids, env),
	m_last_valid_rigid_frame(0),
	m_last_rigid_frame(m_validity_num_frames),
	m_marker_positions(marker_positions),
	m_transform(Affine3f::Identity())
{
	assert(led_ids.size() == marker_positions.size());

	if (m_env) {
		m_plotAxes = PlotAxes::Ptr(new PlotAxes());
		m_env->add(m_plotAxes);
	}
}

void MarkerRigid::init(int tracker)
{
	// create rigid body tracker
	owlTrackeri(tracker, OWL_CREATE, OWL_RIGID_TRACKER);

	// set markers
	for (int ind=0; ind<m_led_ids.size(); ind++) {
		// set markers
		owlMarkeri(MARKER(tracker, getLedId(ind)), OWL_SET_LED, getLedId(ind));
		// set marker positions
		float position[3] = { m_marker_positions[ind](0)*1000.0/METERS, m_marker_positions[ind](1)*1000.0/METERS, m_marker_positions[ind](2)*1000.0/METERS };
		owlMarkerfv(MARKER(tracker, getLedId(ind)), OWL_SET_POSITION, position);
	}

	// activate tracker
	owlTracker(tracker, OWL_ENABLE);

	m_rigid_id = tracker;
}

void MarkerRigid::updateMarkers(OWLMarker markers[], int markers_count, OWLRigid rigids[], int rigids_count) {
	m_last_rigid_frame++;
	assert(m_rigid_id < rigids_count);
	int iRigid = m_rigid_id;
	if(iRigid!=-1 && rigids[iRigid].cond > m_min_confidence) {
		m_last_valid_rigid_frame = m_last_rigid_frame;
		Translation3f translation(rigids[iRigid].pose[0]*METERS/1000.0, rigids[iRigid].pose[1]*METERS/1000.0, rigids[iRigid].pose[2]*METERS/1000.0);
		Quaternionf rotation(rigids[iRigid].pose[3], rigids[iRigid].pose[4], rigids[iRigid].pose[5], rigids[iRigid].pose[6]);
		m_transform = translation * rotation;
	}
	MarkerBody::updateMarkers(markers, markers_count, rigids, rigids_count);
}

void MarkerRigid::plot()
{
	if (m_plotAxes) {
		if (isValid())  m_plotAxes->setup(toBulletTransform(getTransform()), 0.10*METERS);
		else            m_plotAxes->clear();
	}
	MarkerBody::plot();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


MarkerSoft::MarkerSoft(std::vector<ledid_t> led_ids, std::vector<Eigen::Vector3f> marker_positions, EnvironmentObject::Ptr sim, Environment::Ptr env) :
	MarkerBody(SOFT, led_ids, env),
	m_sim(sim)
{
	assert(led_ids.size() == marker_positions.size());

	// set the m_sim2ind_transforms so that the current marker_positions define the simulation positions
	for (int ind=0; ind<m_led_ids.size(); ind++) {
		Affine3f marker_transform = Affine3f(Translation3f(marker_positions[ind]));
		m_ind2simInd.push_back(m_sim->getIndex(toBulletTransform(marker_transform)));
		const Affine3f sim_transform = toEigenTransform(m_sim->getIndexTransform(getSimInd(ind)));
		m_sim2ind_transforms.push_back(sim_transform.inverse() * marker_transform);
	}
}

float MarkerSoft::evaluateError()
{
	vector<Vector3f> position_diffs;

	for (int ind=0; ind<m_led_ids.size(); ind++)
		if (isValid(ind))
			position_diffs.push_back(getSimPosition(ind) - getPosition(ind));
	if (position_diffs.size() == 0) return -1;

	VectorXf diffs(position_diffs.size()*3);
	for (int k=0; k<position_diffs.size(); k++)
		for (int c=0; c<3; c++)
			diffs(3*k+c) = position_diffs[k](c);

	VectorXf diffs_squared = diffs.array().square();
	return sqrt(diffs_squared.sum()/((float) diffs_squared.size()));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MarkerSystem::MarkerSystem(std::vector<MarkerBody::Ptr> marker_bodies) :
	m_marker_bodies(marker_bodies),
	m_rigid_count(0),
	m_marker_count(32)
{
	if(owlInit(PHASESPACE_SERVER_NAME, PHASESPACE_INIT_FLAGS) < 0) { runtime_error("owl initialization failed"); };

	for (int iBody=0; iBody<m_marker_bodies.size(); iBody++) {
		m_marker_bodies[iBody]->init(iBody);
		if (m_marker_bodies[iBody]->m_body_type == MarkerBody::RIGID) m_rigid_count++;
	}

  // flush requests and check for errors
  if(!owlGetStatus()) {
		owl_print_error("error in point tracker setup", owlGetError());
		runtime_error("error in point tracker setup");
	}

  // set default frequency
  owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);

  // start streaming
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);
}

MarkerSystem::~MarkerSystem()
{
	owlDone();
}

void MarkerSystem::updateMarkers()
{
	OWLRigid rigids[m_rigid_count];
	OWLMarker markers[m_marker_count];

	// get the rigid body
	int r = owlGetRigids(rigids, m_rigid_count);

	// get the rigid body markers
	//  note: markers have to be read,
	//  even if they are not used
	int m = owlGetMarkers(markers, m_marker_count);

	// check for error
	int err;
	if((err = owlGetError()) != OWL_NO_ERROR)
	{
		owl_print_error("error", err);
		return;
	}

	for (int iBody=0; iBody<m_marker_bodies.size(); iBody++) {
		m_marker_bodies[iBody]->updateMarkers(markers, m_marker_count, rigids, m_rigid_count);
	}
}

void MarkerSystem::startUpdateLoop() {
	while(!m_exit_loop) {
		updateMarkers();
		usleep(1000);
	}
}

void MarkerSystem::startUpdateLoopThread() {
	m_exit_loop = false;
	m_update_loop_thread.reset(new boost::thread(boost::bind(&MarkerSystem::startUpdateLoop, this)));
}

void MarkerSystem::stopUpdateLoopThread() {
	m_exit_loop = true;
	if (m_update_loop_thread)
		m_update_loop_thread->join();
	m_update_loop_thread.reset();
}

void MarkerSystem::setPose(Eigen::Affine3f transform) {
	Vector3f tf_t(transform.translation());
	tf_t *= 1000.0/METERS;
	Quaternionf tf_q(transform.rotation());
	float pose[7] = { tf_t(0), tf_t(1), tf_t(2), tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z() };
	owlLoadPose(pose);
}

void MarkerSystem::blockUntilAllValid() {
	bool all_valid = false;
	while (!all_valid) {
		updateMarkers();
		int ind;
		for (ind=0; ind<m_marker_bodies.size(); ind++)
			if (!m_marker_bodies[ind]->isValid()) break;
		if (ind == m_marker_bodies.size()) all_valid = true;
		//usleep(15000);
	}
}

void MarkerSystem::owl_print_error(const char *s, int n)
{
  if(n < 0) printf("%s: %d\n", s, n);
  else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
  else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
  else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
  else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
  else printf("%s: 0x%x\n", s, n);
}

MarkerRigid::Ptr createMarkerRigid(std::string rigid_info_filename, Environment::Ptr env) {
	vector<ledid_t> led_ids;
	vector<Vector3f> marker_positions;
	loadPhasespaceRigid(rigid_info_filename, led_ids, marker_positions);
	for (int i=0; i<marker_positions.size(); i++)
		marker_positions[i] *= METERS;
	MarkerRigid::Ptr marker_rigid(new MarkerRigid(led_ids, marker_positions, env));
	return marker_rigid;
}

Eigen::Affine3f waitForRigidBodyTransform(std::string rigid_info_filename, int iter) {
	MarkerRigid::Ptr marker_rigid = createMarkerRigid(rigid_info_filename);
	vector<MarkerBody::Ptr> marker_bodies;
	marker_bodies.push_back(marker_rigid);

	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies));

	cout << "Waiting for rigid body LEDs " << marker_rigid->m_led_ids << " to be seen..." << endl;
	for (int i=0; i<iter; i++)
		marker_system->blockUntilAllValid();

	return marker_rigid->getTransform();
}

vector<Vector3f> waitForMarkerPositions(vector<ledid_t> led_ids, Eigen::Affine3f marker_system_transform) {
	vector<MarkerPoint::Ptr> marker_points;
	for (int ind=0; ind<led_ids.size(); ind++)
		marker_points.push_back(MarkerPoint::Ptr(new MarkerPoint(led_ids[ind])));
	vector<MarkerBody::Ptr> marker_bodies;
	for (int ind=0; ind<marker_points.size(); ind++)
		marker_bodies.push_back(marker_points[ind]);

	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies));
	marker_system->setPose(marker_system_transform);

	cout << "Waiting for LEDs " << led_ids << " to be seen..." << endl;
	marker_system->blockUntilAllValid();

	vector<Vector3f> marker_positions;
	for (int ind=0; ind<led_ids.size(); ind++)
		marker_positions.push_back(marker_points[ind]->getPosition());
	return marker_positions;
}

MarkerSoft::Ptr createMarkerSoftCloth(vector<ledid_t> led_ids_front, vector<ledid_t> led_ids_back, Vector3f frontToBackVector, EnvironmentObject::Ptr sim, Eigen::Affine3f marker_system_transform, Environment::Ptr env) {
	assert(led_ids_front.size() == led_ids_back.size());

	// For the front LEDs, fill marker_positions with the positions of the seen LEDs
	// For the back LEDs, assume the marker_position is frontToBackVector offset from the corresponding seen LED
	vector<Vector3f> marker_positions = waitForMarkerPositions(led_ids_front, marker_system_transform);
	for (int ind=0; ind<led_ids_front.size(); ind++)
		marker_positions.push_back(marker_positions[ind] + frontToBackVector);

	// Append front and back LEDs
	vector<ledid_t> led_ids = led_ids_front;
	led_ids.insert(led_ids.end(), led_ids_back.begin(), led_ids_back.end());

	MarkerSoft::Ptr marker_soft(new MarkerSoft(led_ids, marker_positions, sim, env));
	return marker_soft;
}

bool savePhasespaceRigid(const std::string& filename, const std::vector<ledid_t>& led_ids, const std::vector<Eigen::Vector3f>& positions) {
	ofstream file;
	file.open(filename.c_str());
	if (file.fail()) {
		cout << "Phasespace rigid body information couldn't be saved to " << filename << endl;
		return false;
	}
	file.precision(20);

	assert(led_ids.size() == positions.size());
	file << positions.size() << " ";
	for (int i=0; i<positions.size(); i++) {
		file << led_ids[i] << " ";
		for(int c=0; c<3; c++)
			file << positions[i](c) << " ";
	}
	file << "\n";

	file.close();
	cout << "Phasespace rigid body information saved to " << filename << endl;
	return true;
}

bool loadPhasespaceRigid(const std::string& filename, std::vector<ledid_t>& led_ids, std::vector<Eigen::Vector3f>& positions) {
  ifstream file;
  file.open(filename.c_str());
  if (file.fail()) {
		cout << "Phasespace rigid body information couldn't be loaded from " << filename << endl;
  	return false;
  }

  int positions_size;
  file >> positions_size;
  led_ids.resize(positions_size, 0);
  positions.resize(positions_size, Vector3f::Zero());
  while (!file.eof()) {
  	for (int i=0; i<positions_size; i++) {
			file >> led_ids[i];
			for(int c=0; c<3; c++)
				file >> positions[i](c);
		}
  }

  file.close();
	cout << "Phasespace rigid body information loaded from " << filename << endl;
  return true;
}
