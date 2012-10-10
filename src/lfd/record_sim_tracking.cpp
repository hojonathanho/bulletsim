#include "lfd_rope_common.h"
#include "task_execution.h"
using namespace lfd;

#include "utils_python.h"
#include "utils/vector_io.h"
#include "utils/conversions.h"

#include <H5Cpp.h>
#include <boost/scoped_array.hpp>

struct LocalConfig : public Config {
  static string task;
  static string h5file;
  static double trajSlow;
  static string rope;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("task", &task, "task name"));
    params.push_back(new Parameter<string>("h5file", &h5file, "task library to simulate and annotate with state info. should be a copy of the task library for the specified task."));
    params.push_back(new Parameter<double>("trajSlow", &trajSlow, "slowdown factor for trajectory execution"));
    params.push_back(new Parameter<string>("rope", &rope, "initial rope control points"));
  }
};
string LocalConfig::task;
string LocalConfig::h5file;
double LocalConfig::trajSlow = 2.;
string LocalConfig::rope;

static void stepCallback(LFDRopeScene *s, SegRopeStates *recorded_states, py::dict traj, int step) {
  string seg_name = py::extract<string>(traj["seg_name"]);
  if (recorded_states->find(seg_name) == recorded_states->end()) {
    recorded_states->insert(make_pair(seg_name, vector<RopeState>()));
  }
  (*recorded_states)[seg_name].push_back(s->scene->m_rope->getControlPoints());
}

/**
 * Given a task-library h5 file, this tool initializes a scene with each demo
 * segment, then executes the demo trajectory and records the state at each timestep.
 * Writes a new h5 file with the states in seg_name/tracked_states[].
 */
int main(int argc, char *argv[]) {
  LFDRopeScene s(argc, argv, LocalConfig());

  // Load rope (should be close to 00.00/cloud_xyz)
  vector<btVector3> ropeCtlPts = toBulletVectors(floatMatFromFile(LocalConfig::rope)) * METERS;
  s.resetScene(ropeCtlPts);
  util::setGlobalEnv(s.scene->env);

  // Execute demonstration and track the simulated rope
  SegRopeStates recorded_states;
  TaskExecuter ex(*s.scene);
  ex.setTrajExecSlowdown(LocalConfig::trajSlow);
  ex.setTrajStepCallback(boost::bind(&stepCallback, &s, &recorded_states, _1, _2));
  try {
    ex.run(LocalConfig::task);
  } catch (const py::error_already_set &e) {
    PyErr_Print();
    throw e;
  }

  LOG_INFO("Done playing demo. Need to write " << recorded_states.size() << " segments");

  // Write the tracked states
  try {
    H5::H5File h5f(LocalConfig::h5file, H5F_ACC_RDWR);
    for (SegRopeStates::const_iterator it = recorded_states.begin(); it != recorded_states.end(); ++it) {
      const string &seg_name = it->first;
      const vector<RopeState> &rope_states = it->second;

      H5::Group group = h5f.openGroup(seg_name);

      hsize_t dims[] = { rope_states.size(), 3*rope_states[0].size() };
      LOG_DEBUG("allocating matrix " << dims[0] << "x" << dims[1]);
      boost::scoped_array<float> pts(new float[dims[0]*dims[1]]);
      for (int i = 0; i < dims[0]; ++i) {
        RopeState rope_pts = ropeState_sim2real(rope_states[i], s.scene->pr2m->pr2);
        assert(3*rope_pts.size() == dims[1]);
        for (int j = 0; j < rope_pts.size(); ++j) {
          pts[i*dims[1] + 3*j]   = rope_pts[j].x();
          pts[i*dims[1] + 3*j+1] = rope_pts[j].y();
          pts[i*dims[1] + 3*j+2] = rope_pts[j].z();
        }
      }
      H5::DataSpace dataspace(2, dims);
      string name = seg_name + "/tracked_states";
      LOG_INFO("Writing " << name);
      H5::DataSet dataset = h5f.createDataSet(name, H5::PredType::NATIVE_FLOAT, dataspace);
      dataset.write(pts.get(), H5::PredType::NATIVE_FLOAT);
      pts.reset();

      group.close();
    }
    h5f.close();

  } catch (const H5::Exception &e) {
    e.printError();
    throw e;
  }

  return 0;
}
