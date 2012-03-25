#include "knots.h"
#include "utils_python.h"

npMatrixf toNumpy(const vector<RobotAndRopeState>& rars) {
  assert(rars.size() > 0);
  int nCtrlPts = rars[0].ctrlPts.size();
  int nRows = rars.size();
  int nCols = 18+nCtrlPts*3;
  int dims[2] = {nRows, nCols};
  
  npMatrixf out(dims);
  
  for (int i=0; i < rars.size(); i++) {
    out[i][0] = rars[i].leftPose.getOrigin().x();
    out[i][1] = rars[i].leftPose.getOrigin().y();
    out[i][2] = rars[i].leftPose.getOrigin().z();
    out[i][3] = rars[i].leftPose.getRotation().x();
    out[i][4] = rars[i].leftPose.getRotation().y();
    out[i][5] = rars[i].leftPose.getRotation().z();
    out[i][6] = rars[i].leftPose.getRotation().w();
    out[i][7] = rars[i].leftGrip;
    out[i][8] = rars[i].leftGrab;

    out[i][9] = rars[i].rightPose.getOrigin().x();
    out[i][10] = rars[i].rightPose.getOrigin().y();
    out[i][11] = rars[i].rightPose.getOrigin().z();
    out[i][12] = rars[i].rightPose.getRotation().x();
    out[i][13] = rars[i].rightPose.getRotation().y();
    out[i][14] = rars[i].rightPose.getRotation().z();
    out[i][15] = rars[i].rightPose.getRotation().w();
    out[i][16] = rars[i].rightGrip;
    out[i][17] = rars[i].rightGrab;
    
    for (int i=0; i < nCtrlPts; i++) {
      out[i][18+3*i+0] = rars[i].ctrlPts[i].x();
      out[i][18+3*i+1] = rars[i].ctrlPts[i].y();
      out[i][18+3*i+2] = rars[i].ctrlPts[i].z();
    }
    
  }
  
  return out;
}


py::object toNumpy1(const vector<RobotAndRopeState>& rars) {
  assert(rars.size() > 0);
  int nCtrlPts = rars[0].ctrlPts.size();
  int nRows = rars.size();
  int nCols = 18+nCtrlPts*3;

  py::object np = py::import("numpy");
  py::object out = np.attr("zeros")(py::make_tuple(nRows,nCols));
  
  for (int i=0; i < rars.size(); i++) {
    out[i][0] = rars[i].leftPose.getOrigin().x();
    out[i][1] = rars[i].leftPose.getOrigin().y();
    out[i][2] = rars[i].leftPose.getOrigin().z();
    out[i][3] = rars[i].leftPose.getRotation().x();
    out[i][4] = rars[i].leftPose.getRotation().y();
    out[i][5] = rars[i].leftPose.getRotation().z();
    out[i][6] = rars[i].leftPose.getRotation().w();
    out[i][7] = rars[i].leftGrip;
    out[i][8] = rars[i].leftGrab;

    out[i][9] = rars[i].rightPose.getOrigin().x();
    out[i][10] = rars[i].rightPose.getOrigin().y();
    out[i][11] = rars[i].rightPose.getOrigin().z();
    out[i][12] = rars[i].rightPose.getRotation().x();
    out[i][13] = rars[i].rightPose.getRotation().y();
    out[i][14] = rars[i].rightPose.getRotation().z();
    out[i][15] = rars[i].rightPose.getRotation().w();
    out[i][16] = rars[i].rightGrip;
    out[i][17] = rars[i].rightGrab;
    
    for (int j=0; j < nCtrlPts; j++) {
      out[i][18+3*j+0] = rars[i].ctrlPts[j].x();
      out[i][18+3*j+1] = rars[i].ctrlPts[j].y();
      out[i][18+3*j+2] = rars[i].ctrlPts[j].z();
    }
    
  }
  
  return out;
}


npMatrixf ropeToNumpy(const vector<btVector3>& pts) {
  int dims[2] = {pts.size(), 3};

  npMatrixf out(dims);
  for (int i=0; i < pts.size(); i++) {
    out[i][0] = pts[i].x();
    out[i][1] = pts[i].y();
    out[i][2] = pts[i].z();        
  }
  return out;
}

py::object ropeToNumpy1(const vector<btVector3>& pts) { 
  py::object np = py::import("numpy");
  int nRows = pts.size();
  int nCols = 3;
  py::object out = np.attr("zeros")(py::make_tuple(nRows,nCols));



  assert(pts.size() == N_CTRL_PTS);

  for (int i=0; i < pts.size(); i++) {
    out[i][0] = pts[i].x();
    out[i][1] = pts[i].y();
    out[i][2] = pts[i].z();        
  }
  return out;
}

static inline float f(py::object o) {return py::extract<float>(o);}
static inline float ii(py::object o) {return py::extract<int>(PyGlobals::builtin_module.attr("int")(o));}

vector<RobotAndRopeState> fromNumpy(py::object arr) {
  int n = py::extract<int>(arr.attr("__len__")());
  vector<RobotAndRopeState> out(n);
  for (int i=0; i < out.size(); i++) {
    py::object a = arr[i];
    out[i].leftPose = btTransform(btQuaternion(f(a["quat_l"][0]),f(a["quat_l"][1]),f(a["quat_l"][2]),f(a["quat_l"][3])),
				  btVector3(f(a["xyz_l"][0]),f(a["xyz_l"][1]),f(a["xyz_l"][2])));
    out[i].rightPose = btTransform(btQuaternion(f(a["quat_r"][0]),f(a["quat_r"][1]),f(a["quat_r"][2]),f(a["quat_r"][3])),
				  btVector3(f(a["xyz_r"][0]),f(a["xyz_r"][1]),f(a["xyz_r"][2])));
    out[i].leftGrip = f(a["grip_l"]);
    out[i].rightGrip = f(a["grip_r"]);
    out[i].leftGrab = ii(a["grab_l"]);
    out[i].rightGrab = ii(a["grab_r"]);

    for (int j=0; j < N_CTRL_PTS; j++) {
      out[i].ctrlPts.push_back(btVector3(f(a["rope"][j][0]),f(a["rope"][j][1]),f(a["rope"][j][2])));
    }
  }
  return out;
}

vector<RobotAndRopeState> fromNumpy(const npMatrixf& arr) {

  int nRows = arr.shape()[0];
  int nCols = arr.shape()[1];
  
  vector<RobotAndRopeState> out(nRows);
  
  assert(18+N_CTRL_PTS == nCols);
  
  for (int i=0; i < nRows; i++) {
    out[i].leftPose = btTransform(btQuaternion(arr[i][3],arr[i][4],arr[i][5],arr[i][6]),
      btVector3(arr[i][0],arr[i][1],arr[i][2]));
    out[i].leftGrip = arr[i][7];
    out[i].leftGrab = arr[i][8];

    out[i].rightPose = btTransform(btQuaternion(arr[i][12],arr[i][13],arr[i][14],arr[i][15]),
      btVector3(arr[i][9],arr[i][10],arr[i][11]));
    out[i].rightGrip = arr[i][16];
    out[i].rightGrab = arr[i][17];

    out[i].ctrlPts.reserve(N_CTRL_PTS);
    for (int j=0; j < N_CTRL_PTS; j++)
      out[i].ctrlPts.push_back(btVector3(arr[i][18+3*j+0],arr[i][18+3*j+1],arr[i][18+3*j+2]));

  }
  
  return out;
} 

