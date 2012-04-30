#include "knots.h"
#include "utils_python.h"


// py::object toNumpy1(const vector<RobotAndRopeState>& rars) {
//   assert(rars.size() > 0);
//   
//   
//   int nCtrlPts = rars[0].ctrlPts.size();
//   int nRows = rars.size();
//   int nCols = 18+nCtrlPts*3;
// 
//   py::object np = py::import("numpy");
//   py::object out = np.attr("zeros")(py::make_tuple(nRows,nCols));
//   
//   for (int i=0; i < rars.size(); i++) {
//     out[i][0] = rars[i].leftPose.getOrigin().x();
//     out[i][1] = rars[i].leftPose.getOrigin().y();
//     out[i][2] = rars[i].leftPose.getOrigin().z();
//     out[i][3] = rars[i].leftPose.getRotation().x();
//     out[i][4] = rars[i].leftPose.getRotation().y();
//     out[i][5] = rars[i].leftPose.getRotation().z();
//     out[i][6] = rars[i].leftPose.getRotation().w();
//     out[i][7] = rars[i].leftGrip;
//     out[i][8] = rars[i].leftGrab;
// 
//     out[i][9] = rars[i].rightPose.getOrigin().x();
//     out[i][10] = rars[i].rightPose.getOrigin().y();
//     out[i][11] = rars[i].rightPose.getOrigin().z();
//     out[i][12] = rars[i].rightPose.getRotation().x();
//     out[i][13] = rars[i].rightPose.getRotation().y();
//     out[i][14] = rars[i].rightPose.getRotation().z();
//     out[i][15] = rars[i].rightPose.getRotation().w();
//     out[i][16] = rars[i].rightGrip;
//     out[i][17] = rars[i].rightGrab;
//     
//     for (int j=0; j < nCtrlPts; j++) {
//       out[i][18+3*j+0] = rars[i].ctrlPts[j].x();
//       out[i][18+3*j+1] = rars[i].ctrlPts[j].y();
//       out[i][18+3*j+2] = rars[i].ctrlPts[j].z();
//     }
//     
//   }
//   
//   return out;
// }
// 
// TrajectoryPoint = np.dtype([("xyz_l",float,3),
//                             ("quat_l",float,4),
//                             ("grip_l",float),
//                             ("grab_l",int),                            
//                             ("xyz_r",float,3),
//                             ("quat_r",float,4),
//                             ("grip_r",float),
//                             ("grab_r",int),
//                             ("rope",float,(N_CTRL_PTS,3))],align=True)

py::object toNumpy1(const vector<RobotAndRopeState>& rars) {
  assert(rars.size() > 0);
  py::object tl = py::import("knot_tying.rope_library");
  
  int nRows = rars.size();
  int nCtrlPts = rars[0].ctrlPts.size();

  py::object dtype = tl.attr("RopeTrajectoryPoint");
  py::object out = NP.attr("zeros")(nRows, dtype);
  
  for (int i=0; i < nRows; i++) {
    btVector3 xyz_l = rars[i].leftPose.getOrigin();
    btQuaternion quat_l = rars[i].leftPose.getRotation();
    out["xyz_l"][i][0] = xyz_l.x();
    out["xyz_l"][i][1] = xyz_l.y();
    out["xyz_l"][i][2] = xyz_l.z();
    out["quat_l"][i][0] = quat_l.x();
    out["quat_l"][i][1] = quat_l.y();
    out["quat_l"][i][2] = quat_l.z();
    out["quat_l"][i][3] = quat_l.w();
    out["grip_l"][i] = rars[i].leftGrip;
    out["grab_l"][i] = rars[i].leftGrab;

    btVector3 xyz_r = rars[i].rightPose.getOrigin();
    btQuaternion quat_r = rars[i].rightPose.getRotation();
    out["xyz_r"][i][0] = xyz_r.x();
    out["xyz_r"][i][1] = xyz_r.y();
    out["xyz_r"][i][2] = xyz_r.z();
    out["quat_r"][i][0] = quat_r.x();
    out["quat_r"][i][1] = quat_r.y();
    out["quat_r"][i][2] = quat_r.z();
    out["quat_r"][i][3] = quat_r.w();
    out["grip_r"][i] = rars[i].rightGrip;
    out["grab_r"][i] = rars[i].rightGrab;
    
    for (int j=0; j < nCtrlPts; j++) {
      out["rope"][i][j][0] = rars[i].ctrlPts[j].x();
      out["rope"][i][j][1] = rars[i].ctrlPts[j].y();
      out["rope"][i][j][2] = rars[i].ctrlPts[j].z();
    }
    
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


vector<RobotAndRopeState> fromNumpy(py::object arr) {
  int n = py::extract<int>(arr.attr("__len__")());
  vector<RobotAndRopeState> out(n);
  for (int i=0; i < out.size(); i++) {
    py::object a = arr[i];
    out[i].leftPose = btTransform(btQuaternion(ff(a["quat_l"][0]),ff(a["quat_l"][1]),ff(a["quat_l"][2]),ff(a["quat_l"][3])),
				  btVector3(ff(a["xyz_l"][0]),ff(a["xyz_l"][1]),ff(a["xyz_l"][2])));
    out[i].rightPose = btTransform(btQuaternion(ff(a["quat_r"][0]),ff(a["quat_r"][1]),ff(a["quat_r"][2]),ff(a["quat_r"][3])),
				  btVector3(ff(a["xyz_r"][0]),ff(a["xyz_r"][1]),ff(a["xyz_r"][2])));
    out[i].leftGrip = ff(a["grip_l"]);
    out[i].rightGrip = ff(a["grip_r"]);
    out[i].leftGrab = ii(a["grab_l"]);
    out[i].rightGrab = ii(a["grab_r"]);

    for (int j=0; j < N_CTRL_PTS; j++) {
      out[i].ctrlPts.push_back(btVector3(ff(a["rope"][j][0]),ff(a["rope"][j][1]),ff(a["rope"][j][2])));
    }
  }
  return out;
}


