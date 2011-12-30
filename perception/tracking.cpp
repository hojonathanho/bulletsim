#include "dist_math.h"
#include "tracking.h"
#include <pcl/point_types.h>
#include "utils_perception.h"


vector<bool> checkOccluded(const vector<btVector3>& xyzs_world, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  Affine3f cam2world) {
  // for each point, get distance from camera, and get corresponding pixel
  // check if kinect detects something at that pixel but closer (or nan)
  Affine3f world2cam = cam2world.inverse() * scaling(1./CFG2->scene.scale);
  Matrix<float,Dynamic,3> xyzs_world1 = toEigens(xyzs_world);
  MatrixXf xyzs_cam = (world2cam * xyzs_world1.transpose()).transpose();
  VectorXf dists = xyzs_cam.rowwise().norm();
  MatrixXi uvs = xyz2uv(xyzs_cam);

  // cout << "uvs:" << endl << uvs << endl;
  // cout << "dists:" << endl << dists << endl;

  vector<bool> out(xyzs_world.size());
  cout << "------------------:" << endl;
  for (int row=0; row<uvs.rows(); row++) {
    int u = uvs(row,0);
    int v = uvs(row,1);
    pcl::PointXYZRGB pt = cloud->at(v,u);
    float cloud_dist = (world2cam*Vector3f(pt.x,pt.y,pt.z)).norm();
    out[row] = !isfinite(cloud_dist) || cloud_dist < dists[row]-.04;
    // cout << cloud_dist QQ dists[row] QQ u QQ v << endl;
  }
  cout << "--------------------" << endl;

  return out;
}

MatrixXi xyz2uv(const Matrix<float,Dynamic,3>& xyz) {
  // http://www.pcl-users.org/Using-Kinect-with-PCL-How-to-project-a-3D-point-x-y-z-to-the-depth-rgb-image-and-how-to-unproject-a--td3164499.html

  VectorXf x = xyz.col(0);
  VectorXf y = xyz.col(1);
  VectorXf z = xyz.col(2);

  float cx = 320-.5;
  float cy = 240-.5;
  float f = 525;
  VectorXf v = f*(x.array() / z.array()) + cx;
  VectorXf u = f*(y.array() / z.array()) + cy;
  MatrixXi uv(u.rows(),2);
  uv.col(0) = u.cast<int>();
  uv.col(1) = v.cast<int>();
  return uv;
}



Affine3f getCam2World(const vector<Vector3f>& tableVerts, Vector3f& normal, float scale){
  Matrix3f world2cam_rot;
  Vector3f newY = tableVerts[1] - tableVerts[0];
  Vector3f newX = tableVerts[3] - tableVerts[0];
  Vector3f newZ = newX.cross(newY);
  newX.normalize(); newY.normalize(); newZ.normalize();
  world2cam_rot.col(0) = newX;
  world2cam_rot.col(1) = newY;
  world2cam_rot.col(2) = newZ;
  cout << "det: " << world2cam_rot.determinant() << endl;
  cout << "dot: " << newX.dot(newY) << endl;
  Matrix3f cam2world_rot = world2cam_rot.inverse();

  float tz = 1-(cam2world_rot*tableVerts[0])[2];
  return scaling(scale)*(Translation3f(0,0,tz)*cam2world_rot);
}

void calcOptImpulses(const vector<btVector3>& ctrlPts, const vector<btVector3>& pointCloud, const vector<btVector3>& oldCtrlPts, vector<btVector3>& forces, vector<bool>& occs) {
  MatrixXf ropePts = toEigens(ctrlPts);
  MatrixXf kinPts = toEigens(pointCloud);
  VectorXi indRope2Kin = argminAlongRows(pairwiseSquareDist(ropePts,kinPts));

  vector<btVector3> plot_srcs;
  vector<btVector3> plot_targs;

  float atob = CFG2->atob;
  float btoa = CFG2->btoa;
  float reg = CFG2->reg;

  int nRope = ctrlPts.size();
  forces.resize(nRope);
  //cout << "sizes: " << occs.size() << " " << nRope << endl;
  assert(occs.size() == nRope);
  for (int i_rope=0; i_rope < nRope; i_rope++) {
    if (!occs[i_rope]) {
      btVector3 cloud_pt = pointCloud[indRope2Kin[i_rope]];
      btVector3 rope_pt = ctrlPts[i_rope];
      forces[i_rope] = atob*(cloud_pt - rope_pt);
      plot_srcs.push_back(rope_pt);
      plot_targs.push_back(cloud_pt);
      forces[i_rope] += reg*(oldCtrlPts[i_rope] - rope_pt);
    }

  }

  VectorXi indKin2Rope = argminAlongRows(pairwiseSquareDist(kinPts,ropePts));
  for (int i_cloud=0; i_cloud < indKin2Rope.size(); i_cloud++) {
    int i_rope = indKin2Rope[i_cloud];
    btVector3 rope_pt = ctrlPts[i_rope];
    btVector3 cloud_pt = pointCloud[i_cloud];
    forces[i_rope] += btoa*(cloud_pt - rope_pt);
    plot_srcs.push_back(rope_pt);
    plot_targs.push_back(cloud_pt);
  }

  plots::forcelines->setPoints(plot_srcs,plot_targs);
  plots::targpts->setPoints(pointCloud);

}

void applyImpulses(vector< shared_ptr<btRigidBody> > bodies, vector<btVector3> impulses, float multiplier) {
  for (int i=0; i<bodies.size(); i++) bodies[i]->applyCentralImpulse(impulses[i]*multiplier);
}


void initTrackingPlots() {
  plots::forcelines.reset(new PlotLines(3));
  plots::forcelines->setDefaultColor(0,0,1,1);
  plots::targpts.reset(new PlotPoints(5));
  plots::targpts->setDefaultColor(0,0,1,.5);
}

MyConfigData::MyConfigData() {
  opts.add_options()
    OPT(atob, float, 1, "force multiplier from model to observation")
    OPT(btoa, float, 1, "force multiplier from observation to model")
    OPT(mult, float, 1, "overall force multiplier")
    OPT(reg, float, 1, "regularization factor to slow rope")
    OPT(angDamping, float, 1,"angular damping for rope")
    OPT(linDamping, float, .75,"linear damping for rope")
    OPT(nIter, int, 20, "num iterations")
    ;
}


PlotLines::Ptr plots::forcelines;
PlotPoints::Ptr plots::targpts;
