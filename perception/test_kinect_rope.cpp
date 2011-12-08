#include "get_table.h"
#include "clouds/geom.h"
#include "utils_perception.h"
#include "comm.h"

#include "userconfig.h"
#include "simplescene.h"
#include "util.h"
#include "rope.h"
#include "unistd.h"
#include "dist_math.h"
#include "plotting.h"

#include <json/json.h>
#include <math.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
//#include "pcl/common/eigen.h"
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using boost::shared_ptr;
using namespace Eigen;
using namespace util;
// make table and rope from kinect data

PlotLines::Ptr forcelines;
PlotPoints::Ptr targpts;

const float METERS = 10;

template<class T, class S>
vector<T> operator*(vector<T>& xs, S p) {
  vector<T> ys;
  ys.resize(xs.size());
  for (int i=0; i<xs.size(); i++) ys[i] = xs[i]*p;
  return ys;
}

struct EndInfo {
  btPoint2PointConstraint* ptp0;
  btPoint2PointConstraint* ptp1;
  bool active0;
  bool active1;
};



void verts2boxPars(const vector<btVector3>& verts, btVector3& halfExtents, btVector3& origin, btScalar thickness) {
  origin = (verts[0] + verts[2])/2;
  halfExtents = (verts[2] - verts[0]).absolute()/2;
  origin[2] -= thickness/2;
  halfExtents[2] = thickness/2;
}

Affine3f getCam2World(const vector<Vector3f>& tableVerts, Vector3f& normal){
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
  return Translation3f(0,0,tz)*cam2world_rot;
}

Affine3f scaling(float s) {
  Affine3f T;
  T = s*Matrix3f::Identity();
  return T;
}

void calcOptImpulses(const vector<btVector3>& ctrlPts, const vector<btVector3>& pointCloud, vector<btVector3>& forces, vector<bool>& occs) {
  MatrixXf ropePts = toEigens(ctrlPts);
  MatrixXf kinPts = toEigens(pointCloud);
  VectorXi indRope2Kin = argminAlongRows(pairwiseSquareDist(ropePts,kinPts));

  vector<btVector3> plot_srcs;
  vector<btVector3> plot_targs;

  int nRope = ctrlPts.size();
  forces.resize(nRope);
  cout << "sizes: " << occs.size() << " " << nRope << endl;
  assert(occs.size() == nRope);
  for (int i_rope=0; i_rope < nRope; i_rope++) {
    if (!occs[i_rope]) {
    btVector3 cloud_pt = pointCloud[indRope2Kin[i_rope]];
    btVector3 rope_pt = ctrlPts[i_rope];
    forces[i_rope] = (cloud_pt - rope_pt);
    plot_srcs.push_back(rope_pt);
    plot_targs.push_back(cloud_pt);}
  }

  VectorXi indKin2Rope = argminAlongRows(pairwiseSquareDist(kinPts,ropePts));
  for (int i_cloud=0; i_cloud < indKin2Rope.size(); i_cloud++) {
    int i_rope = indKin2Rope[i_cloud];
    btVector3 rope_pt = ctrlPts[i_rope];
    btVector3 cloud_pt = pointCloud[i_cloud];
    forces[i_rope] += cloud_pt - rope_pt;
    plot_srcs.push_back(rope_pt);
    plot_targs.push_back(cloud_pt);
  }

  forcelines->setPoints(plot_srcs,plot_targs);
  targpts->setPoints(pointCloud);

}

void applyImpulses(vector< shared_ptr<btRigidBody> > bodies, vector<btVector3> impulses, float multiplier) {
  for (int i=0; i<bodies.size(); i++) bodies[i]->applyCentralImpulse(impulses[i]*multiplier);
}

btVector3 jsonVec(Json::Value v) {
  return btVector3(v[0u].asDouble(), v[1u].asDouble(), v[2u].asDouble());
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

vector<bool> checkOccluded(const vector<btVector3>& xyzs_world, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,  Affine3f cam2world) {
   // for each point, get distance from camera, and get corresponding pixel
  // check if kinect detects something at that pixel but closer (or nan)
  Affine3f world2cam = cam2world.inverse() * scaling(1./METERS);
  Matrix<float,Dynamic,3> xyzs_world1 = toEigens(xyzs_world);
  MatrixXf xyzs_cam = (world2cam * xyzs_world1.transpose()).transpose();
  VectorXf dists = xyzs_cam.rowwise().norm();
  MatrixXi uvs = xyz2uv(xyzs_cam);

  // cout << "uvs:" << endl << uvs << endl;
  // cout << "dists:" << endl << dists << endl;
  DPRINT(xyzs_world.size());

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



void updateEnds(const string jsonfile, btDynamicsWorld* world, EndInfo& endinfo, const btTransform& cam2world) {

  std::stringstream buffer;
  std::ifstream infile(jsonfile.c_str());
  assert(!infile.fail());
  buffer << infile.rdbuf();
  Json::Reader reader;
  Json::Value root;
  bool parsedSuccess = reader.parse(buffer.str(), root, false);
  assert(parsedSuccess);

  btVector3 pivot0 = (cam2world*jsonVec(root["front"]["xyz"]))*METERS;
  btVector3 pivot1 = (cam2world*jsonVec(root["back"]["xyz"]))*METERS;

  cout << "end constraints" << endl;
  btVector3 vec = pivot0;
  cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;
  vec = pivot1;
  cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;

  endinfo.ptp0->setPivotB(pivot0);
  endinfo.ptp1->setPivotB(pivot1);

  bool newActive0 = root["front"]["seen"].asBool();
  bool newActive1 = root["back"]["seen"].asBool();
  bool oldActive0 = endinfo.active0;
  bool oldActive1 = endinfo.active1;

  //hack to deal with case where we get an erroneous constraint

  if (pivot0.getZ()/METERS > 1.1) newActive0 = false;
  if (pivot1.getZ()/METERS > 1.1) newActive1 = false;

  cout << newActive0 QQ newActive1 QQ oldActive0 QQ oldActive1 << endl;

  if (newActive0 && !oldActive0) world->addConstraint(endinfo.ptp0);
  if (newActive1 && !oldActive1) world->addConstraint(endinfo.ptp1);
  if (!newActive0 && oldActive0) world->removeConstraint(endinfo.ptp0);
  if (!newActive1 && oldActive1) world->removeConstraint(endinfo.ptp1);

  endinfo.active0 = newActive0;
  endinfo.active1 = newActive1;

}



  int main(int argc, char *argv[]) {


    string first_rope = comm::listenOnce("first_rope.txt");
    string first_ends = comm::listenOnce("first_ends.txt");

    comm::Listener rope_listener("rope");
    comm::Listener pcd_listener("pcds");
    comm::Listener ends_listener("ends");


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = readPCD(pcd_listener.next(MODE_NOUP));
    vector<Vector3f> corners_cam;
    Vector3f normal;
    getTable(cloud,corners_cam,normal);
    Affine3f cam2world = getCam2World(corners_cam,normal);
    btTransform bt_cam2world = toBTTransform(cam2world);
    cout << cam2world.matrix() << endl;
    // btTransform cam2world_bullet;
    // cam2world_bullet.setIdentity();

    vector<btVector3> corners_world = transform_btVectors(toBTs(corners_cam),bt_cam2world);

    BOOST_FOREACH(btVector3 vec, corners_world) cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;
    // vector< btVector3 > xyzs_cam;
    // read_btVectors(xyzs_cam,initial_ropefile); //rope vertices

    vector<btVector3> ropePts = transform_btVectors(read_btVectors(first_rope),bt_cam2world);
    //vector<btVector3> endPts = transform_btVectors(read_btVectors(first_ends),bt_cam2world);
    // btVector3 normal(abcd[0],abcd[1],abcd[2]);

    BOOST_FOREACH(btVector3 vec, ropePts) cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;

    // cout << "cam2world:" << endl;
    // print_matrix<btMatrix3x3,3> (cam2world);
    // btVector3 trans(0,0,1.5);
    // btTransform cam2world1(cam2world,trans);


    // vector<btVector3> verts_world;
    // for (int i=0; i<verts_cam.size(); i++) verts_world.push_back(cam2world1*btVector3(verts_cam[i][0],verts_cam[i][1],verts_cam[i][2]));

    // vector<btVector3> ctrlPts;
    // for (int i=0; i<xyzs_cam.size(); i+=2) {
    //   btVector3 xyz_world = cam2world1*btVector3(xyzs_cam[i][0],xyzs_cam[i][1],xyzs_cam[i][2]);
    //   ctrlPts.push_back(xyz_world);
    // }
    btVector3 halfExtents;
    btVector3 origin;

    verts2boxPars(corners_world,halfExtents,origin,.2);

    pcl::transformPointCloud(*cloud,*cloud,scaling(1*METERS)*cam2world);

    //   pcl::transformPointCloud(*cloud,*cloud,cam2world);

    // Affine3f T;
    // Matrix3f M;
    // for (int i=0; i<3; i++) for (int j=0; j<3; j++) M(i,j) = cam2world[i][j];
    // Vector3f eig_trans(trans.getX(),trans.getY(),trans.getZ());
    // T = Translation3f(eig_trans)*M;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr rectCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // BOOST_FOREACH(btVector3 w, corners_world) rectCloud->push_back(pcl::PointXYZ(w[0],w[1],w[2]));
    // pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
    // viewer.addPointCloud (cloud);
    // viewer.addPolygon<pcl::PointXYZ>(rectCloud,255,0,0);
    // viewer.spin();



    // // const string pcdfile = "../data/0003.pcd";
    // // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    // //   PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
    // //   return -1;
    // // }
    // // cout << "new cloud size: " << cloud->size();


    // /////////////// put stuf into scene //////////////////

    shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ropePts*METERS,.0075*METERS));
    shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin*METERS)));
    shared_ptr<BulletObject> table(new BoxObject(0,halfExtents*METERS,ms));
    Scene s;

    Config::read(argc, argv);
    CFG.scene.enableIK = CFG.scene.enableHaptics = CFG.scene.enableRobot = false;

    //s.manip->state.debugDraw = false;
    PlotPoints::Ptr plot(new PlotPoints());
    plot->setPoints(cloud);

    s.env->add(plot);
    s.env->add(table);
    s.env->add(ropePtr);
    forcelines.reset(new PlotLines(3));
    targpts.reset(new PlotPoints(20));
    s.env->add(forcelines);
    s.env->add(targpts);

    s.startViewer();
    s.step(0,1,.01);
    s.manip->toggleIdle();

    int nSegs = ropePtr->bodies.size();
    btPoint2PointConstraint* ptp0 = new btPoint2PointConstraint(*ropePtr->bodies[0].get(),(ropePts[0]-ropePts[1])/2);
    btPoint2PointConstraint* ptp1 = new btPoint2PointConstraint(*ropePtr->bodies[nSegs-1].get(),(ropePts[nSegs-1]-ropePts[nSegs-2])/2);

    vector<btVector3> forces(ropePts.size());
    vector<btVector3> centers;
    EndInfo endinfo = {ptp0,ptp1,false,false};

    for (int t=0; ; t++) {
      // for (int t=0;  t<pcdfiles.size(); t++) {
      string pcdfile = pcd_listener.next();
      cout << "--------------- reading file " << pcdfile << "------------" << endl;
      cloud = readPCD(pcdfile);
      pcl::transformPointCloud(*cloud,*cloud,scaling(1*METERS)*cam2world);

      string ropefile = rope_listener.next();
      string endfile = ends_listener.next();

      ropePts =   transform_btVectors(read_btVectors(ropefile),bt_cam2world);
      ropePts = ropePts * METERS;
      // cout<<"pts: " << endl;
      // BOOST_FOREACH(btVector3 pt, ropePts) cout << pt.getX() << " " << pt.getY() << " " << pt.getZ() << endl;

      updateEnds(endfile, s.env->bullet->dynamicsWorld, endinfo, bt_cam2world);


      // endPts =   transform_btVectors(read_btVectors(endfile),bt_cam2world);
      // endPts = endPts * METERS;
      // ptp1->setPivotB(endPts[0]);
      // ptp2->setPivotB(endPts[1]);

      plot->setPoints(cloud);
      ropePtr->getPts(centers);
      vector<bool> occs = checkOccluded(centers,cloud,cam2world);
      cout << "occlusions:";
      BOOST_FOREACH(bool occ, occs) cout << occ << " "; cout << endl;

      for (int i=0; i < 20; i++) {

	vector<bool> occs = checkOccluded(centers,cloud,cam2world);



	ropePtr->getPts(centers);

	calcOptImpulses(centers, ropePts, forces,occs);

	applyImpulses(ropePtr->bodies, forces, .25);

	s.step(.01,1,.01);



      


	cout << "iteration " << i  << endl;
	//     usleep(10*1000);
      }

      // s.manip->toggleIdle();
    }
  }

