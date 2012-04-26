#include "clouds/utils_pcl.h"
#include "clouds/preprocessing.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils/vector_io.h"
#include "filtering.h"
#include "cloud_ops.h"
#include "perception/utils_perception.h" // XXX. just for inline stuff
#include "utils/conversions.h"

#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

using namespace Eigen;
using namespace std;

Affine3f getCamToWorldFromTable(const vector<Vector3f>& corners) {
  Vector3f newY = corners[1] - corners[0];
  Vector3f newX = corners[3] - corners[0];
  Vector3f newZ = newX.cross(newY);
  if (newZ.z() > 0) {
    newZ *= -1;
    newX *= -1;
  }
  newX.normalize(); newY.normalize(); newZ.normalize();
  Matrix3f rotWorldToCam;
  rotWorldToCam.col(0) = newX;
  rotWorldToCam.col(1) = newY;
  rotWorldToCam.col(2) = newZ;
  Matrix3f rotCamToWorld = rotWorldToCam.inverse();
  Affine3f r;
  r = rotCamToWorld;
  float tz = 1-(rotCamToWorld*corners[0]).z();
  Translation3f t(0,0,tz);
  Affine3f out;
  out = r*t;
  return out;
}


VectorXb getPointsOnTable(ColorCloudPtr cloudCam, const Affine3f& camToWorld, const MatrixXf& tableWorld, float below, float above) {
    
  Matrix3f rotation;
  rotation = camToWorld.linear();
    
  Eigen::MatrixXf ptsCam = toEigenMatrix(cloudCam);
  Eigen::ArrayXXf ptsWorld = ptsCam * rotation.transpose();
    
  float minX = tableWorld.col(0).minCoeff();
  float maxX = tableWorld.col(0).maxCoeff();
  float minY = tableWorld.col(1).minCoeff();
  float maxY = tableWorld.col(1).maxCoeff();
  float minZ = tableWorld.col(2).minCoeff()-below;
  float maxZ = tableWorld.col(2).maxCoeff()+above;
    
  VectorXb out = (ptsWorld.col(0) >= minX)
    * (ptsWorld.col(0) <= maxX) 
    * (ptsWorld.col(1) >= minY) 
    * (ptsWorld.col(1) <= maxY) 
    * (ptsWorld.col(2) >= minZ)
    * (ptsWorld.col(2) <= maxZ);
        
  cout << out.sum() << endl;
  return out;
}

VectorXi getLabels(ColorCloudPtr cloud,  const MatrixXf& coeffs, const VectorXf& intercepts) {
  int nPts = cloud->size();
  MatrixXu bgr = toBGR(cloud);
  cv::Mat cvmat(cv::Size(nPts,1), CV_8UC3, bgr.data());
  cv::cvtColor(cvmat, cvmat, CV_BGR2Lab);
  Map<MatrixXu>lab(cvmat.data,nPts, 3);
  ArrayXXf feats(nPts,6);
  feats.block(0,0,nPts,3) = lab.cast<float>().array()/256.;
  feats.block(0,3,nPts,3) = feats.block(0,0,nPts,3).square();
    
  MatrixXf results = feats.matrix()*coeffs.transpose();
  MatrixXf results1 = results.rowwise() + intercepts.transpose();
  
  VectorXi out(nPts);
  for (int i=0; i < results.rows(); ++i) results1.row(i).maxCoeff(&out(i));
  return out;

}


TowelPreprocessor::TowelPreprocessor() {

  vector<Vector3f> cornersCam = toEigenVectors(floatMatFromFile(onceFile("table_corners.txt").string()));
  m_camToWorld = getCamToWorldFromTable(cornersCam);
        
  MatrixXf cornersCam1(4,3);
  for (int i=0; i < 4; i++) cornersCam1.row(i) = cornersCam[i];

  Matrix3f rotation; rotation = m_camToWorld.linear();
  m_cornersWorld = cornersCam1 * rotation.transpose();

  vector< vector<float> > coeffs_tmp = floatMatFromFile(EXPAND(BULLETSIM_DATA_DIR)"/pixel_classifiers/towel_on_wood/coeffs.txt");
  vector<float> intercepts_tmp = floatVecFromFile(EXPAND(BULLETSIM_DATA_DIR)"/pixel_classifiers/towel_on_wood/intercepts.txt");
  m_coeffs = toEigenMatrix(coeffs_tmp);
  m_intercepts = toVectorXf(intercepts_tmp);
    
}
  
  
ColorCloudPtr TowelPreprocessor::extractTowelPoints(ColorCloudPtr cloud) {
  VectorXb mask = getPointsOnTable(cloud, m_camToWorld, m_cornersWorld,.01,1);
  ColorCloudPtr cloudOnTable = maskCloud(cloud, mask);
  VectorXi labels = getLabels(cloudOnTable, m_coeffs, m_intercepts);
  ColorCloudPtr towelCloud = maskCloud(cloudOnTable, labels.array()==0);
  ColorCloudPtr downedTowelCloud = removeOutliers(downsampleCloud(towelCloud,.02),1.5);
  return downedTowelCloud;
}  

RopePreprocessor::RopePreprocessor() {

  vector<Vector3f> cornersCam = toEigenVectors(floatMatFromFile(onceFile("table_corners.txt").string()));
  m_camToWorld = getCamToWorldFromTable(cornersCam);
        
  MatrixXf cornersCam1(4,3);
  for (int i=0; i < 4; i++) cornersCam1.row(i) = cornersCam[i];

  Matrix3f rotation; rotation = m_camToWorld.linear();
  m_cornersWorld = cornersCam1 * rotation.transpose();

  vector< vector<float> > coeffs_tmp = floatMatFromFile(EXPAND(BULLETSIM_DATA_DIR)"/pixel_classifiers/rope_sdh_light/coeffs.txt");
  vector<float> intercepts_tmp = floatVecFromFile(EXPAND(BULLETSIM_DATA_DIR)"/pixel_classifiers/rope_sdh_light/intercepts.txt");
  m_coeffs = toEigenMatrix(coeffs_tmp);
  m_intercepts = toVectorXf(intercepts_tmp);
    
}
  
  
ColorCloudPtr RopePreprocessor::extractRopePoints(ColorCloudPtr cloud) {
  VectorXb mask = getPointsOnTable(cloud, m_camToWorld, m_cornersWorld,.01,1);
  ColorCloudPtr cloudOnTable = maskCloud(cloud, mask);
  VectorXi labels = getLabels(cloudOnTable, m_coeffs, m_intercepts);
  for (int i=0; i < labels.size(); ++i) cloudOnTable->points[i].r = labels(i)+1;
  ColorCloudPtr towelCloud = maskCloud(cloudOnTable, labels.array() <= 1);
  ColorCloudPtr downedTowelCloud = removeOutliers(downsampleCloud(towelCloud,.02),1.5);
  return downedTowelCloud;
}  
  

