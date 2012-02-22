#include "utils_pcl.h"
#include "preprocessing.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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


VectorXb getPointsOnTable(ColorCloudPtr cloudCam, const Affine3f& camToWorld, const MatrixXf& tableCam, float below, float above) {
    
  Matrix3f rotation;
  rotation = camToWorld.linear();
    
  Eigen::MatrixXf ptsCam = toEigenMatrix(cloudCam);
  Eigen::ArrayXXf ptsWorld = ptsCam * rotation.transpose();
    
  Eigen::ArrayXXf tableWorld = tableCam * rotation.transpose();
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
  // cout << nPts << " points" << endl;
  // cout << cloud->width << " " << cloud->height << endl;
  MatrixXb bgr = toBGR(cloud);
  // cout << (float)cloud->at(0).r << " "  << (float)cloud->at(0).g << " "  << (float)cloud->at(0).b << endl;
  // cout << "bgr" << endl;
  // cout << bgr.block(0,0,10,3).cast<float>() << endl;
  // cout << "done" << endl;
  cv::Mat cvmat(cv::Size(nPts,1), CV_8UC3, bgr.data());
  //    cv2eigen(cvmat, bgr);
  // cout << "rgb in mat " << (int)cvmat.at<cv::Vec3b>(0,0)[0] << " " <<  (int)cvmat.at<cv::Vec3b>(0,0)[1] << " " << (int)cvmat.at<cv::Vec3b>(0,0)[2] << " " << endl;
  cv::cvtColor(cvmat, cvmat, CV_BGR2Lab);
  // cout << "lab in mat " << (int)cvmat.at<cv::Vec3b>(0,0)[0] << " " <<  (int)cvmat.at<cv::Vec3b>(0,0)[1] << " " << (int)cvmat.at<cv::Vec3b>(0,0)[2] << " " << endl;
  Map<MatrixXb>lab(cvmat.data,nPts, 3);
  //  eigen2cv(lab, cvmat);
  // cout << lab.block(0,0,10,3).cast<float>() << endl;
  ArrayXXf feats(nPts,6);
  feats.block(0,0,nPts,3) = lab.cast<float>().array()/256.;
  feats.block(0,3,nPts,3) = feats.block(0,0,nPts,3).square();
    
  MatrixXf results = feats.matrix()*coeffs.transpose();

  MatrixXf results1 = results.rowwise() + intercepts.transpose();
  // cout << feats.block(0,0,10,6).cast<float>() << endl;
  // cout << results.block(0,0,10,4).cast<float>() << endl;
  // cout << results1.block(0,0,10,5).cast<float>() << endl;
  VectorXi out(nPts);
  for (int i=0; i < results.rows(); ++i) results1.row(i).maxCoeff(&out(i));
  return out;

}

ColorCloudPtr labelCloud(ColorCloudPtr cloudCam, const MatrixXf& coeffs, const MatrixXf& tableCam, const Affine3f& camToWorld) {
  Eigen::MatrixXf pts = toEigenMatrix(cloudCam);
    
  // // transform point cloud
  // VectorXb pointsOnTable = getPointsOnTable(cloudCam, camToWorld, tableCam);
    
  // MatrixXb bgr = toBGR(pointsOnTable);
  // cv::Mat cvmat = cv::eigen2cv(bgr);
  // cv::cvtColor(cvmat, cvmat, CV_BGR2LAB);
    
  // Eigen::ArrayXXf lab;
  // cv::cv2eigen(cvmat, lab.block(0,0, nPts, 3));
  // lab.block(0,3, nPts,3) = lab.block(0,0,nPts,3).cwiseSquare();

  // results = lab.matrix() * coeffs;
    
  // get stuff on table
    
  // mat mult for svm
    
  // label stuff
    
  // downsample it
    
}
