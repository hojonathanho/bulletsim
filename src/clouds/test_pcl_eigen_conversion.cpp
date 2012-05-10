#include "utils_pcl.h"
#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <boost/foreach.hpp>
using namespace std;
using namespace pcl;
using namespace Eigen;

// typedef Matrix<uint8_t, Dynamic, Dynamic> MatrixXu;
// typedef unsigned char uint8_t;
typedef Matrix<uint8_t,Dynamic,Dynamic> MatrixXu;

int main() {
       
    ColorCloudPtr cloud = readPCD("/home/joschu/cpp/clouds/test.pcd");
    MatrixXf xyz = cloud->getMatrixXfMap(3,8,0);
    cout << xyz.transpose() << endl;
    cout << xyz.rows() << " " << xyz.cols() << endl;
    
    MatrixXf bgr = cloud->getMatrixXfMap(1,8,4);
    
    MatrixXu bgr1 = Map<MatrixXu>(reinterpret_cast<uint8_t*>(bgr.data()), 4, bgr.cols());
    MatrixXi bgr2 = bgr1.cast<int>();
    // cout << bgr2.transpose() << endl;
    
    BOOST_FOREACH(ColorPoint& pt, *cloud) printf("%i %i %i\n", pt.r, pt.g, pt.b);
    
    for (int i=0; i < bgr2.cols(); i++) {        
        ColorPoint& pt = cloud->at(i);
        printf("%i %i %i, %i %i %i\n",pt.b, pt.g, pt.r, bgr2(0,i),bgr2(1,i),bgr2(2,i));        
    }
    
    
    // Matrix<uint8_t> bgr1 = Map<  
    // cout << xyz.transpose() << endl;
    // cout << xyz.rows() << " " << xyz.cols() << endl;
    
    
}