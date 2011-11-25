#include <vector>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

MatrixXf pairwiseSquareDist(MatrixXf x_m3, MatrixXf y_n3);
VectorXi argminAlongRows(MatrixXf d_mn);
