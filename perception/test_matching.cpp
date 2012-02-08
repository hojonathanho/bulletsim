#include <Eigen/Dense>
#include <vector>
#include "testing.h"
#include "matching.h"
#include "dist_math.h"
#include "vector_io.h"
#include <my_assert.h>
#include <boost/timer.hpp>
using namespace Eigen;
using namespace std;

void test_matching_equal_size() {
boost::timer watch;
    
  const int nPts = 5;
  const int nDim = 3;

  MatrixXf x(nPts,nDim);
  x.setRandom();
  MatrixXf y = 10+x.array();
    
  MatrixXf costs = pairwiseSquareDist(x,y);
  VectorXi result = matchHardOneWay(costs);
  for (int i=0; i<result.size(); i++) ENSURE(result[i] == i);

  cout << watch.elapsed() << "elapsed" << endl;
}


void test_matching_different_size() {
  MatrixXf costs(2,1);
  costs << 1, 0;
  cout << matchHardMaximal(costs) << endl;
  cout << matchHardOneWay(costs.transpose()) << endl;
}

int main() {
  TEST_FUNC(test_matching_equal_size);
  TEST_FUNC(test_matching_different_size);
}
