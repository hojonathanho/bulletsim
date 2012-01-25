#include <Eigen/Dense>
#include <vector>
#include "testing.h"
#include "matching.h"
#include "dist_math.h"
#include "vector_io.h"
using namespace Eigen;
using namespace std;

void test_matching_equal_size() {
  const int nPts = 10;
  const int nDim = 2;

  MatrixXf x(nPts,nDim);
  x.setRandom();
  MatrixXf y = 10+x.array();
    
  MatrixXf costs = pairwiseSquareDist(x,y);
  vector<int> result = matchHardOneWay(costs);
  for (int i=0; i<result.size(); i++) assert(result[i] == i);

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
