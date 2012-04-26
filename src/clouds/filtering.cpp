#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "utils/my_assert.h"
#include "utils_pcl.h"
#include <iostream>

using namespace std;
using namespace cv;
using namespace pcl;
using boost::shared_ptr;
using namespace Eigen;


