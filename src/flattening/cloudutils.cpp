#include "cloudutils.h"

#include <pcl/kdtree/kdtree_flann.h>

// adapted from pcl::Registration::getFitnessScore()
double calcAlignmentScore(ColorCloudPtr input, ColorCloudPtr target, double max_range) {
  double fitness_score = 0.0;

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  pcl::KdTree<ColorPoint>::Ptr tree(new pcl::KdTreeFLANN<ColorPoint>);
  tree->setInputCloud(target);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input->points.size (); ++i)
  {
    Eigen::Vector4f p1 = Eigen::Vector4f (input->points[i].x,
                                          input->points[i].y,
                                          input->points[i].z, 0);
    // Find its nearest neighbor in the target
    tree->nearestKSearch (input->points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range)
      continue;

    Eigen::Vector4f p2 = Eigen::Vector4f (target->points[nn_indices[0]].x,
                                          target->points[nn_indices[0]].y,
                                          target->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}
