#include "utils/config.h"
#include "storage.h"
#include "clouds/utils_pcl.h"
#include "simulation/logging.h"

#include "clothscene.h" // not really necessary, just for SetCommonConfig
#include "clouds/cloud_ops.h"
#include <pcl/registration/icp.h>

// This program reads an input point cloud and ranks the cloth states
// in the storage by similarity between their corresponding simulated point clouds
// and the given input cloud.

struct MatchObsConfig : Config {
    static string root;
    static string inputcloud;
    static float downsampleSize;
    MatchObsConfig() : Config() {
        params.push_back(new Parameter<string>("root", &root, "data root directory"));
        params.push_back(new Parameter<string>("inputcloud", &inputcloud, "path to input point cloud (.pcd)"));
        params.push_back(new Parameter<float>("downsampleSize", &downsampleSize, "downsampling size for both the input cloud and clouds in the database (pass in negative value for no downsampling)"));
    }
};
string MatchObsConfig::root = "";
string MatchObsConfig::inputcloud = "";
float MatchObsConfig::downsampleSize = -1; // no downsampling

// adapted from pcl::Registration::getFitnessScore()
static double getFitnessScore(ColorCloudPtr input, ColorCloudPtr target, double max_range=std::numeric_limits<double>::max()) {
  double fitness_score = 0.0;

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  pcl::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBA>);
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

static double calcCloudSimilarity(ColorCloudPtr input, ColorCloudPtr target) {
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputCloud(input);
    icp.setInputTarget(target);
    ColorCloudPtr aligned(new ColorCloud);
    icp.align(*aligned);
    double fitness_inToTarget = icp.getFitnessScore();
    double fitness_targetToIn = getFitnessScore(target, aligned);
    return fitness_inToTarget + fitness_targetToIn;
}

static void run(const vector<fs::path> &dbcloudpaths, const fs::path &inputcloudpath) {
    ColorCloudPtr inputcloud = readPCD(inputcloudpath.string());
    if (MatchObsConfig::downsampleSize > 0)
        inputcloud = downsampleCloud(inputcloud, MatchObsConfig::downsampleSize*METERS);

    // calculate similarities between dbclouds and the input cloud
    vector<pair<double, Storage::ID> > similarities;
    similarities.resize(dbcloudpaths.size());

    #pragma omp parallel for shared(similarities)
    for (int i = 0; i < dbcloudpaths.size(); ++i) {
        cout << "num threads: " << omp_get_num_threads() << endl;
        ColorCloudPtr dbcloud = readPCD(dbcloudpaths[i].string());
        if (MatchObsConfig::downsampleSize > 0) {
            LOG_TRACE("downsampling");
            dbcloud = downsampleCloud(dbcloud, MatchObsConfig::downsampleSize*METERS);
        }

        double sim = calcCloudSimilarity(dbcloud, inputcloud);

        Storage::ID id = Storage::idFromCloudPath(dbcloudpaths[i].string());
        similarities[i] = make_pair(sim, id);
    }

    sort(similarities.begin(), similarities.end());

    for (int i = 0; i < similarities.size(); ++i)
        cout << similarities[i].first << '\t' << similarities[i].second << endl;
}

int main(int argc, char *argv[]) {
    SetCommonConfig();
    LoggingInit();

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(MatchObsConfig());
    parser.read(argc, argv);

    fs::path inputcloud = MatchObsConfig::inputcloud;
    if (!fs::exists(inputcloud)) {
        LOG_ERROR("input cloud path " << MatchObsConfig::inputcloud << " does not exist");
        exit(1);
    }

    vector<fs::path> dbcloudpaths;
    Storage::listCloudFiles(MatchObsConfig::root, dbcloudpaths);
    sort(dbcloudpaths.begin(), dbcloudpaths.end());

    run(dbcloudpaths, inputcloud);

    return 0;
}
