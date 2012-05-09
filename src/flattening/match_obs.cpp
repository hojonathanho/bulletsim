#include "utils/config.h"
#include "utils/logging.h"
#include "storage.h"
#include "cloudutils.h"

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


static double calcCloudSimilarity(ColorCloudPtr input, ColorCloudPtr target) {
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputCloud(input);
    icp.setInputTarget(target);
    ColorCloudPtr aligned(new ColorCloud);
    icp.align(*aligned);
    double fitness_inToTarget = icp.getFitnessScore();
    double fitness_targetToIn = calcAlignmentScore(target, aligned);
    return fitness_inToTarget + fitness_targetToIn;
}

static void run(const vector<fs::path> &dbcloudpaths, const fs::path &inputcloudpath) {
    ColorCloudPtr inputcloud = readPCD(inputcloudpath.string());
    if (MatchObsConfig::downsampleSize > 0)
        inputcloud = downsampleCloud(inputcloud, MatchObsConfig::downsampleSize*METERS);

    // calculate similarities between dbclouds and the input cloud
    vector<pair<double, Storage::ID> > similarities;
    similarities.resize(dbcloudpaths.size());

    int count = 0, total = dbcloudpaths.size();
    #pragma omp parallel for schedule(dynamic) shared(similarities, count, total)
    for (int i = 0; i < total; ++i) {
        ColorCloudPtr dbcloud = readPCD(dbcloudpaths[i].string());
        if (MatchObsConfig::downsampleSize > 0) {
            LOG_TRACE("downsampling");
            dbcloud = downsampleCloud(dbcloud, MatchObsConfig::downsampleSize*METERS);
        }

        double sim = calcCloudSimilarity(dbcloud, inputcloud);

        Storage::ID id = Storage::idFromCloudPath(dbcloudpaths[i].string());
        similarities[i] = make_pair(sim, id);

        #pragma omp critical
        LOG_INFO("progress: " << ++count << "/" << total);
    }

    sort(similarities.begin(), similarities.end());

    for (int i = 0; i < similarities.size(); ++i)
        cout << similarities[i].first << '\t' << similarities[i].second << endl;
}

int main(int argc, char *argv[]) {
    SetCommonConfig();

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
