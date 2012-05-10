#include "storage.h"
#include "clothscene.h"

#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"

#include "simulation/plotting.h"

#include "perception/get_nodes.h"
#include "perception/optimization_forces.h"
#include "perception/config_perception.h"
#include "perception/utils_perception.h"
using namespace Eigen;

// This program locally optimizes a cloth state to match an input observation

struct ForceClothConfig : Config {
    static string inputCloth;
    static string cloudToMatch;
    static string outputCloth;
    static bool enableDisplay;
    static float trackingDownsampleSize;
    static int forceSteps;
    static float restTime;
    ForceClothConfig() : Config() {
        params.push_back(new Parameter<string>("inputCloth", &inputCloth, "input cloth file"));
        params.push_back(new Parameter<string>("cloudToMatch", &cloudToMatch, "observation to force the input cloth to"));
        params.push_back(new Parameter<string>("outputCloth", &outputCloth, "output cloth file"));
        params.push_back(new Parameter<bool>("enableDisplay", &enableDisplay, "show display"));
        params.push_back(new Parameter<float>("trackingDownsampleSize", &trackingDownsampleSize, "downsampling resolution for input cloud before doing tracking"));
        params.push_back(new Parameter<int>("forceSteps", &forceSteps, "steps to run tracking"));
        params.push_back(new Parameter<float>("restTime", &restTime, "rest time after tracking"));
    }
};
string ForceClothConfig::inputCloth;
string ForceClothConfig::cloudToMatch;
string ForceClothConfig::outputCloth;
bool ForceClothConfig::enableDisplay = false;
float ForceClothConfig::trackingDownsampleSize = 0.01; // 1 cm
int ForceClothConfig::forceSteps = 200;
float ForceClothConfig::restTime = 0.5; // 0.5 sec

static void forceClothToObs(Scene &scene, Cloth::Ptr cloth, ColorCloudPtr cloud) {
    btSoftBody *psb = cloth->psb();

    VectorXf pVis(psb->m_nodes.size()); pVis.setOnes(); // assume full visibility
    vector<btVector3> clothObsPts = toBulletVectors(cloud);

    for (int t = 0; t < ForceClothConfig::forceSteps; ++t) {
        vector<btVector3> clothEstPts = getNodes(cloth);
        SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(clothEstPts), toEigenMatrix(clothObsPts), pVis, TrackingConfig::sigB, TrackingConfig::outlierParam), TrackingConfig::cutoff);
        vector<btVector3> impulses = calcImpulsesSimple(clothEstPts, clothObsPts, corr, TrackingConfig::impulseSize);
        for (int i = 0; i < impulses.size(); ++i)
            psb->addForce(impulses[i], i);
        scene.step(DT);
    }
}

class CustomScene : public ClothScene {
    PlotPoints::Ptr cloudplot;

public:
    void run();
};

void CustomScene::run() {
    setupScene();

    cloth = Storage::loadCloth(ForceClothConfig::inputCloth, env->bullet->softBodyWorldInfo);
    env->add(cloth);
    cloth->setColor(1, 1, 1, 0.5);

    ColorCloudPtr cloud = readPCD(ForceClothConfig::cloudToMatch);
    cloud = downsampleCloud(cloud, ForceClothConfig::trackingDownsampleSize*METERS);

    if (ForceClothConfig::enableDisplay) {
        // plot the cloud
        cloudplot.reset(new PlotPoints(1));
        vector<btVector3> pts; vector<btVector4> colors;
        for (int i = 0; i < cloud->points.size(); ++i) {
            pts.push_back(btVector3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
            colors.push_back(btVector4(1, 0, 1, 1));
        }
        cloudplot->setPoints(pts, colors);
        env->add(cloudplot);

        startViewer();
    }

    forceClothToObs(*this, cloth, cloud);
    stepFor(DT, ForceClothConfig::restTime);

    if (cloth->fullValidCheck())
        cloth->saveToFile(ForceClothConfig::outputCloth);
    else
        LOG_ERROR("resulting cloth state is invalid");
}

int main(int argc, char *argv[]) {
    SetCommonConfig();
    SceneConfig::enableRobot = false;
    TrackingConfig::impulseSize = 50;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(TrackingConfig());
    parser.addGroup(ForceClothConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
