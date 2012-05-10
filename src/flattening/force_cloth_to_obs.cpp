#include "storage.h"
#include "clothscene.h"
#include "clouds/utils_pcl.h"
#include "simulation/plotting.h"

// This program locally optimizes a cloth state to match an input observation

struct ForceClothConfig : Config {
    static string inputCloth;
    static string cloudToMatch;
    static string outputCloth;
    static bool enableDisplay;
    ForceClothConfig() : Config() {
        params.push_back(new Parameter<string>("inputCloth", &inputCloth, "input cloth file"));
        params.push_back(new Parameter<string>("cloudToMatch", &cloudToMatch, "observation to force the input cloth to"));
        params.push_back(new Parameter<string>("outputCloth", &outputCloth, "output cloth file"));
        params.push_back(new Parameter<bool>("enableDisplay", &enableDisplay, "show display"));
    }
};
string ForceClothConfig::inputCloth;
string ForceClothConfig::cloudToMatch;
string ForceClothConfig::outputCloth;
bool ForceClothConfig::enableDisplay = false;

// for each cloth point, find nearest neighbor in cloud, and apply force
// in that direction
static void forceClothToObs(Scene &scene, PlotLines::Ptr forceplot, Cloth &cloth, ColorCloudPtr cloud) {
    btSoftBody *psb = cloth.psb();

    pcl::KdTree<ColorPoint>::Ptr tree(new pcl::KdTreeFLANN<ColorPoint>);
    tree->setInputCloud(cloud);
    vector<int> nn_indices(1);
    vector<float> nn_dists(1);

    const btScalar nodemass = psb->getTotalMass() / psb->m_nodes.size();
    const btScalar dt = BulletConfig::dt;
    const btScalar numSteps=1000;
    vector<btVector3> plotpts;
    for (int s = 0; s < numSteps; ++s) {
        plotpts.clear();
        cloth->updateAccel();
        for (int i = 0; i < cloud->points.size(); ++i) {
            // find cloth node nearest to cloud point
            cloth->kdtree->nearestKSearch(cloud->points[i], 1, nn_indices, nn_dists);
            const int idx = nn_indices[0];

            // apply force in that direction
            btVector3 cpt(cloud->points[i].x,
                          cloud->points[i].y,
                          cloud->points[i].z);
            btVector3 dir = cpt - psb->m_nodes[idx].m_x;
            btVector3 force = 2*nodemass/dt/dt/250 * dir;
            psb->addForce(force, clothnode);
            if (forceplot) {
                plotpts.push_back(node.m_x);
                plotpts.push_back(cpt);
            }
        }

/*
        for (int i = 0; i < psb->m_nodes.size(); ++i) {
            const btSoftBody::Node &node = psb->m_nodes[i];
            // find nearest neighbor to node in the cloud
            ColorPoint pt; pt.x = node.m_x.x(); pt.y = node.m_x.y(); pt.z = node.m_x.z();
            tree->nearestKSearch(pt, 1, nn_indices, nn_dists);
            btVector3 cpt(cloud->points[nn_indices[0]].x,
                          cloud->points[nn_indices[0]].y,
                          cloud->points[nn_indices[0]].z);
            // apply force in that direction
            btVector3 dir = (cpt - node.m_x).normalize();
            btVector3 force = 2*nodemass/dt/dt/250 * dir;
            psb->addForce(force, i);
            if (forceplot) {
                plotpts.push_back(node.m_x);
                plotpts.push_back(cpt);
            }
        }
        */
        if (forceplot) {
            forceplot->setPoints(plotpts);
        }
        scene.step(dt, 0, dt);
    }
    LOG_INFO("done");
}

class CustomScene : public ClothScene {
    PlotPoints::Ptr cloudplot;
    PlotLines::Ptr forceplot;

public:
    void run();
};

void CustomScene::run() {
    setupScene();

    cloth = Storage::loadCloth(ForceClothConfig::inputCloth, env->bullet->softBodyWorldInfo);
    env->add(cloth);

    ColorCloudPtr cloud = readPCD(ForceClothConfig::cloudToMatch);

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

        forceplot.reset(new PlotLines(1));
        env->add(forceplot);

        startViewer();
        forceClothToObs(*this, forceplot, *cloth, cloud);
        startFixedTimestepLoop(BulletConfig::dt);
    }
}

int main(int argc, char *argv[]) {
    SetCommonConfig();
    SceneConfig::enableRobot = false;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(ForceClothConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
