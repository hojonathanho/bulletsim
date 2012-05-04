#include "clothscene.h"
#include "storage.h"

#include "perception/fake_kinect.h"
#include "perception/utils_perception.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

struct GenCloudConfig : Config {
    static string in, out;
    static float camDistFromCloth;
    GenCloudConfig() : Config() {
        params.push_back(new Parameter<string>("in", &in, "cloth state path (.cloth or .cloth.gz)"));
        params.push_back(new Parameter<string>("out", &out, "output path (.pcd)"));
        params.push_back(new Parameter<float>("camDistFromCloth", &camDistFromCloth, "cam distance from cloth"));
    }
};
string GenCloudConfig::in = "";
string GenCloudConfig::out = "";
float GenCloudConfig::camDistFromCloth = 0.5;

class CloudGenerator {
    Scene &scene; Cloth &cloth;
    btScalar minx, maxx, miny, maxy, minz, maxz;
    vector<btTransform> perspectives;

public:
    CloudGenerator(Scene &scene_, Cloth &cloth_) : scene(scene_), cloth(cloth_) {
        // calculate bounds of cloth
        minx = SIMD_INFINITY; maxx = -SIMD_INFINITY;
        miny = SIMD_INFINITY; maxy = -SIMD_INFINITY;
        minz = SIMD_INFINITY; maxz = -SIMD_INFINITY;
        for (int i = 0; i < cloth.psb()->m_nodes.size(); ++i) {
            const btVector3 &p = cloth.psb()->m_nodes[i].m_x;
            minx = min(minx, p.x()); maxx = max(maxx, p.x());
            miny = min(miny, p.y()); maxy = max(maxy, p.y());
            minz = min(minz, p.z()); maxz = max(maxz, p.z());
        }
    }

    void addPerspective(const btTransform &t) { perspectives.push_back(t); }

    ColorCloudPtr genCloud() {
        CoordinateTransformer CT(btTransform::getIdentity());

        ColorCloudPtr totalcloud;

        for (int c = 0; c < perspectives.size(); ++c) {
            CT.reset(perspectives[c]);

            FakeKinect fk(scene.env->osg, CT.worldFromCamEigen, false);
            ColorCloudPtr cloud1 = fk.snapshot();

            // filter out everything outside of the bounds of the cloth
            ColorCloudPtr cloud2(new ColorCloud(cloud1->width, cloud1->height));
            pcl::PassThrough<pcl::PointXYZRGBA> pass;

            pass.setInputCloud(cloud1);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(minx, maxx);
            pass.filter(*cloud2);

            pass.setInputCloud(cloud2);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(miny, maxy);
            pass.filter(*cloud1);

            pass.setInputCloud(cloud1);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(minz, maxz);
            pass.filter(*cloud2);

            if (!totalcloud)
                totalcloud.reset(new ColorCloud(cloud1->width, cloud1->height));
            *totalcloud += *cloud2;
        }

        return totalcloud;
    }

    void genCloud(const string &out) {
        pcl::io::savePCDFileBinary(GenCloudConfig::out, *genCloud());
    }
};

static btTransform calcTrans(const btVector3 &target, const btVector3 &camoffset, btScalar distfromtarget) {
    btVector3 camcenter = target + camoffset.normalized()*distfromtarget;

    btVector3 v1 = camoffset - btVector3(0, 0, camoffset.z());
    btVector3 v2 = btVector3(1, 0, 0);
    btVector3 v3 = v1.cross(v2);
    btScalar zangle;
    if (btFuzzyZero(v3.length())) zangle = 0;
    else zangle = v1.angle(v2);
    btQuaternion zrot(btVector3(0, 0, v1.cross(v2).z() < 0 ? 1 : -1), zangle);
    btQuaternion yrot(btVector3(0, 1, 0), camoffset.angle(btVector3(0, 0, 1)));
    btQuaternion xrot(btVector3(1, 0, 0), M_PI);

    return btTransform(zrot * yrot * xrot, camcenter/METERS);
}

class CustomScene : public ClothScene {
    void genPerspectives(CloudGenerator &g, const btVector3 &center, btScalar zAngle) {
        for (float a = 0; a < 5; ++a) {
            float angle = 2*M_PI/5*a;
            btTransform T = calcTrans(center,
                    btVector3(cos(angle), sin(angle), tan(zAngle)),
                    GenCloudConfig::camDistFromCloth*METERS);
            g.addPerspective(T);
        }
    }

public:
    void run() {
        setupScene();

        if (!fs::exists(GenCloudConfig::in)) {
            LOG_ERROR("input " << GenCloudConfig::in << " does not exist");
            exit(1);
        }
        cloth = Storage::loadCloth(GenCloudConfig::in, env->bullet->softBodyWorldInfo);
        if (!cloth->fullValidCheck()) {
            LOG_ERROR("cloth exploded");
            exit(1);
        }
        env->add(cloth);

        // remove everything from osg except the cloth
        for (int i = env->osg->root->getNumChildren() - 1; i >= 0; --i)
            if (env->osg->root->getChild(i) != cloth->getOSGNode())
                env->osg->root->removeChild(i);

        CloudGenerator g(*this, *cloth);

        btVector3 clothcenter = cloth->centerPoint();

        // generate views from the sides
        genPerspectives(g, clothcenter, M_PI/3);
        genPerspectives(g, clothcenter, M_PI/10);

        // generate view from the top
        btTransform T = calcTrans(clothcenter,
                btVector3(0, 0, 1),
                GenCloudConfig::camDistFromCloth*METERS);
        g.addPerspective(T);

        startViewer();
        g.genCloud(GenCloudConfig::out);
    };
};

int main(int argc, char *argv[]) {
    SetCommonConfig();
    SceneConfig::enableRobot = false;

    LoggingInit();

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(GenCloudConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
