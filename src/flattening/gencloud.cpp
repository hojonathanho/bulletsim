#include "clothscene.h"
#include "storage.h"

#include "perception/fake_kinect.h"
#include "perception/utils_perception.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

struct GenCloudConfig : Config {
    static string in, out;
    static float camDistFromCloth;
    static float voxelSize;
    GenCloudConfig() : Config() {
        params.push_back(new Parameter<string>("in", &in, "cloth state path (.cloth or .cloth.gz)"));
        params.push_back(new Parameter<string>("out", &out, "output path (.pcd)"));
        params.push_back(new Parameter<float>("camDistFromCloth", &camDistFromCloth, "cam distance from cloth"));
        params.push_back(new Parameter<float>("voxelSize", &voxelSize, "voxel size for downsampling"));
    }
};
string GenCloudConfig::in = "";
string GenCloudConfig::out = "";
float GenCloudConfig::camDistFromCloth = 0.5;
//float GenCloudConfig::voxelSize = 0.01; // 1 cm
float GenCloudConfig::voxelSize = 0.001; // 1 mm

class CloudGenerator {
    Scene &scene; Cloth &cloth;
    vector<btTransform> perspectives;

public:
    CloudGenerator(Scene &scene_, Cloth &cloth_) : scene(scene_), cloth(cloth_) { }

    void addPerspective(const btTransform &t) { perspectives.push_back(t); }

    ColorCloudPtr genCloud() {
        // calculate bounds of cloth
        btScalar minx, maxx, miny, maxy, minz, maxz;
        minx = SIMD_INFINITY; maxx = -SIMD_INFINITY;
        miny = SIMD_INFINITY; maxy = -SIMD_INFINITY;
        minz = SIMD_INFINITY; maxz = -SIMD_INFINITY;
        for (int i = 0; i < cloth.psb()->m_nodes.size(); ++i) {
            const btVector3 &p = cloth.psb()->m_nodes[i].m_x;
            minx = min(minx, p.x()); maxx = max(maxx, p.x());
            miny = min(miny, p.y()); maxy = max(maxy, p.y());
            minz = min(minz, p.z()); maxz = max(maxz, p.z());
        }
        const Eigen::Vector4f cropmin(minx, miny, minz, 0);
        const Eigen::Vector4f cropmax(maxx, maxy, maxz, 0);

        // take a picture with the fake kinect from each perspective
        CoordinateTransformer CT(btTransform::getIdentity());
        ColorCloudPtr totalcloud;
        FakeKinect fk(scene.env->osg, CT.worldFromCamEigen, false);
        pcl::CropBox<ColorPoint> crop;
        for (int c = 0; c < perspectives.size(); ++c) {
            CT.reset(perspectives[c]);
            fk.setWorldFromCam(CT.worldFromCamEigen);
            ColorCloudPtr snapshot = fk.snapshot();

            // filter out everything outside of the bounds of the cloth
            ColorCloudPtr cropped(new ColorCloud(snapshot->width, snapshot->height));
            crop.setInputCloud(snapshot);
            crop.setMin(cropmin);
            crop.setMax(cropmax);
            crop.filter(*cropped);

            if (!totalcloud)
                totalcloud.reset(new ColorCloud(snapshot->width, snapshot->height));
            *totalcloud += *cropped;
        }

        // downsample the result (since we got points from many perspectives)
        ColorCloudPtr downsampled(new ColorCloud(totalcloud->width, totalcloud->height));
        pcl::VoxelGrid<ColorPoint> vgrid;
        vgrid.setInputCloud(totalcloud);
        float s = GenCloudConfig::voxelSize * GeneralConfig::scale;
        vgrid.setLeafSize(s, s, s);
        vgrid.filter(*downsampled);

        return downsampled;
    }

    void genCloud(const string &out) {
        pcl::io::savePCDFileBinary(GenCloudConfig::out, *genCloud());
    }
};

// calculates the transform to have the camera face the target
static btTransform calcCamTrans(const btVector3 &target, const btVector3 &camoffset, btScalar distfromtarget) {
    btVector3 camcenter = target + camoffset.normalized()*distfromtarget;

    btVector3 v1 = camoffset - btVector3(0, 0, camoffset.z());
    btVector3 v2 = btVector3(1, 0, 0);
    btVector3 v3 = v1.cross(v2);
    btScalar zangle = btFuzzyZero(v3.length()) ? 0 : v1.angle(v2);
    btQuaternion zrot(btVector3(0, 0, v1.cross(v2).z() < 0 ? 1 : -1), zangle);
    btQuaternion yrot(btVector3(0, 1, 0), camoffset.angle(btVector3(0, 0, 1)));
    btQuaternion xrot(btVector3(1, 0, 0), M_PI);

    return btTransform(zrot * yrot * xrot, camcenter/METERS);
}

static void genPerspectives(CloudGenerator &g, const btVector3 &center, btScalar zAngle) {
    for (float a = 0; a < 5; ++a) {
        float angle = 2*M_PI/5*a;
        btTransform T = calcCamTrans(center,
                btVector3(cos(angle), sin(angle), tan(zAngle)),
                GenCloudConfig::camDistFromCloth*METERS);
        g.addPerspective(T);
    }
}

class CustomScene : public ClothScene {
public:
    void run() {
        setupScene();

        // load the cloth
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

        step(0); // make sure all pre-draws are called

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
        btTransform T = calcCamTrans(clothcenter,
                btVector3(0, 0, 1),
                GenCloudConfig::camDistFromCloth*METERS);
        g.addPerspective(T);
        g.genCloud(GenCloudConfig::out);
    };
};

int main(int argc, char *argv[]) {
    SetCommonConfig();
    SceneConfig::enableRobot = false;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(GenCloudConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
