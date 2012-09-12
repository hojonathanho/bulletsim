#include "simulation/simplescene.h"
#include "collision_map_tools.h"
#include "robots/pr2.h"
#include "clouds/cloud_ops.h"
class JointPrinter {
public:
  RaveRobotObject::Manipulator::Ptr m_left, m_right;
  JointPrinter(RaveRobotObject::Manipulator::Ptr left, RaveRobotObject::Manipulator::Ptr right) :
    m_left(left), m_right(right) {}
  void doit() {
    cout << "left: " << m_left->getDOFValues() << "right: " << m_right->getDOFValues() << endl;
  }
};

int main(int argc, char* argv[]) {
    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.read(argc, argv);

    Scene scene;
    PR2Manager pr2m(scene);
    RaveRobotObject::Ptr pr2 = pr2m.pr2;
    RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;
    RaveRobotObject::Manipulator::Ptr larm = pr2m.pr2Left;
    JointPrinter jp(larm, rarm);
    scene.addPreDrawCallback(boost::bind(&JointPrinter::doit, &jp));

    pr2m.pr2->setDOFValues(vector<int>(1,12), vector<double>(1,.287));


    ColorCloudPtr cloud = readPCD("/home/joschu/Data/scp/three_objs.pcd");
    cloud = downsampleCloud(cloud, .02);
    CollisionBoxes::Ptr collisionBoxes = collisionBoxesFromPointCloud(cloud, .02);
    scene.env->add(collisionBoxes);


    scene.startViewer();
    scene.startFixedTimestepLoop(.05);
}
