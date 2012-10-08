#include "simulation/simplescene.h"
#include "collision_map_tools.h"
#include "robots/pr2.h"
#include "clouds/cloud_ops.h"
#include "simulation/simulation_fwd.h"
#include "simulation/bullet_io.h"

struct LocalConfig : Config {

//  static bool printJoint;
//  static bool printCart;
  static string loadCloud;
  static string loadEnv;

  LocalConfig() : Config() {
//    params.push_back(new Parameter<bool>("printJoint", &printJoint, "print joints"));
//    params.push_back(new Parameter<bool>("printCart", &printCart, "print cart"));
    params.push_back(new Parameter<string>("loadCLoud", &loadCloud, "load cloud"));
    params.push_back(new Parameter<string>("loadEnv", &loadEnv, "load env"));
  }
};
//bool LocalConfig::printJoint=0;
//bool LocalConfig::printCart=0;
string LocalConfig::loadCloud="";
string LocalConfig::loadEnv="";

class JointPrinter {
public:
  RaveRobotObject::Manipulator::Ptr m_left, m_right;
  JointPrinter(RaveRobotObject::Manipulator::Ptr left, RaveRobotObject::Manipulator::Ptr right) :
    m_left(left), m_right(right) {}
  void doit() {
    cout << "left: " << m_left->getDOFValues() << " right: " << m_right->getDOFValues() << endl;
  }
};

class CartPrinter {
public:
  RaveRobotObject::Manipulator::Ptr m_left, m_right;
  CartPrinter(RaveRobotObject::Manipulator::Ptr left, RaveRobotObject::Manipulator::Ptr right) :
    m_left(left), m_right(right) {}
  void doit() {
    cout << "left: " << m_left->getTransform() << "right: " << m_right->getTransform() << endl;
  }
};



int main(int argc, char* argv[]) {
    Parser parser;
    BulletConfig::internalTimeStep=0;
    GeneralConfig::scale = 1;
    parser.addGroup(GeneralConfig());
    parser.addGroup(LocalConfig());
    parser.read(argc, argv);
    assert((LocalConfig::loadCloud=="") ^ (LocalConfig::loadEnv == ""));

    Scene scene;



    if (LocalConfig::loadEnv != "") {
      Load(scene.env, scene.rave, LocalConfig::loadEnv);
    }
    else if (LocalConfig::loadCloud != "") {
      ColorCloudPtr cloud = readPCD(LocalConfig::loadCloud);
      cloud = downsampleCloud(cloud, .02);
      CollisionBoxes::Ptr collisionBoxes = collisionBoxesFromPointCloud(cloud, .02);
      scene.env->add(collisionBoxes);
      Load(scene.env, scene.rave, "robots/pr2-beta-static/zae");
    }


    RaveRobotObjectPtr  pr2 = getRobotByName(scene.env, scene.rave, "pr2");
    if (!pr2) pr2 = getRobotByName(scene.env, scene.rave, "PR2");
    assert (pr2);

    PR2Manager pr2m(scene);
    assert (pr2m.pr2Right);

    JointPrinter jp(pr2m.pr2Left, pr2m.pr2Right);
    CartPrinter cp(pr2m.pr2Left, pr2m.pr2Right);
    scene.addVoidKeyCallback('j',boost::bind(&JointPrinter::doit, &jp), "print joints");
    scene.addVoidKeyCallback('c',boost::bind(&CartPrinter::doit, &cp), "print cart");

    scene.startViewer();
    while (true) {
      scene.step(0);
      sleep(.05);
    }
}
