#include "simulation/simplescene.h"
#include "collision_map_tools.h"
#include "robots/robot_manager.h"
#include "clouds/cloud_ops.h"
#include "simulation/simulation_fwd.h"
#include "simulation/bullet_io.h"
#include "kinematics_utils.h"

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


int main(int argc, char* argv[]) {
    Parser parser;
    BulletConfig::internalTimeStep=0;
    GeneralConfig::scale = 1;
    parser.addGroup(GeneralConfig());
    parser.addGroup(LocalConfig());
    parser.read(argc, argv);
    assert((LocalConfig::loadCloud=="") || (LocalConfig::loadEnv == ""));

    Scene scene;



    if (LocalConfig::loadEnv != "") {
      Load(scene.env, scene.rave, LocalConfig::loadEnv);
    }
    else if (LocalConfig::loadCloud != "") {
      ColorCloudPtr cloud = readPCD(LocalConfig::loadCloud);
      cloud = downsampleCloud(cloud, .02);
      CollisionBoxes::Ptr collisionBoxes = collisionBoxesFromPointCloud(cloud, .02);
      scene.env->add(collisionBoxes);
      Load(scene.env, scene.rave, "robots/pr2-beta-static.zae");
    }
    else
      Load(scene.env, scene.rave, "robots/pr2-beta-static.zae");


    RaveRobotObjectPtr  pr2 = getRobotByName(scene.env, scene.rave, "pr2");
    if (!pr2) pr2 = getRobotByName(scene.env, scene.rave, "PR2");
    if (!pr2) pr2 = getRobotByName(scene.env, scene.rave, "BarrettWAM");
    assert (pr2);

    RobotManager pr2m(scene);
    assert (pr2m.botRight);

    ArmPrinter ap(pr2m.botLeft, pr2m.botRight);
    scene.addVoidKeyCallback('c',boost::bind(&ArmPrinter::printCarts, &ap), "print cart");
    scene.addVoidKeyCallback('j',boost::bind(&ArmPrinter::printJoints, &ap), "print joints");
    scene.addVoidKeyCallback('a', boost::bind(&ArmPrinter::printAll, &ap), "print all dofs");

    scene.startViewer();
    while (true) {
      scene.step(0);
      sleep(.05);
    }
}
