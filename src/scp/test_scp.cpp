#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"

int main(int argc, char *argv[]) {
	GeneralConfig::scale = 20.;
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;
    SceneConfig::enableRobotCollision = false;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);


    Scene scene;
    PR2Manager pr2m(scene);

    RaveRobotObject::Ptr pr2 = pr2m.pr2;
    RaveRobotObject::Manipulator::Ptr manip = pr2->manip;
    btTransform trans = manip->getTransform();
    manip->moveByIK(trans);

    RobotBasePtr raveRobot = pr2->robot;
    raveRobot->GetName();

    const float table_height = .5;
    const float table_thickness = .05;

    BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1, 0, table_height-table_thickness/2))));
    table->setColor(0,0,1,1);
    scene.env->add(table);


    BoxObject::Ptr box(new BoxObject(1, GeneralConfig::scale * btVector3(.03, .03, .03),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.7, 0.188, table_height + 0.1/2))));
    box->setColor(1,0,0,1);
    scene.env->add(box);

    scene.startViewer();
    scene.startLoop();

}
