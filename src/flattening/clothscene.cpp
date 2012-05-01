#include "clothscene.h"

void SetCommonConfig() {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;
}

void ClothScene::setupScene() {
    // create the table
    tableExtents = GeneralConfig::scale * btVector3(.75,.75,table_thickness/2);
    tableTrans = btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.8, 0, table_height-table_thickness/2));
    BoxObject::Ptr table(new BoxObject(0, tableExtents, tableTrans));
    table->rigidBody->setFriction(0.1);
    env->add(table);
}

void ClothScene::initStandardCloth() {
    // create the cloth
    const int resx = 45, resy = 31;
    const btScalar lenx = GeneralConfig::scale * 0.7/2, leny = GeneralConfig::scale * 0.5/2;
    const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.7, 0.1, table_height+0.01);
    cloth.reset(new Cloth(resx, resy, lenx, leny, clothcenter, env->bullet->softBodyWorldInfo));
    env->add(cloth);
}
