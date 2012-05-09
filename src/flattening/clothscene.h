#ifndef _FL_CLOTHSCENE_H_
#define _FL_CLOTHSCENE_H_

#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/simplescene.h"
#include "utils/logging.h"
#include "cloth.h"

// a scene common to all the cloth flattening programs
// (gives a standardized cloth location, table, etc)
class ClothScene : public Scene {
protected:
    Cloth::Ptr cloth;
    btTransform tableTrans;
    btVector3 tableExtents;

    static const float table_height = .5;
    static const float table_thickness = .05;

    void setupScene();
    void initStandardCloth();

public:

};

void SetCommonConfig();

#endif // _FL_CLOTHSCENE_H_
