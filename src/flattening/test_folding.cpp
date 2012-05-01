#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/plotting.h"
#include "simulation/util.h"
#include "simulation/logging.h"
#include <cstdlib>

#include "edges.h"
#include "facepicker.h"
#include "folding.h"
#include "config_flattening.h"

class CustomScene : public Scene {
    Cloth::Ptr cloth;
    PlotAxes::Ptr leftManipAxes, rightManipAxes;

    PlotPoints::Ptr foldnodeplot;
    PlotPoints::Ptr tofoldplot;
    PlotPoints::Ptr destplot;
    PlotLines::Ptr folddirplot;
    PlotSpheres::Ptr pickplot;
    SoftBodyFacePicker::Ptr facepicker;

    btTransform tableTrans;
    btVector3 tableExtents;

    vector<int> foldnodes;

    btSoftBody::Node *pickedNode, *pickedNode2;
    bool pickToggle;
    void pickCallback(int faceidx) {
        // we have to pick 1 of the 3 nodes on the face
        // just choose one that is a foldnode
        btSoftBody::Node *&toPick = pickToggle ? pickedNode : pickedNode2;
        pickToggle = !pickToggle;
        for (int i = 0; i < 3; ++i) {
            int n = cloth->psb()->m_faces[faceidx].m_n[i] - &cloth->psb()->m_nodes[0];
            if (std::find(foldnodes.begin(), foldnodes.end(), n) != foldnodes.end()) {
                toPick = cloth->psb()->m_faces[faceidx].m_n[i];
                return;
            }
        }
        toPick = cloth->psb()->m_faces[faceidx].m_n[0]; // arbitrary
    }

    void drawPick() {
        osg::ref_ptr<osg::Vec3Array> centers(new osg::Vec3Array);
        osg::ref_ptr<osg::Vec4Array> cols(new osg::Vec4Array);
        std::vector<float> radii;
        if (pickedNode) {
            centers->push_back(osg::Vec3(pickedNode->m_x.x(), pickedNode->m_x.y(), pickedNode->m_x.z()));
            cols->push_back(osg::Vec4(1, 0, 0, 0.5));
            radii.push_back(0.01*METERS);
        }
        if (pickedNode2) {
            centers->push_back(osg::Vec3(pickedNode2->m_x.x(), pickedNode2->m_x.y(), pickedNode2->m_x.z()));
            cols->push_back(osg::Vec4(0, 1, 0, 0.5));
            radii.push_back(0.01*METERS);
        }
        pickplot->plot(centers, cols, radii);
    }

    void doFold() {
        if (pickedNode && pickedNode2)
            Folding::foldClothAlongLine(*this, *cloth, pickedNode->m_x, pickedNode2->m_x);
    }

    void doRandomFolds(int n) {
        srand(time(NULL));
        for (int i = 0; i < n; ++i) {
            int idx1 = rand() % cloth->psb()->m_nodes.size();
            const btVector3 &p1 = cloth->psb()->m_nodes[idx1].m_x;
            int idx2 = rand() % cloth->psb()->m_nodes.size();
            const btVector3 &p2 = cloth->psb()->m_nodes[idx2].m_x;
            Folding::foldClothAlongLine(*this, *cloth, p1, p2);
        }
    }

    void pickUpAndDrop() {
        Folding::pickUpAndDrop(*this, *cloth);
    }

    void markNodesToFold() {
        if (!pickedNode || !pickedNode2) return;

        vector<btVector3> pts; vector<btVector4> colors;
        btSoftBody *psb = cloth->psb();
        btVector3 line = pickedNode2->m_x - pickedNode->m_x; line.setZ(0); line.normalize();
        for (int i = 0; i < psb->m_nodes.size(); ++i) {
            const btVector3 &x = psb->m_nodes[i].m_x;
            btVector3 ptline = x - pickedNode->m_x; ptline.setZ(0);
            if (line.cross(ptline).z() <= 0) continue;

            pts.push_back(x); colors.push_back(btVector4(0, 0, 1, 1));
        }

        tofoldplot->setPoints(pts, colors);
    }

    void markFolds() {
        cloth->updateAccel();
        foldnodes.clear();
        calcFoldNodes(*cloth, foldnodes);

        vector<btVector3> pts;
        vector<btVector4> colors;
        for (int i = 0; i < foldnodes.size(); ++i) {
            pts.push_back(cloth->psb()->m_nodes[foldnodes[i]].m_x);
            colors.push_back(btVector4(0, 0, 0, 1));
        }
        foldnodeplot->setPoints(pts, colors);

        pts.clear();
        for (int i = 0; i < foldnodes.size(); ++i) {
            btVector3 dir = calcFoldLineDir(*cloth, foldnodes[i], foldnodes);
            btVector3 c = cloth->psb()->m_nodes[foldnodes[i]].m_x;
            btScalar l = 0.01*METERS;
            pts.push_back(c - l*dir); pts.push_back(c + l*dir);
        }
        folddirplot->setPoints(pts);
    }

    void saveCloth() {
        cloth->saveToFile("currstate.cloth");
    }

    void loadCloth() {
        if (cloth) {
            env->remove(cloth);
            cloth.reset();
        }
        cloth = Cloth::createFromFile(env->bullet->softBodyWorldInfo, "currstate.cloth");
        env->add(cloth);
    }

public:
    void run() {
        foldnodeplot.reset(new PlotPoints());
        env->add(foldnodeplot);
        folddirplot.reset(new PlotLines(3));
        env->add(folddirplot);
        pickplot.reset(new PlotSpheres());
        env->add(pickplot);
        tofoldplot.reset(new PlotPoints());
        env->add(tofoldplot);
        destplot.reset(new PlotPoints());
        env->add(destplot);
        pickedNode = pickedNode2 = NULL;

        // create the table
        const float table_height = .5;
        const float table_thickness = .05;
        tableExtents = GeneralConfig::scale * btVector3(.75,.75,table_thickness/2);
        tableTrans = btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.8, 0, table_height-table_thickness/2));
        BoxObject::Ptr table(new BoxObject(0, tableExtents, tableTrans));
        table->rigidBody->setFriction(0.1);
        env->add(table);
        cout << "table margin: " << table->rigidBody->getCollisionShape()->getMargin() << endl;

        const int resx = 45, resy = 31;
//        const btScalar lenx = GeneralConfig::scale * 0.7, leny = GeneralConfig::scale * 0.5;
        const btScalar lenx = GeneralConfig::scale * 0.7/2, leny = GeneralConfig::scale * 0.5/2;
//        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.5, 0, table_height+0.01);
        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.7, 0.1, table_height+0.01);
//        cloth = makeSelfCollidingTowel(clothcenter, lenx, leny, resx, resy, env->bullet->softBodyWorldInfo);
        cloth.reset(new Cloth(resx, resy, lenx, leny, clothcenter, env->bullet->softBodyWorldInfo));
        env->add(cloth);

        facepicker.reset(new SoftBodyFacePicker(*this, viewer.getCamera(), cloth->softBody.get()));
        facepicker->setPickCallback(boost::bind(&CustomScene::pickCallback, this, _1));

        addVoidKeyCallback('f', boost::bind(&CustomScene::doFold, this));
        addVoidKeyCallback('g', boost::bind(&CustomScene::doRandomFolds, this, 3));
        addVoidKeyCallback('h', boost::bind(&CustomScene::pickUpAndDrop, this));
        addVoidKeyCallback('s', boost::bind(&CustomScene::saveCloth, this));
        addVoidKeyCallback('l', boost::bind(&CustomScene::loadCloth, this));

        addPreDrawCallback(boost::bind(&CustomScene::markFolds, this));
        addPreDrawCallback(boost::bind(&CustomScene::drawPick, this));
        addPreDrawCallback(boost::bind(&CustomScene::markNodesToFold, this));

        startViewer();
        setSyncTime(false);
        startFixedTimestepLoop(BulletConfig::dt);
    }
};

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    LoggingInit();

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(FlatteningConfig());
    parser.addGroup(FoldingConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
