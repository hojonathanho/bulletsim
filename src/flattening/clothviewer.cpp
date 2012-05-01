#include "simulation/logging.h"
#include <cstdlib>

#include "clothscene.h"
#include "cloth.h"

struct ClothViewerConfig : Config {
    static string file;
    ClothViewerConfig() : Config() {
        params.push_back(new Parameter<string>("file", &file, "cloth state file to open"));
    }
};
string ClothViewerConfig::file = "";

class CustomScene : public ClothScene {
public:
    void run() {
        setupScene();

        cloth = Cloth::createFromFile(env->bullet->softBodyWorldInfo, ClothViewerConfig::file.c_str());
        env->add(cloth);

        startViewer();
        setSyncTime(false);
        startFixedTimestepLoop(BulletConfig::dt);
    }
};

int main(int argc, char *argv[]) {
    SetCommonConfig();
    SceneConfig::enableRobot = false;

    LoggingInit();

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(ClothViewerConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
