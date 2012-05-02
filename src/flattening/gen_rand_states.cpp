#include "simulation/util.h"
#include <cstdlib>

#include "clothscene.h"
#include "folding.h"
#include "config_flattening.h"

#include <boost/filesystem.hpp>

#include <sstream>
using namespace std;

// State generation procedure:
// for nfolds = 1 .. maxRandFolds:
//   repeat statesPerFold times:
//     initialize a flat cloth
//     fold the cloth randomly nfolds times, then record its state
//     repeat dropTimes times:
//       pick up the cloth and drop it, then record its state
//
// This generates maxRandFolds*statesPerFold*(dropTimes+1) total states.
//

struct GenStatesConfig : Config {
    static int maxRandFolds;
    static int statesPerFold;
    static int dropTimes;
    static string outputPath;
    static bool fake;
    static string gzip;

    GenStatesConfig() : Config() {
        params.push_back(new Parameter<int>("maxRandFolds", &maxRandFolds, ""));
        params.push_back(new Parameter<int>("statesPerFold", &statesPerFold, ""));
        params.push_back(new Parameter<int>("dropTimes", &dropTimes, ""));
        params.push_back(new Parameter<string>("outputPath", &outputPath, ""));
        params.push_back(new Parameter<bool>("fake", &fake, "don't actually do anything"));
        params.push_back(new Parameter<string>("gzip", &gzip, "path to gzip to compress output files (pass empty string to leave files uncompressed)"));
    }
};
int GenStatesConfig::maxRandFolds = 3;
int GenStatesConfig::statesPerFold = 10;
int GenStatesConfig::dropTimes = 3;
string GenStatesConfig::outputPath = "";
bool GenStatesConfig::fake = false;
string GenStatesConfig::gzip = "";

static void compress(const string &path) {
    if (GenStatesConfig::gzip.empty()) return;

    stringstream ss;
    ss << '\'' << GenStatesConfig::gzip << "' '" << path << '\'';
    string cmd = ss.str();

    LOG_INFO("compressing: executing " << cmd);
    system(cmd.c_str());
}

static int discarded = 0;
static int calcTotal() {
    static const int total = GenStatesConfig::maxRandFolds
        * GenStatesConfig::statesPerFold
        * (GenStatesConfig::dropTimes + 1);
    return total - discarded;
}
static int gencount = 0;
static void record(Cloth &cloth) {
    if (!cloth.validCheck(true)) {
        LOG_ERROR("cloth explosion detected. discarding state");
        ++discarded;
        return;
    }

    ++gencount;
    boost::filesystem::path path(GenStatesConfig::outputPath);
    stringstream ss;
    ss << setw(10) << setfill('0') << gencount;
    ss << ".cloth";
    string out = (path / ss.str()).string();

    if (!GenStatesConfig::fake)
        cloth.saveToFile(out.c_str());
    LOG_INFO('(' << gencount << '/' << calcTotal() << "): wrote " << out);
    if (!GenStatesConfig::fake)
        compress(out);
}

static void gen(Scene &scene, Cloth &cloth, int nfolds) {
    if (!GenStatesConfig::fake)
        Folding::doRandomFolds(scene, cloth, nfolds);
    record(cloth);

    for (int i = 0; i < GenStatesConfig::dropTimes; ++i) {
        if (!GenStatesConfig::fake)
            Folding::pickUpAndDrop(scene, cloth);
        record(cloth);
    }
}

class CustomScene : public ClothScene {
public:
    void run();
};

void CustomScene::run() {
    setupScene();

    LOG_INFO("Generating " << calcTotal() << " cloth states.");
    LOG_INFO("Output directory: " << GenStatesConfig::outputPath);

    for (int nfolds = 1; nfolds <= GenStatesConfig::maxRandFolds; ++nfolds) {
        for (int i = 0; i < GenStatesConfig::statesPerFold; ++i) {
            if (cloth) {
                env->remove(cloth);
                cloth.reset();
            }
            initStandardCloth();
            gen(*this, *cloth, nfolds);
        }
    }
}

int main(int argc, char *argv[]) {
    SetCommonConfig();
    LoggingInit();

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(Folding::FoldingConfig());
    parser.addGroup(GenStatesConfig());
    parser.read(argc, argv);

    srand(time(NULL));

    CustomScene().run();

    return 0;
}
