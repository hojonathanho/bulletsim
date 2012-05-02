#include "simulation/logging.h"
#include <cstdlib>

#include "clothscene.h"
#include "cloth.h"

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

struct ClothViewerConfig : Config {
    static string path;
    ClothViewerConfig() : Config() {
        params.push_back(new Parameter<string>("path", &path, "cloth state path (either a file or a directory)"));
    }
};
string ClothViewerConfig::path = "";

class CustomScene : public ClothScene {
    vector<string> files;
    int currpos;

public:
    void displayCloth(const string &filename) {
        LOG_INFO("loading " << filename);
        if (cloth) {
            env->remove(cloth);
            cloth.reset();
        }
        try {
            cloth = Cloth::createFromFile(env->bullet->softBodyWorldInfo, filename.c_str());
            if (cloth->checkExplosion()) {
                LOG_ERROR("cloth exploded");
            }
        } catch (...) {
            LOG_ERROR("could not load " << filename);
        }
        env->add(cloth);
    }

    void nextCloth(int d) {
        currpos = (currpos + d) % files.size();
        displayCloth(files[currpos]);
    }

    void run() {
        setupScene();

        fs::path p(ClothViewerConfig::path.c_str());
        if (!fs::exists(p)) {
            LOG_ERROR("input path " << ClothViewerConfig::path << " does not exist");
            exit(1);
        }

        if (fs::is_directory(p)) {
            fs::directory_iterator end_iter;
            for (fs::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
                if ( fs::is_regular_file( dir_itr->status() ) )
                    files.push_back(dir_itr->path().string());
            sort(files.begin(), files.end());
        } else {
            files.push_back(p.string());
        }

        addVoidKeyCallback('j', boost::bind(&CustomScene::nextCloth, this, 1));
        addVoidKeyCallback('k', boost::bind(&CustomScene::nextCloth, this, -1));

        currpos = 0;
        displayCloth(files[0]);

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
