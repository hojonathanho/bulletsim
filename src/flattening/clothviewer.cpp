#include "simulation/logging.h"
#include <cstdlib>

#include "clothscene.h"
#include "cloth.h"
#include "storage.h"

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

struct ClothViewerConfig : Config {
    static string path;
    static string gzip;
    ClothViewerConfig() : Config() {
        params.push_back(new Parameter<string>("path", &path, "cloth state path (either a file or a directory)"));
        params.push_back(new Parameter<string>("gzip", &gzip, "path to gzip to decompress input files"));
    }
};
string ClothViewerConfig::path = "";
string ClothViewerConfig::gzip = "/bin/gzip";

class CustomScene : public ClothScene {
    vector<fs::path> files;
    int currpos;

public:
    void displayCloth(const fs::path &filename) {
        if (cloth) {
            env->remove(cloth);
            cloth.reset();
        }
        cloth = Storage::loadCloth(filename, env->bullet->softBodyWorldInfo);
        if (!cloth->fullValidCheck()) {
            LOG_ERROR("cloth exploded");
            return;
        }
        env->add(cloth);
    }

    void nextCloth(int d) {
        currpos = (currpos + d) % (int) files.size();
        if (currpos < 0) currpos += files.size();
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
                    files.push_back(dir_itr->path());
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
