#include "simulation/logging.h"
#include <cstdlib>

#include "clothscene.h"
#include "cloth.h"

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
        LOG_INFO("loading " << filename);

        bool usingTempPath = false;
        fs::path tmpPath, tmpDecompressed;
        // decompress if needed
        if (filename.extension() == ".gz") {
            // copy to /tmp first
            usingTempPath = true;
            tmpDecompressed = fs::temp_directory_path() / fs::unique_path();
            tmpPath = tmpDecompressed; tmpPath.replace_extension(".gz");
            fs::copy_file(filename, tmpPath);

            // run gunzip
            stringstream ss;
            ss << '\'' << ClothViewerConfig::gzip << "' -d " << tmpPath;
            string cmd = ss.str();
            LOG_INFO("decompressing: executing " << cmd);
            system(cmd.c_str());
        }

        if (cloth) {
            env->remove(cloth);
            cloth.reset();
        }
        cloth = Cloth::createFromFile(env->bullet->softBodyWorldInfo,
                usingTempPath ? tmpDecompressed.string() : filename.string());
        if (!cloth->fullValidCheck()) {
            LOG_ERROR("cloth exploded");
            goto exit;
        }
        env->add(cloth);

exit:
        // remove temporary file if needed
        if (usingTempPath)
            fs::remove(tmpDecompressed);
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
