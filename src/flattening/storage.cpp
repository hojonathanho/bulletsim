#include "storage.h"
#include "simulation/logging.h"
#include <boost/lexical_cast.hpp>

static const char GZIP_PATH[] = "/bin/gzip";

static const char STATES_SUBDIR[] = "states";
static const char CLOUDS_SUBDIR[] = "clouds";

namespace Storage {

Cloth::Ptr loadCloth(const fs::path &filename, btSoftBodyWorldInfo &worldInfo) {
    LOG_INFO("loading " << filename.string());

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
        ss << '\'' << GZIP_PATH << "' -d " << tmpPath;
        string cmd = ss.str();
        LOG_TRACE("decompressing: executing " << cmd);
        system(cmd.c_str());
    }

    Cloth::Ptr cloth = Cloth::createFromFile(worldInfo,
            usingTempPath ? tmpDecompressed.string() : filename.string());

    // remove temporary file if needed
    if (usingTempPath)
        fs::remove(tmpDecompressed);

    return cloth;
}

/*Cloth::Ptr loadClothByID(const fs::path &root, ID id, btSoftBodyWorldInfo &worldInfo) {
    return loadCloth(clothFileFromID(root, id));
}*/

static string genFilenameStem(ID id) {
    stringstream ss;
    ss << setw(10) << setfill('0') << id;
    return ss.str();
}

fs::path clothFileFromID(const fs::path &root, ID id) {
    return root / STATES_SUBDIR / genFilenameStem(id) / ".cloth";
}

fs::path cloudFileFromID(const fs::path &root, ID id) {
    return root / CLOUDS_SUBDIR / genFilenameStem(id) / ".pcd";
}

// returns filename with all extensions stripped
static fs::path rawStem(const fs::path &p) {
    fs::path q = p;
    while (!q.extension().empty())
        q = q.stem();
    return q;
}

ID idFromCloudPath(const fs::path &filename) {
    return boost::lexical_cast<ID>(rawStem(filename).string());
}

void listCloudFiles(const fs::path &root, vector<fs::path> &files) {
    fs::path p = root / CLOUDS_SUBDIR;
    fs::directory_iterator end_iter;
    for (fs::directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
        if ( fs::is_regular_file( dir_itr->status() ) )
            files.push_back(dir_itr->path());
}

}
