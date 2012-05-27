#ifndef _FL_STORAGE_H_
#define _FL_STORAGE_H_

#include "cloth.h"

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace Storage {

typedef int ID;

fs::path clothPathFromID(const fs::path &root, ID id);
fs::path cloudPathFromID(const fs::path &root, ID id);

ID idFromCloudPath(const fs::path &filename);

Cloth::Ptr loadCloth(const fs::path &filename, btSoftBodyWorldInfo &worldInfo);
//Cloth::Ptr loadClothByID(const fs::path &root, ID id, btSoftBodyWorldInfo &worldInfo);

void listCloudFiles(const fs::path &root, vector<fs::path> &files);
};


#endif // _FL_STORAGE_H_
