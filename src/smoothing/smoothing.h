#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

fs::path SMOOTHING_DATA  = fs::path(EXPAND(BULLETSIM_DATA_DIR) "/knots");