#include "config.h"
using namespace std;

void Parser::read(int argc, char* argv[]) {
  // create boost options_description based on variables, parser
  po::options_description od;
  od.add_options()("help,h", "produce help message");
  BOOST_FOREACH(Config config, m_configs){
    BOOST_FOREACH(ParameterBase* param, config.params) {
param->addToBoost(od);
    }
  }
  po::variables_map vm;        
  po::store(po::command_line_parser(argc, argv)
      .options(od)
      .run()
      , vm);
  if (vm.count("help")) {
    std::cout << "usage: " << argv[0] << " [options]" << std::endl;
    std::cout << od << std::endl;
    exit(0);
  }
  po::notify(vm);    

}


bool GeneralConfig::verbose = false;
float GeneralConfig::scale = 1.;
