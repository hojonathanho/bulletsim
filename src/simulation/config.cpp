#include "config.h"
#include "logging.h"
#include <log4cplus/logger.h>




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

  LoggingInit();
  log4cplus::Logger::getRoot().setLogLevel(GeneralConfig::verbose);
}


int GeneralConfig::verbose = log4cplus::WARN_LOG_LEVEL;
float GeneralConfig::scale = 1.;
