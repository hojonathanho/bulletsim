#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;


struct ParameterBase {
  string m_name;
  string m_desc;
  virtual void addToBoost(po::options_description&) = 0;
};

template <typename T>
struct Parameter : ParameterBase {
  T* m_value;
  Parameter(string name, T* value, string desc) {
    m_name = name;
    m_value = value;
    m_desc = desc;
  }
  void addToBoost(po::options_description& od) {
    od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value), m_desc.c_str());
  }
};

struct Config {
  vector<ParameterBase*> params;
};

class Parser {

  vector<Config> m_configs;
public:
  void addGroup(Config config) {
    m_configs.push_back(config);
  }

  void read(int argc, char* argv[]) {
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
      cout << "usage: " << argv[0] << " [options]" << endl;
      cout << od << endl;
      exit(0);
    }
    po::notify(vm);    

  } 
};

struct GeneralConfig : Config {
  static bool verbose;
  static float scale;
  GeneralConfig() : Config() {
    params.push_back(new Parameter<bool>("verbose", &verbose, "verbose switch")); 
    params.push_back(new Parameter<float>("scale", &scale, "scale factor applied to distances that are assumed to be in meters")); 
  }
};

#define METERS GeneralConfig::scale
