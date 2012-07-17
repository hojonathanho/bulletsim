#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include "vector_alg.h"

namespace po = boost::program_options;


struct ParameterBase {
  std::string m_name;
  std::string m_desc;
  virtual void addToBoost(po::options_description&) = 0;
};

template <typename T>
struct Parameter : ParameterBase {
  T* m_value;
  Parameter(std::string name, T* value, std::string desc) {
    m_name = name;
    m_value = value;
    m_desc = desc;
  }

  template <typename TYPE>
  std::string textualRepresentation(const TYPE& v) {
  	std::stringstream ss;
  	ss << v;
  	return ss.str();
  }

  void addToBoost(po::options_description& od) {
    od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value, textualRepresentation(*m_value)), m_desc.c_str());
  }
};

struct Config {
  std::vector<ParameterBase*> params;
};

class Parser {
  std::vector<Config> m_configs;
public:
  void addGroup(Config config) {
      m_configs.push_back(config);
  }

  void read(int argc, char* argv[]);
};

struct GeneralConfig : Config {
  static int verbose;
  static float scale;
  GeneralConfig() : Config() {
    params.push_back(new Parameter<int>("verbose", &verbose, "verbosity: 0: debug, 10000:info, 20000: warn, 30000: error")); 
    params.push_back(new Parameter<float>("scale", &scale, "scale factor applied to distances that are assumed to be in meters")); 
  }
};

#define METERS GeneralConfig::scale
