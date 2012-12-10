#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "utils_vector.h"

namespace po = boost::program_options;

template <typename TYPE>
std::string toString(const TYPE& v) {
  std::ostringstream ss;
  ss << v;
  return ss.str();
}

template <typename TYPE>
TYPE fromString(const std::string& s) {
  std::istringstream ss;
  ss.str(s);
  TYPE x;
  ss >> x;
  return x;
}

struct ParameterBase {
  std::string m_name;
  std::string m_desc;
  virtual void addToBoost(po::options_description&) = 0;
};

// Deprecated, use the specialized templated class
template <typename T>
struct ParameterVec : ParameterBase {
  std::vector<T>* m_value;
  ParameterVec(std::string name, std::vector<T>* value, std::string desc) {
    m_name = name;
    m_value = value;
    m_desc = desc;
  }
  void addToBoost(po::options_description& od) {
    od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value, toString(*m_value))->multitoken(), m_desc.c_str());
  }
};

template <typename T>
struct Parameter : ParameterBase {
  T* m_value;
  Parameter(std::string name, T* value, std::string desc) {
    m_name = name;
    m_value = value;
    m_desc = desc;
  }
  void addToBoost(po::options_description& od) {
    od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value, toString(*m_value)), m_desc.c_str());
  }
};

template <typename T>
struct Parameter<std::vector<T> > : ParameterBase {
  std::vector<T>* m_value;
  Parameter(std::string name, std::vector<T>* value, std::string desc) {
    m_name = name;
    m_value = value;
    m_desc = desc;
  }
  void addToBoost(po::options_description& od) {
  	cout << "especial" << endl;
  	od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value, toString(*m_value))->multitoken(), m_desc.c_str());
  }
};

template <typename T, typename U>
struct Parameter<std::map<T,U> > : ParameterBase {
  std::map<T,U>* m_value;
  vector<string> m_value_vector;

  void toVector(std::map<T,U> m, std::vector<std::string>& v) {
		typename map<T,U>::iterator it;
		for ( it=m.begin() ; it != m.end(); it++ ) {
			v.push_back(boost::lexical_cast<std::string>(it->first));
			v.push_back(boost::lexical_cast<std::string>(it->second));
		}
	}

	void toMap(std::vector<std::string> v, std::map<T,U>& m) {
		vector<string>::iterator it;
		for ( it=v.begin(); it<v.end(); it+=2)
			m[boost::lexical_cast<T>(*it)] = boost::lexical_cast<U>(*(it+1));
	}

	void updateValue(std::vector<std::string> v) {
		if ((v.size()%2) != 0) {
			stringstream ss;
			ss << "The option " << m_name << " should have an even number of parameters but it has " << v.size() << " parameters";
			throw std::runtime_error(ss.str());
		}
		toMap(v, *m_value);
	}

  Parameter(std::string name, std::map<T,U>* value, std::string desc) {
    m_name = name;
    m_value = value;
    toVector(*m_value, m_value_vector);
    m_desc = desc;
  }

  void addToBoost(po::options_description& od) {
  	od.add_options()(m_name.c_str(), po::value(&m_value_vector)->default_value(m_value_vector, toString(*m_value))->multitoken()
  		->notifier(boost::bind(&Parameter::updateValue, this, _1)), m_desc.c_str());
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

#ifdef __CDT_PARSER__
#undef BOOST_FOREACH
#define BOOST_FOREACH(a, b) for(a; ; )
#undef BOOST_REVERSE_FOREACH
#define BOOST_REVERSE_FOREACH(a, b) for(a; ; )
#endif

#define METERS GeneralConfig::scale
