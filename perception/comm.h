#pragma once

#include <string>
#include <map>
#include <vector>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>
using namespace boost::filesystem;
using namespace std;

enum COMM_MODE {
  MODE_INC,
  MODE_LAST,
  MODE_NOUP
};



namespace comm {

  string publishOnce(string fname) {
      string data_root = getenv("DATA_ROOT");
      assert(!data_root.empty());
      return (path(data_root) /"once"/ fname).string();
  }

  string listenOnce(string fname) {
      string data_root = getenv("DATA_ROOT");
      assert(!data_root.empty());
      string out = (path(data_root) /"once"/ fname).string();
      assert(exists(out));
      return out;
  }


  map <int,string> getMessages(path p) {
    map<int,string> ind2fname;
    for( directory_iterator i( p ); i != directory_iterator(); ++i ) {
      string basename=i->filename();
      if (boost::starts_with(basename,"msg")) {
	int ind = boost::lexical_cast<int>(basename.substr(3,12));
	ind2fname.insert(pair<int,string>(ind,i->string()));
      }
    }
 
    return ind2fname;
  }


  class Listener {
  public:
    path m_directory;
    int m_current_ind;
    Listener(const string topicname) {

      m_current_ind = -1;
      string data_root = getenv("DATA_ROOT");
      assert(!data_root.empty());
      m_directory =  path(data_root) / topicname;
    }
    string next(COMM_MODE mode = MODE_INC) {
      map<int,string> ind2fname;
      while (true) {
	ind2fname = getMessages(m_directory);
	if (!ind2fname.empty() && m_current_ind < ind2fname.rbegin()->first) break;
	usleep(10*1000);


      }

      string fname;
      if (mode == MODE_INC) {
	m_current_ind++;
	fname = ind2fname[m_current_ind];
      }
      else if (mode == MODE_LAST) {
	m_current_ind = ind2fname.rbegin()->first;
	fname = ind2fname[m_current_ind];
      }
      else fname = ind2fname[m_current_ind+1];


      return fname;
    }
  };

  class Publisher {
    int m_nKeep;
    int m_current_ind;
    path m_directory;
    string m_extension;
  public:
    Publisher(string topicname, string extension = "txt", int nKeep=10) {
      m_current_ind = 0;
      m_extension = extension;
      m_nKeep = nKeep;
      string data_root = getenv("DATA_ROOT");
      assert(!data_root.empty());
      m_directory = path(data_root) / topicname;
      if (exists(m_directory)) remove_all(m_directory);
      assert(create_directory(m_directory));
    }
    string next() {
      stringstream ss;
      ss << (m_directory / "msg").string();
      ss << setw(12) << setfill('0') << m_current_ind;
      ss << "." << m_extension;
      m_current_ind++;
      return ss.str();
    }
    void cleanup() {
      map<int,string> ind2fname = getMessages(m_directory);
      if (ind2fname.size() > m_nKeep) {
	int last_ind =  ind2fname.rbegin()->first;
	for (int i = last_ind - m_nKeep+1; i <= last_ind; i++) remove(ind2fname[i]);
  }

    }
  };
}
