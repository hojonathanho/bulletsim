#pragma once
#include <string>
#include <fstream>
#include <exception>
#include <sstream>
#include <map>
#include <iostream>
#include <vector>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <json/json.h>
#include "utils/my_exceptions.h"
#include "utils/my_assert.h"

namespace comm {

namespace fs = boost::filesystem;
typedef std::pair<fs::path,fs::path> PathPair; // dataname, infoname

double timeOfDay();

void askToResetDir(fs::path p);

void writeJson(const Json::Value& v, fs::path p);
Json::Value readJson(fs::path p);

void setDataRoot(fs::path p);
void setDataRoot();
fs::path getDataRoot();

void setLive(bool live);
void setTimeout(float timeout);

void initComm();

fs::path topicPath(std::string topic);
fs::path filePath(std::string basename, std::string topic);
fs::path onceFile(std::string basename);

void delTopic(std::string topic);

int infoID(std::string infoName);
int dataID(std::string dataName);
fs::path makeDataName(int id, std::string extension, std::string topic);
fs::path makeInfoName(int id, std::string topic);
PathPair makePathPair(int id, std::string extension, std::string topic);
Json::Value readJson(fs::path p);
bool waitFor(fs::path p, bool enableWait);
void waitIfThrottled(std::string topic);
bool getThrottled(std::string topic);

class Names {
public:
  int m_id;
  std::string m_extension;
  std::string m_topic;
  Names(std::string topic, std::string extension) : m_topic(topic), m_extension(extension), m_id(0) {
    ENSURE(!topic.empty());
    ENSURE(!extension.empty());
  }
  PathPair getCur() const;
  void step();
  PathPair getCurAndStep();
};

class Message { 
public: 
  Json::Value m_info;
  Message() {m_info["time"] = timeOfDay();  }
  Message(Json::Value info) : m_info(info) {
    if (!m_info.isMember("time")) m_info["time"] = timeOfDay();
  }
  void toFiles(PathPair) const;
  void fromFiles(PathPair);
  double getTime() const;
  virtual void writeDataTo(fs::path) const {};
  virtual void readDataFrom(fs::path) {};
};

class Publisher {
public:
  virtual void send(const Message&)=0;
};

class FilePublisher : public Publisher {
public:
  Names m_names;
  FilePublisher(std::string topic, std::string extension);
  void send(const Message& message);
};

class Subscriber {
public:
  virtual bool recv(Message&, bool enableWait=true)=0;
  virtual bool skip() = 0;
};

class FileSubscriber : public Subscriber {
public:
  Names m_names;
  FileSubscriber(std::string topic, std::string extension);
  bool recv(Message& message, bool enableWait=true);
  bool skip();
};

template <typename T> 
struct GenericMessage :  Message {
  T m_data;
  GenericMessage() : Message() {}
  GenericMessage(T data) : Message(), m_data(data) {}
  GenericMessage(T data, Json::Value info) : Message(info), m_data(data) {}
  void writeDataTo(fs::path p) const {
    std::ofstream outfile(p.string().c_str());
    if (outfile.fail()) throw FileOpenError(p.string());
    outfile << m_data;
    outfile.close();
  }
  void readDataFrom(fs::path p) {
    std::ifstream infile(p.string().c_str());
    if (infile.fail()) throw FileOpenError(p.string());
    infile >> m_data;
    infile.close();
  }
};

typedef GenericMessage<int> IntMessage;
typedef GenericMessage<float> FloatMessage;
typedef GenericMessage<std::string> StrMessage;


template <typename T>
struct VectorMessage :  Message {
  std::vector<T> m_data;
  VectorMessage() : Message() {}
  VectorMessage(std::vector<T>& data) : Message(), m_data(data) {}
  VectorMessage(std::vector<T>& data, Json::Value info) : Message(info), m_data(data) {}
  void writeDataTo(fs::path p) const {
    std::ofstream outfile(p.string().c_str());
    if (outfile.fail()) throw FileOpenError(p.string());
    for (int i=0; i<m_data.size(); i++) outfile << m_data[i] << " ";
    outfile.close();
  }
  void readDataFrom(fs::path p) {
    std::ifstream infile(p.string().c_str());
    if (infile.fail()) throw FileOpenError(p.string());
    T x;				
    m_data.clear();		
    while (!infile.eof()) {				
      infile >> x;					
      if (!infile.fail()) m_data.push_back(x);
    }							
  }
};

typedef VectorMessage<int> VectorIMessage;
typedef VectorMessage<float> VectorFMessage;
typedef VectorMessage<double> VectorDMessage;


template <typename T>
struct VecVecMessage :  Message {
  std::vector< std::vector<T> > m_data;
  VecVecMessage() : Message() {}
  VecVecMessage(std::vector< std::vector<T> >& data) : Message(), m_data(data) {}
  VecVecMessage(std::vector< std::vector<T> >& data, Json::Value info) : Message(info), m_data(data) {}
  void writeDataTo(fs::path p) const {
    std::ofstream outfile(p.string().c_str());
    if (outfile.fail()) throw FileOpenError(p.string());
    for (int i=0; i<m_data.size(); i++) {
      for (int j=0; j<m_data[i].size(); j++)
	outfile << m_data[i][j] << " ";
      outfile << std::endl;
    }
    outfile.close();
  }
  void readDataFrom(fs::path p) {
    std::ifstream infile(p.string().c_str());
    if (infile.fail()) throw FileOpenError(p.string());
    T x;				
    m_data.clear();		
    std::string line;
    while (!infile.eof()) {
      getline(infile, line, '\n');				
      std::stringstream ss(std::stringstream::in | std::stringstream::out);
      std::vector<T> v;
      ss << line;						

      T x;
      while (!ss.eof()) {
	ss >> x;					
	if (!ss.fail()) v.push_back(x);
      }

      if (v.size() > 0) m_data.push_back(v);			
    }							
  }
};

class AbstractRetimer {
public:
  AbstractRetimer(Subscriber* sub);
  Message* msgAt(double time);
protected:
  Subscriber* m_sub;
  bool m_new;
  bool m_done;
  Message* m_msg0;
  Message* m_msg1;
  bool isBetween(double time);
  bool step();
  Message* closerMsg(double time);
};

template <typename MsgT>
class Retimer : public AbstractRetimer {
public:
  Retimer(Subscriber* sub) : AbstractRetimer(sub) {
    m_msg0 = new MsgT();
    m_msg1 = new MsgT();
  }
  MsgT* msgAt(double time) {
    return dynamic_cast<MsgT*>(AbstractRetimer::msgAt(time));
  }
};

struct MultiSubscriber {
  
  std::vector<Message*> m_msgs;
  std::vector<FileSubscriber*> m_subs;
  std::vector<bool> m_gotEm;
    
  bool recvAll();
  void prepare();

};
}

