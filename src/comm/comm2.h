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

using namespace std;
namespace fs = boost::filesystem;
using fs::path;
using Json::Value;
using namespace Json;
typedef pair<path,path> PathPair;


double timeOfDay();

void askToResetDir(path p);

void writeJson(const Value& v, path p);
Value readJson(path p);

void setDataRoot(path p);
void setDataRoot();
path getDataRoot();

void setLive(bool live);
void setTimeout(float timeout);

void initComm();



//void absPath(string rel);
path topicPath(string topic);
path filePath(string basename, string topic);
path onceFile(string basename);

void delTopic(string topic);

int infoID(string infoName);
int dataID(string dataName);
path makeDataName(int id, string extension, string topic);
path makeInfoName(int id, string topic);
PathPair makePathPair(int id, string extension, string topic);
Value readJson(path p);

class Names {
public:
  int m_id;
  string m_extension;
  string m_topic;
  Names(string topic, string extension) : m_topic(topic), m_extension(extension), m_id(0) {
    ENSURE(!topic.empty());
    ENSURE(!extension.empty());
  }
  PathPair getCur() const;
  void step();
  PathPair getCurAndStep();
};

class Message { 
public: 
  Value m_info; 
  Message() {m_info["time"] = timeOfDay();  }
  Message(Value info) : m_info(info) {
    if (!m_info.isMember("time")) m_info["time"] = timeOfDay();
  }
  void toFiles(PathPair) const;
  void fromFiles(PathPair);
  double getTime() const;
  virtual void writeDataTo(path) const {};
  virtual void readDataFrom(path) {};
};

class Publisher {
public:
  virtual void send(const Message&)=0;
};

class FilePublisher : public Publisher {
public:
  Names m_names;
  FilePublisher(string topic, string extension);
  void send(const Message& message);
};

class Subscriber {
public:
  virtual bool recv(Message& )=0;
  virtual bool skip() = 0;
};

class FileSubscriber : public Subscriber {
public:
  Names m_names;
  FileSubscriber(string topic, string extension);
  bool recv(Message& message);
  bool skip();
};

template <typename T> 
struct GenericMessage :  Message {
  T m_data;
  GenericMessage() : Message() {}
  GenericMessage(T data) : Message(), m_data(data) {}
  GenericMessage(T data, Value info) : Message(info), m_data(data) {}
  void writeDataTo(path p) const {
    ofstream outfile(p.string().c_str());
    if (outfile.fail()) throw FileOpenError(p.string());
    outfile << m_data;
    outfile.close();
  }
  void readDataFrom(path p) {
    ifstream infile(p.string().c_str());
    if (infile.fail()) throw FileOpenError(p.string());
    infile >> m_data;
    infile.close();
  }
};

typedef GenericMessage<int> IntMessage;
typedef GenericMessage<float> FloatMessage;
typedef GenericMessage<string> StrMessage;


template <typename T>
struct VectorMessage :  Message {
  vector<T> m_data;
  VectorMessage() : Message() {}
  VectorMessage(vector<T>& data) : Message(), m_data(data) {}
  VectorMessage(vector<T>& data, Value info) : Message(info), m_data(data) {}
  void writeDataTo(path p) const {
    ofstream outfile(p.string().c_str());
    if (outfile.fail()) throw FileOpenError(p.string());
    for (int i=0; i<m_data.size(); i++) outfile << m_data[i] << " ";
    outfile.close();
  }
  void readDataFrom(path p) {
    ifstream infile(p.string().c_str());
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
  vector< vector<T> > m_data;
  VecVecMessage() : Message() {}
  VecVecMessage(vector< vector<T> >& data) : Message(), m_data(data) {}
  VecVecMessage(vector< vector<T> >& data, Value info) : Message(info), m_data(data) {}
  void writeDataTo(path p) const {
    ofstream outfile(p.string().c_str());
    if (outfile.fail()) throw FileOpenError(p.string());
    for (int i=0; i<m_data.size(); i++) {
      for (int j=0; j<m_data[i].size(); j++)
	outfile << m_data[i][j] << " ";
      outfile << endl;
    }
    outfile.close();
  }
  void readDataFrom(path p) {
    ifstream infile(p.string().c_str());
    if (infile.fail()) throw FileOpenError(p.string());
    T x;				
    m_data.clear();		
    string line;
    while (!infile.eof()) {
      getline(infile, line, '\n');				
      stringstream ss(stringstream::in | stringstream::out);	
      vector<T> v;
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

// class Synchronizer {
// public:
//   Subscriber m_leader;
//   vector<Retimer> m_retimers;
//   Synchronizer(Subscriber m_leader&, vector<Retimer> retimers);  
//   void recvMulti(Message& leaderMsg, vector<Message>& followerMsgs);
// };
