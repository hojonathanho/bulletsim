#include "comm.h"
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include "utils/logging.h"
using namespace std;
namespace fs = boost::filesystem;
using fs::path;
using Json::Value;
using namespace Json;

namespace comm {

fs::path DATA_ROOT = "/dont/forget/to/set";
bool LIVE = false;
float TIMEOUT = 1; //second 

double timeOfDay() {
  timeval tim;
  gettimeofday(&tim, NULL);
  return tim.tv_sec+(tim.tv_usec/1000000.0);
}

bool yesOrNo(char message[]) {
  while (true) {
    cout << message << " (y/n): ";
    char yn;
    cin >> yn;
    if (yn == 'y') return true;
    else if (yn == 'n') return false;
  }
}

void askToResetDir(fs::path p) {
  if (fs::exists(p)) {
    char buffer[150];
    sprintf(buffer, "%s already exists. Delete it?", p.string().c_str());
    bool consent = yesOrNo(buffer);
    if (consent) {
      cout << "deleting " << p.string() << endl;
      fs::remove_all(p);
    }
    else throw IOError();
  }
  ENSURE(fs::create_directory(p));
}


void writeJson(const Value& v, fs::path p) {
  Json::StyledWriter writer;
  string outString = writer.write(v);
  ofstream outfile(p.string().c_str());
  if (outfile.fail()) throw FileOpenError(p.string());
  outfile << outString;
  outfile.close();
}

Value readJson(fs::path jsonfile) { 
  // occasionally it fails, presumably when the json isn't done being written. so repeat 10 times
  for (int i_try = 0; i_try < 10; i_try++) {
    try {

      std::ifstream infile(jsonfile.string().c_str());
      if(infile.fail()) throw FileOpenError(jsonfile.string());

      Json::Reader reader;
      Value root;
      infile >> root;
      return root;
    }
    catch (std::runtime_error e) {
      usleep(1000);
    }
  }
  throw std::runtime_error("tried 10 times but failed to read json file " + jsonfile.string());
}

void setDataRoot(fs::path newDataRoot) {
  // if the fs::path starts with ~, expand it
  if (!newDataRoot.empty() && newDataRoot.string()[0] == '~') {
    const char *home = getenv("HOME");
    if (home) {
        DATA_ROOT = home;
        DATA_ROOT /= newDataRoot.string().substr(1);
    }
  } else {
    DATA_ROOT = newDataRoot;
  }
  bool success=true;
  if (!fs::exists(DATA_ROOT)) ENSURE(fs::create_directory(DATA_ROOT));
}

void setDataRoot() {
  const char *home = getenv("DATA_ROOT");
  if (home==NULL) throw runtime_error("DATA_ROOT not set");
  fs::path p  = home;
  ENSURE(fs::exists(p));
  DATA_ROOT = p;
}
fs::path getDataRoot() {return DATA_ROOT;}

void setLive(bool live){ LIVE = live;}
void setTimeout(float timeout) {TIMEOUT = timeout;}

void initComm() {
  setDataRoot();
  char* maybeLive = getenv("COMM_LIVE");
  if (maybeLive) LIVE = boost::lexical_cast<bool>(maybeLive);
  char* maybeTimeout = getenv("COMM_TIMEOUT");
  if (maybeTimeout) TIMEOUT = boost::lexical_cast<float>(maybeTimeout);
}

fs::path topicPath(string topic) {
  return DATA_ROOT / topic;
}
fs::path filePath(string basename, string topic) {
  return DATA_ROOT / topic / basename;
}
fs::path onceFile(string basename) {
  return DATA_ROOT / "once" / basename;
}

void delTopic(string topic) {
  fs::path fullPath = topicPath(topic);
  fs::remove_all(fullPath);
}

int infoID(string infoName) {
  return atoi(infoName.substr(4,12).c_str());
}
int dataID(string dataName) {
  return atoi(dataName.substr(4,12).c_str());
}


fs::path makeDataName(int id, string extension, string topic) {
  stringstream ss;
  ss << "data" << setw(12) << setfill('0') << id << "." << extension;
  return filePath(ss.str(),topic);
}

fs::path makeInfoName(int id, string topic) {
  stringstream ss;
  ss << "info" << setw(12) << setfill('0') << id << "." << "json";
  return filePath(ss.str(), topic);
}
PathPair makePathPair(int id, string extension, string topic) {
  return PathPair(makeDataName(id,extension,topic),
		  makeInfoName(id,topic));
}

bool waitFor(fs::path p, bool enableWait) {
  // Checks if file exists. If LIVE, waits up to TIMEOUT if it doesn't exist
  if (enableWait) {
    double tStart = timeOfDay();
    while(timeOfDay() - tStart < TIMEOUT) {
      if (exists(p)) return true;
      else usleep(1000);
    }
    return false;
  }
  else return exists(p);
}
void waitIfThrottled(string topic) {
  fs::path throttleFile = filePath("STOP", topic);
  while (exists(throttleFile)) usleep(10000);
}
bool getThrottled(string topic) {
  fs::path throttleFile = filePath("STOP", topic);
  return exists(throttleFile);
}



PathPair Names::getCur() const {
  return makePathPair(m_id, m_extension, m_topic);
}

void Names::step() {m_id++;}

PathPair Names::getCurAndStep() {
  PathPair pair = getCur();
  step();
  return pair;
}

// TopicWatcher::TopicWatcher(string topic) 
//   : watch(topicPath(topic),IN_CLOSE_WRITE);
// {
//   notify.Add(watch);
// }

// TopicWatcher::update () {
//   size_t nEvents = notify.GetEventCount();
//   while (count > 0) {
//     InotifyEvent event;
//     bool got_event = notify.GetEvent(&event);
//     if (got_event) {
//       string mask_str;
//       event.DumpTypes(mask_str);
//       string filename = event.GetName();
//       cout << "[watch " << watch_dir << "] ";
//       cout << "event mask: \"" << mask_str << "\", ";
//       cout << "filename: \"" << filename << "\"" << endl;
//     }
//     else cout << "huh?" << endl;
//     nEvents--;
//     m_counter++;
//   }
// }

void Message::toFiles(PathPair pair) const {
  cout << "writing " << pair.first << "...";
  writeDataTo(pair.first);
  cout << "OK" << endl;
  writeJson(m_info,pair.second);
}

void Message::fromFiles(PathPair pair) {
//  cout << "reading " << pair.first << "...";
  readDataFrom(pair.first);
//  cout << "OK"<<endl;
//  cout << "reading " << pair.second << "...";
  m_info = readJson(pair.second);
//  cout << "OK"<<endl;
}

double Message::getTime() const {
  return m_info["time"].asDouble();
}


FilePublisher::FilePublisher(string topic, string extension) : m_names(topic, extension) {
  askToResetDir(topicPath(topic));
}

void FilePublisher::send(const Message& message) {
  PathPair pair = m_names.getCurAndStep();
  message.toFiles(pair);
}

FileSubscriber::FileSubscriber(string topic, string extension) : m_names(topic, extension) {}
bool FileSubscriber::recv(Message& message, bool enableWait)  {
  PathPair namePair = m_names.getCur();
  bool gotIt = waitFor(namePair.second, LIVE && enableWait);
  if (gotIt) {
    message.fromFiles(namePair);
    m_names.step();
      }
  if (!gotIt) {
    LOG_INFO("didn't get " << namePair.second);
  }
  return gotIt;
}

bool FileSubscriber::skip() {
  m_names.step();
  return true;
}

// problem: you might do msgAt, and then call it again and lose your first message
// maybe it would be better to pass msg into msgAt, and then clone it twice. Then use copy constructor to return it.

AbstractRetimer::AbstractRetimer(Subscriber* sub) : m_sub(sub), m_new(true), m_done(false) {}

Message* AbstractRetimer::msgAt(double time) {
  //printf("msgAt: %10.10f %10.10f %10.10f\n", time, m_msg0->getTime(), m_msg1->getTime());
  if (m_new) {
    ENSURE(m_sub->recv(*m_msg0));
    ENSURE(m_sub->recv(*m_msg1));
    m_new = false;
  }
  if (m_done) return m_msg0;
  if (time < m_msg0->getTime()) return m_msg0;
  while (!isBetween(time)) {
    bool reachedEnd = !step();
    if (reachedEnd) {
      cout << "reached the end of the topic. done" << endl;
      m_done = true;
      return m_msg0;
    }
  }
  return closerMsg(time);
}

bool AbstractRetimer::isBetween(double time) {
  return (time >= m_msg0->getTime()) && (time <= m_msg1->getTime());
}

bool AbstractRetimer::step() {
  //cout << "before step: " << m_msg0->getTime() << " " << m_msg1->getTime() << endl;

    swap(m_msg0,m_msg1);
    bool gotOne = m_sub->recv(*m_msg1);
    //cout << "after step: " << m_msg0->getTime() << " " << m_msg1->getTime() << endl;

    return gotOne;

}


Message* AbstractRetimer::closerMsg(double time) {
  double diff0 = time - m_msg0->getTime();
  double diff1 = m_msg1->getTime() - time;
  return (diff0 < diff1) ? m_msg0 : m_msg1;
}


bool allTrue(const vector<bool>& x) {
  bool out = true;
  BOOST_FOREACH(bool b, x) out &= b;
  return out;
}

bool MultiSubscriber::recvAll() {
    for (int i=0; i < m_subs.size(); i++)
      if (!m_gotEm[i]) m_gotEm[i] = m_subs[i]->recv(*m_msgs[i], false);
    return allTrue(m_gotEm);
};

void MultiSubscriber::prepare() {
  m_gotEm = vector<bool>(m_msgs.size(), false);
}

}
// Synchronizer::Synchronizer(vector<Subscriber>& subs) {
//   m_leader = subs[0];
//   for (int i=1; i<subs.size(); i++) m_retimers.push_back(Retimer(subs[i]));
// }

// void Synchronizer::recvMulti(Message& leaderMsg, vector<Message>& followerMsgs) {
// vector<Message*> Synchronizer::recvMulti{
//   vector<Message*> out;
//   out.push_back(
// }
