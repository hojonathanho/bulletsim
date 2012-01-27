#include "comm2.h"
#include "my_exceptions.h"

#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>

using namespace std;


path DATA_ROOT = "/dont/forget/to/set";

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
    if (yn == 'y') break;
    else if (yn == 'n') exit(0);
  }
}

void askToResetDir(path p) {
  if (fs::exists(p)) {
    char buffer[150];
    sprintf(buffer, "%s already exists. Delete it?", p.string().c_str());
    bool consent = yesOrNo(buffer);
    if (consent) {
      cout << "deleting " << p.string() << endl;
      fs::remove_all(p);
    }
    else assert(false);
  }
  bool success = fs::create_directory(p);
  assert(success);
}


void writeJson(const Value& v, path p) {
  Json::StyledWriter writer;
  string outString = writer.write(v);
  ofstream outfile(p.string().c_str());
  if (outfile.fail()) throw FileOpenError(p.string());
  outfile << outString;
  outfile.close();
}

Value readJson(path jsonfile) {
  std::stringstream buffer;
  std::ifstream infile(jsonfile.string().c_str());
  if(infile.fail()) throw FileOpenError(jsonfile.string());
  buffer << infile.rdbuf();
  Json::Reader reader;
  Value root;
  bool parsedSuccess = reader.parse(buffer.str(), root, false);
  if (!parsedSuccess) throw FileParseError(jsonfile.string());
  return root;
}

void setDataRoot(path newDataRoot) {
  // if the path starts with ~, expand it
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
  if (!fs::exists(DATA_ROOT)) success = fs::create_directory(DATA_ROOT);
  assert(success);
}
void setDataRoot() {
  path p  = getenv("DATA_ROOT");
  assert(fs::exists(p));
  DATA_ROOT = p;
}
path getDataRoot() {return DATA_ROOT;}
path absPath(string path) {
  return DATA_ROOT / path;
}
path topicPath(string topic) {
  return DATA_ROOT / topic;
}
path filePath(string basename, string topic) {
  return DATA_ROOT / topic / basename;
}
path onceFile(string basename) {
  return DATA_ROOT / "once" / basename;
}

void delTopic(string topic) {
  path fullPath = topicPath(topic);
  fs::remove_all(fullPath);
}

int infoID(string infoName) {
  return atoi(infoName.substr(4,12).c_str());
}
int dataID(string dataName) {
  return atoi(dataName.substr(4,12).c_str());
}


path makeDataName(int id, string extension, string topic) {
  stringstream ss;
  ss << "data" << setw(12) << setfill('0') << id << "." << extension;
  return filePath(ss.str(),topic);
}

path makeInfoName(int id, string topic) {
  stringstream ss;
  ss << "info" << setw(12) << setfill('0') << id << "." << "json";
  return filePath(ss.str(), topic);
}
PathPair makePathPair(int id, string extension, string topic) {
  return PathPair(makeDataName(id,extension,topic),
		  makeInfoName(id,topic));
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
  writeDataTo(pair.first);
  writeJson(m_info,pair.second);
}

void Message::fromFiles(PathPair pair) {
  cout << "reading " << pair.first << "...";
  readDataFrom(pair.first);
  cout << "OK"<<endl;
  cout << "reading " << pair.second << "...";
  m_info = readJson(pair.second);
  cout << "OK"<<endl;
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
bool FileSubscriber::recv(Message& message)  {
  PathPair namePair = m_names.getCurAndStep();
  try {
    message.fromFiles(namePair);
    return true;
  }
  catch (FileOpenError err) {
    cout << "recv failed for " << err.m_filename << endl;
    return false;
  }
}

// problem: you might do msgAt, and then call it again and lose your first message
// maybe it would be better to pass msg into msgAt, and then clone it twice. Then use copy constructor to return it.

AbstractRetimer::AbstractRetimer(Subscriber* sub) : m_sub(sub), m_new(true), m_done(false) {}

Message* AbstractRetimer::msgAt(double time) {
  //printf("msgAt: %10.10f %10.10f %10.10f\n", time, m_msg0->getTime(), m_msg1->getTime());
  if (m_new) {
    bool success0 = m_sub->recv(*m_msg0);
    assert(success0);
    bool success1 = m_sub->recv(*m_msg1);
    assert(success1);
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



// Synchronizer::Synchronizer(vector<Subscriber>& subs) {
//   m_leader = subs[0];
//   for (int i=1; i<subs.size(); i++) m_retimers.push_back(Retimer(subs[i]));
// }

// void Synchronizer::recvMulti(Message& leaderMsg, vector<Message>& followerMsgs) {
// vector<Message*> Synchronizer::recvMulti{
//   vector<Message*> out;
//   out.push_back(
// }
