#include <iostream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <json/json.h>


#include "comm2.h"
#include "comm_eigen.h"
#include "testing.h"
namespace fs = boost::filesystem;
using fs::path;
using Json::Value;
using namespace std;
using namespace Eigen;

void test_names() {
  string topic = "test_Names";
  Names names("test_Names", "txt");
  PathPair pair0 = names.getCur();
  ENSURE(pair0.first==getDataRoot()/topic/"data000000000000.txt");
  ENSURE(pair0.second==getDataRoot()/topic/"info000000000000.json");
}

void test_publisher() {
  string topic = "test_TopicWatcher";
  delTopic(topic);
  FilePublisher pub(topic,"txt");
  Value info;
  info["a"] = 2;
  StrMessage m("hi",info);
  pub.send(m);
  FileSubscriber sub(topic,"txt");
  StrMessage message;
  sub.recv(message);
  ENSURE(message.m_data==string("hi"));
  ENSURE(message.m_info["a"].asFloat()==2);
}

void test_syncer() {
  string topicA = "test_SyncerA";
  string topicB = "test_SyncerB";
  delTopic(topicA);
  delTopic(topicB);
  FilePublisher pubA(topicA,"txt");
  FilePublisher pubB(topicB,"txt");
  FileSubscriber subA(topicA,"txt");
  FileSubscriber subB(topicB, "txt");

  float tA [] = {0,1,2,3};
  float tB [] = {-1,0,.1,.9,1,2.9,3.1};

  BOOST_FOREACH(float t, tA) {
    Value info;
    info["time"] = t;
    pubA.send(FloatMessage(t, info));
  }

  BOOST_FOREACH(float t, tB) {
    Value info;
    info["time"] = t;
    pubB.send(FloatMessage(t,info));
  }

  Retimer<FloatMessage> r(&subA);
  for (float t=0; t < 4; t+= .1){
    FloatMessage* mA =   r.msgAt(t);
    cout << t << " " << mA->m_data << endl;
  }

  // vector<float> aData;
  // typedef vector<Message*> MessagePair;
  // MessagePair recMsgs;

  // for (int i=0; i<10; i++) {
  //   recMsgs = syncer.recvMulti(); 
  //   if recMsgs.size() == 0;
  //   aData.push_back(dynamic_cast<FloatMessage>(recMsgs[0]).m_data);
  // }


  // for (int i=0; i<aData.size(); i++)
  //   ENSURE(tA[i] == aData[i]);

}

void test_vector_message() {
  string topic = "test_vector_message";
  delTopic(topic);
  FilePublisher pub(topic,"txt");

  vector<double> vBefore;
  for (int i=0; i < 10; i++) vBefore.push_back(i);

  VectorDMessage msgBefore(vBefore);
  pub.send(msgBefore);

  FileSubscriber sub(topic,"txt");
  VectorDMessage msgAfter;
  sub.recv(msgAfter);

  for (int i=0; i< 10; i++)  ENSURE(msgAfter.m_data[i] == msgBefore.m_data[i]);

}


void test_vecvec_message() {
  string topic = "test_vector_message";
  delTopic(topic);
  FilePublisher pub(topic,"txt");

  vector< vector<double> > vBefore;
  for (int i=0; i < 10; i++) {
    vector<double> v;
    for (int j=0; j< 3; j++) 
      v.push_back(3*i+j);
    vBefore.push_back(v);
  }

  VecVecMessage<double> msgBefore(vBefore);
  pub.send(msgBefore);

  FileSubscriber sub(topic,"txt");
  VecVecMessage<double> msgAfter;
  sub.recv(msgAfter);

  for (int i=0; i< 10; i++) for (int j=0; j<3; j++)  ENSURE(msgAfter.m_data[i][j] == msgBefore.m_data[i][j]);

}

void test_eigen_message() {
  string topic = "test_eigen_message";
  delTopic(topic);
  FilePublisher pub(topic,"eig");

  MatrixXf matBefore(10,3);
  for (int i=0; i < 10; i++) 
    for (int j=0; j< 3; j++) 
      matBefore(i,j) = i+j;



  EigenMessage msgBefore(matBefore);
  pub.send(msgBefore);

  FileSubscriber sub(topic,"eig");
  EigenMessage msgAfter;
  sub.recv(msgAfter);

  for (int i=0; i< 10; i++) for (int j=0; j<3; j++)  ENSURE(msgAfter.m_data(i,j) == msgBefore.m_data(i,j));


}




int main() {
 setDataRoot("~/Data/test_comm_cpp");
 TEST_FUNC(test_names);
 TEST_FUNC(test_publisher);
 TEST_FUNC(test_syncer);
 TEST_FUNC(test_vector_message);
 TEST_FUNC(test_vecvec_message);
 TEST_FUNC(test_eigen_message);
}
