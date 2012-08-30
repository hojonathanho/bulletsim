#include <ros/serialization.h>
#include <fstream>
#include <sstream>
namespace ser = ros::serialization;


template<class T>
boost::shared_array<uint8_t> serializeMsg(const T& msg, int& size) {
  size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[size]);
  ser::OStream stream(buffer.get(), size);
  ser::serialize(stream, msg);
  return buffer;
}

template<class T>
T deserializeMsg(boost::shared_array<uint8_t> buffer, int size) {
  T msg;
  ser::IStream stream(buffer.get(), size);
  ser::deserialize(stream, msg);
  return msg;
}

template<class T>
void msgToFile(const T& msg, const std::string& fname) {
  std::ofstream outfile(fname.c_str());
  int size;
  boost::shared_array<uint8_t> buffer = serializeMsg(msg, size);
  outfile << std::string((char*)buffer.get(), size);
}

template<class T>
T msgFromFile(const std::string& fname) {
  std::ifstream infile(fname.c_str());
  std::stringstream ss;
  ss << infile.rdbuf();
  std::string str = ss.str();
  boost::shared_array<uint8_t> buffer(new uint8_t[ss.str().size()]);
  memcpy(buffer.get(), str.c_str(), str.size());
  return deserializeMsg<T>(buffer, str.size());
}
