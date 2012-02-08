#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include "my_exceptions.h"
using namespace std;




#define IO_IMPL(T)							\
  ostream &operator<<(ostream &stream, const vector<T>& v)		\
  {									\
    for (int i=0; i < v.size(); i++) stream << v[i] << " ";		\
    return stream;							\
  }									\
  istream &operator>>(istream &stream, vector<T>& v)			\
  {									\
    T x;								\
    while (!stream.eof()) {						\
      stream >> x;							\
      if (!stream.fail()) v.push_back(x);				\
    }									\
    return stream;							\
  }									\
  ostream &operator<<(ostream &stream, const vector< vector<T> >& vv)	\
  {									\
    for (int i=0; i < vv.size(); i++) stream << vv[i] << "\n";		\
    return stream;							\
  }									\
  istream &operator>>(istream &stream, vector< vector<T> >& vv)		\
  {									\
    string line;							\
    vector<T> v;							\
    while (!stream.eof()) {						\
      getline(stream, line, '\n');					\
      stringstream ss(stringstream::in | stringstream::out);		\
      ss << line;							\
      v.clear();							\
      ss >> v;								\
      if (v.size() > 0) vv.push_back(v);				\
    }									\
    return stream;							\
  }									\
  vector<T> T##VecFromFile(string fname) {				\
    ifstream infile(fname.c_str());					\
    if (infile.fail()) throw FileOpenError(fname);			\
    vector<T> out;							\
    operator>>(infile,out);						\
    return out;								\
  }									\
  vector< vector<T> > T##MatFromFile(string fname) {			\
    ifstream infile(fname.c_str());					\
    if (infile.fail()) throw FileOpenError(fname);			\
    vector< vector<T> > out;						\
    operator>>(infile,out);						\
    infile.close();							\
    return out;								\
  }									\
  void vecToFile(string fname, const vector<T> in) {			\
    ofstream outfile(fname.c_str());					\
    if (outfile.fail()) throw FileOpenError(fname);			\
    outfile << in;							\
    outfile.close();							\
  }									\
  void matToFile(string fname, const vector< vector<T> >& in) {		\
    ofstream outfile(fname.c_str());					\
    if (outfile.fail()) throw FileOpenError(fname);			\
    outfile << in;							\
    outfile.close();							\
  }									\


IO_IMPL(int)
IO_IMPL(float)
IO_IMPL(double)

#undef IO_IMPL
