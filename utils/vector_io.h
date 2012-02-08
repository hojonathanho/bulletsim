#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "my_exceptions.h"
using namespace std;

#define IO_DECL(T)							\
  ostream &operator<<(ostream &stream, const vector<T>& v);		\
  istream &operator>>(istream &stream, vector<T>& v);			\
  ostream &operator<<(ostream &stream, const vector< vector<T> >& vv);	\
  istream &operator>>(istream &stream, vector< vector<T> >& vv);	\
  vector<T> T##VecFromFile(string fname);				\
  vector< vector<T> > T##MatFromFile(string fname);			\
  void vecToFile(string fname, const vector<T> in);			\
  void matToFile(string fname, const vector< vector<T> >& in) ;		\


IO_DECL(int);
IO_DECL(float);
IO_DECL(double);

#undef IO_DECL
