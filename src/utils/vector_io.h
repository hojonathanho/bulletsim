#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "my_exceptions.h"

#define IO_DECL(T)							\
  std::ostream &operator<<(std::ostream &stream, const std::vector<T>& v);		\
  std::istream &operator>>(std::istream &stream, std::vector<T>& v);			\
  std::ostream &operator<<(std::ostream &stream, const std::vector< std::vector<T> >& vv);	\
  std::istream &operator>>(std::istream &stream, std::vector< std::vector<T> >& vv);	\
  std::vector<T> T##VecFromFile(std::string fname);				\
  std::vector< std::vector<T> > T##MatFromFile(std::string fname);			\
  void vecToFile(std::string fname, const std::vector<T> in);			\
  void matToFile(std::string fname, const std::vector< std::vector<T> >& in) ;		\


IO_DECL(int);
IO_DECL(float);
IO_DECL(double);

#undef IO_DECL
