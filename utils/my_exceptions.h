#pragma once
#include <exception>
#include <string>
#include <cstdio>
using namespace std;

struct IOError :  exception {};

struct FileOpenError : IOError {
  string m_filename;
  FileOpenError(const string& filename="") : m_filename(filename) {}
  const char* what() const throw() {
    return ("could not open " + m_filename).c_str();
  }
  ~FileOpenError() throw() {};
};

struct FileParseError : IOError {
  string m_filename;
  FileParseError(const string& filename="") : m_filename(filename) {}
  const char* what() const throw() {
    return ("unable to parse " + m_filename).c_str();
  }
  ~FileParseError() throw() {};
};

struct StopIteration : exception {
  StopIteration() {};
  ~StopIteration() throw() {};
};
