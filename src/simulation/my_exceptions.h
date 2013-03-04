#pragma once
#include <exception>
#include <string>
#include <cstdio>

struct IOError :  std::exception {};

struct FileOpenError : IOError {
  std::string m_filename;
  FileOpenError(const std::string& filename="") : m_filename(filename) {}
  const char* what() const throw() {
    return ("could not open " + m_filename).c_str();
  }
  ~FileOpenError() throw() {};
};

struct FileParseError : IOError {
  std::string m_filename;
  FileParseError(const std::string& filename="") : m_filename(filename) {}
  const char* what() const throw() {
    return ("unable to parse " + m_filename).c_str();
  }
  ~FileParseError() throw() {};
};

struct StopIteration : std::exception {
  StopIteration() {};
  ~StopIteration() throw() {};
};
