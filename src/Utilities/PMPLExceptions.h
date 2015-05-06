#ifndef PMPLEXCEPTIONS_H_
#define PMPLEXCEPTIONS_H_

#include <string>
#include <sstream>
#include <stdexcept>
using namespace std;

template<typename X, typename Y, typename Z>
string WhereAt(X x, Y y, Z z) {
  ostringstream oss;
  oss << "File: " << x << "\n\tFunction: " << y << "\n\tLine: " << z;
  return oss.str();
};

#define WHERE WhereAt(__FILE__, __PRETTY_FUNCTION__, __LINE__)

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Exceptions
/// @brief Base exception class for PMPL use.
///
/// PMPLExceptions are objects which methods can throw for their various errors.
/// These are all runtime errors and facilitate recovery for programs like
/// Vizmo and aid users in identifying where and how errors occur.
////////////////////////////////////////////////////////////////////////////////
class PMPLException : public runtime_error {
  public:
    PMPLException(const string& _type, const string& _where, const string& _message) :
      runtime_error(
          "\nError:\n\t" + _type +
          "\nWhere:\n\t" + _where +
          "\nWhy:\n\t" + _message +
          "\n") {
      }
};

class ParseException : public PMPLException {
  public:
    ParseException(const string& _where, const string& _message) :
      PMPLException("Parse Exception", _where, _message) {}
};

class WriteException : public PMPLException {
  public:
    WriteException(const string& _where, const string& _message) :
      PMPLException("Write Exception", _where, _message) {}
};

class RunTimeException : public PMPLException {
  public:
    RunTimeException(const string& _where, const string& _message) :
      PMPLException("Runtime Exception", _where, _message) {}
};

#endif
