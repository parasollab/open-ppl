#ifndef PMPL_EXCEPTIONS_H_
#define PMPL_EXCEPTIONS_H_

#include <string>
#include <sstream>
#include <stdexcept>

#include "Utilities/RuntimeUtils.h"

using namespace std;


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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Exceptions
/// @brief Parse exceptions
////////////////////////////////////////////////////////////////////////////////
class ParseException : public PMPLException {
  public:
    ParseException(const string& _where, const string& _message) :
      PMPLException("Parse Exception", _where, _message) {}
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Exceptions
/// @brief Write exceptions
////////////////////////////////////////////////////////////////////////////////
class WriteException : public PMPLException {
  public:
    WriteException(const string& _where, const string& _message) :
      PMPLException("Write Exception", _where, _message) {}
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Exceptions
/// @brief Run time exceptions
////////////////////////////////////////////////////////////////////////////////
class RunTimeException : public PMPLException {
  public:
    RunTimeException(const string& _where, const string& _message) :
      PMPLException("Runtime Exception", _where, _message) {}
};

#endif
