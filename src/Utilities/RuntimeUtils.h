#ifndef RUNTIME_UTILS_H_
#define RUNTIME_UTILS_H_

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

///@name Runtime Utilities
///@{

////////////////////////////////////////////////////////////////////////////////
/// Describe the source-code location where an error occurs.
/// @param _file The file name.
/// @param _func The function name.
/// @param _line The line number.
/// @return A string describing the source-code location of a runtime error.
template <typename FilenameType, typename FunctionNameType, typename LineType>
std::string
WhereAt(FilenameType _file, FunctionNameType _func, LineType _line) {
  std::ostringstream oss;
  oss << "File: " << _file
      << "\n\tFunction: " << _func
      << "\n\tLine: " << _line;
  return oss.str();
};


////////////////////////////////////////////////////////////////////////////////
/// Macro for retrieving info about file, function, and line number.
#define WHERE WhereAt(__FILE__, __PRETTY_FUNCTION__, __LINE__)


////////////////////////////////////////////////////////////////////////////////
/// Terminate exection and display a message.
/// @param _msg The message to display.
inline
void
ExitMsg(const std::string& _msg) noexcept {
  std::cerr << _msg << std::endl;
  std::exit(-1);
}


////////////////////////////////////////////////////////////////////////////////
/// Terminate exection and display a message if the asserted condition is false.
/// @param _condition The condition to assert.
/// @param _msg The message to display on failure.
inline
void
AssertMsg(const bool _condition, const std::string& _msg) noexcept {
  if(!_condition)
    ExitMsg(_msg);
}

///@}

#endif
