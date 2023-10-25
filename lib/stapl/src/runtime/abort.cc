/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/runtime.hpp>
#include <iostream>
#include <cstdlib>
#include <sstream>

namespace stapl {

namespace runtime {

// Prints the given arguments and aborts
void assert_fail(const char* s,
                 const char* file, unsigned int line, const char* function)
{
  std::ostringstream oss;
  oss << s
      << " (file: "     << file
      << ", function: " << function
      << ", line: "     << line
      << ")";
  abort(oss.str());
}


// Prints the given argument and aborts
void assert_fail(const char* s, const char* function)
{
  std::ostringstream oss;
  oss << s << " (function: " << function << ")";
  abort(oss.str());
}


// Prints the given arguments
void warning(const char* s, const char* function)
{
  std::ostringstream oss;
  if (!is_initialized())
    oss << "process [" << getpid() << "] ";
  else
    oss << '[' << get_location_id() << "] ";
  oss << s
      << " (function: " << function << ")";
  std::cerr << oss.str() << std::endl;
}


// Prints the given arguments
void warning(const char* s,
             const char* file, unsigned int line, const char* function)
{
  std::ostringstream oss;
  if (!is_initialized())
    oss << "process [" << getpid() << "] ";
  else
    oss << '[' << get_location_id() << "] ";
  oss << s
      << " (file: "     << file
      << ", function: " << function
      << ", line: "     << line
      << ")";
  std::cerr << oss.str() << std::endl;
}

} // namespace runtime


// Prints the given string and aborts execution
void abort(std::string const& err)
{
  std::ostringstream oss;
  if (!is_initialized())
    oss << "process [" << getpid() << "] ";
  else
    oss << '[' << get_location_id() << "] ";
  oss << "Error - Aborting.";
  if (!err.empty())
    oss << ' ' << err;
  std::cerr << oss.str() << std::endl;
  std::abort();
}

} // namespace stapl
