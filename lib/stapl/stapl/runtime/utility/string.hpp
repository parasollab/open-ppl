/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_STRING_HPP
#define STAPL_RUNTIME_UTILITY_STRING_HPP

#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Splits the given @c std::string to a vector of @p T, based on the
///        delimiters.
///
/// @param s      String to split.
/// @param delims List of delimiters to split the string with.
///
/// @return An @c std::vector of the tokens or an empty @c std::vector if an
///         error occurred.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T>
std::vector<T> split_string_to_vector(std::string const& s, const char* delims)
{
  using separator_type = boost::char_separator<char>;
  using tokenizer_type = boost::tokenizer<separator_type>;

  tokenizer_type tokens{s, separator_type{delims}};
  std::vector<T> v;
  for (auto&& t : tokens) {
    try {
      v.push_back(boost::lexical_cast<T>(t));
    }
    catch (boost::bad_lexical_cast const&) {
      return std::vector<T>{};
    }
  }
  return v;
}

} // namespace runtime

} // namespace stapl

#endif
