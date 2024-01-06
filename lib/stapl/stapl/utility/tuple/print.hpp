/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_PRINT_HPP
#define STAPL_UTILITY_TUPLE_PRINT_HPP

#include <ostream>
#include <stapl/utility/tuple/tuple.hpp>
#include <string>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Static functor that uses template recursion to print all
///  elements of a tuple with a comma delimiter.
//////////////////////////////////////////////////////////////////////
template<typename Tuple, int Index, int LastIndex>
struct print_tuple_impl
{
  static void apply(std::ostream& os, std::string sep, const Tuple& value)
  {
    os << std::get<Index>(value) << sep;

    print_tuple_impl<Tuple, Index + 1, LastIndex>::apply(os, sep, value);
  }
};


template<typename Tuple, int Index>
struct print_tuple_impl<Tuple, Index, Index>
{
  static void apply(std::ostream& os, std::string sep, const Tuple& value)
  {
    os << std::get<Index>(value);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Fallback implementation of print_tuple for an empty tuple.
/// @param os Output stream to write to.
//////////////////////////////////////////////////////////////////////
inline
std::ostream&
print_tuple(std::ostream& os, tuple<> const&)
{
  return os << "( )";
}

//////////////////////////////////////////////////////////////////////
/// @brief Simple tuple printer with default separator ','
/// @param os Output stream to write tuple to.
/// @param t The tuple to be printed.
//////////////////////////////////////////////////////////////////////
template<typename ...Args>
std::ostream&
print_tuple(std::ostream& os, tuple<Args...> const& t)
{
  os << "(";
  print_tuple_impl<
    tuple<Args...>, 0, sizeof...(Args)-1>::apply(os, ", ", t);
  return os << ")";
}

//////////////////////////////////////////////////////////////////////
/// @brief Simple tuple printer.
/// @param os Output stream to write tuple to.
/// @param sep Separator character
/// @param t The tuple to be printed.
//////////////////////////////////////////////////////////////////////
template<typename ...Args>
std::ostream&
print_tuple(std::ostream& os, std::string sep, tuple<Args...> const& t)
{
  os << "(";
  print_tuple_impl<
    tuple<Args...>, 0, sizeof...(Args)-1>::apply(os, sep, t);
  return os << ")";
}

template<typename T>
std::ostream&
print_tuple(std::ostream& os, T const& t)
{
  return os << t;
}

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_PRINT_HPP
