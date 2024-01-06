/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//////////////////////////////////////////////////////////////////////
/// @file printers.hpp
/// @brief Standalone functions for skeletons framework debugging.
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_SKELETONS_UTILITY_PRINTERS_HPP
#define STAPL_SKELETONS_UTILITY_PRINTERS_HPP

#include <ostream>
#include <fstream>
#include <iomanip>
#include <type_traits>
#include <stapl/runtime/system.hpp>
#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {
namespace skeletons {

inline void replace_string(std::string& text,
                           std::string oldstr, std::string newstr)
{
  std::size_t pos = 0;
  while ((pos = text.find(oldstr, pos)) != std::string::npos)
  {
    text.replace(pos, oldstr.length(), newstr);
     pos += newstr.length();
  }
}

template <typename Element, typename O>
void show_demangled_trimmed(O& o = std::cout)
{
  std::string name(stapl::runtime::demangle(typeid(Element).name()));
  replace_string(name, "spans::", "");
  replace_string(name, "elem_f::", "");
  replace_string(name, "stapl::skeletons::flows::", "");
  replace_string(name, "stapl::skeletons::flows::", "");
  replace_string(name, "stapl::skeletons::", "");
  replace_string(name, "skeletons_impl::", "");
  replace_string(name, "stapl::", "");
  replace_string(name, "prange_impl::", "");
  replace_string(name, "boost::fusion::", "");
  replace_string(name, "boost::", "");
  if (name.length() > 150)
    o << name.substr(0, 150) << "...";
  else
    o << name;
}

template <typename Element, typename O = std::ostream>
void show_demangled(O& o = std::cout)
{
  show_demangled_trimmed<Element>(o);
}

template<int index>
struct show_tuple
{
  template <typename... T, typename O>
  static void call(tuple<T...> const& t, O& o)
  {
    show_tuple<index-1>::call(t, o);
    o << stapl::get<index-1>(t) << " ";
  }
};

template<>
struct show_tuple<0>
{
  template <typename... T, typename O>
  static void call(tuple<T...> const& t, O& o)
  { }
};

template <typename Item, typename O = std::ostream>
void show(O& o = std::cout)
{
  show_demangled<Item>(o);
}

template <typename Item, typename O = std::ostream>
void show(Item&&, O& o = std::cout)
{
  show_demangled<Item>(o);
}

template <typename... Args, typename O = std::ostream>
void show_value(stapl::tuple<Args...> const& t, O& o = std::cout)
{
  show_tuple<sizeof...(Args)>::call(t, o);
}

template <typename T, typename O = std::ostream>
void show_value(T const& t, O& o = std::cout)
{
  o << t;
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_PRINTERS_HPP
