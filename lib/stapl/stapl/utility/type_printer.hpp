/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_UTILITY_TYPE_PRINTER_HPP
#define STAPL_UTILITY_TYPE_PRINTER_HPP

#include <stapl/runtime/system.hpp>
#include <iosfwd>
#include <string>
#include <typeinfo>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief Prints the given type through the @ref apply(std::ostream&) function.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_printer
{
  static std::ostream& apply(std::ostream& os)
  { return os << runtime::demangle(typeid(T).name()); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref type_printer for @c const types.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_printer<const T>
{
  static std::ostream& apply(std::ostream& os)
  { return type_printer<T>::apply(os) << " const"; }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref type_printer for @c volatile types.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_printer<volatile T>
{
  static std::ostream& apply(std::ostream& os)
  { return type_printer<T>::apply(os) << " volatile"; }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref type_printer for @c const @c volatile types.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_printer<const volatile T>
{
  static std::ostream& apply(std::ostream& os)
  { return type_printer<T>::apply(os) << " const volatile"; }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref type_printer for references.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_printer<T&>
{
  static std::ostream& apply(std::ostream& os)
  { return type_printer<T>::apply(os) << '&'; }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref type_printer for pointers.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_printer<T*>
{
  static std::ostream& apply(std::ostream& os)
  { return type_printer<T>::apply(os) << '*'; }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref type_printer for rvalue references.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_printer<T&&>
{
  static std::ostream& apply(std::ostream& os)
  { return type_printer<T>::apply(os) << "&&"; }
};


template<typename T>
std::ostream& operator<<(std::ostream& os, type_printer<T> const&)
{
  return type_printer<T>::apply(os);
}



////////////////////////////////////////////////////////////////////
/// @brief Prints the given list of types to the @c std::ostream.
///
/// A delimiter between the types can be specified at the constructor.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename... T>
struct typelist_printer;


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typelist_printer for two or more types.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename H, typename... T>
struct typelist_printer<H, T...>
{
  std::string delimiter;

  explicit typelist_printer(const char* delim = ", ")
  : delimiter(delim)
  { }

  explicit typelist_printer(std::string const& delim)
  : delimiter(delim)
  { }

  std::ostream& apply(std::ostream& os) const
  {
    return os << type_printer<H>() << delimiter
              << typelist_printer<T...>(delimiter);
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typelist_printer for one type.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct typelist_printer<T>
{
  explicit typelist_printer(const char* = ", ")
  { }

  explicit typelist_printer(std::string const&)
  { }

  static std::ostream& apply(std::ostream& os)
  { return os << type_printer<T>(); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typelist_printer for no types.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<>
struct typelist_printer<>
{
  explicit typelist_printer(const char* = ", ")
  { }

  explicit typelist_printer(std::string const&)
  { }

  static std::ostream& apply(std::ostream& os)
  { return os; }
};


template<typename... T>
std::ostream& operator<<(std::ostream& os, typelist_printer<T...> const& p)
{
  return p.apply(os);
}



////////////////////////////////////////////////////////////////////
/// @brief Creates a @ref typelist_printer from the list of objects.
///
/// @related typelist_printer
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename... U>
typelist_printer<U...> object_type_printer(U...)
{
  return typelist_printer<U...>();
}

} // namespace stapl

#endif
