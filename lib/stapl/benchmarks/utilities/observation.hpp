/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_OBSERVATION_HPP
#define STAPL_BENCHMARK_OBSERVATION_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a type is a string. By a string,
///        it checks for std::string and char*.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct is_string
    : std::integral_constant<
          bool,
          std::is_same<typename std::remove_cv<typename std::remove_reference<
                           typename std::remove_cv<T>::type>::type>::type,
                       std::string>::value ||
              std::is_same<typename std::remove_cv<
                               typename std::remove_pointer<T>::type>::type,
                           char>::value> {};

//////////////////////////////////////////////////////////////////////
/// @brief Abstract base class for all fields in an observation
//////////////////////////////////////////////////////////////////////
struct observation_field_base
{
  virtual std::string key() const = 0;
  virtual std::string value() const = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief A key-value pair that will be stored in an observation.
//////////////////////////////////////////////////////////////////////
template<typename T>
class observation_field
  : public observation_field_base
{
  std::string m_key;
  T m_value;

  std::string value_as_string(std::true_type) const
  {
    return m_value;
  }

  std::string value_as_string(std::false_type) const
  {
    std::stringstream ss;
    ss << m_value;
    return ss.str();
  }

public:
  observation_field(std::string k, T v)
    : m_key(std::move(k)), m_value(std::move(v))
  { }

  virtual std::string key() const
  {
    return m_key;
  }

  virtual std::string value() const
  {
    return this->value_as_string(is_string<T>{});
  }

};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief An observation (measured data for a benchmark) in the Tidy
///        Data send of the word. Used to capture statistics about a
///        run of a benchmark, typically time or resource usage, and
///        supports any polymorphic type to be added.
///
///        For example, if you want to report the results of a benchmark,
///        you can use it as follows:
///
///          observation o;
///
///          o("num_sources", 10);
///          o("algorithm", "BFS");
///          o("cutoff", 5.3);
///          o("passed", true);
///          o("time", 1.333);
///
///          std::cout << o << std::endl;
//////////////////////////////////////////////////////////////////////
struct observation
{
  std::vector<std::unique_ptr<detail::observation_field_base>> m_fields;

  template<typename T>
  void operator()(std::string key, T value)
  {
    m_fields.emplace_back(
      new detail::observation_field<T>(std::move(key), std::move(value))
    );
  }

  friend std::ostream& operator<<(std::ostream&, observation const&);
};

std::ostream& operator<<(std::ostream& os, observation const& o)
{
  os << "dbx.obs {";

  bool first = true;
  for (auto&& p : o.m_fields)
  {
    if (!first) os << ", ";
    first = false;
    os << "\"" << p->key() << "\": \"" << p->value() << "\"";
  }

  os << "}";

  return os;
}

} // namespace stapl

#endif
