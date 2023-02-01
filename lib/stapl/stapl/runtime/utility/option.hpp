/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_OPTION_HPP
#define STAPL_RUNTIME_UTILITY_OPTION_HPP

#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

namespace stapl {

void abort(std::string const&);


//////////////////////////////////////////////////////////////////////
/// @brief Used to create a list of options.
///
/// The @ref option object is a map of keys and objects. The objects can be
/// retrieved with the key, but they have to be cast to the right type.
/// Depending on the function, if the object does not exist, a default value can
/// be returned or a value from the environment.
///
/// You can create a list of options by combining multiple @ref option objects:
/// @code
/// option opts = option("key1", int(2)) & option("key2", "object");
/// @endcode
///
/// You can also pass the @c argc and @c argv from @c main() using the
/// following:
/// @code
/// option opts = option(argc, argv) & option("key2", "object");
/// @endcode
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
class option
{
private:
  typedef std::map<std::string, boost::any> map_type;
public:
  typedef map_type::key_type                key_type;
  typedef map_type::size_type               size_type;

private:
  map_type m_opts;
  int*     m_argc;
  char***  m_argv;

public:
  option(void)
  : m_argc(nullptr),
    m_argv(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref option object with the @c argc and @c argv from
  ///        a @c main() function.
  //////////////////////////////////////////////////////////////////////
  option(int& argc, char**& argv)
  : m_argc(&argc),
    m_argv(&argv)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref option object with the string associated with
  ///        the key.
  ///
  /// @param key Key to add to the map of options.
  /// @param s   String to associate with the @p key.
  //////////////////////////////////////////////////////////////////////
  option(key_type const& key, const char* s)
  : m_argc(nullptr),
    m_argv(nullptr)
  { m_opts[key] = std::string{s}; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref option object with the object associated with
  ///        the key.
  ///
  /// @param key Key to add to the map of options.
  /// @param t   Object to associate with the @p key.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  option(key_type const& key, T&& t)
  : m_argc(nullptr),
    m_argv(nullptr)
  { m_opts[key] = std::forward<T>(t); }

  bool has_argv(void) const noexcept
  { return m_argc; }

  int& get_argc(void) const noexcept
  { return *m_argc; }

  char**& get_argv(void) const noexcept
  { return *m_argv; }

  size_type count(key_type const& key) const noexcept
  {
    const size_type c = m_opts.count(key);
    if (c>0)
      return c;
    char* const var = std::getenv(key.c_str());
    return (!var ? 0 : 1);
  }

  size_type count(const char* key) const noexcept
  { return count(std::string(key)); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the value associated with the key if it is in the options
  ///        map.
  ///
  /// @tparam T Type of the object associated with the key.
  ///
  /// This function will not attempt return an option from the environment.
  ///
  /// @param key Key of the required option.
  /// @param t   Space to save the object that is associated with the key.
  ///
  /// @return @c true if the key exists in the map and the object saved to @p t,
  ///         otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  bool try_get_noenv(key_type const& key, T& t) const
  {
    auto it = m_opts.find(key);
    if (it==m_opts.end())
      return false;
    t = boost::any_cast<T>(it->second);
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the object associated with the key if it is in the options
  ///        map.
  ///
  /// @tparam T Type of the object associated with the key.
  ///
  /// This function will attempt to return an option from the environment.
  ///
  /// @param key Key of the required option.
  /// @param t   Space to save the object that is associated with the key.
  ///
  /// @return @c true if the key exists in the map and the object saved to @p t,
  ///         otherwise @c false.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  bool try_get(key_type const& key, T& t) const
  {
    // try to find the value already stored
    if (try_get_noenv(key, t))
      return true;
    // try to load envariable
    char* const var = std::getenv(key.c_str());
    try {
      if (var) {
        t = boost::lexical_cast<T>(var);
        return true;
      }
    }
    catch (boost::bad_lexical_cast&) {
      std::ostringstream oss;
      oss << key << ": incorrect value \"" << var << "\"";
      abort(oss.str());
    }
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the C string associated with the key if it is in the
  ///        options map or the default value if it is not.
  ///
  /// This function will attempt to return an option from the environment.
  ///
  /// @param key           Key of the required option.
  /// @param default_value String to return if the key is not in the options map
  ///                      or the environment.
  ///
  /// @return The string associated with the key.
  //////////////////////////////////////////////////////////////////////
  const char* get(key_type const& key, const char* default_value) const
  {
    // try to find the value already stored
    auto it = m_opts.find(key);
    if (it!=m_opts.end())
      return boost::any_cast<const char*>(it->second);

    // try to load envariable
    char* const var = std::getenv(key.c_str());
    if (var)
      return var;

    // return default
    return default_value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the object associated with the key if it is in the options
  ///        map or the default value if it is not.
  ///
  /// @tparam T Type of the object associated with the key.
  ///
  /// This function will attempt to return an option from the environment.
  ///
  /// @param key           Key of the required option.
  /// @param default_value Object to return if the key is not in the options map
  ///                      or the environment.
  ///
  /// @return The object associated with the key.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  T get(key_type const& key, T const& default_value) const
  {
    // try to find the value already stored
    auto it = m_opts.find(key);
    if (it!=m_opts.end())
      return boost::any_cast<T>(it->second);

    // try to load envariable
    char* const var = std::getenv(key.c_str());
    try {
      if (var)
        return boost::lexical_cast<T>(var);
    }
    catch (boost::bad_lexical_cast&) {
      std::ostringstream oss;
      oss << key << ": incorrect value \"" << var << "\"";
      abort(oss.str());
    }

    // return default
    return default_value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Merges @c *this with @p other, to create a bigger option map.
  //////////////////////////////////////////////////////////////////////
  void merge(option const& other)
  {
    m_opts.insert(other.m_opts.begin(), other.m_opts.end());
    if (!has_argv() && other.has_argv()) {
      m_argc = other.m_argc;
      m_argv = other.m_argv;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Merges @p x with @p y, to create a bigger option map.
  //////////////////////////////////////////////////////////////////////
  friend option operator&(option const& x, option y)
  {
    y.merge(x);
    return y;
  }
};

} // namespace stapl

#endif
