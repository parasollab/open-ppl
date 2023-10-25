/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_C_STRING_HPP
#define STAPL_RUNTIME_UTILITY_C_STRING_HPP

#include <cstring>
#include <cstdlib>
#include <new>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Wraps a C string into a class.
///
/// Provides functions to clone and find the size of the C string.
///
/// @ingroup runtimeUtility
////////////////////////////////////////////////////////////////////
class c_string
{
public:
  typedef std::size_t size_type;
private:
  typedef char        char_type;

  size_type  m_size;
  char_type* m_string;

  static char_type* clone_string(const char_type* s, const std::size_t size)
  {
    const std::size_t sz = (size+1);
    void* p = std::malloc(sz*sizeof(char_type));
    if (!p)
      throw std::bad_alloc();
    return static_cast<char_type*>(std::memcpy(p, s, (sz*sizeof(char_type))));
  }

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Creates a new object from the given array.
  ///
  /// It will allocate and copy the string internally.
  ////////////////////////////////////////////////////////////////////
  explicit c_string(const char_type* s = nullptr)
  : m_size(0),
    m_string(nullptr)
  {
    if (!s)
      return;
    m_size   = std::strlen(s);
    m_string = clone_string(s, m_size);
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Creates a new object from the given array.
  ///
  /// It will allocate and copy the string internally. The length of the string
  /// is given by the length parameter.
  ////////////////////////////////////////////////////////////////////
  c_string(const char_type* s, const std::size_t length)
  : m_size(length),
    m_string(nullptr)
  {
    if (!s) {
      m_size = 0;
      return;
    }
    m_string = clone_string(s, m_size);
  }

  c_string(c_string const& other)
  : m_size(other.m_size),
    m_string(nullptr)
  {
    if (m_size==0)
      return;
    m_string = clone_string(other.m_string, m_size);
  }

  c_string(c_string&& other) noexcept
  : m_size(other.m_size),
    m_string(other.m_string)
  {
    other.m_size   = 0;
    other.m_string = nullptr;
  }

  ~c_string(void)
  { std::free(m_string); }

  c_string& operator=(c_string const& other)
  {
    if (&other!=this) {
      std::free(m_string);
      m_size = other.m_size;
      if (m_size==0)
        m_string = nullptr;
      else
        m_string = clone_string(other.m_string, m_size);
    }
    return *this;
  }

  c_string& operator=(c_string&& other) noexcept
  {
    if (&other!=this) {
      std::free(m_string);
      m_size         = other.m_size;
      m_string       = other.m_string;
      other.m_size   = 0;
      other.m_string = nullptr;
    }
    return *this;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the size of the string.
  ///
  /// @return The length of the stored string without the @c \0 .
  ///
  /// @see c_string::length()
  ////////////////////////////////////////////////////////////////////
  size_type size(void) const noexcept
  { return m_size; }

  ////////////////////////////////////////////////////////////////////
  /// @copydoc size() const
  /// @see c_string::size()
  ////////////////////////////////////////////////////////////////////
  size_type length(void) const noexcept
  { return m_size; }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the internal C representation of the string.
  ///
  /// The string returned is @c '\0' terminated and is valid as long as the
  /// associated c_string object is alive.
  ////////////////////////////////////////////////////////////////////
  const char_type* c_str(void) const noexcept
  { return m_string; }

  operator const char_type*(void) noexcept
  { return m_string; }

  ////////////////////////////////////////////////////////////////////
  /// @brief Resets the c_string with the given string and size.
  ///
  /// @see c_string::c_string()
  ////////////////////////////////////////////////////////////////////
  void reset(char_type* s = nullptr, const std::size_t len = 0)
  {
    std::free(m_string);
    if (!s) {
      m_size   = 0;
      m_string = nullptr;
    }
    else {
      if (len==0)
        m_size = std::strlen(s);
      else
        m_size = len;
      m_string = s;
    }
  }
};

} // namespace runtime

} // namespace stapl

#endif
