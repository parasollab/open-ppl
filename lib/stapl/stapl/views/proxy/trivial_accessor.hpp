/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TRIVIAL_ACCESSOR_HPP
#define STAPL_VIEWS_TRIVIAL_ACCESSOR_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an accessor over a constant value.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct trivial_accessor
{
private:
  const T m_t;

public:
  using value_type = T;

  explicit trivial_accessor(T const& t)
    : m_t(t)
  { }

  explicit trivial_accessor(T&& t)
    : m_t(std::move(t))
  { }

  bool is_local(void) const
  {
    return true;
  }

  value_type read(void) const
  {
    return m_t;
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const pmf)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    return (m_t.*pmf)(args...);
  }

  void define_type(typer &t)
  {
    t.member(m_t);
  }
}; // struct trivial_accessor

template<typename T>
struct accessor_traits<trivial_accessor<T>>
{
  typedef std::true_type is_localized;
};

} // namespace stapl

#endif // STAPL_VIEWS_TRIVIAL_ACCESSOR_HPP
