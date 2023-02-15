/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

////////////////////////////////////////////////////////////////////////////////
/// @file
/// Common work functions used by the profilers.
///
/// @todo Add more complicated set/get functors.
////////////////////////////////////////////////////////////////////////////////

#ifndef TEST_CONTAINERS_PROFILE_WORKFUNCTIONS_HPP_
#define TEST_CONTAINERS_PROFILE_WORKFUNCTIONS_HPP_

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Sets an element to a given value.
////////////////////////////////////////////////////////////////////////////////
template <class T>
struct set_op
{
  set_op(T val)
    : m_val(std::move(val))
  { }

  template<typename Reference>
  void operator()(Reference&& elem) const
  {
    elem = m_val;
  }

  T const& result() const
  {
    return m_val;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_val);
  }

private:
  T m_val;
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns the element passed in (possibly unwrapping it from
/// its proxy representation).
////////////////////////////////////////////////////////////////////////////////
template <class T>
struct get_op
{
  using result_type = T;

  template<typename Reference>
  T operator()(Reference&& elem) const
  {
    return elem;
  }
};

} // namespace profiling

} // namespace stapl

#endif /* TEST_CONTAINERS_PROFILE_WORKFUNCTIONS_HPP_ */
