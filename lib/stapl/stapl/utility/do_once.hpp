/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_DO_ONCE_HPP
#define STAPL_UTILITY_DO_ONCE_HPP

#include <stapl/runtime.hpp>
#include <boost/optional.hpp>

namespace stapl {

namespace utility {

//////////////////////////////////////////////////////////////////////
/// @brief Helper unary function object that returns input value.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template<typename T>
struct identity
  : public p_object
{
  T operator()(T const& t) const
  {
    return t;
  }
};

} // namespace utility


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct for @ref do_once which uses partial specialization
///   to dispatch to correct implementation for void and non-void
///   return types of @p F.
/// @tparam F Function object passed to @ref do_once.
/// @tparam Rtn Return type of @p F.
/// @tparam Args List of arguments to pass to function object invocation.
//////////////////////////////////////////////////////////////////////
template<typename F, typename Rtn, typename...Args>
struct do_once_impl
{
  static Rtn apply(F&& f, Args&&... args)
  {
    typedef utility::identity<Rtn> identity_type;

    identity_type io;

    boost::optional<Rtn> ret;

    rmi_fence();

    const bool root = (get_location_id() == 0);
    if (root)
    {
      gang g;
      ret = f(std::forward<Args>(args)...);
      rmi_fence();
    }

    rmi_fence();

    if (root)
      return broadcast_rmi(root_location, io.get_rmi_handle(),
                           &identity_type::operator(), *ret).get();
    else
      return broadcast_rmi(0, &identity_type::operator()).get();
  }
};


template<typename F, typename...Args>
struct do_once_impl<F, void, Args...>
{
  static void apply(F&& f, Args&&... args)
  {
    rmi_fence();

    if (get_location_id() == 0)
    {
      gang g;
      f(std::forward<Args>(args)...);
      rmi_fence();
    }

    rmi_fence();
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief When called in SPMD code section, invokes specified function
///   once globally (i.e., on one and only one location of computation).
/// @param f Function object to be called once.
/// @param args List of arguments to pass to functor invocation.
/// @ingroup utility
///
/// Right now, location 0 always invokes function.
/// Primary template matches function objects.
//////////////////////////////////////////////////////////////////////
template<typename F, typename... Args>
auto do_once(F&& f, Args&&... args)
  ->decltype(f(std::forward<Args>(args)...))
{
  typedef decltype(f(std::forward<Args>(args)...)) return_type;

  return detail::do_once_impl<F, return_type, Args...>::apply(
    std::forward<F>(f), std::forward<Args>(args)...
  );
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature matching pointer to function with void return type.
/// @param fun Pointer to C function returning void.
/// @param args List of arguments to pass to function call.
//////////////////////////////////////////////////////////////////////
template<typename... Args>
void do_once(void (*fun)(Args...), Args... args)
{
  rmi_fence();

  if (get_location_id() == 0)
  {
    gang g;
    fun(std::forward<Args>(args)...);
    rmi_fence();
  }

  rmi_fence();
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature matching pointer to function with non-void return type.
/// @param fun Pointer to C function with return type @p Rtn.
/// @param args List of arguments to pass to function call.
//////////////////////////////////////////////////////////////////////
template<typename Rtn, typename... Args>
Rtn do_once(Rtn (*fun)(Args...), Args... args)
{
  typedef utility::identity<Rtn> identity_type;

  identity_type io;

  boost::optional<Rtn> ret;

  rmi_fence();

  const bool root = (get_location_id() == 0);
  if (root)
  {
    gang g;
    ret = fun(std::forward<Args>(args)...);
    rmi_fence();
  }
  rmi_fence();

  if (root)
    return broadcast_rmi(root_location, io.get_rmi_handle(),
                         &identity_type::operator(), *ret).get();
  else
    return broadcast_rmi(0, &identity_type::operator()).get();
}

} // namespace stapl

#endif // STAPL_UTILITY_DO_ONCE_HPP

