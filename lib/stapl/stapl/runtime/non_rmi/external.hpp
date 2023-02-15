/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NON_RMI_EXTERNAL_HPP
#define STAPL_RUNTIME_NON_RMI_EXTERNAL_HPP

#include "../context.hpp"
#include "../instrumentation.hpp"
#include "../runqueue.hpp"
#include "../collective/barrier_object.hpp"
#include <functional>
#include <set>
#include <type_traits>
#include <utility>
#include <boost/optional.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that calls a function passed from
///        @ref external_call().
///
/// @tparam R Function result type.
/// @tparam T Argument types.
///
/// It performs a barrier to make sure that all locations are on the same point
/// in execution and then locks the communication library so that no other
/// primitive can use it.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R>
struct external_caller
{
  using result_type = boost::optional<R>;

  template<typename F, typename... T>
  static result_type apply(F&& f, T&&... t)
  {
    auto* const ctx = this_context::try_get();
    if (ctx) {
      // wait for all locations to arrive
      barrier_object b{*ctx};
      b();
      b.wait();

      // only the leader will call f
      if (!ctx->get_location_md().is_leader())
        return boost::optional<R>{};
    }

    // call the function
    runqueue::lock_type lock;
    return boost::optional<R>{f(std::forward<T>(t)...)};
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref external_caller for @c void return type.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<>
struct external_caller<void>
{
  using result_type = bool;

  template<typename F, typename... T>
  static result_type apply(F&& f, T&&... t)
  {
    auto* const ctx = this_context::try_get();
    if (ctx) {
      // wait for all locations to arrive
      barrier_object b{*ctx};
      b();
      b.wait();

      // only the leader will call f
      if (!ctx->get_location_md().is_leader())
        return false;
    }

    // call the function
    runqueue::lock_type lock;
    f(std::forward<T>(t)...);
    return true;
  }
};

} // namespace runtime


////////////////////////////////////////////////////////////////////
/// @brief Returns the location ids that are going to make the external call.
///
/// @ingroup ARMI
////////////////////////////////////////////////////////////////////
std::set<unsigned int> external_callers(void);


//////////////////////////////////////////////////////////////////////
/// @brief Calls an external library function.
///
/// @tparam F Function type.
/// @tparam T Argument types.
///
/// This function is useful for calling functions that are not STAPL-aware or
/// thread-safe, such as MPI-based libraries. It is going to call @p f only from
/// one location per process.
///
/// It is the user's responsibility to call the @ref external_call() in a gang
/// that @p f can be called correctly. Most of the times @ref external_call()
/// should be called in @ref stapl_main().
///
/// @warning Calling any runtime primitive inside @p f is undefined behavior.
///
/// @param f External library function to be called.
/// @param t Arguments to pass to the function.
///
/// @return If @p R is not @c void, the result of @p f(t...) is returned in a
///         @c boost::optional<R> which has a value in all locations that @p f
///         has been called.
///         If @p R is @c void, then it returns @c true in all locations that
///         @p f has been called, otherwise @c false.
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
template<typename F, typename... T>
typename runtime::external_caller<
  typename std::result_of<F(T...)>::type
>::result_type
external_call(F&& f, T&&... t)
{
  using namespace stapl::runtime;

  STAPL_RUNTIME_PROFILE("external_call()", (primitive_traits::blocking |
                                            primitive_traits::coll     |
                                            primitive_traits::sync));

  using wf_type = external_caller<typename std::result_of<F(T...)>::type>;
  return wf_type::apply(std::forward<F>(f), std::forward<T>(t)...);
}

} // namespace stapl

#endif
