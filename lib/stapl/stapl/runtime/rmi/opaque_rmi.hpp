/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_OPAQUE_RMI_HPP
#define STAPL_RUNTIME_RMI_OPAQUE_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../future.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../rmi_handle.hpp"
#include "../tags.hpp"
#include "../value_handle.hpp"
#include "../yield.hpp"
#include "../non_rmi/response.hpp"
#include "../request/sync_rmi_request.hpp"
#include "../type_traits/callable_traits.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include <memory>
#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous RMI primitive.
///
/// The given member function is called on the object in the destination
/// location.
///
/// If the return value is not needed, it is recommended that @ref async_rmi()
/// is used.
///
/// @param dest Destination location.
/// @param h    Handle to the target object.
/// @param pmf  Member function to invoke.
/// @param t    Arguments to pass to the member function.
///
/// @return A @ref future object with the return value of the invoked function.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
future<typename callable_traits<MemFun>::result_type>
opaque_rmi(unsigned int dest, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");
  STAPL_RUNTIME_ASSERT_MSG(h.is_valid(dest),
                           "p_object does not exist in destination" );

  typedef typename callable_traits<MemFun>::result_type result_type;
  typedef value_handle<result_type>                     return_handle_type;
  typedef response<return_handle_type>                  response_type;

  std::unique_ptr<return_handle_type> p{new return_handle_type};
  {
    STAPL_RUNTIME_PROFILE("opaque_rmi()", (primitive_traits::non_blocking |
                                           primitive_traits::ordered      |
                                           primitive_traits::p2p          |
                                           primitive_traits::comm));

    aggregator a{ctx, h, dest, no_implicit_flush};
    const bool on_shmem = a.is_on_shmem();

    if (on_shmem) {
      typedef sync_rmi_request<
                response_type, packed_handle_type,
                MemFun,
                typename transport_qualifier<decltype(t)>::type...
              > request_type;
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{*p, h, pmf, std::forward<T>(t)...};
    }
    else {
      typedef sync_rmi_request<
                response_type, packed_handle_type,
                MemFun,
                typename std::remove_reference<T>::type...
              > request_type;
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{*p, h, pmf, std::forward<T>(t)...};
    }
  }

  scheduling_point(ctx);
  return future<result_type>{std::move(p)};
}


//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous RMI primitive to all locations the object it exists on.
///
/// The given member function is called on the object in all the locations it
/// exists on. The return values are ordered by location id.
///
/// If the return values are not needed, it is recommended that @ref async_rmi()
/// is used.
///
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref futures object with the return values from each member
///         function invocation.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
futures<typename callable_traits<MemFun>::result_type>
opaque_rmi(all_locations_t, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  typedef typename callable_traits<MemFun>::result_type result_type;
  typedef values_handle<result_type>                    return_handle_type;
  typedef indexed_response<return_handle_type>          response_type;

  std::unique_ptr<return_handle_type>
    p{new return_handle_type{h.get_num_locations()}};
  {
    STAPL_RUNTIME_PROFILE("opaque_rmi(all_locations)",
                          (primitive_traits::non_blocking |
                           primitive_traits::ordered      |
                           primitive_traits::p2m          |
                           primitive_traits::comm));

    bcast_aggregator a{ctx, h};
    typedef sync_rmi_request<
              response_type,
              packed_handle_type,
              MemFun,
              typename std::remove_reference<T>::type...
            > request_type;
    const std::size_t size =
      request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{*p, h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
  return futures<result_type>{std::move(p)};
}


namespace unordered {

//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous RMI primitive to all locations the object it exists on.
///
/// The given member function is called on the object in all the locations it
/// exists on. The return values are ordered by location id.
///
/// If the return values are not needed, it is recommended that
/// @ref unordered::async_rmi() is used.
///
/// This is an unordered version of the @ref stapl::opaque_rmi() that may break
/// RMI ordering rules.
///
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref futures object with the return values from each member
///         function invocation.
///
/// @ingroup ARMIUnordered
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
futures<typename callable_traits<MemFun>::result_type>
opaque_rmi(all_locations_t, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  typedef typename callable_traits<MemFun>::result_type result_type;
  typedef values_handle<result_type>                    return_handle_type;
  typedef indexed_response<return_handle_type>          response_type;

  std::unique_ptr<return_handle_type>
    p{new return_handle_type{h.get_num_locations()}};
  {
    STAPL_RUNTIME_PROFILE("opaque_rmi(all_locations)",
                          (primitive_traits::non_blocking |
                           primitive_traits::unordered    |
                           primitive_traits::p2m          |
                           primitive_traits::comm));

    bcast_aggregator a{ctx, h, false};
    typedef sync_rmi_request<
              response_type,
              packed_handle_type,
              MemFun,
              typename std::remove_reference<T>::type...
            > request_type;
    const std::size_t size =
      request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{*p, h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
  return futures<result_type>{std::move(p)};
}

} // namespace unordered

} // namespace stapl

#endif
