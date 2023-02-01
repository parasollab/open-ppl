/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_SYNC_RMI_HPP
#define STAPL_RUNTIME_RMI_SYNC_RMI_HPP

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
#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Synchronous RMI primitive.
///
/// The given member function is called on the object in the destination
/// location.
///
/// @warning This is a function that can harm scalability. It exists to
///          facilitate one-sided synchronization. Use @ref opaque_rmi() or
///          @ref async_rmi() if possible.
///
/// @param dest Destination location.
/// @param h    Handle to the target object.
/// @param pmf  Member function to invoke.
/// @param t    Arguments to pass to the member function.
///
/// @return The return value of the invoked function.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
typename callable_traits<MemFun>::result_type
sync_rmi(unsigned int dest, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");
  STAPL_RUNTIME_ASSERT_MSG(h.is_valid(dest),
                           "p_object does not exist in destination");

  STAPL_RUNTIME_PROFILE("sync_rmi()", (primitive_traits::blocking |
                                       primitive_traits::ordered  |
                                       primitive_traits::p2p      |
                                       primitive_traits::comm));

  typedef typename callable_traits<MemFun>::result_type result_type;
  typedef value_handle<result_type>                     return_handle_type;
  typedef response<return_handle_type>                  response_type;

  return_handle_type rh;
  {
    aggregator a{ctx, h, dest};
    const bool on_shmem = a.is_on_shmem();

    if (on_shmem) {
      typedef sync_rmi_request<
                response_type,
                packed_handle_type,
                MemFun,
                typename transport_qualifier<decltype(t)>::type...
              > request_type;
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{rh, h, pmf, std::forward<T>(t)...};
    }
    else {
      typedef sync_rmi_request<
                response_type,
                packed_handle_type,
                MemFun,
                typename std::remove_reference<T>::type...
              > request_type;
      const std::size_t size =
        request_type::expected_size(std::forward<T>(t)...);
      new(a.allocate(size)) request_type{rh, h, pmf, std::forward<T>(t)...};
    }
  }
  return rh.get(ctx);
}

} // namespace stapl

#endif
