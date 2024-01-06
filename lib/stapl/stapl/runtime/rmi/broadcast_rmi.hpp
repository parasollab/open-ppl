/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_BROADCAST_RMI_HPP
#define STAPL_RUNTIME_RMI_BROADCAST_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../future.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../yield.hpp"
#include "../non_rmi/response.hpp"
#include "../collective/broadcast_object.hpp"
#include "../request/sync_rmi_request.hpp"
#include "../type_traits/callable_traits.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include <memory>
#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Broadcast RMI primitive for root locations.
///
/// The given member function is called on the root location and the result of
/// the invocation is broadcast to all locations.
///
/// @warning This primitive is supposed to be called only from the root
///          location. Non-root locations have to call
///          @ref broadcast_rmi(unsigned int,MemFun const&).
///
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref future object with the result of the function call.
///
/// @ingroup ARMICollectives
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
future<typename callable_traits<MemFun>::result_type>
broadcast_rmi(root_location_t, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");
  STAPL_RUNTIME_ASSERT_MSG((ctx.is_base() &&
                            ctx.get_gang_id()==h.get_gang_id()),
                           "Only allowed in SPMD");

  typedef typename callable_traits<MemFun>::result_type result_type;
  typedef broadcast_object<result_type>                 return_handle_type;
  typedef active_handle_response<
            packed_handle_type, return_handle_type
          >                                             response_type;

  std::unique_ptr<return_handle_type> p{new return_handle_type{ctx}};
  {
    STAPL_RUNTIME_PROFILE("broadcast_rmi()", (primitive_traits::non_blocking |
                                              primitive_traits::ordered      |
                                              primitive_traits::coll         |
                                              primitive_traits::comm));

    aggregator a{ctx, h, ctx.get_location_id(), no_implicit_flush};
    typedef sync_rmi_request<
              response_type,
              packed_handle_type,
              MemFun,
              typename transport_qualifier<decltype(t)>::type...
            > request_type;
    const std::size_t size = request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{*p, h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
  return future<result_type>{std::move(p)};
}


//////////////////////////////////////////////////////////////////////
/// @brief Broadcast RMI primitive for non-root locations.
///
/// @warning This primitive is supposed to be called only from the non-root
///          locations. The root location has to call
/// @ref broadcast_rmi(root_location_t,Handle const&,MemFun const&,T&&...).
///
/// @param root Root location of the broadcast.
/// @param pmf  Member function to invoke.
///
/// @return A @ref future object with the result of the function call on the
///         root location.
///
/// @ingroup ARMICollectives
//////////////////////////////////////////////////////////////////////
template<typename MemFun>
future<typename callable_traits<MemFun>::result_type>
broadcast_rmi(unsigned int root, MemFun const&)
{
  using namespace stapl::runtime;

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(ctx.is_base(), "Only allowed in SPMD");

  if (root==ctx.get_location_id())
    STAPL_RUNTIME_ERROR("Intended to only be called from non-root locations.");

  typedef typename callable_traits<MemFun>::result_type result_type;
  typedef broadcast_object<result_type>                 return_handle_type;

  std::unique_ptr<return_handle_type> p{new return_handle_type{ctx}};
  {
    STAPL_RUNTIME_PROFILE("broadcast_rmi()", (primitive_traits::non_blocking |
                                              primitive_traits::ordered      |
                                              primitive_traits::coll         |
                                              primitive_traits::comm));
  }

  scheduling_point(ctx);
  return future<result_type>{std::move(p)};
}

} // namespace stapl

#endif
