/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_ALLREDUCE_RMI_HPP
#define STAPL_RUNTIME_RMI_ALLREDUCE_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../future.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../yield.hpp"
#include "../collective/allreduce_object.hpp"
#include "../non_rmi/response.hpp"
#include "../request/sync_rmi_request.hpp"
#include "../type_traits/callable_traits.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include <memory>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Allreduce RMI primitive.
///
/// The given member function is called on all locations the object is defined
/// on and the result of the local invocations of the member function are
/// combined using @p op and the result is distributed to all locations that
/// made the call. This result can be retrieved through the returned @ref future
/// object.
///
/// @param op  Reduction operator.
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref future object with the combined return values.
///
/// @ingroup ARMICollectives
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation,
         typename Handle,
         typename MemFun,
         typename... T>
future<
  decltype(
    std::declval<BinaryOperation>()(
      std::declval<typename callable_traits<MemFun>::result_type>(),
      std::declval<typename callable_traits<MemFun>::result_type>())
  )
>
allreduce_rmi(BinaryOperation op, Handle const& h, MemFun const& pmf, T&&... t)
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

  using result_type = decltype(
            op(std::declval<typename callable_traits<MemFun>::result_type>(),
               std::declval<typename callable_traits<MemFun>::result_type>()));
  using return_handle_type = allreduce_object<result_type, BinaryOperation>;
  using response_type =
    active_handle_response<packed_handle_type, return_handle_type>;

  std::unique_ptr<return_handle_type>
    p{new return_handle_type{ctx, std::move(op)}};
  {
    STAPL_RUNTIME_PROFILE("allreduce_rmi()", (primitive_traits::non_blocking |
                                              primitive_traits::ordered      |
                                              primitive_traits::coll         |
                                              primitive_traits::comm));

    aggregator a{ctx, h, ctx.get_location_id(), no_implicit_flush};
    using request_type =
      sync_rmi_request<response_type,
                       packed_handle_type,
                       MemFun,
                       typename transport_qualifier<decltype(t)>::type...>;
    const std::size_t size = request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{*p, h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
  return future<result_type>{std::move(p)};
}

} // namespace stapl

#endif
