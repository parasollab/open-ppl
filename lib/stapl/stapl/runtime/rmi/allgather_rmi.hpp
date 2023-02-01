/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_ALLGATHER_RMI_HPP
#define STAPL_RUNTIME_RMI_ALLGATHER_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../future.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../yield.hpp"
#include "../collective/allgather_object.hpp"
#include "../non_rmi/response.hpp"
#include "../request/sync_rmi_request.hpp"
#include "../type_traits/callable_traits.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include <memory>
#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Allgather RMI primitive.
///
/// The given member function is called on all locations the object is defined
/// on and returns a @ref futures object which can be used to retrieve the
/// return values from all the member function invocations.
///
/// Each location passes the locally provided arguments to the member function.
///
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref futures object with the return values from all the locations,
///         ordered per location id.
///
/// @ingroup ARMICollectives
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
futures<typename callable_traits<MemFun>::result_type>
allgather_rmi(Handle const& h, MemFun const& pmf, T&&... t)
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
  typedef allgather_object<result_type>                 return_handle_type;
  typedef active_handle_response<
            packed_handle_type, return_handle_type
          >                                             response_type;

  std::unique_ptr<return_handle_type> p{new return_handle_type{ctx}};
  {
    STAPL_RUNTIME_PROFILE("allgather_rmi()", (primitive_traits::non_blocking |
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
  return futures<result_type>{std::move(p)};
}

} // namespace stapl

#endif
