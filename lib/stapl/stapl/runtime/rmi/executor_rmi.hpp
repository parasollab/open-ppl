/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_EXECUTOR_RMI_HPP
#define STAPL_RUNTIME_RMI_EXECUTOR_RMI_HPP

#include "../aggregator.hpp"
#include "../exception.hpp"
#include "../context.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../rmi_handle.hpp"
#include "../request/arg_storage.hpp"
#include "../request/rmi_request.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Request for executing the given function on the destination location.
///
/// @tparam Function    Function type.
/// @tparam RetFunction Function retrieval type.
///
/// When the request executes, the @ref gang_executor of the destination is
/// given to the function as an argument.
///
/// @see executor_rmi()
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Function, typename RetFunction>
class executor_rmi_request final
: public rmi_request,
  private arg_storage_t<Function, RetFunction>
{
private:
  using function_storage_type = arg_storage_t<Function, RetFunction>;

public:
  template<typename F>
  static std::size_t expected_size(F&& f) noexcept
  {
    return (sizeof(executor_rmi_request) +
            function_storage_type::packed_size(std::forward<F>(f)));
  }

  template<typename F>
  explicit executor_rmi_request(F&& f) noexcept
  : rmi_request(sizeof(*this)),
    function_storage_type(std::forward<F>(f), this, this->size())
  { }

  bool operator()(context& ctx) final
  {
    auto& ex = ctx.get_location_md().get_executor();
    function_storage_type::get(this)(ex);
    this->~executor_rmi_request();
    return true;
  }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Invokes the given function on the @ref gang_executor of the location
///        that @p h lives on.
///
/// @param dest Destination location.
/// @param h    Handle to an object that is on the same gang as the target
///             @ref gang_executor.
/// @param f    Function to be called.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename F>
void executor_rmi(unsigned int dest, Handle const& h, F&& f)
{
  using namespace stapl::runtime;

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");
  STAPL_RUNTIME_ASSERT_MSG(h.is_valid(dest),
                           "p_object does not exist in destination");

  {
    STAPL_RUNTIME_PROFILE("executor_rmi()", (primitive_traits::non_blocking |
                                             primitive_traits::ordered      |
                                             primitive_traits::p2p          |
                                             primitive_traits::comm));

    aggregator a{ctx, h, dest};
    const bool on_shmem = a.is_on_shmem();
    if (on_shmem) {
      typedef executor_rmi_request<
                typename transport_qualifier<decltype(f)>::type,
                typename std::remove_reference<F>::type&
              > request_type;
      const std::size_t size = request_type::expected_size(std::forward<F>(f));
      new(a.allocate(size)) request_type{std::forward<F>(f)};
    }
    else {
      typedef executor_rmi_request<
                typename std::remove_reference<F>::type,
                typename std::remove_reference<F>::type&
              > request_type;
      const std::size_t size = request_type::expected_size(std::forward<F>(f));
      new(a.allocate(size)) request_type{std::forward<F>(f)};
    }
  }

  scheduling_point(ctx);
}

} // namespace stapl

#endif
