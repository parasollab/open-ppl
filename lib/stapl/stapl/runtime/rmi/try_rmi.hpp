/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_TRY_RMI_HPP
#define STAPL_RUNTIME_RMI_TRY_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../rmi_handle.hpp"
#include "../yield.hpp"
#include "../request/arguments.hpp"
#include "../request/rmi_request.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief RMI request that will not execute if the distributed object has been
///        destroyed and that discards the return value.
///
/// @tparam MemFun Member function pointer type.
/// @tparam T      Argument types.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename MemFun, typename... T>
class try_rmi_request final
: public rmi_request,
  private arguments_t<MemFun, T...>
{
private:
  using args_type = arguments_t<MemFun, T...>;
  using seq_type  = make_index_sequence<sizeof...(T)>;

  packed_handle_epoch m_handle;
  const MemFun        m_pmf;

public:
  template<typename... U>
  static std::size_t expected_size(U&&... u) noexcept
  {
    return (sizeof(try_rmi_request) +
            dynamic_size<args_type>(seq_type{}, std::forward<U>(u)...));
  }

  template<typename Handle, typename F, typename... U>
  try_rmi_request(Handle&& h, F&& f, U&&... u) noexcept
  : rmi_request(sizeof(*this)),
    args_type(std::forward_as_tuple(std::forward<U>(u),
                                    static_cast<void*>(this),
                                    this->size())...),
    m_handle(std::forward<Handle>(h)),
    m_pmf(std::forward<F>(f))
  { }

  bool operator()(context& ctx) final
  {
    using object_type = typename callable_traits<MemFun>::object_type;

    auto* const t = m_handle.get<object_type>(ctx.get_location_md());
    if (t) {
      // object exists; unpack arguments and call the function
      invoke(m_pmf, *t, static_cast<args_type&>(*this),
             static_cast<void*>(this), seq_type{});
      this->~try_rmi_request();
    }
    else {
      // object does not exist; clean-up arguments
      cleanup(static_cast<args_type&>(*this), seq_type{});
    }

    return true;
  }
};

} //  namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Asynchronous best-effort RMI primitive.
///
/// The given member function is called on the object in the destination
/// location. It discards any return value.
///
/// If the object does not exist in the destination, this primitive silently
/// drops the request without errors.
///
/// @param dest Destination location.
/// @param h    Handle to the target object.
/// @param pmf  Member function to invoke.
/// @param t    Arguments to pass to the member function.
///
/// @ingroup ARMIOneSided
///
/// @todo Enable shared-memory optimizations.
//////////////////////////////////////////////////////////////////////
template<typename Handle, typename MemFun, typename... T>
void try_rmi(unsigned int dest, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");
  STAPL_RUNTIME_ASSERT_MSG(h.is_valid(dest),
                           "p_object does not exist in destination" );
  STAPL_RUNTIME_ASSERT_MSG((h.get_flags() & allow_try_rmi)!=0,
                           "try_rmi() not supported for this p_object");

  {
    STAPL_RUNTIME_PROFILE("try_rmi()", (primitive_traits::non_blocking |
                                        primitive_traits::ordered      |
                                        primitive_traits::p2p          |
                                        primitive_traits::comm));

    aggregator a{ctx, h, dest, no_implicit_flush};
    typedef try_rmi_request<
              MemFun,
              typename std::remove_reference<T>::type...
            > request_type;
    const std::size_t size = request_type::expected_size(std::forward<T>(t)...);
    new(a.allocate(size)) request_type{h, pmf, std::forward<T>(t)...};
  }

  scheduling_point(ctx);
}

} // namespace stapl

#endif
