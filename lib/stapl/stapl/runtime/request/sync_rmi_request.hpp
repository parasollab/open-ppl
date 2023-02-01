/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_SYNC_RMI_REQUEST_HPP
#define STAPL_RUNTIME_REQUEST_SYNC_RMI_REQUEST_HPP

#include "arguments.hpp"
#include "rmi_request.hpp"
#include "../context.hpp"
#include "../type_traits/callable_traits.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief RMI request that calls the given callback with the return value of
///        the function.
///
/// @tparam ReturnCallback Callback type.
/// @tparam ObjectHandle   Distributed object handle type.
/// @tparam MemFun         Member function pointer type.
/// @tparam T              Argument types.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ReturnCallback,
         typename ObjectHandle,
         typename MemFun,
         typename... T>
class sync_rmi_request final
: public rmi_request,
  private arguments_t<MemFun, T...>
{
private:
  using args_type   = arguments_t<MemFun, T...>;
  using seq_type    = make_index_sequence<sizeof...(T)>;
  using object_type = typename callable_traits<MemFun>::object_type;

  const MemFun   m_pmf;
  ReturnCallback m_callback;
  ObjectHandle   m_handle;

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes the request when the return type is @c void.
  //////////////////////////////////////////////////////////////////////
  void apply(context& ctx, object_type& t, std::true_type)
  {
    invoke(m_pmf, t, static_cast<args_type&>(*this),
           static_cast<void*>(this), seq_type{});
    m_callback(ctx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes the request when the return type is not @c void.
  //////////////////////////////////////////////////////////////////////
  void apply(context& ctx, object_type& t, std::false_type)
  {
    m_callback(
      ctx,
      invoke(m_pmf, t, static_cast<args_type&>(*this),
             static_cast<void*>(this), seq_type{}));
  }

public:
  template<typename... U>
  static std::size_t expected_size(U&&... u) noexcept
  {
    return (sizeof(sync_rmi_request) +
            dynamic_size<args_type>(seq_type{}, std::forward<U>(u)...));
  }

  template<typename Callback, typename Handle, typename F, typename... U>
  sync_rmi_request(Callback&& c, Handle&& h, F&& f, U&&... u) noexcept
  : rmi_request(sizeof(*this)),
    args_type(std::forward_as_tuple(std::forward<U>(u),
                                    static_cast<void*>(this),
                                    this->size())...),
    m_pmf(std::forward<F>(f)),
    m_callback(std::forward<Callback>(c)),
    m_handle(std::forward<Handle>(h))
  { }

  bool operator()(context& ctx) final
  {
    using pmf_result_type = typename callable_traits<MemFun>::result_type;

    location_md& l = ctx.get_location_md();

    auto* const t = m_handle.template get<object_type>(l);
    if (!t)
      STAPL_RUNTIME_ERROR("p_object does not exist.");
    apply(ctx, *t, typename std::is_void<pmf_result_type>::type{});

    this->~sync_rmi_request();
    return true;
  }
};

} // namespace runtime

} // namespace stapl

#endif
