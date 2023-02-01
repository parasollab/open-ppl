/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_ASYNC_RMI_REQUEST_HPP
#define STAPL_RUNTIME_REQUEST_ASYNC_RMI_REQUEST_HPP

#include "arguments.hpp"
#include "rmi_request.hpp"
#include "../context.hpp"
#include "../type_traits/callable_traits.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief RMI request that discards the return value.
///
/// @tparam ObjectHandle Distributed object handle type.
/// @tparam MemFun       Member function pointer type.
/// @tparam T            Argument types.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ObjectHandle, typename MemFun, typename... T>
class nc_async_rmi_request final
: public rmi_request,
  private arguments_t<MemFun, T...>
{
private:
  using args_type = arguments_t<MemFun, T...>;
  using seq_type  = make_index_sequence<sizeof...(T)>;

  ObjectHandle m_handle;
  const MemFun m_pmf;

public:
  template<typename... U>
  static std::size_t expected_size(U&&... u) noexcept
  {
    return (sizeof(nc_async_rmi_request) +
            dynamic_size<args_type>(seq_type{}, std::forward<U>(u)...));
  }

  template<typename Handle, typename F, typename... U>
  nc_async_rmi_request(Handle&& h, F&& f, U&&... u) noexcept
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

    auto* const t = m_handle.template get<object_type>(ctx.get_location_md());
    if (!t)
      STAPL_RUNTIME_ERROR("p_object does not exist.");
    invoke(m_pmf, *t, static_cast<args_type&>(*this),
           static_cast<void*>(this), seq_type{});
    this->~nc_async_rmi_request();

    return true;
  }
};


#if !defined(STAPL_RUNTIME_DISABLE_COMBINING)

//////////////////////////////////////////////////////////////////////
/// @brief RMI request that discards the return value and supports combining.
///
/// @tparam ObjectHandle Distributed object handle type.
/// @tparam MemFun       Member function pointer type.
/// @tparam T            Argument types.
///
/// This RMI request supports combining. It is detected at runtime if two
/// requests can be combined and if so, they are. Combining consists of keeping
/// only the set of arguments instead of also the object handle and the
/// member function pointer, saving on space and required function calls.
///
/// In general combining is detected if the size of the sent request is bigger
/// than the size of the current executed request (given by the arguments).
/// However, if the argument size is 0, an additional counter that gives the
/// number of aggregated requests exists.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename ObjectHandle, typename MemFun, typename... T>
class async_rmi_request final
: public combinable_rmi_request<
           std::is_empty<arguments_t<MemFun, T...>>::value
         >,
  private arguments_t<MemFun, T...>
{
public:
  using args_type = arguments_t<MemFun, T...>;
private:
  using seq_type  = make_index_sequence<sizeof...(T)>;

  static constexpr bool is_empty         = std::is_empty<args_type>::value;
  static const std::size_t s_static_size = (is_empty ? 0 : sizeof(args_type));

  using base_type = combinable_rmi_request<is_empty>;

  ObjectHandle m_handle;
  const MemFun m_pmf;

public:
  template<typename... U>
  static combined_request_size expected_size(U&&... u) noexcept
  {
    return combined_request_size(
             sizeof(async_rmi_request),
             s_static_size,
             dynamic_size<args_type>(seq_type{}, std::forward<U>(u)...));
  }

  template<typename Handle, typename F, typename... U>
  async_rmi_request(Handle&& h, F&& f, U&&... u) noexcept
  : base_type(sizeof(*this)),
    args_type(std::forward_as_tuple(std::forward<U>(u),
                                    static_cast<void*>(this),
                                    this->size())...),
    m_handle(std::forward<Handle>(h)),
    m_pmf(std::forward<F>(f))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to the end of this request.
  //////////////////////////////////////////////////////////////////////
  const void* end(void) const noexcept
  { return (reinterpret_cast<const char*>(this) + this->size()); }

  bool operator()(context& ctx) final
  {
    using object_type = typename callable_traits<MemFun>::object_type;

    auto* const t = m_handle.template get<object_type>(ctx.get_location_md());
    if (!t)
      STAPL_RUNTIME_ERROR("p_object does not exist.");

    if (base_type::has_counter) {
      // combined requests have sizeof(args_type)==0
      for (typename base_type::size_type n = this->num_requests(); n>0; --n)
        invoke(m_pmf, *t, static_cast<args_type&>(*this),
               static_cast<void*>(this), seq_type{});
    }
    else {
      // execute first request
      std::size_t offset = sizeof(async_rmi_request); // static size
      invoke(m_pmf, *t, static_cast<args_type&>(*this),
             static_cast<void*>(this), offset, seq_type{});

      // execute combined requests
      const void* const end_of_request = end();
      for (void* p = (reinterpret_cast<char*>(this) + offset);
             p<end_of_request;
               p = (reinterpret_cast<char*>(this)+offset)) {
        auto* const creq = static_cast<args_type*>(p);
        // static size of the arguments
        offset += s_static_size;
        // packed size calculated at unpacking
        invoke(m_pmf, *t, *creq, static_cast<void*>(creq), offset, seq_type{});
        creq->~args_type();
      }
    }

    this->~async_rmi_request();
    return true;
  }
};

#endif

} // namespace runtime

} // namespace stapl

#endif
