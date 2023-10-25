/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NON_RMI_RPC_HPP
#define STAPL_RUNTIME_NON_RMI_RPC_HPP

#include "../aggregator.hpp"
#include "../request/arguments.hpp"
#include "../request/rpc_request.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Remote Procedure Call (RPC) request that discards return values.
///
/// @tparam FunPtr   Function pointer type.
/// @tparam T        Argument types.
///
/// RPC requests are only inter-process, therefore no shared-memory
/// optimizations are applicable.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename FunPtr, typename... T>
class async_rpc_request final
: public rpc_request,
  private arguments_t<FunPtr, T...>
{
private:
  using args_type = arguments_t<FunPtr, T...>;
  using seq_type  = make_index_sequence<sizeof...(T)>;

  FunPtr m_mem_fun_ptr;

public:
  template<typename... U>
  static std::size_t expected_size(U&&... u) noexcept
  {
    return (sizeof(async_rpc_request) +
            dynamic_size<args_type>(seq_type{}, std::forward<U>(u)...));
  }

  template<typename... U>
  explicit async_rpc_request(FunPtr mem_fun_ptr, U&&...u) noexcept
  : rpc_request(sizeof(*this)),
    args_type(std::forward_as_tuple(std::forward<U>(u),
                                    static_cast<void*>(this),
                                    this->size())...),
    m_mem_fun_ptr(mem_fun_ptr)
  { }

  void operator()(message_shared_ptr&) final
  {
    invoke(m_mem_fun_ptr,
           static_cast<args_type&>(*this),
           static_cast<void*>(this),
           seq_type{});

    this->~async_rpc_request();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Remote Procedure Call (RPC) to process primitive.
///
/// This function calls @p f on the destination processes.
///
/// @param f    Callable object.
/// @param pids Destination processes.
/// @param t    Arguments to pass to the function.
///
/// @ingroup requestBuildingBlock
///
/// @todo It's possible to statically encode the function pointer as a
/// template parameter, but this causes symbol size carnage.  An alternate
/// strategy that should still avoid runtime indirection is a lambda, but
/// with arg_storage's current implementation, this causes carnage as well.
/// These should be investigated to avoid the unnecessary indirection
/// cost currently incurred.
//////////////////////////////////////////////////////////////////////
template<typename F, typename Range, typename... T>
void rpc(F&& f, Range&& pids, T&&... t)
{
  using request_type = async_rpc_request<
                           typename std::decay<F>::type,
                           typename std::remove_reference<T>::type...>;

  rpc_aggregator a{std::forward<Range>(pids)};
  const auto size = request_type::expected_size(std::forward<T>(t)...);
  new(a.allocate(size)) request_type{std::forward<F>(f), std::forward<T>(t)...};
}

} // namespace runtime

} // namespace stapl

#endif
