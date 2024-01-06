/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_REDUCE_RMI_HPP
#define STAPL_RUNTIME_RMI_REDUCE_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../future.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../rmi_handle.hpp"
#include "../yield.hpp"
#include "../non_rmi/response.hpp"
#include "../request/packed_value.hpp"
#include "../request/sync_rmi_request.hpp"
#include "../type_traits/callable_traits.hpp"
#include "../type_traits/is_non_commutative.hpp"
#include "../type_traits/lazy_storage.hpp"
#include <atomic>
#include <memory>
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Handle to wait for values from @ref reduce_rmi().
///
/// @tparam T               Value type.
/// @tparam BinaryOperation Reduction operator type.
///
/// This is an aggregator that extends @ref future_base so that it is possible
/// to return a @ref future from @ref reduce_rmi().
///
/// @see future
/// @ingroup requestBuildingBlock
///
/// @todo It needs some kind of one-sided object registration to do the
///       reduction in a tree fashion. Right now it works by sending all the
///       values to the calling location.
//////////////////////////////////////////////////////////////////////
template<typename T, typename BinaryOperation>
class reduce_rmi_handle
: public future_base<T>
{
private:
  typedef std::size_t                              size_type;
  typedef packed_value<T>                          packed_value_type;
public:
  typedef typename packed_value_type::storage_type storage_type;

private:
  /// Number of values waiting to be received.
  const size_type        m_size;
  /// Number of received values.
  std::atomic<size_type> m_count;
  BinaryOperation        m_op;
  lazy_storage<T>        m_storage;

  template<typename U>
  void acc_impl(U&& v)
  {
    std::lock_guard<std::mutex> lock{this->m_mtx};
    STAPL_RUNTIME_ASSERT(!valid_no_yield());
    if (m_count==0) { // first data to arrive
      m_storage.construct(std::forward<U>(v));
    }
    else {
      m_storage.construct(m_op(m_storage.moveout(), std::forward<U>(v)));
    }
    if (++m_count==m_size)
      this->schedule_continuation();
  }

public:
  reduce_rmi_handle(BinaryOperation op, const size_type size)
  : m_size(size),
    m_count(0),
    m_op(std::move(op)),
    m_storage()
  { }

  ~reduce_rmi_handle(void)
  { STAPL_RUNTIME_ASSERT(valid_no_yield()); }

protected:
  bool valid_no_yield(void) const noexcept
  { return (m_size==m_count); }

public:
  T get(void)
  {
    wait();
    std::lock_guard<std::mutex> lock{this->m_mtx};
    return m_storage.moveout();
  }

  T get(context& ctx)
  {
    wait(ctx);
    std::lock_guard<std::mutex> lock{this->m_mtx};
    return m_storage.moveout();
  }

  bool valid(void) const
  { return yield_if_not([this] { return this->valid_no_yield(); }); }

  void wait(void) const
  { yield_until([this] { return this->valid_no_yield(); }); }

  void wait(context& ctx) const
  { yield_until(ctx, [this] { return this->valid_no_yield(); }); }

  void set_value(storage_type* const p, void* const base, message_shared_ptr& m)
  { acc_impl(packed_value_type{p, base, m}.get()); }

  void set_value(T const& v)
  { acc_impl(v); }

  void set_value(T&& v)
  { acc_impl(std::move(v)); }
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Reduction RMI primitive.
///
/// The given member function is called on all locations the object is defined
/// on with the given arguments and returns an object that is the result of the
/// reduction using the supplied function.
///
/// If the operator @p op is non-commutative, this has to be declared by using
/// @ref non_commutative.
///
/// @warning This is a function that can harm scalability. It exists to
///          facilitate one-sided synchronization. Use @ref reduce_rmi() if
///          possible.
///
/// @param op  Reduction operator.
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return The result of the reduction of the return values from all the
///         locations the object exists on.
///
/// @ingroup ARMIOneSided
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation,
         typename Handle,
         typename MemFun,
         typename... T>
decltype(
  std::declval<BinaryOperation>()(
    std::declval<typename callable_traits<MemFun>::result_type>(),
    std::declval<typename callable_traits<MemFun>::result_type>())
)
sync_reduce_rmi(BinaryOperation op,
                Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  STAPL_RUNTIME_PROFILE("sync_reduce_rmi()", (primitive_traits::blocking |
                                              primitive_traits::ordered  |
                                              primitive_traits::p2m      |
                                              primitive_traits::comm));

  typedef decltype(
            op(std::declval<typename callable_traits<MemFun>::result_type>(),
               std::declval<typename callable_traits<MemFun>::result_type>())
          )                                               result_type;
  typedef reduce_rmi_handle<result_type, BinaryOperation> return_handle_type;
  typedef response<return_handle_type>                    response_type;

  return_handle_type rh{std::move(op), h.get_num_locations()};
  {
    bcast_aggregator a{ctx, h};
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
  return rh.get(ctx);
}


//////////////////////////////////////////////////////////////////////
/// @brief Reduction RMI primitive.
///
/// The given member function is called on all locations the object is defined
/// on with the given arguments and returns an object that is the result of the
/// reduction using the supplied function.
///
/// If the operator @p binary_op is non-commutative, this has to be declared by
/// using @ref non_commutative().
///
/// @param op  Reduction operator.
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref future object with the result of the reduction of the return
///         values from all the locations the object exists on.
///
/// @ingroup ARMIOneSided
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
reduce_rmi(BinaryOperation op, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  typedef decltype(
            op(std::declval<typename callable_traits<MemFun>::result_type>(),
               std::declval<typename callable_traits<MemFun>::result_type>())
          )                                               result_type;
  typedef reduce_rmi_handle<result_type, BinaryOperation> return_handle_type;
  typedef response<return_handle_type>                    response_type;

  std::unique_ptr<return_handle_type>
    p{new return_handle_type{std::move(op), h.get_num_locations()}};
  {
    STAPL_RUNTIME_PROFILE("reduce_rmi()", (primitive_traits::non_blocking |
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
  return future<result_type>{std::move(p)};
}


namespace unordered {

//////////////////////////////////////////////////////////////////////
/// @brief Reduction RMI primitive.
///
/// The given member function is called on all locations the object is defined
/// on with the given arguments and returns an object that is the result of the
/// reduction using the supplied function.
///
/// If the operator @p op is non-commutative, this has to be declared by using
/// @ref stapl::non_commutative().
///
/// This is an unordered version of the @ref stapl::sync_reduce_rmi() that may
/// break RMI ordering rules.
///
/// @warning This is a function that can harm scalability. It exists to
///          facilitate one-sided synchronization. Use
///          @ref unordered::reduce_rmi() if possible.
///
/// @param op  Reduction operator.
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return The result of the reduction of the return values from all the
///         locations the object exists on.
///
/// @ingroup ARMIUnordered
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation,
         typename Handle,
         typename MemFun,
         typename... T>
decltype(
  std::declval<BinaryOperation>()(
    std::declval<typename callable_traits<MemFun>::result_type>(),
    std::declval<typename callable_traits<MemFun>::result_type>())
)
sync_reduce_rmi(BinaryOperation op,
                Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  STAPL_RUNTIME_PROFILE("sync_reduce_rmi(unodered)",
                        (primitive_traits::blocking  |
                         primitive_traits::unordered |
                         primitive_traits::p2m       |
                         primitive_traits::comm));

  typedef decltype(
            op(std::declval<typename callable_traits<MemFun>::result_type>(),
               std::declval<typename callable_traits<MemFun>::result_type>())
          )                                               result_type;
  typedef reduce_rmi_handle<result_type, BinaryOperation> return_handle_type;
  typedef response<return_handle_type>                    response_type;

  return_handle_type rh{std::move(op), h.get_num_locations()};
  {
    bcast_aggregator a{ctx, h, false};
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
  return rh.get(ctx);
}


//////////////////////////////////////////////////////////////////////
/// @brief Reduction RMI primitive.
///
/// The given member function is called on all locations the object is defined
/// on with the given arguments and returns an object that is the result of the
/// reduction using the supplied function.
///
/// If the operator @p op is non-commutative, this has to be declared by using
/// @ref stapl::non_commutative().
///
/// This is an unordered version of the @ref stapl::reduce_rmi() that may break
/// RMI ordering rules.
///
/// @param op  Reduction operator.
/// @param h   Handle to the target object.
/// @param pmf Member function to invoke.
/// @param t   Arguments to pass to the member function.
///
/// @return A @ref future object with the result of the reduction of the return
///         values from all the locations the object exists on.
///
/// @ingroup ARMIUnordered
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
reduce_rmi(BinaryOperation op, Handle const& h, MemFun const& pmf, T&&... t)
{
  using namespace stapl::runtime;

  static_assert(std::is_convertible<
                  Handle, typename appropriate_handle<MemFun>::type
                >::value, "handle discards qualifiers");

  context& ctx = this_context::get();

  STAPL_RUNTIME_ASSERT_MSG(h.valid(), "Invalid handle");

  typedef decltype(
            op(std::declval<typename callable_traits<MemFun>::result_type>(),
               std::declval<typename callable_traits<MemFun>::result_type>())
          )                                               result_type;
  typedef reduce_rmi_handle<result_type, BinaryOperation> return_handle_type;
  typedef response<return_handle_type>                    response_type;

  std::unique_ptr<return_handle_type>
    p{new return_handle_type{std::move(op), h.get_num_locations()}};
  {
    STAPL_RUNTIME_PROFILE("reduce_rmi(unodered)",
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
  return future<result_type>{std::move(p)};
}

} // namespace unordered

} // namespace stapl

#endif
