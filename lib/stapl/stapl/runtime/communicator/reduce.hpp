/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COMMUNICATOR_ALLREDUCE_HPP
#define STAPL_RUNTIME_COMMUNICATOR_ALLREDUCE_HPP

#include "collective.hpp"
#include "../aggregator.hpp"
#include "../exception.hpp"
#include "../message.hpp"
#include "../request/arg_storage.hpp"
#include "../request/rpc_request.hpp"
#include <mutex>
#include <utility>
#include <boost/function.hpp>
#include <boost/optional.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Distributed memory collective reduce.
///
/// @tparam T               Object type.
/// @tparam BinaryOperation Binary operation function object type to be applied.
///
/// @warning This class supports only commutative reductions.
///
/// @see collective
/// @ingroup runtimeCollectives
///
/// @todo Use platform optimized reduce implementation.
//////////////////////////////////////////////////////////////////////
template<typename T, typename BinaryOperation>
class reduce
{
public:
  using value_type = T;
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Gather phase request.
  //////////////////////////////////////////////////////////////////////
  class gather_request final
  : public rpc_request,
    private arg_storage_t<T, T const&>
  {
  private:
    using storage_type = arg_storage_t<T, T const&>;

    collective::id m_cid;

  public:
    template<typename U>
    static std::size_t expected_size(U&& u) noexcept
    {
      return (sizeof(gather_request) +
              storage_type::packed_size(std::forward<U>(u)));
    }

    template<typename U>
    gather_request(collective::id const& cid, U&& u)
    : rpc_request{sizeof(*this)},
      storage_type{std::forward<U>(u), this, this->size()},
      m_cid{cid}
    { }

    void operator()(message_shared_ptr& m) final
    {
      auto& c = collective::get(m_cid);
      STAPL_RUNTIME_ASSERT(!c.get_topology().is_leaf());
      c.notify_arrival(m);
    }

    auto get(void)
      -> decltype(this->get(this))
    { return storage_type::get(this); }
  };


  collective&                 m_handle;
  BinaryOperation             m_op;
  boost::optional<value_type> m_value;
  boost::function<void(T)>    m_signal;
  std::mutex                  m_mtx;

  ////////////////////////////////////////////////////////////////////
  /// @brief Accumulates the values in @p l.
  ////////////////////////////////////////////////////////////////////
  T accumulate(message_slist l)
  {
    // get first value
    auto m  = l.pop_front();
    char* b = m->payload().begin();
    auto* p = reinterpret_cast<gather_request*>(b);
    T t     = p->get();
    p->~gather_request();

    // accumulate the rest of the values
    while (!l.empty()) {
      m = l.pop_front();
      b = m->payload().begin();
      p = reinterpret_cast<gather_request*>(b);
      t = m_op(std::move(t), p->get());
      p->~gather_request();
    }

    return t;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Sends @p u to the parent process or signals the end of the
  //         reduction if this is the root process.
  ////////////////////////////////////////////////////////////////////
  template<typename U>
  void send_partial_result(U&& u)
  {
    if (m_handle.get_topology().is_root()) {
      // root; signal that reduction is done
      m_signal(std::forward<U>(u));
    }
    else {
      // non-root; send to result to parent and wait for release
      rpc_aggregator a{m_handle.get_topology().parent_id()};
      const auto size = gather_request::expected_size(std::forward<U>(u));
      new(a.allocate(size)) gather_request{m_handle.get_id(),
                                           std::forward<U>(u)};
    }
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Notifies that all messages from children process have arrived.
  ////////////////////////////////////////////////////////////////////
  void all_messages_arrived(message_slist l)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    STAPL_RUNTIME_ASSERT(m_value);
    send_partial_result(m_op(std::move(*m_value), accumulate(std::move(l))));
    m_value = boost::none;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Reduction implementation.
  ////////////////////////////////////////////////////////////////////
  template<typename U>
  void init(U&& u)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    STAPL_RUNTIME_ASSERT(!m_value);
    if (m_handle.get_topology().is_leaf()) {
      send_partial_result(std::forward<U>(u));
    }
    else {
      auto l =
        m_handle.try_collect([this](message_slist l)
                             { this->all_messages_arrived(std::move(l)); });
      if (!l.empty()) {
        send_partial_result(m_op(std::forward<U>(u), accumulate(std::move(l))));
      }
      else {
        // wait for children
        m_value = std::forward<U>(u);
      }
    }
  }

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Constructs an @ref reduce object.
  ///
  /// @param gid Id of the gang the collective executes in.
  /// @param cid Collective operation id.
  /// @param t   @ref topology object associated with the gang.
  /// @param f   Function to call when the collective has finished.
  /// @param op  Reduction operator.
  ////////////////////////////////////////////////////////////////////
  template<typename Function>
  reduce(const gang_id gid,
         const collective_id cid,
         topology const& t,
         Function&& f,
         BinaryOperation op = BinaryOperation{})
  : m_handle(collective::get(collective::id{gid, cid}, t)),
    m_op(std::move(op)),
    m_signal(std::forward<Function>(f))
  { }

  ~reduce(void)
  { m_handle.try_destroy(); }

  void operator()(T const& t)
  { init(t); }

  void operator()(T&& t)
  { init(std::move(t)); }
};

} // namespace runtime

} // namespace stapl

#endif
