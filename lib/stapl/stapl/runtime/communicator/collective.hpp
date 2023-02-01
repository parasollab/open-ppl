/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COMMUNICATOR_COLLECTIVE_HPP
#define STAPL_RUNTIME_COMMUNICATOR_COLLECTIVE_HPP

#include "../config.hpp"
#include "topology.hpp"
#include "../exception.hpp"
#include "../message.hpp"
#include <functional>
#include <mutex>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Handle for distributed memory collective operations.
///
/// @ingroup runtimeCollectives
//////////////////////////////////////////////////////////////////////
class collective
{
public:
  using id            = std::pair<gang_id, collective_id>;
private:
  using size_type     = topology::size_type;
  using function_type = std::function<void(message_slist)>;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the requested collective.
  //////////////////////////////////////////////////////////////////////
  static collective& get(id const&, topology const&);

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the requested collective.
  //////////////////////////////////////////////////////////////////////
  static collective& get(id const&);

private:
  const id           m_id;
  topology const&    m_topology;

  /// Messages from children.
  message_slist      m_arrived;
  size_type          m_num_arrived;
  function_type      m_notify_arrived;
  mutable std::mutex m_mtx;

public:
  collective(id i, topology const& t)
  : m_id(std::move(i)),
    m_topology(t),
    m_num_arrived(0)
  { }

  ~collective(void)
  { STAPL_RUNTIME_ASSERT(!bool(m_notify_arrived) && m_arrived.empty()); }

  void try_destroy(void) noexcept;

  id const& get_id(void) const noexcept
  { return m_id; }

  topology const& get_topology(void) const noexcept
  { return m_topology; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Notifies that @p sm has arrived from a child process.
  //////////////////////////////////////////////////////////////////////
  void notify_arrival(message_shared_ptr& sm)
  {
    auto m = make_message_ptr(sm.detach());
    const auto num_waiting = m_topology.children().size();
    message_slist l;
    function_type f;

    {
      std::lock_guard<std::mutex> lock{m_mtx};
      m_arrived.push_back(std::move(m));
      ++m_num_arrived;
      if ((m_num_arrived!=num_waiting) || !bool(m_notify_arrived))
        return;
      m_num_arrived = 0;
      f = std::move(m_notify_arrived);
      l = std::move(m_arrived);
    }

    f(std::move(l));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the messages from children if they have all arrived or sets
  ///        @p f as a notifier to be called when they have arrived.
  //////////////////////////////////////////////////////////////////////
  template<typename Function>
  message_slist try_collect(Function&& f)
  {
    const auto num_waiting = m_topology.children().size();

    std::lock_guard<std::mutex> lock{m_mtx};
    STAPL_RUNTIME_ASSERT(m_num_arrived<=num_waiting && !bool(m_notify_arrived));
    if (m_num_arrived==num_waiting) {
      m_num_arrived = 0;
      return std::move(m_arrived);
    }
    m_notify_arrived = std::forward<Function>(f);
    return message_slist{};
  }
};

} // namespace runtime

} // namespace stapl

#endif
