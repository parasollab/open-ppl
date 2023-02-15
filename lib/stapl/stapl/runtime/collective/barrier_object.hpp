/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COLLECTIVE_BARRIER_OBJECT_HPP
#define STAPL_RUNTIME_COLLECTIVE_BARRIER_OBJECT_HPP

#include "../context.hpp"
#include "../rmi_handle.hpp"
#include "../value_handle.hpp"
#include "../yield.hpp"
#include "../communicator/barrier.hpp"
#include "../concurrency/reduction.hpp"
#include "../non_rmi/response.hpp"
#include <functional>
#include <memory>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Performs a barrier over all locations of the current gang.
///
/// @ingroup runtimeCollectives
///
/// @todo Use shared-memory optimized barrier implementation.
//////////////////////////////////////////////////////////////////////
class barrier_object
: public value_handle<void>
{
public:
  using value_type           = void;
private:
  using distmem_barrier_type = barrier;
  using shmem_reduce_type    = reduction<bool, std::logical_and<bool>>;
  using response_type        =
    handle_response<packed_handle_type, barrier_object>;

  friend response_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Base class for internal implementation.
  //////////////////////////////////////////////////////////////////////
  class impl_base
  {
  public:
    virtual ~impl_base(void) = default;
    virtual void operator()(const location_md::size_type) = 0;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation for distributed memory only gangs.
  //////////////////////////////////////////////////////////////////////
  class dist_impl final
  : public impl_base
  {
  private:
    distmem_barrier_type m_barrier;

  public:
    dist_impl(barrier_object& o, const collective_id cid)
    : m_barrier(o.get_gang_md().get_id(),
                cid,
                o.get_gang_md().get_topology(),
                [&o] { o.bcast_release(); })
    { }

    void operator()(const location_md::size_type) final
    { m_barrier(); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation for shared memory only gangs.
  //////////////////////////////////////////////////////////////////////
  class shmem_impl final
  : public impl_base
  {
  private:
    barrier_object&   m_parent;
    shmem_reduce_type m_red;

  public:
    explicit shmem_impl(barrier_object& o)
    : m_parent(o),
      m_red(o.get_gang_md().local_size())
    { }

    void operator()(const location_md::size_type idx) final
    {
      if (m_red(idx, true)) {
        m_red.get();
        m_parent.bcast_release();
      }
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation for mixed-mode gangs.
  //////////////////////////////////////////////////////////////////////
  class mixed_impl final
  : public impl_base
  {
  private:
    shmem_reduce_type    m_shmem_red;
    distmem_barrier_type m_distmem_barrier;

  public:
    mixed_impl(barrier_object& o, const collective_id cid)
    : m_shmem_red(o.get_gang_md().local_size()),
      m_distmem_barrier(o.get_gang_md().get_id(),
                        cid,
                        o.get_gang_md().get_topology(),
                        [&o] { o.bcast_release(); })
    { }

    void operator()(const location_md::size_type idx) final
    {
      if (m_shmem_red(idx, true)) {
        m_shmem_red.get();
        m_distmem_barrier();
      }
    }
  };

  rmi_handle                 m_handle;
  std::shared_ptr<impl_base> m_impl;

public:
  explicit barrier_object(context& ctx)
  : m_handle(ctx, this)
  {
    if (m_handle.get_num_locations()==1) {
      // single location gang
      return;
    }

    auto& g = ctx.get_gang_md();

    if (g.local_size()==1) {
      // single location on the process
      const collective_id cid = m_handle.internal_handle().abstract();
      m_impl = std::make_shared<dist_impl>(std::ref(*this), cid);
    }
    else if (g.get_description().is_on_shmem()) {
      // shared memory gang
      m_impl = g.get_shared_object<shmem_impl>(m_handle, std::ref(*this));
    }
    else {
      // multiple locations on the process, multiple processes
      const collective_id cid = m_handle.internal_handle().abstract();
      m_impl =
        g.get_shared_object<mixed_impl>(m_handle, std::ref(*this), cid);
    }
  }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  { return m_handle; }

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  { return m_handle; }

private:
  gang_md const& get_gang_md(void) const noexcept
  { return m_handle.get_location_md().get_gang_md(); }

  using value_handle<void>::set_value;

  void bcast_release(void)
  { response_type{}(m_handle); }

public:
  void operator()(void)
  {
    if (!m_impl)
      this->set_value();
    else
      (*m_impl)(m_handle.get_location_md().local_index());
  }
};

} // namespace runtime

} // namespace stapl

#endif
