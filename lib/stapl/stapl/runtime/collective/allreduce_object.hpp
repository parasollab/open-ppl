/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COLLECTIVE_ALLREDUCE_OBJECT_HPP
#define STAPL_RUNTIME_COLLECTIVE_ALLREDUCE_OBJECT_HPP

#include "../context.hpp"
#include "../rmi_handle.hpp"
#include "../tags.hpp"
#include "../value_handle.hpp"
#include "../yield.hpp"
#include "../communicator/reduce.hpp"
#include "../concurrency/reduction.hpp"
#include "../non_rmi/response.hpp"
#include "../request/async_rmi_request.hpp"
#include "../type_traits/is_basic.hpp"
#include "../type_traits/is_non_commutative.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include <functional>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <boost/optional.hpp>
#ifndef STAPL_DONT_USE_MPI
#include <boost/mpi/operations.hpp>
#endif

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Performs an allreduce over all locations of the current gang.
///
/// @tparam T                Value type.
/// @tparam BinaryOperation  Reduction operator type.
/// @tparam NonCommutative   @c true for non-commutative operators, otherwise
///                          @c false.
/// @tparam ComplexOperation @c true for operations without MPI equivalents or
///                          on types which are non-basic, otherwise @c false.
///
/// @ingroup runtimeCollectives
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename BinaryOperation,
         bool NonCommutative = is_non_commutative<BinaryOperation>::value,
         bool ComplexOperation
           = !((std::is_empty<BinaryOperation>::value
#ifndef STAPL_DONT_USE_MPI
                 || boost::mpi::is_mpi_op<BinaryOperation, T>::value
#endif
               ) && is_basic<T>::value)
        >
class allreduce_object
: public value_handle<T>
{
public:
  using value_type    = T;
private:
  using cache_type    = std::unordered_map<location_md::id, T>;
  using response_type =
    handle_response<packed_handle_type, allreduce_object>;

  friend response_type;

  rmi_handle          m_handle;
  gang_md&            m_gang;
  BinaryOperation     m_op;
  boost::optional<T>  m_t;

  /// Number of reduction steps.
  const unsigned int  m_steps;

  /// Current reduction step.
  unsigned int        m_step;

  /// From which location a value is expected.
  location_md::id     m_wait;

  /// Out-of-order values.
  cache_type          m_cache;

  //////////////////////////////////////////////////////////////////////
  /// @brief Sends @p t to location @p id.
  //////////////////////////////////////////////////////////////////////
  void send(const location_md::id id, T&& t)
  {
    const location_md::id myid = m_handle.get_location_id();

    aggregator a{this_context::get(), m_handle, id};
    const bool on_shmem = a.is_on_shmem();

    if (on_shmem)
    {
      constexpr auto pmf = &allreduce_object::template receive_operand<T&&>;
      using request_type = nc_async_rmi_request<
                             packed_handle_type,
                             decltype(pmf),
                             location_md::id,
                             typename transport_qualifier<T&&>::type>;
      const std::size_t size = request_type::expected_size(myid, std::move(t));
      new(a.allocate(size)) request_type{m_handle, pmf, myid, std::move(t)};
    }
    else
    {
      constexpr auto pmf =
        &allreduce_object::template receive_operand<T const&>;
      using request_type = nc_async_rmi_request<
                             packed_handle_type,
                             decltype(pmf),
                             location_md::id,
                             T>;
      const std::size_t size = request_type::expected_size(myid, std::move(t));
      new(a.allocate(size)) request_type{m_handle, pmf, myid, std::move(t)};
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sends @p t to location @p id.
  //////////////////////////////////////////////////////////////////////
  void send(const location_md::id id, T const& t)
  {
    const location_md::id myid = m_handle.get_location_id();

    aggregator a{this_context::get(), m_handle, id};
    constexpr auto pmf =
      &allreduce_object::template receive_operand<T const&>;
    using request_type = nc_async_rmi_request<
                           packed_handle_type,
                           decltype(pmf),
                           location_md::id,
                           T>;
    const std::size_t size = request_type::expected_size(myid, t);
    new(a.allocate(size)) request_type{m_handle, pmf, myid, t};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Makes progress on the reduction.
  //////////////////////////////////////////////////////////////////////
  void make_progress(void)
  {
    const location_md::id myid = m_handle.get_location_id();

    for (++m_step; m_step <= m_steps; ++m_step)
    {
      // find the receiver location
      const location_md::id recv_id = (myid - myid % (0x1<<m_step));

      if (myid != recv_id)
      {
        // this location is not a receiver of a value
        send(recv_id, std::move(*m_t));
        break;
      }

      // find the location id that a value is expected from
      m_wait = (myid + (0x1<<(m_step-1)));

      if (m_wait >= m_handle.get_num_locations())
      {
        // no more values expected
        continue;
      }

      // check if operand is already here
      auto it = m_cache.find(m_wait);

      if (it == m_cache.end())
      {
        return;
      }

      m_t = m_op(std::move(*m_t), it->second);
      m_cache.erase(it);
    }

    STAPL_RUNTIME_ASSERT(m_cache.empty());

    if (myid == 0)
    {
      // broadcasts the value to all locations
      response_type{}(m_handle, std::move(*m_t));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receives @p u from @p id.
  //////////////////////////////////////////////////////////////////////
  template<typename U>
  void receive_operand(const location_md::id id, U&& u)
  {
    if (id != m_wait)
    {
      m_cache.emplace(id, std::forward<U>(u));
    }
    else
    {
      m_t = m_op(std::move(*m_t), std::forward<U>(u));
      make_progress();
    }
  }

public:
  allreduce_object(context& ctx, BinaryOperation op = BinaryOperation{})
  : m_handle(ctx, this),
    m_gang(ctx.get_gang_md()),
    m_op(std::move(op)),
    m_steps(integral_ceil_log2(m_handle.get_num_locations())),
    m_step(1),
    m_wait(invalid_location_id)
  { }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  {
    return m_handle;
  }

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  {
    return m_handle;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initiates the allreduce.
  ///
  /// @tparam u Value to contribute to the reduction.
  //////////////////////////////////////////////////////////////////////
  template<typename U>
  void operator()(U&& u)
  {
    if (m_handle.get_num_locations() == 1)
    {
      // only one location; set the reduction value directly
      this->set_value(std::forward<U>(u));
      return;
    }

    m_wait = ((m_handle.get_location_id() % 2 == 0)
        ? (m_handle.get_location_id() + 1) : m_handle.get_num_locations());

    if (m_wait < m_handle.get_num_locations())
    {
      // this location waits value from another one; check if it arrived prior
      // to the execution of this function
      auto it = m_cache.find(m_wait);

      if (it == m_cache.end())
      {
        // value not in cache; reduction will be triggered by incoming request
        m_t = std::forward<U>(u);
        return;
      }
      // value in cache; process it and do some progress
      m_t = m_op(std::forward<U>(u), it->second);
      m_cache.erase(it);
      make_progress();
    }
    else
    {
      // does not wait from any other location; send to the receiver location
      const location_md::id myid = m_handle.get_location_id();

      for ( ; m_step <= m_steps; ++m_step)
      {
        const location_md::id recv_id = (myid - myid % (0x1<<m_step));

        if (myid != recv_id)
        {
          send(recv_id, std::forward<U>(u));
          break;
        }
      }
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref allreduce_object for commutative
///        reductions with complex operations.
///
/// @ingroup runtimeCollectives
//////////////////////////////////////////////////////////////////////
template<typename T, typename BinaryOperation>
class allreduce_object<T, BinaryOperation, false, true>
: public value_handle<T>
{
public:
  using value_type    = T;

private:
  using response_type =
    handle_response<packed_handle_type, allreduce_object>;

  rmi_handle          m_handle;
  gang_md&            m_gang;
  BinaryOperation     m_op;
  boost::optional<T>  m_t;

  /// Number of values that are pending from children.
  unsigned int        m_cnt;

  //////////////////////////////////////////////////////////////////////
  /// @brief Sends @p t to location @p id.
  //////////////////////////////////////////////////////////////////////
  void send(const location_md::id id, T&& t)
  {
    aggregator a{this_context::get(), m_handle, id};
    const bool on_shmem = a.is_on_shmem();

    if (on_shmem)
    {
      constexpr auto pmf = &allreduce_object::template receive_operand<T&&>;
      using request_type = nc_async_rmi_request<
                             packed_handle_type,
                             decltype(pmf),
                             typename transport_qualifier<T&&>::type>;
      const std::size_t size = request_type::expected_size(std::move(t));
      new(a.allocate(size)) request_type{m_handle, pmf, std::move(t)};
    }
    else
    {
      constexpr auto pmf =
        &allreduce_object::template receive_operand<T const&>;
      using request_type = nc_async_rmi_request<
                             packed_handle_type,
                             decltype(pmf),
                             T>;
      const std::size_t size = request_type::expected_size(std::move(t));
      new(a.allocate(size)) request_type{m_handle, pmf, std::move(t)};
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sends @p t to location @p id.
  //////////////////////////////////////////////////////////////////////
  void send(const location_md::id id, T const& t)
  {
    aggregator a{this_context::get(), m_handle, id};
    constexpr auto pmf =
      &allreduce_object::template receive_operand<T const&>;
    using request_type = nc_async_rmi_request<
                           packed_handle_type,
                           decltype(pmf),
                           T>;
    const std::size_t size = request_type::expected_size(t);
    new(a.allocate(size)) request_type{m_handle, pmf, t};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receives @p u, performs the reduction and forwards the result to
  ///        the parent if it is not the root, otherwise broadcasts the value to
  ///        all locations.
  //////////////////////////////////////////////////////////////////////
  template<typename U>
  void receive_operand(U&& u)
  {
    STAPL_RUNTIME_ASSERT(m_cnt > 0);

    --m_cnt;

    if (m_cnt > 0)
    {
      // more values expected
      if (!m_t)
        m_t = std::forward<U>(u);
      else
        m_t = m_op(std::move(*m_t), std::forward<U>(u));
    }
    else
    {
      // last value to arrive
      const location_md::id myid = m_handle.get_location_id();

      if (myid == 0)
      {
        // broadcast the value to all locations
        STAPL_RUNTIME_ASSERT(m_t);
        response_type{}(m_handle, m_op(std::move(*m_t), std::forward<U>(u)));
      }
      else
      {
        const location_md::id parent_id = ((myid - 1) / 2);

        if (!m_t)
          send(parent_id, std::forward<U>(u));
        else
          send(parent_id, m_op(std::move(*m_t), std::forward<U>(u)));
      }
    }
  }

public:
  allreduce_object(context& ctx, BinaryOperation op = BinaryOperation{})
  : m_handle(ctx, this),
    m_gang(ctx.get_gang_md()),
    m_op(std::move(op)),
    m_cnt(1)
  {
    const location_md::id myid = m_handle.get_location_id();
    const location_md::id n    = m_handle.get_num_locations();

    if ((2u*(myid+1u)) < n) // check if left and right children exist
      m_cnt += 2;
    else if (((2u*(myid+1u)) - 1) < n) // check if left child exists
      m_cnt += 1;
  }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  {
    return m_handle;
  }

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  {
    return m_handle;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initiates the allreduce.
  ///
  /// @tparam u Value to contribute to the reduction.
  //////////////////////////////////////////////////////////////////////
  template<typename U>
  void operator()(U&& u)
  {
    if (m_handle.get_num_locations() == 1)
    {
      // one location, set the reduction value directly
      --m_cnt;
      this->set_value(std::forward<U>(u));
    }
    else
    {
      receive_operand(std::forward<U>(u));
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref allreduce_object for commutative
///        reductions with well-known operations.
///
/// @ingroup runtimeCollectives
///
/// @todo Use platform optimized allreduce implementation (e.g.,MPI_Iallreduce).
//////////////////////////////////////////////////////////////////////
template<typename T, typename BinaryOperation>
class allreduce_object<T, BinaryOperation, false, false>
: public value_handle<T>
{
public:
  using value_type          = T;
private:
  using distmem_reduce_type = reduce<value_type, BinaryOperation>;
  using shmem_reduce_type   = reduction<value_type, BinaryOperation>;
  using response_type       =
    handle_response<packed_handle_type, allreduce_object>;

  friend response_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Base class for internal implementation.
  //////////////////////////////////////////////////////////////////////
  class impl_base
  {
  public:
    virtual ~impl_base(void) = default;
    virtual void operator()(const location_md::size_type, T const&) = 0;
    virtual void operator()(const location_md::size_type, T&&) = 0;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation for distributed memory only gangs.
  //////////////////////////////////////////////////////////////////////
  class dist_impl final
  : public impl_base
  {
  private:
    distmem_reduce_type m_red;

  public:
    dist_impl(allreduce_object& o, const collective_id cid, BinaryOperation op)
    : m_red(o.get_gang_md().get_id(),
            cid,
            o.get_gang_md().get_topology(),
            [&o](T t) { o.bcast_value(std::move(t)); },
            std::move(op))
    { }

    void operator()(const location_md::size_type, T const& t) final
    { m_red(t); }

    void operator()(const location_md::size_type, T&& t) final
    { m_red(std::move(t)); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation for shared memory only gangs.
  //////////////////////////////////////////////////////////////////////
  class shmem_impl final
  : public impl_base
  {
  private:
    allreduce_object& m_parent;
    shmem_reduce_type m_red;

  public:
    shmem_impl(allreduce_object& o, BinaryOperation op)
    : m_parent(o),
      m_red(o.get_gang_md().local_size(), std::move(op))
    { }

    void operator()(const location_md::size_type idx, T const& t) final
    {
      if (m_red(idx, t))
        m_parent.bcast_value(m_red.get());
    }

    void operator()(const location_md::size_type idx, T&& t) final
    {
      if (m_red(idx, std::move(t)))
        m_parent.bcast_value(m_red.get());
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation for mixed-mode gangs.
  //////////////////////////////////////////////////////////////////////
  class mixed_impl final
  : public impl_base
  {
  private:
    shmem_reduce_type   m_shmem_red;
    distmem_reduce_type m_distmem_red;

  public:
    mixed_impl(allreduce_object& o, const collective_id cid, BinaryOperation op)
    : m_shmem_red(o.get_gang_md().local_size(), op),
      m_distmem_red(o.get_gang_md().get_id(),
                    cid,
                    o.get_gang_md().get_topology(),
                    [&o](T t) { o.bcast_value(std::move(t)); },
                    std::move(op))
    { }

    void operator()(const location_md::size_type idx, T const& t) final
    {
      if (m_shmem_red(idx, t))
        m_distmem_red(m_shmem_red.get());
    }

    void operator()(const location_md::size_type idx, T&& t) final
    {
      if (m_shmem_red(idx, std::move(t)))
        m_distmem_red(m_shmem_red.get());
    }
  };

  rmi_handle                 m_handle;
  std::shared_ptr<impl_base> m_impl;

public:
  explicit allreduce_object(context& ctx,
                            BinaryOperation op = BinaryOperation{})
  : m_handle(ctx, this)
  {
    if (m_handle.get_num_locations()==1)
    {
      // single location gang
      return;
    }

    auto& g = ctx.get_gang_md();

    if (g.local_size()==1)
    {
      // single location on the process
      const collective_id cid = m_handle.internal_handle().abstract();
      m_impl = std::make_shared<dist_impl>(std::ref(*this), cid, std::move(op));
    }
    else if (g.get_description().is_on_shmem())
    {
      // shared memory gang
      m_impl = g.get_shared_object<shmem_impl>(m_handle,
                                               std::ref(*this),
                                               std::move(op));
    }
    else
    {
      // multiple locations on the process, multiple processes
      const collective_id cid = m_handle.internal_handle().abstract();
      m_impl = g.get_shared_object<mixed_impl>(m_handle,
                                               std::ref(*this),
                                               cid,
                                               std::move(op));
    }
  }

  rmi_handle::reference const& get_rmi_handle(void) noexcept
  { return m_handle; }

  rmi_handle::const_reference const& get_rmi_handle(void) const noexcept
  { return m_handle; }

private:
  gang_md const& get_gang_md(void) const noexcept
  { return m_handle.get_location_md().get_gang_md(); }

  using value_handle<T>::set_value;

  template<typename U>
  void bcast_value(U&& u)
  { response_type{}(m_handle, std::forward<U>(u)); }

public:
  void operator()(T const& t)
  {
    if (!m_impl)
      set_value(t);
    else
      (*m_impl)(m_handle.get_location_md().local_index(), t);
  }

  void operator()(T&& t)
  {
    if (!m_impl)
      set_value(std::move(t));
    else
      (*m_impl)(m_handle.get_location_md().local_index(), std::move(t));
  }
};

} // namespace runtime

} // namespace stapl

#endif
