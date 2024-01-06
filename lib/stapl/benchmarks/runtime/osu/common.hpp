/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Common facilities for OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_BENCHMARKS_RUNTIME_OSU_COMMON_HPP
#define STAPL_BENCHMARKS_RUNTIME_OSU_COMMON_HPP

#include <stapl/runtime.hpp>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <utility>
#include <vector>


//////////////////////////////////////////////////////////////////////
/// @brief Prints the given string and exits with a failure.
//////////////////////////////////////////////////////////////////////
#define error(...)                    \
 {                                    \
   std::fprintf(stderr, __VA_ARGS__); \
   std::exit(EXIT_FAILURE);           \
 }


//////////////////////////////////////////////////////////////////////
/// @brief Distributed helper object to transport raw bytes of memory.
//////////////////////////////////////////////////////////////////////
class distributed_object
: public stapl::p_object
{
private:
  unsigned int m_peer;
  unsigned int m_counter;
  unsigned int m_cur_counter;
  bool         m_done;
  char*        m_data;

  void set_done(void) noexcept
  { m_done = true; }

  template<typename T>
  void put_pong(T const&) noexcept
  { m_done = true; }

  template<typename T>
  void add_pong(T const& t) noexcept
  {
    std::size_t i = 0;
    for (auto it = std::begin(t); it != std::end(t); ++it, ++i) {
      m_data[i] += *it;
    }
    m_done = true;
  }

public:
  explicit distributed_object(const unsigned int flags, char* p = nullptr)
  : stapl::p_object(flags),
    m_peer((this->get_location_id() + 1) % this->get_num_locations()),
    m_counter(0),
    m_cur_counter(0),
    m_done(false),
    m_data(p)
  { }

  void set_peer(const unsigned int i) noexcept
  { m_peer = i; }

  unsigned int get_peer(void) const noexcept
  { return m_peer; }

  void set_counter(const unsigned int n) noexcept
  {
    m_counter = n;
    if (m_cur_counter!=0)
      error("Benchmark in incorrect state.\n");
  }

  void set_data_ptr(char* p) noexcept
  {
    if (!p)
      error("nullptr detected.\n");
    m_data = p;
  }

  char* get_data_ptr(void) const noexcept
  { return m_data; }

  void recv_sync(void) const noexcept
  { }

  template<typename T>
  void put(T const&) noexcept
  { }

  template<typename T>
  void put_ack(T const&)
  {
    if (++m_cur_counter==m_counter) {
      stapl::async_rmi(m_peer, this->get_rmi_handle(),
                       &distributed_object::set_done);
      stapl::rmi_flush();
      m_cur_counter = 0;
    }
  }

  template<typename T>
  void put_ping(T const& t)
  {
    stapl::async_rmi(m_peer, this->get_rmi_handle(),
                     &distributed_object::template put_pong<T>,
                     T{m_data, t.size()});
  }

  void wait_done(void)
  {
    stapl::block_until([this] { return this->m_done; });
    m_done = false;
  }

  std::vector<char>
  get(const std::size_t disp, const std::size_t size) const noexcept
  { return std::vector<char>(m_data + disp, m_data + disp + size); }

  void get_promise(stapl::promise<std::vector<char>> p,
                   const std::size_t disp,
                   const std::size_t size) const
  { p.set_value(get(disp, size)); }

  std::vector<char> get_nodisp(const std::size_t size) const noexcept
  { return std::vector<char>{m_data, m_data + size}; }

  void get_nodisp_promise(stapl::promise<std::vector<char>> p,
                          const std::size_t size) const
  { p.set_value(get_nodisp(size)); }

  template<typename T>
  void add(T const& t) noexcept
  {
    std::size_t i = 0;
    for (auto it = std::begin(t); it != std::end(t); ++it, ++i) {
      m_data[i] += *it;
    }
  }

  template<typename T>
  void add_ack(T const& t)
  {
    std::size_t i = 0;
    for (auto it = std::begin(t); it != std::end(t); ++it, ++i) {
      m_data[i] += *it;
    }
    if (++m_cur_counter==m_counter) {
      stapl::async_rmi(m_peer, this->get_rmi_handle(),
                       &distributed_object::set_done);
      stapl::rmi_flush();
      m_cur_counter = 0;
    }
  }

  template<typename T>
  void add_ping(T const& t)
  {
    std::size_t i = 0;
    for (auto it = std::begin(t); it != std::end(t); ++it, ++i) {
      m_data[i] += *it;
    }
    stapl::async_rmi(m_peer, this->get_rmi_handle(),
                     &distributed_object::template add_pong<T>,
                     T{m_data, t.size()});
  }

  template<typename T>
  std::vector<char> get_add(T const& t)
  {
    const std::vector<char> v{m_data, m_data + t.size()};
    std::size_t i = 0;
    for (auto it = std::begin(t); it != std::end(t); ++it, ++i) {
      m_data[i] += *it;
    }
    return v;
  }

  template<typename T>
  void get_add_promise(stapl::promise<std::vector<char>> p, T const& t)
  {
    p.set_value(std::vector<char>(m_data, m_data + t.size()));
    std::size_t i = 0;
    for (auto it = std::begin(t); it != std::end(t); ++it, ++i) {
      m_data[i] += *it;
    }
  }

  template<typename T>
  void put_done(T const&) noexcept
  { m_done = true; }

  std::vector<char> get_done(const std::size_t size) noexcept
  {
    m_done = true;
    return std::vector<char>{m_data, m_data + size};
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Distributed object with one value per part.
//////////////////////////////////////////////////////////////////////
template<typename T>
class distributed_value
: public stapl::p_object
{
private:
  T    m_t;
  bool m_done;

  struct add_wf
  {
    T&       m_t1;
    T const& m_t2;

    add_wf(T& t1, T const& t2)
    : m_t1(t1),
      m_t2(t2)
    { }

    ~add_wf(void)
    { m_t1 += m_t2; }
  };

public:
  struct proxy_type
  {
    T* m_t;

    explicit proxy_type(T* t = nullptr)
    : m_t(t)
    { }

    void define_type(stapl::typer& t)
    { t.member(m_t); }
  };

  explicit distributed_value(const bool aggregation)
  : stapl::p_object(aggregation ? 0 : stapl::no_aggregation),
    m_t(),
    m_done(false)
  { }

  void set_value(T const& t)
  { m_t = t; }

  void set_value(T&& t)
  { m_t = std::move(t); }

  void wait_done(void)
  {
    stapl::block_until([this] { return this->m_done; });
    m_done = false;
  }

  T get_value(void) const
  { return m_t; }

  T get_value_done(void) noexcept
  {
    m_done = true;
    return m_t;
  }

  T const& get_value_ref(void) const noexcept
  { return m_t; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Performs a compare-and-swap on the stored value.
  ///
  /// If it succeeds, then the @ref proxy_type has no value, otherwise it stores
  /// the stored value as a pointer.
  //////////////////////////////////////////////////////////////////////
  proxy_type compare_and_swap(T const& expected, T const& desired)
  {
    if (m_t==expected) {
      m_t = desired;
      return proxy_type{};
    }
    return proxy_type{&m_t};
  }

  void compare_and_swap_promise(stapl::promise<proxy_type> p,
                                T const& expected, T const& desired)
  {
    if (m_t==expected) {
      m_t = desired;
      p.set_value(proxy_type());
    }
    else {
      p.set_value(proxy_type{&m_t});
    }
  }

  T fetch_and_add(T const& t)
  {
    add_wf a{m_t, t};
    return m_t;
  }

  void fetch_and_add_promise(stapl::promise<T> p, T const& t)
  {
    p.set_value(m_t);
    m_t += t;
  }

  template<typename BinaryOperation>
  T reduce(BinaryOperation&& op) const
  {
    return stapl::unordered::sync_reduce_rmi(std::forward<BinaryOperation>(op),
                                             this->get_rmi_handle(),
                                             &distributed_value::get_value_ref);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pointer in buffer pointed to by @p p such that the required
///        alignment is fulfilled.
//////////////////////////////////////////////////////////////////////
inline void* align_buffer(void* p, const std::size_t align_size) noexcept
{
  return reinterpret_cast<void*>(
          (reinterpret_cast<uintptr_t>(p) + (align_size-1)) /
           align_size * align_size);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sets a value to the buffer pointed to by @p p, so that the memory
///        addresses are touched.
//////////////////////////////////////////////////////////////////////
inline void touch_data(void* p, const std::size_t size) noexcept
{
  std::memset(p, 'a', size);
}


//////////////////////////////////////////////////////////////////////
/// @brief RMI primitive type.
//////////////////////////////////////////////////////////////////////
enum class rmi_primitive
{
  UNKNOWN = 0x0,
  ASYNC_RMI,
  TRY_RMI,
  SYNC_RMI,
  OPAQUE_RMI,
  PROMISE_WITH_ASYNC_RMI,
  REDUCE_RMI,
  UNORDERED_REDUCE_RMI,
  ASYNC_RMI_ALL_LOCS,
  UNORDERED_ASYNC_RMI,
  UNORDERED_ASYNC_RMI_ALL_LOCS,
  OPAQUE_RMI_ALL_LOCS,
  UNORDERED_OPAQUE_RMI_ALL_LOCS,
  ALLGATHER_RMI,
  ALLREDUCE_RMI,
  BROADCAST_RMI,
  BIND_RMI,
  FENCE,
  BARRIER,
  PING_PONG
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the RMI primitive type as a <tt>const char*</tt>.
///
/// @related rmi_primitive
//////////////////////////////////////////////////////////////////////
inline const char* get_primitive_name(rmi_primitive pr)
{
  switch (pr) {
    case rmi_primitive::ASYNC_RMI:
      return "async_rmi()";
    case rmi_primitive::TRY_RMI:
      return "try_rmi()";
    case rmi_primitive::SYNC_RMI:
      return "sync_rmi()";
    case rmi_primitive::OPAQUE_RMI:
      return "opaque_rmi()";
    case rmi_primitive::PROMISE_WITH_ASYNC_RMI:
      return "async_rmi(promise)";

    case rmi_primitive::REDUCE_RMI:
      return "reduce_rmi()";
    case rmi_primitive::UNORDERED_REDUCE_RMI:
      return "unordered::reduce_rmi()";

    case rmi_primitive::ASYNC_RMI_ALL_LOCS:
      return "async_rmi(all_locations)";
    case rmi_primitive::OPAQUE_RMI_ALL_LOCS:
      return "opaque_rmi(all_locations)";
    case rmi_primitive::UNORDERED_ASYNC_RMI:
      return "unordered::async_rmi()";
    case rmi_primitive::UNORDERED_ASYNC_RMI_ALL_LOCS:
      return "unordered::async_rmi(all_locations)";
    case rmi_primitive::UNORDERED_OPAQUE_RMI_ALL_LOCS:
      return "unordered::opaque_rmi(all_locations)";

    case rmi_primitive::ALLGATHER_RMI:
      return "allgather_rmi()";
    case rmi_primitive::ALLREDUCE_RMI:
      return "allreduce_rmi()";
    case rmi_primitive::BROADCAST_RMI:
      return "broadcast_rmi()";

    case rmi_primitive::BIND_RMI:
      return "bind_rmi()";

    case rmi_primitive::FENCE:
      return "rmi_fence()";
    case rmi_primitive::BARRIER:
      return "rmi_barrier()";

    case rmi_primitive::PING_PONG:
      return "ping-pong";

    default:
      error("%s(): Incorrect primitive.\n", __func__);
      return "";
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the reduction of the given value among all locations on
///        location @c 0.
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation, typename T>
T reduce_value(BinaryOperation op, T const& t)
{
  distributed_value<T> d{false};
  d.set_value(t);
  stapl::rmi_fence(); // wait for set_value to finish on all locations
  return stapl::allreduce_rmi(op, d.get_rmi_handle(),
                              &distributed_value<T>::get_value).get();
}


//////////////////////////////////////////////////////////////////////
/// @brief Distributes the given value to all locations.
//////////////////////////////////////////////////////////////////////
template<typename T>
void distribute_value(const unsigned int root, T& t)
{
  distributed_value<T> d{false};
  if (stapl::get_location_id()==root) {
    void (distributed_value<T>::*pmf)(T const&) =
      &distributed_value<T>::set_value;
    stapl::unordered::async_rmi(stapl::all_locations, d.get_rmi_handle(),
                                pmf, t);
    stapl::rmi_fence();
  }
  else {
    stapl::rmi_fence();
    t = d.get_value();
  }
}

#endif
