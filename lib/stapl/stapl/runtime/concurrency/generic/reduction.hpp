/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_GENERIC_REDUCTION_HPP
#define STAPL_RUNTIME_CONCURRENCY_GENERIC_REDUCTION_HPP

#include "../config.hpp"
#include "../../exception.hpp"
#include "../../utility/cache_line_alignment.hpp"
#include <atomic>
#include <utility>
#include <vector>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Shared memory reduction using atomic counter.  Last thread through
/// does serial reduction and returns true, signaling value is ready for
/// consumption.
///
/// @todo Size of @p m_values is not dynamic, use array / unique_ptr vs vector.
//////////////////////////////////////////////////////////////////////
template<typename T, typename BinaryOperation>
class reduction
{
public:
  using value_type    = T;
  using size_type     = std::size_t;
  using operator_type = BinaryOperation;

private:
  operator_type                m_op;
  std::atomic<size_type>       m_count;
  cache_line_aligned_vector<T> m_values;

public:
  explicit reduction(const size_type nth, operator_type op = operator_type{})
  : m_op(std::move(op)),
    m_count(nth),
    m_values(nth)
  { }

  size_type size(void) const noexcept
  { return m_values.size(); }

  bool operator()(const size_type tid, T const& t)
  {
    STAPL_RUNTIME_ASSERT(m_count.load(std::memory_order_relaxed)>0);
    m_values[tid] = t;
    return (m_count.fetch_sub(1, std::memory_order_release)==1);
  }

  bool operator()(const size_type tid, T&& t)
  {
    STAPL_RUNTIME_ASSERT(m_count.load(std::memory_order_relaxed)>0);
    m_values[tid] = std::move(t);
    return (m_count.fetch_sub(1, std::memory_order_release)==1);
  }

  T get(void)
  {
    STAPL_RUNTIME_ASSERT(m_count.load(std::memory_order_relaxed)==0);
    std::atomic_thread_fence(std::memory_order_acquire);
    auto t = m_values[0];
    for (size_type i=1; i<m_values.size(); ++i)
      t = m_op(t, m_values[i]);
    m_count.fetch_add(m_values.size(), std::memory_order_relaxed);
    return t;
  }
};

} // namespace runtime

} // namespace stapl

#endif
