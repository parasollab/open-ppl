/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_GENERIC_CENTRALIZED_FUZZY_BARRIER_HPP
#define STAPL_RUNTIME_CONCURRENCY_GENERIC_CENTRALIZED_FUZZY_BARRIER_HPP

#include "../config.hpp"
#include "../../exception.hpp"
#include <atomic>
#include <cstddef>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Centralized fuzzy barrier.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
class centralized_fuzzy_barrier
{
public:
  typedef std::size_t size_type;

private:
  /// Total number of threads.
  const size_type                                        m_nth;
  /// Sense flag for barrier release.
  STAPL_RUNTIME_CACHELINE_ALIGNED std::atomic<bool>      m_flag;
  /// Number of threads not yet arrived to the barrier.
  STAPL_RUNTIME_CACHELINE_ALIGNED std::atomic<size_type> m_count;

public:
  explicit centralized_fuzzy_barrier(const size_type nth = 1) noexcept
  : m_nth(nth),
    m_flag(false),
    m_count(nth)
  { }

  template<typename Function>
  void wait(const size_type /*tid*/, Function&& f)
  {
    const bool flag = m_flag.load();
    if (--m_count==0) {
      m_count = m_nth;
      m_flag  = !flag;
    }
    else {
      while (flag==m_flag.load())
        f();
    }
  }
};


typedef centralized_fuzzy_barrier fuzzy_barrier;

} // namespace runtime

} // namespace stapl

#endif
