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
/// Benchmark for small object pools vs @c std::malloc().
///
/// A loop of allocations, @c std::memset() calls to zero the memory and
/// deallocations is performed for each kernel. There is a warm-up round before
/// each kernel.
///
/// The following are benchmarked:
/// -# @c std::malloc()
/// -# @ref stapl::memory_allocator
/// -# @c boost::pool
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <boost/lexical_cast.hpp>
#include <boost/pool/pool.hpp>
#include <cstring>

// Kernel that benchmarks new/delete
struct alloc_wf
{
  typedef void result_type;

  const std::size_t m_size;

  explicit alloc_wf(const std::size_t size)
  : m_size(size)
  { }

  std::string name(void) const
  {
    return (std::string("malloc     ") +
            boost::lexical_cast<std::string>(m_size) + std::string(" "));
  }

  void operator()(void)
  {
    char* p = new char[m_size];
    if (!p)
      std::abort();
    std::memset(p, 0, m_size);
    delete[] p;
  }
};


// Kernel that benchmarks pools
struct pool_wf
{
  typedef void result_type;

  const std::size_t m_size;

  explicit pool_wf(const std::size_t size)
  : m_size(size)
  { }

  std::string name(void) const
  {
    return (std::string("stapl_pool ") +
            boost::lexical_cast<std::string>(m_size) + std::string(" "));
  }

  void operator()(void)
  {
    void* p = stapl::runtime::memory_allocator::allocate(m_size);
    std::memset(p, 0, m_size);
    stapl::runtime::memory_allocator::deallocate(p, m_size);
  }
};


// Kernel that benchmarks local pools
struct lpool_wf
{
  typedef void result_type;

  const std::size_t m_size;
  boost::pool<>     m_p;

  explicit lpool_wf(const std::size_t size)
  : m_size(size),
    m_p(size)
  { }

  std::string name(void) const
  {
    return (std::string("boost_pool ") +
            boost::lexical_cast<std::string>(m_size) + std::string(" "));
  }

  void operator()(void)
  {
    void* p = m_p.malloc();
    std::memset(p, 0, m_size);
    m_p.free(p);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const std::size_t min_alloc = sizeof(int);
  const std::size_t max_alloc = 2048;
  const std::size_t step = 2;
  const unsigned int num_allocs = 10000;

  stapl::runtime_profiler<> p(argc, argv);

  for (unsigned int i=min_alloc; i<=max_alloc; i += step)
  {
    alloc_wf wf(i);
    p.warmup(wf);
  }

  for (unsigned int i=min_alloc; i<=max_alloc; i += step)
  {
    alloc_wf wf(i);
    p.benchmark(wf, num_allocs);
  }

  for (unsigned int i=min_alloc; i<=max_alloc; i += step)
  {
    lpool_wf wf(i);
    p.warmup(wf);
    p.benchmark(wf, num_allocs);
  }

  for (unsigned int i=min_alloc; i<=max_alloc; i += step)
  {
    pool_wf wf(i);
    p.warmup(wf);
  }

  for (unsigned int i=min_alloc; i<=max_alloc; i += step)
  {
    pool_wf wf(i);
    p.benchmark(wf, num_allocs);
  }

  return EXIT_SUCCESS;
}
