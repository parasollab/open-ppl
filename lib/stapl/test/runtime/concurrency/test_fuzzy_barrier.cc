/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE fuzzy_barrier
#include "utility.h"
#include <stapl/runtime/concurrency/fuzzy_barrier.hpp>
#include <algorithm>
#include <atomic>

struct empty_wf
{
  void operator()(void) const
  { }
};


using stapl::runtime::fuzzy_barrier;
using std::atomic;


struct multi_barrier
{
  typedef void result_type;

  const unsigned int m_tid;
  const unsigned int m_nth;
  fuzzy_barrier&     m_barrier;

  multi_barrier(fuzzy_barrier& barrier,
                unsigned int tid,
                unsigned int nth) noexcept
  : m_tid(tid),
    m_nth(nth),
    m_barrier(barrier)
  { }

  void operator()(void)
  {
    for (unsigned int i=0; i<m_nth; ++i) {
      m_barrier.wait(m_tid, empty_wf());
    }
  }
};


BOOST_AUTO_TEST_CASE( multiple_calls )
{
#if defined(STAPL_RUNTIME_USE_OMP)

  const unsigned int NTHREADS = omp_get_max_threads();
  fuzzy_barrier barrier(NTHREADS);

# pragma omp parallel
  {
#  pragma omp single
    BOOST_CHECK_EQUAL(int(NTHREADS), omp_get_num_threads());

    multi_barrier wf(barrier, omp_get_thread_num(), NTHREADS);
    wf();
  }

#else

  const unsigned int NTHREADS = 32;

  fuzzy_barrier barrier(NTHREADS);

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back(multi_barrier(barrier, i, NTHREADS));
  }

  BOOST_CHECK_EQUAL(NTHREADS, threads.size());

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

#endif
}

template<typename WF>
struct increase
{
  fuzzy_barrier&    m_barrier;
  atomic<int>*      m_values;
  const std::size_t m_size;
  const std::size_t m_pos;

  increase(fuzzy_barrier& barrier,
           atomic<int>* const values,
           const std::size_t size,
           const std::size_t pos)
  : m_barrier(barrier),
    m_values(values),
    m_size(size),
    m_pos(pos)
  { }

  void operator()(void)
  {
    m_values[m_pos] = -1;
    m_barrier.wait(m_pos, WF());
    if (m_pos==0) {
      for (std::size_t i=0; i<m_size; ++i) {
        BOOST_CHECK_EQUAL(m_values[i], -1);
      }
    }
  }
};

BOOST_AUTO_TEST_CASE( test_empty_wf )
{
#if defined(STAPL_RUNTIME_USE_OMP)

  const unsigned int NTHREADS = omp_get_max_threads();

  atomic<int>* values = new atomic<int>[NTHREADS];
  std::fill(values, values + NTHREADS, 0);

  fuzzy_barrier barrier(NTHREADS);

# pragma omp parallel
  {
#  pragma omp single
    BOOST_CHECK_EQUAL(int(NTHREADS), omp_get_num_threads());

    int i = omp_get_thread_num();
    increase<empty_wf> wf(barrier, values, NTHREADS, i);
    wf();
  }

#else

  const unsigned int NTHREADS = 32;

  atomic<int>* values = new atomic<int>[NTHREADS];
  std::fill(values, values + NTHREADS, 0);

  fuzzy_barrier barrier(NTHREADS);

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back(increase<empty_wf>(barrier, values, NTHREADS, i));
  }

  BOOST_CHECK_EQUAL(NTHREADS, threads.size());

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

#endif

  for (std::size_t i=0; i<NTHREADS; ++i) {
    BOOST_CHECK_EQUAL(values[i], -1);
  }

  delete[] values;
}

static atomic<int> I;

struct non_empty_wf
{
  void operator()(void) const
  { ++I; }
};

BOOST_AUTO_TEST_CASE( test_non_empty_wf )
{
  I = 0;

#if defined(STAPL_RUNTIME_USE_OMP)

  const unsigned int NTHREADS = omp_get_max_threads();

  atomic<int>* values = new atomic<int>[NTHREADS];
  std::fill(values, values + NTHREADS, 0);

  fuzzy_barrier barrier(NTHREADS);

# pragma omp parallel
  {
#  pragma omp single
    BOOST_CHECK_EQUAL(int(NTHREADS), omp_get_num_threads());

    int i = omp_get_thread_num();
    increase<non_empty_wf> wf(barrier, values, NTHREADS, i);
    wf();
  }

  BOOST_REQUIRE(NTHREADS==1 || I > 0);

#else

  const unsigned int NTHREADS = 32;

  atomic<int>* values = new atomic<int>[NTHREADS];
  std::fill(values, values + NTHREADS, 0);

  fuzzy_barrier barrier(NTHREADS);

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back(increase<non_empty_wf>(barrier, values, NTHREADS, i));
  }

  BOOST_CHECK_EQUAL(NTHREADS, threads.size());

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

  BOOST_REQUIRE(NTHREADS==1 || I > 0);

#endif

  for (std::size_t i=0; i<NTHREADS; ++i) {
    BOOST_CHECK_EQUAL(values[i], -1);
  }

  delete[] values;
}
