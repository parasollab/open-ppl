/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE reduction<T>
#include "utility.h"
#include <stapl/runtime/concurrency/fuzzy_barrier.hpp>
#include <stapl/runtime/concurrency/reduction.hpp>
#include <functional>
#include <mutex>

using namespace stapl::runtime;

static std::mutex mtx;

template<typename T, typename BinaryFunction>
struct wrapper
{
  typedef reduction<T, BinaryFunction>       reduction_type;
  typedef typename reduction_type::size_type size_type;

  reduction_type m_red;
  fuzzy_barrier  m_barrier;

  wrapper(const std::size_t n, BinaryFunction op)
  : m_red(n, op),
    m_barrier(n)
  { }

  wrapper(wrapper const&) = delete;
  wrapper& operator=(wrapper const&) = delete;

  bool operator()(const size_type n, T const& t) noexcept
  { return m_red(n, t); }

  T get(void)
  { return m_red.get(); }

  void wait(const size_type n)
  { m_barrier.wait(n, [] { }); }
};

template<typename T, typename BinaryFunction>
struct multi_reduce
{
  typedef wrapper<T, BinaryFunction> wrapper_type;

  wrapper_type&      m_wrapper;
  T                  m_res;
  const unsigned int m_tid;

  multi_reduce(wrapper_type& w,
               unsigned int tid,
               unsigned int nth)
  : m_wrapper(w),
    m_res(),
    m_tid(tid)
  {
    BinaryFunction f;
    m_res = 1;
    for (unsigned int i=1; i<nth; ++i)
      m_res = f(m_res, i+1);
  }

  void operator()(void)
  {
    for (int i=0; i<10; ++i) {
      if (m_wrapper(m_tid, m_tid+1)) {
        T t = m_wrapper.get();
        std::lock_guard<std::mutex> l(mtx);
        BOOST_CHECK_EQUAL(t, m_res);
      }
      m_wrapper.wait(m_tid);
    }
  }
};

BOOST_AUTO_TEST_CASE( reduction_int )
{
  typedef int                                value_type;
  typedef std::plus<value_type>              operator_type;
  typedef wrapper<value_type, operator_type> wrapper_type;

  operator_type op;

#if defined(STAPL_RUNTIME_USE_OMP)

  const unsigned int NTHREADS = omp_get_max_threads();
  wrapper_type w(NTHREADS, op);

# pragma omp parallel
  {
#  pragma omp single
    {
    BOOST_CHECK_EQUAL(int(NTHREADS), omp_get_num_threads());
    }

    multi_reduce<value_type, operator_type> wf(w,
                                               omp_get_thread_num(),
                                               NTHREADS);
    wf();
  }

#else

  const unsigned int NTHREADS = 32;

  wrapper_type w(NTHREADS, op);

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back(
      multi_reduce<value_type, operator_type>(w, i, NTHREADS));
  }

  BOOST_CHECK_EQUAL(NTHREADS, threads.size());

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

#endif

}
