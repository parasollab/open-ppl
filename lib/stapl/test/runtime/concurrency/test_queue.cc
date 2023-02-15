/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE queue<T>
#include "utility.h"
#include <stapl/runtime/concurrency/queue.hpp>

using stapl::runtime::queue;

// adds to the queue
template<typename T>
struct add_to_queue
{
  stapl::runtime::queue<T>& m_q;
  const unsigned int        m_n;
  const T                   m_t;

  add_to_queue(queue<T>& q,
               const unsigned int n,
               T const& t)
  : m_q(q),
    m_n(n),
    m_t(t)
  { }

  void operator()(void)
  {
    for (unsigned int i = 0; i<m_n; ++i) {
      m_q.push(m_t);
    }
  }
};

BOOST_AUTO_TEST_CASE( queue_writes )
{
  unsigned int NTHREADS = 32;

  queue<int> q;

#if defined(STAPL_RUNTIME_USE_OMP)

# pragma omp parallel
  {
#  pragma omp single
    NTHREADS = omp_get_num_threads();
    int i = omp_get_thread_num();

    add_to_queue<int> a(q, i+1, i);
    a();
  }

#else

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back(add_to_queue<int>(q, i+1, i));
  }

  BOOST_CHECK_EQUAL(NTHREADS, threads.size());

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

#endif

  std::vector<unsigned int> vals(NTHREADS, 0);
  int x = 0;
  while (q.try_pop(x)) {
    ++vals[x];
  }
  for (unsigned int i=0; i<NTHREADS; ++i) {
    BOOST_CHECK_EQUAL(vals.at(i), (i+1));
  }
}
