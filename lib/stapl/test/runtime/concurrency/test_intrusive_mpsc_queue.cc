/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE intrusive_mpsc_queue<T>
#include "utility.h"
#include <stapl/runtime/concurrency/intrusive_mpsc_queue.hpp>
#include <algorithm>
#include <numeric>
#include <vector>

using namespace stapl::runtime;


struct A
: public intrusive_mpsc_queue_hook
{
  unsigned int thread;
  unsigned int value;

  A(unsigned int th, unsigned int v)
  : thread(th),
    value(v)
  { }
};


static intrusive_mpsc_queue<A> q;


void do_it(unsigned int tid, unsigned int N)
{
  for (unsigned int i=0; i<N; ++i) {
    A* a = new A(tid, i);
    q.push(*a);
  }
}


BOOST_AUTO_TEST_CASE( queue_writes )
{
  unsigned int NTHREADS = 32;

#if defined(STAPL_RUNTIME_USE_OMP)

# pragma omp parallel shared(q)
  {
#  pragma omp single
    NTHREADS = omp_get_num_threads();
    int i = omp_get_thread_num();

    do_it(i, (i*i));
  }

#else

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back([i] { do_it(i, (i*i)); });
  }

  BOOST_CHECK_EQUAL(NTHREADS, threads.size());

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

#endif

  // gather all values from all threads pushed to the queue
  std::vector<std::vector<unsigned int> > v(NTHREADS);
  for (A* a = q.try_pop(); a != 0; a = q.try_pop()) {
    v[a->thread].push_back(a->value);
    delete a;
  }

  // values between threads can interleave, but for the same thread they have
  // to be in the same order
  for (unsigned int i=0; i<NTHREADS; ++i) {
    std::vector<unsigned int> const& tmp_v = v[i];
    std::vector<unsigned int> check_v(i*i);
    std::iota(check_v.begin(), check_v.end(), 0);
    BOOST_CHECK_EQUAL(tmp_v.size(), check_v.size());
    BOOST_REQUIRE(
      std::equal(check_v.begin(), check_v.end(), tmp_v.begin())==true
    );
  }
}
