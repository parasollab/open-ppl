/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE thread_local_storage
#include "utility.h"
#include <stapl/runtime/concurrency/thread_local_storage.hpp>
#include <mutex>
#include <vector>

using namespace stapl::runtime;

struct A
{
  int i;

  A(void) noexcept
  : i(0)
  { }

  void inc(void) noexcept
  { ++i; }
};

static std::mutex mtx;
STAPL_RUNTIME_THREAD_LOCAL(A, tls_a)
STAPL_RUNTIME_THREAD_LOCAL(A, tls_b)

void foo(void)
{
  ++tls_a.get().i;
  ++tls_b.get().i;
  std::lock_guard<std::mutex> l(mtx);
  BOOST_CHECK_EQUAL(tls_a.get().i, 1);
  BOOST_CHECK_EQUAL(tls_b.get().i, 1);
}


BOOST_AUTO_TEST_CASE( thread_local_storage_A )
{
#if defined(STAPL_RUNTIME_USE_OMP)

# pragma omp parallel
  {
    foo();
  }

#else

  const unsigned int NTHREADS = 100;

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back(&foo);
  }

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

#endif

}
