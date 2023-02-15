/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE task_queue<T>
#include "utility.h"
#include <stapl/runtime/concurrency/task_queue.hpp>
#include <algorithm>
#include <mutex>
#include <vector>
#include <utility>

using namespace stapl::runtime;

template<typename T>
class shared_object
{
private:
  std::vector<std::pair<int, T> >  m_vector;
  task_queue<void(shared_object*)> m_q;
  std::mutex                       m_mtx;

  void add_internal(unsigned int i, T const& t)
  { m_vector.push_back(std::make_pair(i, t)); }

public:
  shared_object(void) = default;

  shared_object(shared_object const&) = delete;
  shared_object& operator=(shared_object const&) = delete;

  void add(unsigned int i, T const& t)
  {
    std::unique_lock<std::mutex> lock(m_mtx, std::try_to_lock);
    if (!lock.owns_lock()) {
      m_q.add([i, t](shared_object* p) { p->add_internal(i, t); });
      return;
    }
    m_q.drain(this);
    add_internal(i, t);
  }

  void done(void)
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    m_q.drain(this);
  }

  std::vector<std::vector<T> > get(const unsigned int size) const
  {
    std::vector<std::vector<T> > v(size);
    for (unsigned int i=0; i<m_vector.size(); ++i) {
      const unsigned int tid = m_vector[i].first;
      BOOST_REQUIRE(tid<v.size());
      v[tid].push_back(m_vector[i].second);
    }
    return v;
  }
};

void do_it(shared_object<int>& o, int i)
{
  const int N  = 100000;
  for (int n = 0; n<N; ++n)
    o.add(i, n);
}

BOOST_AUTO_TEST_CASE( queue_writes )
{
  unsigned int NTHREADS = 32;

  shared_object<int> so;

#if defined(STAPL_RUNTIME_USE_OMP)

# pragma omp parallel shared(so)
  {
#  pragma omp single
    NTHREADS = omp_get_num_threads();
    int i = omp_get_thread_num();

    do_it(so, i);
  }

#else

  // create some threads
  std::vector<std::thread> threads;
  threads.reserve(NTHREADS);
  for (unsigned int i=0; i<NTHREADS; ++i) {
    threads.emplace_back([&so, i] { do_it(so, i); });
  }

  BOOST_CHECK_EQUAL(NTHREADS, threads.size());

  // wait for them
  for (unsigned int i=0; i<threads.size(); ++i) {
    threads[i].join();
  }

#endif

  so.done();

  std::vector<std::vector<int> > vals = so.get(NTHREADS);
  for (unsigned int i=0; i<vals.size(); ++i) {
    BOOST_REQUIRE(std::is_sorted(vals[i].begin(), vals[i].end()));
  }
}
