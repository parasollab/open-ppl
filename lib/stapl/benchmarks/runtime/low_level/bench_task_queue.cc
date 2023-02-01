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
/// Benchmark for @ref stapl::runtime::task_queue.
///
/// The benchmark attempts to output to a file through multiple threads.
///
/// The following are benchmarked:
/// -# Output with regular @c std::mutex.
/// -# Output with @ref stapl::runtime::task_queue.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <stapl/runtime/concurrency/task_queue.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <omp.h>

using namespace stapl::runtime;

template<std::size_t N>
class data
{
private:
  char m_data[N];

  template<std::size_t M>
  friend std::ostream& operator<<(std::ostream&, data<M> const&);

public:
  data(void)
  {
    std::fill(m_data, m_data + N - 1, 'a');
    m_data[N-1] = '\0';
  }
};

template<std::size_t M>
inline std::ostream& operator<<(std::ostream& os, data<M> const& d)
{
  return os << d.m_data;
}


class my_stream
{
private:
  std::ofstream                m_ofs;
  std::mutex                   m_mtx;
  task_queue<void(my_stream&)> m_q;

  template<typename T>
  void immediate_print(T const& t)
  {
    m_ofs << t << '\n';
  }

public:
  explicit my_stream(const char* s)
  : m_ofs(s)
  { }

  ~my_stream(void)
  {
    if (!m_q.empty())
      std::abort();
  }

  template<typename T>
  void immediate(T const& t)
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    immediate_print(t);
  }

  template<typename T>
  void deferred(T const& t)
  {
    std::unique_lock<std::mutex> lock(m_mtx, std::try_to_lock);
    if (!lock.owns_lock()) {
      m_q.add([t](my_stream& s) { s.immediate_print(t); });
      return;
    }
    m_q.drain(*this);
    immediate_print(t);
  }

  void flush(void)
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    m_q.drain(*this);
  }
};

const int N = 1000000;

// Kernel that benchmarks deferred section
template<std::size_t Size>
struct deferred_wf
{
  typedef void result_type;

  my_stream& m_os;

  deferred_wf(my_stream& os)
  : m_os(os)
  { }

  static std::string name(void)
  { return std::string("deferred"); }

  void operator()(void)
  {
    my_stream& os = m_os;
#pragma omp parallel default(none) shared(os)
    {
      data<Size> data;
      for (int i = 0; i < N; ++i) {
        os.deferred(data);
      }
    } // end omp parallel
    m_os.flush();
  }
};


// Kernel that benchmarks locked section
template<std::size_t Size>
struct locked_wf
{
  typedef void result_type;

  my_stream& m_os;

  locked_wf(my_stream& os)
  : m_os(os)
  { }

  static std::string name(void)
  { return std::string("locked"); }

  void operator()(void)
  {
    my_stream& os = m_os;
#pragma omp parallel default(none) shared(os)
    {
      data<Size> data;
      for (int i = 0; i < N; ++i) {
        os.immediate(data);
      }
    } // end omp parallel
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    my_stream os("file2.txt");
    locked_wf<8> wf(os);
    p.benchmark(wf);
  }

  {
    my_stream os("file1.txt");
    deferred_wf<8> wf(os);
    p.benchmark(wf);
  }

  {
    my_stream os("file2.txt");
    locked_wf<32> wf(os);
    p.benchmark(wf);
  }

  {
    my_stream os("file1.txt");
    deferred_wf<32> wf(os);
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}
