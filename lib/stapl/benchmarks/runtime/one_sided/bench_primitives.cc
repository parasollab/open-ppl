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
/// Benchmark for point-to-point RMI primitives.
///
/// The following are benchmarked:
/// -# @ref stapl::async_rmi() to right neighbor (no aggregation).
/// -# @ref stapl::async_rmi() to location 0 (no aggregation).
/// -# @ref stapl::async_rmi() to right neighbor (no combining).
/// -# @ref stapl::async_rmi() to location 0 (no combining).
/// -# @ref stapl::async_rmi() to right neighbor.
/// -# @ref stapl::async_rmi() to location 0.
/// -# @ref stapl::sync_rmi() to right neighbor.
/// -# @ref stapl::sync_rmi() to location 0.
/// -# @ref stapl::opaque_rmi() to right neighbor.
/// -# @ref stapl::opaque_rmi() to location 0.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <boost/lexical_cast.hpp>

class A
: public stapl::p_object
{
private:
  unsigned int m_next;

public:
  A(void)
  : m_next((this->get_location_id()+1)%this->get_num_locations())
  { }

  explicit A(unsigned int lid)
  : m_next(lid)
  { }

  int foo(void)
  { return 0; }

  int goo(void)
  { return 0; }

  void call_async_rmi(void)
  {
    stapl::async_rmi(m_next, this->get_rmi_handle(), &A::foo);
    stapl::async_rmi(m_next, this->get_rmi_handle(), &A::goo);
  }

  void call_async_rmi_comb(void)
  {
    stapl::async_rmi(m_next, this->get_rmi_handle(), &A::foo);
    stapl::async_rmi(m_next, this->get_rmi_handle(), &A::foo);
  }

  void call_sync_rmi(void)
  {
    stapl::sync_rmi(m_next, this->get_rmi_handle(), &A::foo);
    stapl::sync_rmi(m_next, this->get_rmi_handle(), &A::goo);
  }

  void call_opaque_rmi(void)
  {
    stapl::future<int> h1 =
      stapl::opaque_rmi(m_next, this->get_rmi_handle(), &A::foo);
    stapl::future<int> h2 =
      stapl::opaque_rmi(m_next, this->get_rmi_handle(), &A::goo);
    h2.get();
    h1.get();
  }
};


class B
{
private:
  stapl::rmi_handle m_handle;
  unsigned int      m_next;

public:
  B(void)
  : m_handle(this, stapl::no_aggregation),
    m_next((stapl::get_location_id()+1)%stapl::get_num_locations())
  { }

  explicit B(unsigned int lid)
  : m_handle(this, stapl::no_aggregation),
    m_next(lid)
  { }

  int foo(void)
  { return 0; }

  int goo(void)
  { return 0; }

  void call_async_rmi(void)
  {
    stapl::async_rmi(m_next, m_handle, &B::foo);
    stapl::async_rmi(m_next, m_handle, &B::goo);
  }
};


// Kernel that benchmarks async_rmi() with no aggregation
struct async_noagg_bench_wf
{
  typedef void result_type;

  B           obj;
  std::string s;

  async_noagg_bench_wf(void)
  : s("right")
  { }

  explicit async_noagg_bench_wf(unsigned int id)
  : obj(id),
    s(boost::lexical_cast<std::string>(id))
  { }

  void operator()(void)
  { obj.call_async_rmi(); }

  std::string name(void) const
  { return std::string("async_rmi(no aggregation)") + "[" + s + "]"; }
};


// Kernel that benchmarks async_rmi()
struct async_bench_wf
{
  typedef void result_type;

  A           obj;
  std::string s;

  async_bench_wf(void)
  : s("right")
  { }

  explicit async_bench_wf(unsigned int id)
  : obj(id),
    s(boost::lexical_cast<std::string>(id))
  { }

  void operator()(void)
  { obj.call_async_rmi(); }

  std::string name(void) const
  { return std::string("async_rmi()") + "[" + s + "]"; }
};


// Kernel that benchmarks async_rmi() with combining
struct async_comb_bench_wf
{
  typedef void result_type;

  A           obj;
  std::string s;

  async_comb_bench_wf(void)
  : s("right")
  { }

  explicit async_comb_bench_wf(unsigned int id)
  : obj(id),
    s(boost::lexical_cast<std::string>(id))
  { }

  void operator()(void)
  { obj.call_async_rmi_comb(); }

  std::string name(void) const
  { return std::string("async_rmi(combining)") + "[" + s + "]"; }
};


// Kernel that benchmarks sync_rmi()
struct sync_bench_wf
{
  typedef void result_type;

  A           obj;
  std::string s;

  sync_bench_wf(void)
  : s("right")
  { }

  explicit sync_bench_wf(unsigned int id)
  : obj(id),
    s(boost::lexical_cast<std::string>(id))
  { }

  void operator()(void)
  { obj.call_sync_rmi(); }

  std::string name(void) const
  { return std::string("sync_rmi()") + "[" + s + "]"; }
};

// Kernel that benchmarks opaque_rmi()
struct opaque_bench_wf
{
  typedef void result_type;

  A           obj;
  std::string s;

  opaque_bench_wf(void)
  : s("right")
  { }

  explicit opaque_bench_wf(unsigned int id)
  : obj(id),
    s(boost::lexical_cast<std::string>(id))
  { }

  void operator()(void)
  { obj.call_opaque_rmi(); }

  std::string name(void) const
  { return std::string("opaque_rmi()") + "[" + s + "]"; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    async_noagg_bench_wf wf;
    p.warmup(wf);
    p.benchmark(wf);
  }

  {
    async_noagg_bench_wf wf(0);
    p.benchmark(wf);
  }

  {
    async_bench_wf wf;
    p.benchmark(wf);
  }

  {
    async_bench_wf wf(0);
    p.benchmark(wf);
  }

  {
    async_comb_bench_wf wf;
    p.benchmark(wf);
  }

  {
    async_comb_bench_wf wf(0);
    p.benchmark(wf);
  }

  {
    sync_bench_wf wf;
    p.benchmark(wf);
  }

  {
    sync_bench_wf wf(0);
    p.benchmark(wf);
  }

  {
    opaque_bench_wf wf;
    p.benchmark(wf);
  }

  {
    opaque_bench_wf wf(0);
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}
