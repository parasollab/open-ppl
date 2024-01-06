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
/// Benchmark where each location communicates with sqrt(N) locations, where N
/// is the total number of locations.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

class A
: public stapl::p_object
{
private:
  std::vector<unsigned int> m_locs;
  unsigned int              m_num;
  std::mt19937              m_rng;

  struct randomizer
  {
    std::mt19937& m_rng;

    explicit randomizer(std::mt19937& rng)
    : m_rng(rng)
    { }

    unsigned int operator()(unsigned int i)
    {
      return std::uniform_int_distribution<unsigned int>(0, i)(m_rng);
    }
  };

public:
  A(void)
  : m_locs(this->get_num_locations(), 0),
    m_num(int(std::sqrt(double(m_locs.size()))))
  {
    if (m_locs.size()>1) {
      std::fill(m_locs.begin() + 1, m_locs.end(), 1);
      std::partial_sum(m_locs.begin(), m_locs.end(), m_locs.begin());
    }
    this->advance_epoch();
  }

  void pong(void)
  { }

  void ping(unsigned int l)
  {
    stapl::async_rmi(l, this->get_rmi_handle(), &A::pong);
  }

  void call_pong(void)
  {
    randomizer r(m_rng);
    std::random_shuffle(m_locs.begin(), m_locs.end(), r);
    for (unsigned int i=0; i<m_num; ++i)
      stapl::async_rmi(m_locs[i], this->get_rmi_handle(), &A::pong);
  }

  void call_ping_pong(void)
  {
    unsigned int myid = get_location_id();
    randomizer r(m_rng);
    std::random_shuffle(m_locs.begin(), m_locs.end(), r);
    for (unsigned int i=0; i<m_num; ++i)
      stapl::async_rmi(m_locs[i], this->get_rmi_handle(), &A::ping, myid);
  }
};


// Kernel that does asyncs to random sqrt(p) locations
struct pong_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("multipong"); }

  A obj;

  void operator()(void)
  { obj.call_pong(); }
};


// Kernel that does asyncs to random sqrt(p) locations, followed by a reply
struct ping_pong_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("multipingpong"); }

  A obj;

  void operator()(void)
  { obj.call_ping_pong(); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    pong_wf wf;
    p.benchmark(wf);
  }

  {
    ping_pong_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}
