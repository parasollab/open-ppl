/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <cmath>
#include <ctime>
#include <vector>
#include <algorithm>

#include <boost/nondet_random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_integral.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>

using namespace stapl;

template<typename T>
T uniform_random(T min_size, T max_size, int seed = 0)
{
  boost::random_device                      seed_gen;
  boost::mt19937                            gen(seed == 0 ? seed_gen() : seed);
  boost::random::uniform_int_distribution<> dist(min_size, max_size);

  return dist(gen);
}


class random_sizes_wf
{
private:
  typedef std::size_t size_type;

  size_type                m_min_size;
  size_type                m_max_size;

public:
  typedef void result_type;

  random_sizes_wf(size_type min, size_type max)
    : m_min_size(min), m_max_size(max)
  { }

  template<typename Reference>
  result_type operator()(Reference elem) const
  {
    elem = uniform_random(m_min_size, m_max_size);
  }

  void define_type(typer& t)
  {
    t.member(m_min_size);
    t.member(m_max_size);
  }
};


bool test1d(void)
{
  array<int> ar(100);
  stapl::rmi_fence();

  return ar.size() == 100;
}


bool test2d(void)
{
  typedef array<size_t>           sizes_ct_t;
  typedef array_view<sizes_ct_t>  sizes_vw_t;

  sizes_ct_t sizes_ct(50);
  sizes_vw_t sizes_vw(sizes_ct);

  map_func(random_sizes_wf(100, 150), sizes_vw);

  array<array<int> > ar(sizes_vw);
  stapl::rmi_fence();

  bool b_passed = ar.size() == sizes_ct.size();

  for (size_t idx = 0; idx < sizes_ct.size(); ++idx)
    b_passed = b_passed && (ar[idx].size() == sizes_ct[idx]);

  stapl::rmi_fence();

  return b_passed;
}


struct get_size_wf
{
  typedef size_t result_type;

  template<typename T>
  size_t operator()(T const& t) const
  {
    return t.get().size();
  }
};


struct check_sizes_wf
{
  std::vector<std::vector<size_t> >&   m_l2_sizes_ct;
  bool&                                m_b_passed;

  typedef void result_type;

  check_sizes_wf(std::vector<std::vector<size_t> >& l2_sizes_ct, bool& b_passed)
    : m_l2_sizes_ct(l2_sizes_ct), m_b_passed(b_passed)
  { }

  template<typename View>
  void operator()(View& v)
  {
    m_b_passed = (v.size() == m_l2_sizes_ct.size());

    for (size_t idx1 = 0; idx1 < m_l2_sizes_ct.size(); ++idx1)
    {
      m_b_passed = m_b_passed && (m_l2_sizes_ct[idx1].size() == v[idx1].size());

      for (size_t idx2 = 0; idx2 < m_l2_sizes_ct[idx1].size(); ++idx2)
      {
        const size_t expected_size = m_l2_sizes_ct[idx1][idx2];

        const size_t actual_size = v[idx1].apply_get(idx2, get_size_wf());

        m_b_passed = m_b_passed && (actual_size == expected_size);
      }
    }
  }
};


bool test3d(void)
{
  boost::mt19937                            gen(17);
  boost::random::uniform_int_distribution<> dist1(10, 20);
  boost::random::uniform_int_distribution<> dist2(100, 150);
  std::vector<std::vector<size_t> >         l2_sizes_ct(20);

  for (size_t idx1 = 0; idx1 < l2_sizes_ct.size(); ++idx1)
  {
    l2_sizes_ct[idx1].resize(dist1(gen));

    for (size_t idx2 = 0; idx2 < l2_sizes_ct[idx1].size(); ++idx2)
      l2_sizes_ct[idx1][idx2] = dist2(gen);
  }

  array<array<array<int> > >               ar(l2_sizes_ct);
  array_view<array<array<array<int> > > >  ar_vw(ar);

  stapl::rmi_fence();

  bool b_passed = true;

  stapl::do_once(check_sizes_wf(l2_sizes_ct, b_passed), ar_vw);

  // Simple test to verify operator[] compiles and works
  // (i.e., properly view packing).
  stapl::do_once([&]() { ar[6][5][4] = 78; });
  stapl::do_once([&]() { b_passed = b_passed && (ar[6][5][4] == 78); });

  return b_passed;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (stapl::get_location_id() == 0)
    std::cout << "Testing variable sized multi-level array construction: ";

  bool b_passed = test1d();

  b_passed = b_passed && test2d();
  b_passed = b_passed && test3d();

  if (stapl::get_location_id() == 0)
  {
    if (b_passed)
      std::cout << "PASSED\n";
    else
      std::cout << "FAILED\n";
  }

  return EXIT_SUCCESS;
}
