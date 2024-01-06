/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <cstdlib>
#include <iostream>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>

#include "../../test_report.hpp"

using namespace std;
using namespace stapl;

struct unmerge_gen
  : public p_object
{
  typedef size_t                  value_type;
  typedef value_type              reference;
  typedef const value_type        const_reference;
  typedef indexed_domain<size_t>  domain_type;
  typedef size_t                  gid_type;

  enum gen_mode
  {
    overlap_mode,
    separate_mode,
    offset_mode
  };

  size_t m_nelem;
  size_t m_num;
  gen_mode m_mode;

  unmerge_gen(size_t nelem, size_t num, gen_mode mode)
    : m_nelem(nelem)
    , m_num(num)
    , m_mode(mode)
  {
  }

  value_type get_element(size_t ix) const
  {
    //avoid sorting
    switch (m_mode)
    {
    case overlap_mode:
      return ix*2 + m_num;
    case separate_mode:
      return ix + m_num*m_nelem;
    case offset_mode:
      return ix*2 + m_num + ((m_num == 1) ? 2*m_nelem/get_num_locations() : 0);
    default:
      stapl_assert(false, "Invalid test case");
      return 0;
    }
  }

  template <typename F>
  value_type apply_get(gid_type const& idx, F const& f) const
  { return f(this->get_element(idx)); }

  domain_type domain(void) const
  {
    return domain_type(0,m_nelem-1);
  }

  size_t version(void) const
  {
    return 0;
  }

  bool validate(void)
  {
    return true;
  }

  size_t size(void) const
  {
    return m_nelem;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef static_array<size_t>      array_type;
  typedef array_view<array_type>    array_view_t;
  typedef array_view<unmerge_gen>   unmerge_view;

  unsigned int nelem = 10000;
  if (argc>1) nelem = atoi(argv[1]);

  unmerge_gen::gen_mode mode = unmerge_gen::separate_mode;
  if (argc>2) mode = (unmerge_gen::gen_mode)(argv[2][0]-'0');

  unmerge_gen gen1(nelem, 0, mode);
  unmerge_gen gen2(nelem, 1, mode);
  unmerge_view gview1(gen1);
  unmerge_view gview2(gen2);

  array_type array1(nelem);
  array_type array2(nelem);
  array_view_t view1(array1);
  array_view_t view2(array2);

  // Create sorted arrays
  copy(gview1, view1);
  copy(gview2, view2);

  array_type merged(2*nelem);
  array_view_t mergev(merged);
  merge(view1, view2, mergev);

  array_type merged2(2*nelem);
  array_view_t mergev2(merged2);
  std::merge(view1.begin(), view1.end(),
             view2.begin(), view2.end(), mergev2.begin());

  if (nelem <= 10)
    do_once([&](void) {
      for (size_t i=0; i<nelem*2; ++i)
        cout << mergev[i] << " - ";
      cout << endl;
      for (size_t i=0; i<nelem*2; ++i)
        cout << mergev2[i] << " - ";
      cout << endl;
    });

  bool passed = stapl::equal(mergev, mergev2);

  STAPL_TEST_REPORT(passed, argv[0]);

  return EXIT_SUCCESS;
}

