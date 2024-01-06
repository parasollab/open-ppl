/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/runtime.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>

using namespace stapl;


class test_wf
{
private:
  size_t m_nlocs;

public:
  typedef void result_type;

  test_wf(size_t nlocs)
    : m_nlocs(nlocs)
  { };

  template <typename T, typename ReplicatedElements>
  void operator()(T elem, ReplicatedElements vw) const
  {
    const size_t loc = elem + 1 == m_nlocs ? 0 : elem + 1;
    const size_t sz  = vw.size();

    vw[loc] = sz;
  }

  void define_type(typer& t)
  {
    t.member(m_nlocs);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef array<size_t>    ct_t;
  typedef array_view<ct_t> vw_t;

  if (get_location_id() == 0)
    std::cout << "Testing PARAGRAPH repeated view writes... ";

  size_t nlocs = get_num_locations();

  ct_t nums(nlocs);

  vw_t vw(nums);

  fill(vw, 0);

  map_func(test_wf(nlocs), counting_view<size_t>(nlocs), make_repeat_view(vw));

  if (get_location_id() == 0)
  {
    bool flag = true;
    for (size_t i=0; i<nlocs; ++i)
    {
      if (nlocs != vw[i])
        flag = false;
    }

    if (flag)
      std::cout <<"Passed\n";
    else
      std::cout <<"Failed\n";
  }

  return EXIT_SUCCESS;
}
