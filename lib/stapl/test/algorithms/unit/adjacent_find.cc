/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/views/array_ro_view.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/views/counting_view.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/list.hpp>

#include "../../test_report.hpp"

using namespace stapl;


class custom_gen_cont
  : public p_object
{
private:
  size_t m_size;

public:
  using value_type      = int;
  using reference       = value_type;
  using const_reference = const value_type;
  using domain_type     = indexed_domain<size_t>;
  using gid_type        = size_t;

  custom_gen_cont(size_t size)
    : m_size(size)
  {
    srand(time(NULL) + (get_location_id() + 100) * 3 + get_location_id());
  }

  const_reference operator[](size_t n) const
  {
    return ((n == m_size-1) || (n == m_size-2)) ? 0 : n + 1;
  }

  const_reference get_element(size_t n) const
  {
    return ((n == m_size-1) || (n == m_size-2)) ? 0 : n + 1;
  }

  template<typename Functor>
  const_reference apply_get(size_t idx, Functor const& f) const
  {
    return f(get_element(idx));
  }

  size_t size(void) const
  {
    return m_size;
  }

  domain_type domain(void) const
  {
    return domain_type(0, m_size-1);
  }

  void define_type(typer& t)
  {
    t.member(m_size);
  }

  size_t version(void) const
  {
    return 0;
  }
}; // class custom_gen_cont


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t n = 100;

  if (argc == 2)
  {
    n = atoi(argv[1]);

    if (n < 2)
    {
      do_once([](void)
        { std::cout << "Problem size too small, should be greater than 2\n"; });

      exit(0);
    }
  }

  custom_gen_cont                cgc(n);
  array_ro_view<custom_gen_cont> cgenv(cgc);

  //
  // p_array test
  //
  using parray_t = array<int>;
  using view_t   = array_view<parray_t>;
  using ref_t    = view_t::reference;

  parray_t pa(n);
  view_t view(pa);

  //
  // Test match found.
  //
  copy(cgenv, view);
  const ref_t ref1   = adjacent_find(view);
  const bool result1 = !is_null_reference(ref1);

  //
  // Test NO match found
  //
  copy(counting_view<int>(n),view);
  const ref_t ref2   = adjacent_find(view);
  const bool result2 = is_null_reference(ref2);

  STAPL_TEST_REPORT(result1 && result2, "Testing adjacent_find over array")

/* Fails assert - mapping invalid cid to location
  //
  // list test
  //
  using list_t  = list<int>;
  using viewl_t = list_view<list_t>;
  using refl_t  = viewl_t::reference;

  list_t pl(n);
  viewl_t viewl(pl);

  //
  // Test match found
  //
  copy(cgenv, viewl);

  const refl_t ref3  = adjacent_find(viewl);
  const bool result3 = !is_null_reference(ref3);

  //
  // Test NO match found
  //
  copy(counting_view<int>(n), viewl);

  const refl_t ref4  = adjacent_find(viewl);
  const bool result4 = is_null_reference(ref4);

  STAPL_TEST_REPORT(result3 && result4, "Testing adjacent_find over list")
*/

  return EXIT_SUCCESS;
}
