/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <iostream>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/counting_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  int numelem=100;
  if(argc>1)
  {
    numelem=atoi(argv[1]);
  }

  typedef int                          value_t;
  typedef array<value_t>               p_array_type;
  typedef array_view<p_array_type>     view_type;
  typedef view_type::reference         ref_t;
  p_array_type array(numelem);
  view_type view(array);

  copy(counting_view<value_t>(numelem),view);
  ref_t ref=search_n(view,2,3);

  STAPL_TEST_REPORT(is_null_reference(ref),"Testing for search_n over array");


  //** TO DO Uncomment below after list promotion is fixed **//

  //p_list test
  //typedef p_list<int> p_list_t;
  //typedef list_view<p_list_t> viewl_t;
  //p_list_t pl(numelem);
  //viewl_t viewl(pl);
  //copy(counting_view<int>(numelem),viewl);
  //do_once(std::cout<<constant("Testing search over p_list:"));
  //typedef viewl_t::reference refl_t;
  //refl_t refer=search_n(viewl,vec_view);
  //std::cout << "ref: " << refer << " index: " << index_of(refer) << std::endl;
  //do_once(if_then_else(constant(!is_null_reference(refer)),
  //std::cout << constant("test for search_n over list: PASSED\n"),
  //std::cout << constant("test for search_n over list: FAILED\n")
  //   )
  //  );

  return EXIT_SUCCESS;
}
