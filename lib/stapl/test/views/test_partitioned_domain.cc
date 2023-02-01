/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/domains/indexed.hpp>
#include <stapl/domains/partitioned_domain.hpp>

#include "../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef indexed_domain<int> domain_type;

  domain_type dom(0, 99);

  partitioned_domain<domain_type> pdom(dom);

  printf("%d: lower = %d / upper = %d\n",
         stapl::get_location_id(),
         pdom.local_subdomains()[0].first(),
         pdom.local_subdomains()[0].last());

///  size_t res = accumulate(stapl::domain_view(view1),0);
///
///  size_t m = n*(n-1)/2;
///  STAPL_TEST_REPORT(res == m,"Testing domain_view [0..99]");
///
///
///  view_t view4(pa1,vec_dom_t(10,n-11));
///
///  res = accumulate(stapl::domain_view(view4),0);
///
///  m = ((n-10)*(n-11)/2)-(45);
///  STAPL_TEST_REPORT(res == m,"Testing domain_view [10..89]");
///
///
///  copy(counting_view<size_t>(view4.size()),view4);
///
///  size_t idx = my_find(view1,7);
///  STAPL_TEST_REPORT(idx == 17,"Testing domain_view (find)");
///
  return EXIT_SUCCESS;
}
