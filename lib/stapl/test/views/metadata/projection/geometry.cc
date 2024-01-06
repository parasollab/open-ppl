/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/projection/multidimensional.hpp>
#include <stapl/domains/indexed.hpp>

#define REPORT_WITH_COLOR
#include "../../../test_report.hpp"

using namespace stapl;

void two_dimensions()
{
  typedef indexed_domain<std::size_t, 2> domain_type;
  typedef geometry_impl::is_mergeable<domain_type>   mergeable;

  //    0 1 2 3 4 5 6 7 8 9
  //    -------------------
  // 0  |      |   |      |
  // 1  |      |   |      |
  // 2  |  1   | 2 |   3  |
  // 3  |      |   |      |
  // 4  |      |   |      |
  //    |------|---|------|
  // 5  |      |   |      |
  // 6  |  4   | 5 |   6  |
  //    |------|---|------|
  // 7  |      |   |      |
  // 8  |      |   |      |
  // 9  |  7   | 8 |   9  |
  // 10 |      |   |      |
  // 11 |      |   |      |
  //    -------------------
  domain_type r1(make_tuple(0,0), make_tuple(4,3));
  domain_type r2(make_tuple(0,4), make_tuple(4,5));
  domain_type r3(make_tuple(0,6), make_tuple(4,9));
  domain_type r4(make_tuple(5,0), make_tuple(6,3));
  domain_type r5(make_tuple(5,4), make_tuple(6,5));
  domain_type r6(make_tuple(5,6), make_tuple(6,9));
  domain_type r7(make_tuple(7,0), make_tuple(11,3));
  domain_type r8(make_tuple(7,4), make_tuple(11,5));
  domain_type r9(make_tuple(7,6), make_tuple(11,9));

  bool passed = mergeable::apply(r1, r2);
  STAPL_TEST_REPORT(passed, "r1 and r2");

  passed = mergeable::apply(r2, r3);
  STAPL_TEST_REPORT(passed, "r2 and r3");

  passed = !mergeable::apply(r1, r3);
  STAPL_TEST_REPORT(passed, "r1 and r3");

  passed = mergeable::apply(r1, r4);
  STAPL_TEST_REPORT(passed, "r1 and r4");

  passed = mergeable::apply(r5, r2);
  STAPL_TEST_REPORT(passed, "r5 and r2");

  passed = mergeable::apply(r5, r4);
  STAPL_TEST_REPORT(passed, "r5 and r4");

  passed = mergeable::apply(r5, r8);
  STAPL_TEST_REPORT(passed, "r5 and r8");

  passed = mergeable::apply(r5, r6);
  STAPL_TEST_REPORT(passed, "r5 and r6");

  passed = !mergeable::apply(r5, r9);
  STAPL_TEST_REPORT(passed, "r5 and r9");

  //    0 1 2 3 4
  //    ----------
  // 0  |    |   |
  // 1  |    | 2 |
  // 2  | 1  |   |
  //    |    |---|
  // 3  |    | 3 |
  // 4  |    |   |
  //    ----------
  domain_type s1(make_tuple(0,0), make_tuple(4,2));
  domain_type s2(make_tuple(0,3), make_tuple(2,4));
  domain_type s3(make_tuple(3,3), make_tuple(4,4));

  passed = !mergeable::apply(s1, s2);
  STAPL_TEST_REPORT(passed, "s1 and s2");

  passed = !mergeable::apply(s1, s3);
  STAPL_TEST_REPORT(passed, "s1 and s3");

  passed = mergeable::apply(s2, s3);
  STAPL_TEST_REPORT(passed, "s2 and s3");

  passed = mergeable::apply(s3, s2);
  STAPL_TEST_REPORT(passed, "s3 and s2");

  passed = !mergeable::apply(s3, s1);
  STAPL_TEST_REPORT(passed, "s3 and s1");

  passed = !mergeable::apply(s2, s1);
  STAPL_TEST_REPORT(passed, "s2 and s1");
}


void three_dimensions()
{
  typedef indexed_domain<std::size_t, 3> domain_type;
  typedef geometry_impl::is_mergeable<domain_type>   mergeable;

  //        -----------
  //     2 / 4  / 5  /|
  //      /--- /--- / |
  //   1 /    /    /|5|
  //  0 /  1 /  2 / | |
  //    ----------  |/|
  // 0  |    |   |2 / |
  // 1  |    | 2 | /|6|
  // 2  | 1  |   |/ |/
  //    |    |---| 3/
  // 3  |    | 3 | /
  // 4  |    |   |/
  //    ----------
  //    0 1 2 3 4

  domain_type s1(make_tuple(0,0,0), make_tuple(4,2,1));
  domain_type s2(make_tuple(0,3,0), make_tuple(2,4,1));
  domain_type s3(make_tuple(3,3,0), make_tuple(4,4,1));
  domain_type s4(make_tuple(0,0,2), make_tuple(4,2,2));
  domain_type s5(make_tuple(0,3,2), make_tuple(2,4,2));
  domain_type s6(make_tuple(3,3,2), make_tuple(4,4,2));

  bool passed = !mergeable::apply(s1, s2);
  STAPL_TEST_REPORT(passed, "s1 and s2");

  passed = !mergeable::apply(s1, s3);
  STAPL_TEST_REPORT(passed, "s1 and s3");

  passed = mergeable::apply(s2, s3);
  STAPL_TEST_REPORT(passed, "s2 and s3");

  passed = mergeable::apply(s3, s2);
  STAPL_TEST_REPORT(passed, "s3 and s2");

  passed = !mergeable::apply(s3, s1);
  STAPL_TEST_REPORT(passed, "s3 and s1");

  passed = !mergeable::apply(s2, s1);
  STAPL_TEST_REPORT(passed, "s2 and s1");

  passed = mergeable::apply(s1, s4);
  STAPL_TEST_REPORT(passed, "s1 and s4");

  passed = mergeable::apply(s4, s1);
  STAPL_TEST_REPORT(passed, "s4 and s1");

  passed = mergeable::apply(s6, s3);
  STAPL_TEST_REPORT(passed, "s6 and s3");

  passed = mergeable::apply(s6, s5);
  STAPL_TEST_REPORT(passed, "s6 and s5");

  passed = !mergeable::apply(s6, s2);
  STAPL_TEST_REPORT(passed, "s6 and s2");
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  two_dimensions();
  three_dimensions();

  return EXIT_SUCCESS;
}
