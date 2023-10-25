/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/static_match.hpp>

using namespace stapl;

using std::true_type;
using std::false_type;
using std::pair;
using std::tuple;

struct foo { };
struct bar { };
struct baz { };

void complete_options()
{
  // 0 0 0  baz
  // 0 0 1  baz
  // 0 1 0  bar
  // 0 1 1  bar
  // 1 0 0  foo
  // 1 0 1  foo
  // 1 1 0  foo
  // 1 1 1  foo
  typedef tuple<
    pair<tuple<true_type,  dont_care,  dont_care>, foo>,
    pair<tuple<false_type, true_type,  dont_care>, bar>,
    pair<tuple<dont_care,  dont_care,  dont_care>, baz>
  > options;

  typedef static_match<
    tuple<true_type,  true_type,  true_type >, options>::type sm_111;
  typedef static_match<
    tuple<true_type,  true_type,  false_type>, options>::type sm_110;
  typedef static_match<
    tuple<true_type,  false_type, true_type >, options>::type sm_101;
  typedef static_match<
    tuple<true_type,  false_type, false_type>, options>::type sm_100;
  typedef static_match<
    tuple<false_type, true_type,  true_type >, options>::type sm_011;
  typedef static_match<
    tuple<false_type, true_type,  false_type>, options>::type sm_010;
  typedef static_match<
    tuple<false_type, false_type, true_type >, options>::type sm_001;
  typedef static_match<
    tuple<false_type, false_type, false_type>, options>::type sm_000;

  static_assert(std::is_same<sm_000, baz>::value, "000");
  static_assert(std::is_same<sm_001, baz>::value, "001");
  static_assert(std::is_same<sm_010, bar>::value, "010");
  static_assert(std::is_same<sm_011, bar>::value, "011");
  static_assert(std::is_same<sm_100, foo>::value, "100");
  static_assert(std::is_same<sm_101, foo>::value, "101");
  static_assert(std::is_same<sm_110, foo>::value, "110");
  static_assert(std::is_same<sm_111, foo>::value, "111");
}


void incomplete_options()
{
  // 0 0  foo
  // 0 1  bar
  // 1 0  baz
  // 1 1  -
  typedef tuple<
    pair<tuple<false_type, false_type>, foo>,
    pair<tuple<false_type, true_type >, bar>,
    pair<tuple<true_type,  false_type>, baz>
  > options;

  typedef static_match<tuple<true_type,  true_type >, options>::type sm_11;
  typedef static_match<tuple<true_type,  false_type>, options>::type sm_10;
  typedef static_match<tuple<false_type, true_type >, options>::type sm_01;
  typedef static_match<tuple<false_type, false_type>, options>::type sm_00;

  static_assert(std::is_same<sm_00, foo>::value, "00");
  static_assert(std::is_same<sm_01, bar>::value, "01");
  static_assert(std::is_same<sm_10, baz>::value, "10");
  static_assert(std::is_same<sm_11, not_matched>::value, "11");
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  complete_options();
  incomplete_options();

  return EXIT_SUCCESS;
}
