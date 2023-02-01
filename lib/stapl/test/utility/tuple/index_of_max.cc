/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#define REPORT_WITH_COLOR

#include <stapl/runtime.hpp>
#include <stapl/utility/tuple/index_of_max.hpp>
#include <boost/mpl/int.hpp>

#include "../../test_report.hpp"

using namespace stapl;


exit_code stapl_main(int argc, char** argv)
{
  using tuple0 = std::tuple<
    boost::mpl::int_<0>,
    boost::mpl::int_<3>,
    boost::mpl::int_<2>
  >;

  static_assert(tuple_ops::index_of_max<tuple0>::value == 1,
      "Test with 3-tuple");

  using tuple1 = std::tuple<
    boost::mpl::int_<2>,
    boost::mpl::int_<0>,
    boost::mpl::int_<1>
  >;

  static_assert(tuple_ops::index_of_max<tuple1>::value == 0,
      "Test with max as first");

  using tuple2 = std::tuple<
    boost::mpl::int_<2>
  >;

  static_assert(tuple_ops::index_of_max<tuple2>::value == 0,
      "Test with 1-tuple");

  return EXIT_SUCCESS;
}
