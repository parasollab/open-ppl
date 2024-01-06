/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/array.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/strided_view.hpp>
#include <stapl/views/array_view.hpp>

#include <boost/mpl/int.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/equal.hpp>

#include "../../test_report.hpp"

namespace stapl {

struct some_tag_class
{ };

template<>
struct dimension_traits<some_tag_class>
{
  typedef boost::mpl::int_<42> type;
};

} // end namespace stapl

using namespace stapl;

template <typename Element, typename Element2>
struct equal_to_impl
  : public std::integral_constant<
             bool,
             Element::value == Element2::value>
{ };

stapl::exit_code stapl_main(int argc, char* argv[])
{
/*
  // create vector of dimensions for multiarray types
  typedef boost::mpl::vector<
            dimension_traits<multiarray<1, int> >::type,
            dimension_traits<multiarray<2, int> >::type,
            dimension_traits<multiarray<3, int> >::type
          > multiarray_dimensions;

  // create vector of dimensions for multiarray view types
  typedef boost::mpl::vector<
            dimension_traits<multiarray_view<multiarray<1, int> > >::type,
            dimension_traits<multiarray_view<multiarray<2, int> > >::type,
            dimension_traits<multiarray_view<multiarray<3, int> > >::type
          > multiarray_view_dimensions;

  // create vector of dimensions for strided view types
  typedef boost::mpl::vector<
            dimension_traits<
              strided_view<
                multiarray_view<multiarray<1, int> >
              >::type
            >::type,
            dimension_traits<
              strided_view<
                multiarray_view<multiarray<2, int> >
              >::type
            >::type,
            dimension_traits<
              strided_view<
                multiarray_view<multiarray<3, int> >
              >::type
            >::type
          > strided_view_dimensions;
*/

  // create vector of dimensions for tuple types
  typedef boost::mpl::vector<
            dimension_traits<tuple<size_t> >::type,
            dimension_traits<tuple<size_t, char> >::type,
            dimension_traits<tuple<size_t, char, float> >::type
          > tuple_dimensions;

  // create vector of dimensions for various types
  typedef boost::mpl::vector<
            dimension_traits<array<int> >::type,
            dimension_traits<array_view<array<int > > >::type,
            dimension_traits<some_tag_class>::type
          > various_dimensions;

/*
  // test multiarray dimension types
  typedef boost::mpl::equal<
            boost::mpl::vector<
              boost::mpl::int_<1>,
              boost::mpl::int_<2>,
              boost::mpl::int_<3>
            >,
            multiarray_dimensions
          >::type multiarray_test;

  // test multiarray view dimension types
  typedef boost::mpl::equal<
            boost::mpl::vector<
              boost::mpl::int_<1>,
              boost::mpl::int_<2>,
              boost::mpl::int_<3>
            >,
            multiarray_view_dimensions
          >::type multiarray_view_test;

  // test strided view dimension types
  typedef boost::mpl::equal<
            boost::mpl::vector<
              boost::mpl::int_<1>,
              boost::mpl::int_<2>,
              boost::mpl::int_<3>
            >,
            strided_view_dimensions
          >::type strided_view_test;
*/

  // test tuple dimension types
  typedef boost::mpl::equal<
            boost::mpl::vector<
              boost::mpl::int_<1>,
              boost::mpl::int_<2>,
              boost::mpl::int_<3>
            >,
            tuple_dimensions,
            equal_to_impl<boost::mpl::_, boost::mpl::_>
          >::type tuple_test;

  // test various dimension types
  typedef boost::mpl::equal<
            boost::mpl::vector<
              boost::mpl::int_<1>,
              boost::mpl::int_<1>,
              boost::mpl::int_<42>
            >,
            various_dimensions,
            equal_to_impl<boost::mpl::_, boost::mpl::_>
          >::type various_test;

  //BOOST_MPL_ASSERT(( multiarray_test ));
  //BOOST_MPL_ASSERT(( multiarray_view_test ));
  //BOOST_MPL_ASSERT(( strided_view_test ));
  BOOST_MPL_ASSERT(( tuple_test ));
  BOOST_MPL_ASSERT(( various_test ));

  STAPL_TEST_REPORT(true, "Testing dimension_traits");

  return EXIT_SUCCESS;
}
