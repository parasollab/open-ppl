/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/views/type_traits/is_trivially_coarsenable.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>


using namespace stapl;

struct functor
{
  using index_type  = std::size_t;
  using result_type = std::size_t;

  result_type operator()(index_type const&) const
  {
    return 0ul;
  }

};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  using view_t = decltype(functor_view(10, std::declval<functor>()));

  static_assert(
   is_trivially_coarsenable<view_t>::type::value,
    "Functor view trivially coarsenable"
  );

  using multiarray_type = multiarray<3, int>;
  using multiarray_view_type = multiarray_view<multiarray_type>;

  static_assert(
   !is_trivially_coarsenable<multiarray_view_type>::type::value,
    "Multiarray view is not trivially coarsenable"
  );

  return EXIT_SUCCESS;
}
