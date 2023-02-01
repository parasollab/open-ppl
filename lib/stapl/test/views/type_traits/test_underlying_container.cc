/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/views/type_traits/underlying_container.hpp>

using namespace stapl;

void segmented_1_level()
{
  using traversal_t     = index_sequence<2, 1, 0>;
  using sliced_dims     = index_sequence<0, 1>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<3, std::size_t, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  using slices_view = decltype(make_slices_view<0, 1>(
    std::declval<view_type>())
  );

  static_assert(
    std::is_same<
      typename underlying_container<slices_view>::type,
      multiarray_type
    >::value, "Testing slices 1 level"
  );
}


void segmented_2_levels()
{
  using traversal_t     = index_sequence<4, 3, 2, 1, 0>;
  using sliced_dims     = index_sequence<0>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<5, bool, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  using slices_view = decltype(make_slices_view<0, 1>(
    std::declval<view_type>())
  );
  using slices_slices_view = decltype(
    make_slices_view<0>(std::declval<slices_view>())
  );

  static_assert(
    std::is_same<
      underlying_container<slices_slices_view>::type,
      multiarray_type>::value,
    "Testing slices 2 levels"
  );
}

exit_code stapl_main(int argc, char** argv)
{
  using m_t = multiarray<3, bool>;
  using mv_t = multiarray_view<m_t>;
  using mvv_t = multiarray_view<mv_t>;
  using mvvv_t = multiarray_view<mvv_t>;

  static_assert(
    std::is_same<underlying_container<mv_t>::type, m_t>::value,
    "Testing 1 level"
  );

  static_assert(
    std::is_same<underlying_container<mvv_t>::type, m_t>::value,
    "Testing 2 levels"
  );

  static_assert(
    std::is_same<underlying_container<mvvv_t>::type, m_t>::value,
    "Testing 3 levels"
  );

  segmented_1_level();
  segmented_2_levels();

  return EXIT_SUCCESS;
}
