/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TEST_VIEWS_SLICE_TEST_COMPOSED_SLICES_FULL_PARAGRAPH_HPP_
#define TEST_VIEWS_SLICE_TEST_COMPOSED_SLICES_FULL_PARAGRAPH_HPP_
 
#include <stapl/array.hpp>

using namespace stapl;

struct set_is_local
{
  using result_type = void;

  template<typename RefA, typename RefB>
  void operator()(RefA x, RefB y)
  {
    const bool is_local =
      accessor_core_access::is_local(proxy_core_access::accessor(x));

    x = is_local;
  }
};

struct lev3
{
  using result_type = void;

  template<typename SlicedViewA, typename SlicedViewB>
  void operator()(SlicedViewA vwa, SlicedViewB vwb)
  {
    map_func(set_is_local(), vwa, vwb);
  }
};

struct lev2
{
  using result_type = void;

  template<typename SlicedViewA, typename SlicedViewB>
  void operator()(SlicedViewA vwa, SlicedViewB vwb)
  {
    map_func(lev3(), vwa, vwb);
  }
};

struct lev2_intermediate_view
{
  using result_type = void;

  template<typename SlicedViewA, typename SlicedViewB>
  void operator()(SlicedViewA vwa, SlicedViewB vwb)
  {
    map_func(lev3(), make_array_view(vwa), make_array_view(vwb));
  }
};

struct lev1
{
  using result_type = void;

  template<typename SlicedViewA, typename SlicedViewB>
  void operator()(SlicedViewA vwa, SlicedViewB vwb)
  {
    map_func(lev2(), vwa, vwb);
  }
};

struct lev1_intermediate_view
{
  using result_type = void;

  template<typename SlicedViewA, typename SlicedViewB>
  void operator()(SlicedViewA vwa, SlicedViewB vwb)
  {
    map_func(lev2_intermediate_view(), vwa, vwb);
  }
};

#endif // TEST_VIEWS_SLICE_TEST_COMPOSED_SLICES_FULL_PARAGRAPH_HPP_
