/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_NUMERIC_FWD_HPP
#define STAPL_ALGORITHMS_NUMERIC_FWD_HPP

#include <stapl/views/view_traits.hpp>

namespace stapl {

template<typename View, typename Oper>
typename view_traits<View>::value_type
accumulate(View const& view,
           typename view_traits<View>::value_type init, Oper oper);

template<typename View>
typename view_traits<View>::value_type
accumulate(View const& view, typename view_traits<View>::value_type init);

template<typename View1, typename View2, typename Oper>
inline
void adjacent_difference(View1 const& view1, View2& view2, Oper oper);

template<typename View1, typename View2>
inline
void adjacent_difference(View1 const& view1, View2& view2);

template<typename View1, typename View2, typename Sum,
         typename Product>
inline
typename view_traits<View1>::value_type
inner_product(View1 const& view1, View2 const& view2,
              typename view_traits<View1>::value_type init, Sum op1,
              Product op2);

template<typename View1, typename View2>
inline
typename view_traits<View1>::value_type
inner_product(View1 const& view1, View2 const& view2,
              typename view_traits<View1>::value_type init);

template <typename View1, typename View2, typename View3,
          typename Sum, typename Product>
typename view_traits<View1>::value_type
weighted_inner_product(View1 const& view1, View2 const& view2, View3 const& wt,
                       typename view_traits<View1>::value_type init,
                       Sum op1, Product op2);

template <typename View1, typename View2, typename View3>
typename view_traits<View1>::value_type
weighted_inner_product(View1 const& view1, View2 const& view2, View3 const& wt,
                       typename view_traits<View1>::value_type init);

template <typename View1, typename View2, typename Sum, typename Product>
typename View1::value_type
weighted_norm(View1 const& view1, View2 const& wt, Sum op1, Product op2);

template <typename View1, typename View2>
typename View1::value_type
weighted_norm(View1 const& view1, View2 const& wt);

template<typename View0, typename View1, typename BinaryFunction>
void
partial_sum(View0 const& view0, View1 const& view1, BinaryFunction binary_op,
            const bool shift = false);


template<typename View0, typename View1>
void
partial_sum(View0 const& view0, View1 const& view1, const bool shift = false);

template <typename View0, typename View1, typename BinaryFunction>
typename View0::value_type
partial_sum_accumulate(View0 const& view0,
                       View1 const& view1,
                       typename view_traits<View0>::value_type init,
                       BinaryFunction binary_op,
                       const bool shift = false);

template <typename View0, typename View1>
typename View0::value_type
partial_sum_accumulate(View0 const& view0,
                       View1 const& view1,
                       typename view_traits<View0>::value_type init,
                       const bool shift = false);

template<typename View0>
void iota(View0 const& view, typename View0::value_type const& value);

} // namespace stapl
#endif
