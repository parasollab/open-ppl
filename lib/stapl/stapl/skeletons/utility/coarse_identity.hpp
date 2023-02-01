/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_COARSE_IDENTITY_HPP
#define STAPL_SKELETONS_UTILITY_COARSE_IDENTITY_HPP

#include <vector>
#include <algorithm>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>
#include <stapl/utility/domain_ops.hpp>
#include <stapl/views/view_traits.hpp>

namespace stapl {

template <typename T, int dims = 1>
struct coarse_identity;


//////////////////////////////////////////////////////////////////////
/// @brief A workaround for the cases that a multidimensional coarsened
/// view is passed to @c stapl::identity<T>. This struct brings a
/// multidimensional coarsened view to @c PARAGRAPH environment.
///
/// @tparam T    type of the fine-grain elements
/// @tparam dims the dimensionality of the input
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename T, int dims>
struct coarse_identity
{
  using result_type = lightweight_multiarray<T, dims>;

  template <typename V>
  result_type operator()(V&& v)
  {
    using index_t = typename
                      view_traits<typename std::decay<V>::type>::index_type;

    auto dom = v.domain();
    result_type result(dom.dimensions());
    auto first = dom.first();

    std::size_t j = 0;

    domain_map(result.domain(), [&](index_t i) {
      result[i] = v[dom.advance(first, j)];
      ++j;
    });

    return result;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief @copybrief coarse_identity.
///
/// This specialization is for the 1D case.
///
/// @tparam T type of the fine-grain elements
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename T>
struct coarse_identity<T, 1>
{
  typedef std::vector<T> result_type;

  template <typename V>
  result_type operator()(V const& v)
  {
    result_type r;
    r.reserve(v.size());

    std::copy(v.begin(), v.end(), std::back_inserter(r));
    return r;
  }
};

} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_COARSE_IDENTITY_hpp
