/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_COARSE_IDENTITY_OP_HPP
#define STAPL_SKELETONS_UTILITY_COARSE_IDENTITY_OP_HPP

#include <stapl/skeletons/utility/lightweight_vector.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief A workaround for the cases that a coarsened view is passed
/// to @c stapl::identity<T>. This struct brings a coarsened view to
/// @c PARAGRAPH environment.
///
/// @tparam T type of the fine-grain elements
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename T>
struct coarse_identity_op
{
  using result_type = stapl::lightweight_vector<T>;

  template <typename V>
  result_type operator()(V const& v)
  {
    result_type r;
    r.reserve(v.size());

    std::copy(v.begin(), v.end(), std::back_inserter(r));
    return r;
  }

  template <typename A>
  result_type operator()(stapl::proxy<result_type, A> const& p)
  {
    return p;
  }
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_COARSE_IDENTITY_OP_HPP
