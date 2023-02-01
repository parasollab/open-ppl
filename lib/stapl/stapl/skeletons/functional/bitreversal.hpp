/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_BITREVERSAL_HPP
#define STAPL_SKELETONS_FUNCTIONAL_BITREVERSAL_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/bitreversal_pd.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

////////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an bitreversal skeleton
/// by exposing only the necessary information in its representation.
///
/// This abstraction not only makes the reconstruction of a
/// bitreversal skeleton easier, but also provides access to the
/// underlying operation.
/// Furthermore, it reduces the symbol size for a bitreversal skeleton,
/// hence, reducing the total compilation time.
///
/// @tparam Span the iteration space for elements of the bitreversal
///          skeleton
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Span>
struct bitreversal
  : public decltype(
             skeletons::elem<Span>(skeletons::bitreversal_pd()))
{
  using skeleton_tag_type = tags::bitreversal;
  using base_type = decltype(
                      skeletons::elem<Span>(skeletons::bitreversal_pd()));

  bitreversal(void)
    : base_type(
        skeletons::elem<Span>(skeletons::bitreversal_pd())
      )
  { }

  auto get_op(void) const ->
    decltype(
      std::declval<base_type>().nested_skeleton().get_op()
    )
  {
    return base_type::nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}


namespace result_of {

template <typename Span>
using bitreversal = skeletons_impl::bitreversal<Span>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief This skeleton can be used with skeletons that need their
/// input to be permuted in a bitreversed order such as Cooley Tukey
/// FFT skeletons.
///
/// @tparam Span the inner span on which the bitreversal should be defined
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Span = stapl::use_default>
result_of::bitreversal<Span>
bitreversal(void)
{
  return result_of::bitreversal<Span>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_BITREVERSAL_HPP
