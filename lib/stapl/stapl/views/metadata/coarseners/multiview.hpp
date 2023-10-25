/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_COARSEN_MULTIVIEWS_HPP
#define STAPL_VIEWS_COARSEN_MULTIVIEWS_HPP

#include <stapl/runtime.hpp>
#include <stapl/paragraph/paragraph_fwd.h>

#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/metadata/coarsen_utility.hpp>
#include <stapl/views/metadata/alignment/guided.hpp>

#include <stapl/views/metadata/utility/are_aligned.hpp>
#include <stapl/views/metadata/utility/have_equal_sizes.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief helper class template for @ref multiview_coarsener, dispatching
///   coarsener implementation based on the number of finite views found
///   in the set of views passed in tuple parameter @p Views.
///
/// Primary template is default implementation, used when multiple finite
/// domain views are detected, requiring alignment checks / enforcement.
//////////////////////////////////////////////////////////////////////
template<typename Views, bool Align,
         int FiniteViews = count_finite_domains<Views>::value>
struct multiview_coarsener_impl
{
  static auto apply(Views const& views)
    -> decltype(coarsen_views_native<Views>::apply(
         views, metadata_from_container<Views>::apply(views)))
  {
    constexpr int guide = first_finite_domain_index<Views>::value;

    stapl_assert(metadata::have_equal_sizes<guide>(views),
      "Attempting to coarsen views that have unequal sizes");

    // get metadata from the views
    auto md_conts = metadata_from_container<Views>::apply(views);

    // check to see if the metadata containers have the same number of entries
    const bool b_same_num_entries = metadata::have_equal_sizes(md_conts);

    // if the metadata containers have the same number of entries and they
    // are already aligned, then just transform the fine-grained views
    // using the standard metadata
    if (b_same_num_entries && metadata::entries_are_aligned<guide>(md_conts))
      return coarsen_views_native<Views>::apply(views, md_conts);

    // views are not aligned, so invoke the alignment algorithm
    return metadata::guided_alignment<Views>::template apply<guide>(
      views, md_conts, metadata::have_static_metadata(md_conts));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when one finite view is found.  No alignment
///   (or corresponding check needed).  Call native coarsener.
//////////////////////////////////////////////////////////////////////
template<typename Views>
struct multiview_coarsener_impl<Views, false, 1>
{
private:
  using base_coarsener = coarsen_views_native<Views>;

public:
  static auto apply(Views const& views)
  STAPL_AUTO_RETURN(
    base_coarsener::apply(
      views, base_coarsener::metadata_extractor_type::apply(views)))
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when one finite view is found.  No alignment
///   (or corresponding check needed).  Call native coarsener.
//////////////////////////////////////////////////////////////////////
template<typename Views>
struct multiview_coarsener_impl<Views, true, 1>
{
private:
  using base_coarsener = coarsen_views_native<Views>;

public:
  static auto apply(Views const& views)
  STAPL_AUTO_RETURN(
    base_coarsener::apply(
      views, base_coarsener::metadata_extractor_type::apply(views)))
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when no finite views are found.
///   Statically assert as this case does not make sense for this
///   coarsening approach.
//////////////////////////////////////////////////////////////////////
template<typename Views>
struct multiview_coarsener_impl<Views, true, 0>
{
  static_assert(sizeof(Views) == 0,
                "Must provide at least one view with a finite domain.");
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when no finite views are found and alignment
/// is not requested. Just call the base coarsener.
//////////////////////////////////////////////////////////////////////
template<typename Views>
struct multiview_coarsener_impl<Views,false, 0>
{
private:
  using base_coarsener = coarsen_views_native<Views>;

public:
  static auto apply(Views const& views)
  STAPL_AUTO_RETURN(
    base_coarsener::apply(
      views, base_coarsener::metadata_extractor_type::apply(views)))
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the case when alignment is explicitly disabled.
//////////////////////////////////////////////////////////////////////
template<typename Views, int FiniteViews>
struct multiview_coarsener_impl<Views, false, FiniteViews>
{
private:
  typedef metadata_from_container<Views>                  md_extractor_t;
  typedef typename md_extractor_t::type                   md_t;

public:
  static
  auto apply(Views const& views)
    -> decltype(coarsen_views_native<Views>::apply(
         views, md_extractor_t::apply(views)))
  {
    using guide = typename first_finite_domain_index<Views>::type;

    stapl_assert(metadata::have_equal_sizes<guide::value>(views),
      "Attempting to coarsen views that have unequal sizes");

    // create metadata containers for each view using extractors
    // and projection algorithms
    md_t md_conts = md_extractor_t::apply(views);

    // create coarsened views from the metadata containers
    return coarsen_views_native<Views>::apply(views, md_conts);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the case when a @ref partitioned_mix_view
///   is present in the input view set.
///
/// Do not process the partitioned_mix_views as they already represent
/// coarsened views.
///
/// @todo We currently assume that the partitioned_mix_view comes first
///   in the view set. This could be generalized by using
///   @ref stapl::find_first_index.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename Part, typename CC,
         bool Align, int FiniteViews, typename... Views>
struct multiview_coarsener_impl<
  tuple<partitioned_mix_view<View0, Part, CC>, Views...>, Align, FiniteViews>
{
private:
  using ViewSet = tuple<partitioned_mix_view<View0, Part, CC>, Views...>;

public:
  static auto apply(ViewSet const& views)
    -> decltype(coarsen_views_native<ViewSet>::apply(
         views, metadata_from_container<ViewSet>::apply(views)))
  {
    // get metadata from the views
    auto md_conts = metadata_from_container<ViewSet>::apply(views);

    // check to see if the metadata containers have the same number of entries
    // and are properly aligned
    stapl_assert(metadata::have_equal_sizes(md_conts)
      && metadata::entries_are_aligned<0>(md_conts),
      "Attempting to use coarsened views with incompatible metadata containers."
      " When passing multiple pre-coarsened views to a paragraph, make sure"
      " they have been properly aligned.");

    return coarsen_views_native<ViewSet>::apply(views, md_conts);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the single @ref partitioned_mix_view.
///
/// Do not process the partitioned_mix_view as it already represents
/// a coarsened view.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename Part, typename CC, bool Align>
struct multiview_coarsener_impl<
  tuple<partitioned_mix_view<View0, Part, CC>>, Align, 1>
{
private:
  using Views          = tuple<partitioned_mix_view<View0, Part, CC>>;
  using base_coarsener = coarsen_views_native<Views>;

public:
  static auto apply(Views const& views)
  STAPL_AUTO_RETURN(
    base_coarsener::apply(
      views, base_coarsener::metadata_extractor_type::apply(views)))
};

} // namespace metadata


//////////////////////////////////////////////////////////////////////
/// @brief Functor to coarsen a set of given views.
///
/// If all views are aligned a native coarsener is used.  Otherwise, an
/// aligning coarsener using one of the finite views as the "guide" is
/// employed. Instances of partitioned_mix_view are not coarsened.
///
/// @todo Currently, coarsening acts as an identity on partitioned_mix_view,
///   which is not semantically correct - we should rather produce a
///   partitioned_mix_view<partitioned_mix_view<View, ...>, ...>. See
///   GFORGE #1500.
//////////////////////////////////////////////////////////////////////
template<bool Align>
struct multiview_coarsener
{
  template<typename Views>
  auto operator()(Views const& views) const
  STAPL_AUTO_RETURN(
    STAPL_PROXY_CONCAT(
      metadata::multiview_coarsener_impl<Views, Align>::apply(views)
    )
  )
};

} // namespace stapl

// TODO(mani) remove these from here and add includes to the places that use
// these files.
#include <stapl/views/metadata/container/growable.hpp>
#include <stapl/views/metadata/container/flat.hpp>
#include <stapl/views/metadata/container/projected.hpp>
#include <stapl/views/metadata/container/infinite.hpp>
#include <stapl/views/metadata/container/generator.hpp>
#include <stapl/views/metadata/container/metadata_view.hpp>

#endif // STAPL_VIEWS_COARSEN_MULTIVIEWS_HPP
