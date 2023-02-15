/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_SUBVIEW_TYPE_HPP
#define STAPL_PARAGRAPH_SUBVIEW_TYPE_HPP

namespace stapl {

namespace detail {

template<typename View>
struct make_reference;

} // namespace detail


template<typename View>
struct ptr_wrapper;


namespace paragraph_impl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(subview_type)


//////////////////////////////////////////////////////////////////////
/// @brief Function operator responsible for transforming View*/index pair
///   passed to @p add_task into the actual view used by a PARAGRAPH task.
/// @ingroup pgViewOps
/// @tparam View The first element of pair passed as view specifier to
///   @ref add_task (pointer removed from type).
///
/// Primary template for non edge views having get_subview method.
///
/// @todo I don't understand why the view framework has two interfaces.
///   (get_subview / make_reference). They should either be merged, or at the
///   very least, this dispatch function should be moved into the view codebase.
///////////////////////////////////////////////////////////////////////
template<typename View, bool = has_subview_type<View>::value>
struct subview_type
{
  using type = typename View::subview_type;

  static type apply(View const& vw, typename View::cid_type cid, bool)
  { return vw.get_subview(cid); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for for non edge views not having get_subview method.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct subview_type<View, false>
{
  using type = typename View::reference;

  static type apply(View const& vw, typename View::index_type cid, bool)
  { return detail::make_reference<View>()(vw, cid); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for views passed to @p add_task using
///   @ref ptr_wrapper. This happens with @ref edge_view or
///   @ref aggregated_edge_view, so it captures all PARAGRAPH data
///   flow parameters to a task.
/// @ingroup pgViewOps
/// @sa consume
///
/// Uses the bracket operator (the preferred syntax that all views should
/// eventually use.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct subview_type<ptr_wrapper<View>, false>
{
  using type = typename View::reference;

  template<typename ViewParam>
  static type apply(ViewParam&& vw, typename View::index_type idx,
                    bool b_transporting)
  { return b_transporting ? vw.transporter_reference(idx) : vw[idx]; }
};

} // namespace paragraph_impl

} // namespace stapl

#endif // STAPL_PARAGRAPH_SUBVIEW_TYPE_HPP
