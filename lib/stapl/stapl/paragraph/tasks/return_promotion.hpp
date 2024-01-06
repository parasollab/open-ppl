/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_RETURN_PROMOTION_HPP
#define STAPL_PARAGRAPH_RETURN_PROMOTION_HPP

#include <stapl/paragraph/wf_invoke.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/iterator/ref_iterator.h>

namespace stapl {

template<typename C>
class local_accessor;

template<typename C>
class local_accessor_graph;


//////////////////////////////////////////////////////////////////////
/// @brief Called by @p promotion resolver to perform actual promotion of
///   localized workfunction return value.
/// @ingroup pgTasks
///
/// @tparam GReturn Return type of the workfunction when invoked with a
///   non-localized view set.  The target type of the promotion operation.
/// @tparam LReturn Return type of the workfunction when invoked with a
///   localized view set.  The input result to the promotion operation.
/// @tparam LView Localized view type.
/// @tparam GView Type of view before localization transformation.
/// @tparam IsPView Boolean type parameter denoting whether @p GView is
///   a partitioned view.
/// @return The promoted return value.
///
/// @p GReturn explicitly specified by caller, others are deduced.
///
/// This least specialized, default signature asserts out at compile time
/// because we don't know how to promote @p LReturn.
//////////////////////////////////////////////////////////////////////
template<typename GReturn, typename LReturn,
         typename LView, typename GView, typename IsPView>
inline
GReturn promote_ref(LReturn const&, LView const&, GView const&, IsPView)
{
  // This means we don't know how to promote LReturn to GReturn
  static_assert(sizeof(GReturn) == 0, "Undefined promotion");

  return GReturn();
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for promoting a local_accessor to GReturn.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename GReturn, typename C, typename T,
         typename LView, typename GView>
inline
GReturn
promote_ref(proxy<T, local_accessor<C> > ref, LView const& lv,
            GView const& gv, std::integral_constant<bool, false>)
{
  if (is_null_reference(ref))
    return GReturn(null_reference());

  local_accessor<C> const& accessor = proxy_core_access::accessor(ref);
  return gv.make_reference(accessor.index());
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for promoting a ref_accessor to GReturn.
/// @ingroup pgTasks
///
/// @todo accessor.t is a pointer and lv.container().begin() is a pointer
/// if using parray while they are iterators if using pvector using "&*"
/// is not good. We need to change the proxy framework to let it use iterator.
//////////////////////////////////////////////////////////////////////
template<typename GReturn, typename T, typename LView, typename GView>
inline
GReturn
promote_ref(proxy<T, ref_accessor<T> > ref, LView const& lv,
            GView const& gv, std::integral_constant<bool, false>)
{
  if (is_null_reference(ref))
    return GReturn(null_reference());

  ref_accessor<T> const& accessor = proxy_core_access::accessor(ref);
  return *(gv.begin() + (accessor.t - &*lv.container().begin()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for promoting a ref_accessor to GReturn when:
///  (1) GView is a partitioned view and
///  (2) ref is a reference to an element of an element of that view.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename GReturn, typename T, typename LView, typename GView>
inline
GReturn
promote_ref(proxy<T, ref_accessor<T> > ref, LView const& lv,
            GView const& gv, std::integral_constant<bool, true>)
{
  if (is_null_reference(ref))
    return GReturn(null_reference());

  ref_accessor<T> const& accessor = proxy_core_access::accessor(ref);

  typename LView::index_type lvfirst = lv.domain().first();

  size_t offset = gv.container().get_view()->domain().distance(
    gv.container().get_view()->domain().first(),lv[lvfirst].domain().first()
  );

  return *(gv.container().get_view()->begin() + offset +
         (accessor.t - lv.container().get_container()->begin()) );
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for promoting a local_accessor to GReturn when:
///  (1) GView is a partitioned view and
///  (2) ref is a reference to an element of an element of that view.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename GReturn, typename T, typename C,
         typename LView, typename GView>
inline
GReturn
promote_ref(proxy<T, local_accessor<C> > ref, LView const& lv,
            GView const& gv, std::integral_constant<bool, true>)
{
  if (is_null_reference(ref))
    return GReturn(null_reference());

  local_accessor<C> const& accessor = proxy_core_access::accessor(ref);
  return gv.container().get_view()->make_reference(accessor.index());
}

//////////////////////////////////////////////////////////////////////
/// @brief Signature for promoting a local_accessor_graph to GReturn.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename GReturn, typename C, typename T,
         typename LView, typename GView>
inline
GReturn
promote_ref(proxy<T, local_accessor_graph<C> > ref, LView const& lv,
            GView const& gv, std::integral_constant<bool, false>)
{
  if (is_null_reference(ref))
    return GReturn(null_reference());

  auto const& accessor = proxy_core_access::accessor(ref);
  return gv.make_reference(accessor.index());
}


namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor that attempts to deduce what input parameter to the
///   workfunction the return value is based on and exactly of what it is a
///   reference to (e.g., reference to some portion of a view element, etc). It
///   then redirects to an appropriate invocation of @ref promote_ref to perform
///   the actual promotion and returns the promoted result.
/// @ingroup pgTasks
///
/// Primary template used when framework doesn't know how to perform promotion.
/// Unconditionally asserts at compile time.
///
/// @todo Transparently avoid view localization may be preferable to static
///   assertion in unmatched cases.
///
/// @tparam WF The workfunction type passed to task creation.
/// @tparam GlobalVS The pre-localization view set type.
/// @tparam Result Computed type representing return type of workfunction is
///   invoked with pre-localization viewset.
///
/// @todo We're making some assumptions here right now because we haven't
/// the typing system enough to deduce more exact information about the
/// the nature of the return type / value. These include (not limited to):
///   (1) The return value comes from the first input parameter.
///   (2) View::reference type != View::reference::reference (probably ok).
///   Until we implement (1), at least find an assertion condition in the views
///   to at least recognize we're doing this incorrectly.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename GlobalVS,
         typename Result = typename wf_invoke<WF, GlobalVS>::result_type>
struct promotion_resolver
{
  // This mean we can't resolve what the return is in relation
  // to the wf input view/proxy parameters.
  static_assert(sizeof(WF) == 0, "promotion resolver undefined");
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor specialization used when WF::result_type == View0::reference
///   and View::reference is a proxy.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename WF, typename GlobalVS>
struct promotion_resolver<
  WF, GlobalVS, typename tuple_element<0, GlobalVS>::type::reference
>
{
  typedef typename tuple_element<0, GlobalVS>::type::reference Result;

  //////////////////////////////////////////////////////////////////////
  /// @param ref Result of base workfunction application.
  /// @param lvs Localized view set.
  /// @param gvs Initial viewset passed to task.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Acc, typename LocalVS>
  Result operator()(proxy<T, Acc> ref, LocalVS const& lvs, GlobalVS& gvs) const
  {
    typedef typename tuple_element<0,GlobalVS>::type      view0_t;
    typedef typename is_segmented_view<view0_t>::type   is_part_view;

    return promote_ref<Result>(ref, get<0>(lvs), get<0>(gvs), is_part_view());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor specialization used when
///   WF::result_type == View0::reference::reference
///   (e.g., reference from element (view) of a partitioned view)
///   and View0::reference::reference is a proxy.
/// @ingroup pgTasks
//////////////////////////////////////////////////////////////////////
template<typename WF, typename GlobalVS>
struct promotion_resolver<
  WF, GlobalVS, typename tuple_element<0, GlobalVS>::type::reference::reference
>
{
  typedef typename tuple_element<
    0, GlobalVS
  >::type::reference::reference Result;

  //////////////////////////////////////////////////////////////////////
  /// @param ref Result of base workfunction application.
  /// @param lvs Localized view set.
  /// @param gvs Initial viewset passed to task.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Acc, typename LocalVS>
  Result operator()(proxy<T, Acc> ref, LocalVS const& lvs, GlobalVS& gvs) const
  {
    typedef typename tuple_element<0,GlobalVS>::type      view0_t;
    typedef typename is_segmented_view<view0_t>::type   is_part_view;

    return promote_ref<Result>(ref, get<0>(lvs), get<0>(gvs), is_part_view());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction wrapper that facilitates promotion of a localized
///   return value to the global scope (i.e., that of the PARAGRAPH instead
///   of the task.
/// @ingroup pgTasks
/// @tparam WF The workfunction type passed to task creation.
/// @tparam GlobalVS The pre-localization view set type.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename GlobalVS>
struct promote_wf
  : private WF
{
  /// @brief task graph callback used to release any @ref edge_container
  /// backed @ref lazy_edge_reference objects stored in viewset member.
  tg_callback m_cb;

  /// @brief A copy of the viewset passed to task creation before any
  /// localization transformation.
  GlobalVS    m_gvs;

  promote_wf(tg_callback cb, WF const& wf, GlobalVS const& gvs)
    : WF(wf), m_cb(std::move(cb)), m_gvs(gvs)
  { }

  ~promote_wf()
  {
    vs_map(lazy_ref_release_func(m_cb), m_gvs);
  }

  typedef typename wf_invoke<WF, GlobalVS>::result_type result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Forward parameters to WF invocation, take return value and pass
  ///   on to promotion_resolver::operator()/promote() call chain with tuple
  ///   of input params as an additional parameter. Generalize the base
  ///   workfunction's localized result.
  /// @param views... Variadic list of views that are passed to the base
  ///   workfunction.
  /// @return The promoted return value of the workfunction.
  //////////////////////////////////////////////////////////////////////
  template<typename... Views>
  result_type operator()(Views const&... views)
  {
    typedef tuple<Views const&...> tuple_t;

    return promotion_resolver<WF, GlobalVS>()(
      static_cast<WF&>(*this)(views...), tuple_t(views...), m_gvs
    );
  }
}; // class promote_wf


//////////////////////////////////////////////////////////////////////
/// @brief Functor invoked by the task creation process.  Compares the return
///   types of the workfunction when given the global and local viewsets.
///   If they differ the workfunction that task uses is changed to facilitate
///   the promotion of the return value based on the localized viewset to a form
///   valid in the global / enclosing scope.
/// @ingroup pgTasks
///
/// @tparam WF The base workfunction type.
/// @tparam GlobalVS The pre-localization view set type.
/// @tparam LocalVS The post-localization view set type.
/// @tparam b Computed boolean parameter that is true if the application of the
///   workfunction of viewsets of type @p LocalVS and @ref GlobalVS return the
///   value type.  Used to dispatch to the appropriate specialization.
///
/// Primary template matches the case when the return types of localized and
/// non localized views are the same.  No promotion wrapper is needed.  The
/// functor invocation is an identify operation.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename GlobalVS, typename LocalVS,
         bool b = std::is_same<
           typename wf_invoke<WF, GlobalVS>::result_type,
           typename wf_invoke<WF, LocalVS>::result_type
         >::value>
struct compute_promotion_wf
{
  typedef WF result_type;

  template<typename ...ViewParams>
  static result_type
  apply(tg_callback const&, WF const& wf, ViewParams const&...)
  { return wf; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor class specialization matching cases where a promotion wrapper
///   is needed to generalize the localized return value from the workfunction.
/// @ingroup pgTasks
///
/// Wrap the workfunction with @p promote_wf, initialized the wrapper with
/// both the base workfunction and the non localized view set.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename GlobalVS, typename LocalVS>
struct compute_promotion_wf<WF, GlobalVS, LocalVS, false>
{
  typedef promote_wf<WF, GlobalVS> result_type;

  template<typename ...ViewParams>
  static result_type
  apply(tg_callback const& cb, WF const& wf, ViewParams const&... views)
  { return result_type(cb, wf, std::make_tuple(views...)); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor that recovers the original workfunction passed to task
///   creation prior to any wrappers to support the promotion of localized
///   return values.
/// @ingroup pgTasks
///
/// @tparam WF The type of the workfunction that is held in a @ref Task that
/// may have been transformed to support promotion.
///
/// The functor supports task migration, allowing the original workfunction to
/// be recovered to be paired with the original, non localized viewset.
/// Localization will be attempted again on the new execution location, once
/// migration has completed.
///
/// Primary template matches case where @ref promote_wf wrapper was not applied.
/// Return the input parameter.
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct compute_demotion_wf
{
  typedef WF result_type;

  static result_type apply(WF const& wf)
  { return wf; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor class specialization matching task workfunctions that were
///   wrapped with @p promote_wf to support localization.
/// @ingroup pgTasks
///
/// Extracts and return the original workfunction.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename GlobalVS>
struct compute_demotion_wf<promote_wf<WF, GlobalVS> >
{
  typedef WF result_type;

  static result_type apply(promote_wf<WF, GlobalVS> const& pwf)
  { return pwf.m_wf; }
};

} // namespace paragraph_impl

} // namespace stapl

#endif // STAPL_PARAGRAPH_RETURN_PROMOTION_HPP
