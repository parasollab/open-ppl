/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TASK_CREATION_HPP
#define STAPL_PARAGRAPH_TASK_CREATION_HPP

#include <stapl/utility/pack_ops.hpp>
#include <stapl/paragraph/edge_container/views/edge_accessor_fwd.hpp>

namespace stapl {

namespace detail {

struct edge_local_notifier_base;

BOOST_MPL_HAS_XXX_TRAIT_DEF(disable_localization)

template<typename T>
struct is_lazy_edge_reference
{
  static bool apply(T const&)
  { return false; }
};


template<typename T>
struct is_lazy_edge_reference<lazy_edge_reference<T>>
{
  static bool apply(lazy_edge_reference<T> const&)
  { return true; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction reflecting @p std::true_type if a view
/// has chosen to disable localization and @p std::false_type
/// otherwise.
///
/// Primary template matches case when @p disable_localization is not
/// defined by the view.  Localization is attempted.
//////////////////////////////////////////////////////////////////////
template<typename View, bool = has_disable_localization<View>::value>
struct check_localizable
  : std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// Specialization matches case when @p disable_localization is defined
/// by the view.  Inspect the reflected integral constant to determine
/// whether localization should be attempted.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct check_localizable<View, true>
  : std::integral_constant<bool, !View::disable_localization::value>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Recursive type metafunction which returns false if any of the
/// views in the parameter pack have disabled localization.
//////////////////////////////////////////////////////////////////////
template<typename ...Views>
struct is_localizable;


template<>
struct is_localizable<>
  : std::true_type
{ };


template<typename View, typename ...Views>
struct is_localizable<View, Views...>
  : std::integral_constant<
      bool, check_localizable<View>::value && is_localizable<Views...>::value>
{ };


enum view_wrapper_type {Standard, Fast, Mixed};


//////////////////////////////////////////////////////////////////////
/// @brief Functor used during initializing of each view for a task.
/// view storage classes forward original view along with type booleans
/// stating whether (a) the view is to be localized and (b) whether
/// the view supports deferred localization.  Based on these parameters,
/// appropriately forward the original view or called deferred localized
/// constructor and invoke deferred callback if appropriated.
//////////////////////////////////////////////////////////////////////
struct compute_view_params
{
  // Localization will not be performed. View is not deferred localizable.
  template<typename FastView, typename ViewParam>
  static ViewParam&&
  apply(ViewParam&& view, std::false_type, std::false_type,
        edge_local_notifier_base*, tg_callback const&)
  { return std::forward<ViewParam>(view); }

  // Localization will not be performed. View is deferred localizable.
  template<typename FastView, typename ViewParam>
  static ViewParam&&
  apply(ViewParam&& view, std::false_type, std::true_type,
        edge_local_notifier_base* callback, tg_callback const& cb)
  {
    callback->operator()(cb.tg().executor());

    return std::forward<ViewParam>(view);
  }

  // Localization will be performed. View is not deferred localizable.
  template<typename FastView, typename ViewParam>
  static ViewParam&&
  apply(ViewParam&& view, std::true_type, std::false_type,
        edge_local_notifier_base*, tg_callback const&)
  { return std::forward<ViewParam>(view); }

  // Localization will be performed. View is deferred localizable.
  template<typename FastView, typename ViewParam>
  static FastView
  apply(ViewParam&& view, std::true_type, std::true_type,
        edge_local_notifier_base* callback_ptr,
        tg_callback const& cb)
  {
    executor_base* exec_ptr = &cb.tg().executor();
    return FastView(
      std::forward<ViewParam>(view),
      [callback_ptr, exec_ptr] { callback_ptr->operator()(*exec_ptr); }
    );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief View storage used for tasks when localization is not performed.
/// @ingroup pgViewOps
//////////////////////////////////////////////////////////////////////
template<typename ...Views>
class std_storage
  : private tuple<Views...>
{
public:
  using base_view_set_type = tuple<Views...>;
  using view_set_type      = tuple<Views...>;

  template<typename ...ViewParams>
  std_storage(tg_callback const& cb,
              edge_local_notifier_base* notifier_ptr,
              ViewParams&&... views)
    : view_set_type(
        compute_view_params::apply<
          typename get_fast_view_type<Views>::type
        >(
         std::forward<ViewParams>(views),
         std::false_type(),
         is_deferred_localizable<ViewParams>(),
         notifier_ptr, cb
        )...
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by ephemeral paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(tg_callback const& cb, bool b_migrating = false)
  {
    vs_map(lazy_ref_release_func(cb), this->views());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by persistent paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(void)
  {
    vs_map(lazy_ref_release_persistent_func(), this->views());
  }

  base_view_set_type& get_orig_vs(void)
  { return static_cast<view_set_type&>(*this); }

  view_set_type& views(void)
  { return static_cast<view_set_type&>(*this); }
};


//////////////////////////////////////////////////////////////////////
/// @brief View Storage used for tasks when localization is performed
///    and migration is enabled. Store both original and localized views.
/// @ingroup pgViewOps
/// @todo Specialize for cases where localization is a noop
/// (e.g., edge flowed values) to avoid double storage.
//////////////////////////////////////////////////////////////////////
template<typename ...Views>
class fast_mg_storage
{
public:
  using base_view_set_type = tuple<Views...>;
  using view_set_type      = tuple<typename get_fast_view_type<Views>::type...>;

private:
  /// @brief Copy of pre localization viewset.  Maintained to revert
  /// localization prior to task migration.
  base_view_set_type m_orig_vs;

  /// @brief the transformed (i.e., possibly localized) view_set for task.
  view_set_type      m_views;

public:
  template<typename ...ViewParams>
  fast_mg_storage(tg_callback const& cb,
                  edge_local_notifier_base* notifier_ptr,
                  ViewParams&&... views)
    : m_orig_vs(views...),
      m_views(
        compute_view_params::apply<
          typename get_fast_view_type<Views>::type
        >(
         std::forward<ViewParams>(views),
         std::true_type(),
         is_deferred_localizable<ViewParams>(),
         notifier_ptr, cb
        )...
     )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by ephemeral paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(tg_callback const& cb, bool b_migrating = false)
  {
    if (!b_migrating)
      vs_map(lazy_ref_release_func(cb), this->views());

    vs_map(lazy_ref_release_func(cb), m_orig_vs);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by persistent paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(void)
  {
    vs_map(lazy_ref_release_persistent_func(), this->views());
    vs_map(lazy_ref_release_persistent_func(), m_orig_vs);
  }

  ~fast_mg_storage() = default;

  base_view_set_type& get_orig_vs(void)
  { return m_orig_vs; }

  view_set_type& views(void)
  { return m_views; }
}; // class view_wrapper


//////////////////////////////////////////////////////////////////////
/// @brief View storage used for tasks when localization is
///   performed and migration is disabled. Only store the localized views.
/// @ingroup pgViewOps
//////////////////////////////////////////////////////////////////////
template<typename ...Views>
class fast_no_mg_storage
  : private tuple<typename get_fast_view_type<Views>::type...>
{
public:
  using base_view_set_type = tuple<Views...>;
  using view_set_type      = tuple<typename get_fast_view_type<Views>::type...>;

  template<typename ...ViewParams>
  fast_no_mg_storage(tg_callback const& cb,
                     edge_local_notifier_base* notifier_ptr,
                     ViewParams&&... views)
    :  view_set_type(
        compute_view_params::apply<
          typename get_fast_view_type<Views>::type
        >(
         std::forward<ViewParams>(views),
         std::true_type(),
         is_deferred_localizable<ViewParams>(),
         notifier_ptr, cb
        )...
     )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by ephemeral paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(tg_callback const& cb, bool b_migrating = false)
  {
    vs_map(lazy_ref_release_func(cb), this->views());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by persistent paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(void)
  {
    vs_map(lazy_ref_release_persistent_func(), this->views());
  }

  view_set_type& views(void)
  { return static_cast<view_set_type&>(*this); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper struct that computes the type of the tuple used to
///        store views.  The type of view in each tuple element varies
///        based on whether the view is local or not.
///
/// The base case doesn't change the type of the views in the tuple.
/// This allows us to control the number of views that we'll transform
/// in order to avoid large increases in compile time.
///
/// @tparam Localize If true, use the computed localizable vector to
/// perform mixed viewset localization.  Otherwise, do not localize any
/// views.
/// @tparam FastVec   Boost.MPL vector of bools indicating whether each
/// view in the tuple is local (true) or non-local (false)
/// @tparam Views     Tuple of the views to be used to create a task
//////////////////////////////////////////////////////////////////////
template<bool Localize, typename FastVec, typename ...Views>
struct compute_mixed_viewset
{
  using type = std::tuple<typename std::decay<Views>::type...>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the case where there are two views in
/// the PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename... Fast, typename ...Views>
struct compute_mixed_viewset<true, tuple<Fast...>, Views...>
{
  using type =
    tuple<
      typename std::conditional<
        Fast::value, typename get_fast_view_type<Views>::type, Views
      >::type...
    >;
};


//////////////////////////////////////////////////////////////////////
/// @brief View Storage used for tasks when per-view localization is performed
///    and migration is enabled. Store both original and localized views.
///
/// @tparam FastVec Tuple of boolean integral constants representing whether
/// each view should be localized or not.
/// view in the tuple is local (true) or non-local (false)
/// @tparam Views     Tuple of the views to be used to create a task
///
/// @ingroup pgViewOps
/// @todo Specialize for cases where localization is a noop
/// (e.g., edge flowed values) to avoid double storage.
//////////////////////////////////////////////////////////////////////
template<typename FastTuple, typename ...Views>
class mixed_mg_storage;


template<typename... Fast, typename ...Views>
class mixed_mg_storage<tuple<Fast...>, Views...>
{
private:
  static constexpr bool b_localize_views = sizeof...(Views) <= 3;

  using FastVec = tuple<Fast...>;

public:
  using base_view_set_type = std::tuple<Views...>;
  using view_set_type      =
    typename compute_mixed_viewset<b_localize_views, FastVec, Views...>::type;

private:
  /// @brief Copy of pre localization viewset.  Maintained to revert
  /// localization prior to task migration.
  base_view_set_type  m_orig_vs;

  /// @brief the transformed (i.e., possibly localized) view_set for task.
  view_set_type       m_views;

public:
  template<typename ...ViewParams>
  mixed_mg_storage(tg_callback const& cb,
                   edge_local_notifier_base* notifier_ptr,
                   ViewParams&&... views)
    : m_orig_vs(views...),
      m_views(
        compute_view_params::apply<
          typename get_fast_view_type<Views>::type
        >(
         std::forward<ViewParams>(views),
         std::integral_constant<bool, Fast::value && b_localize_views>(),
         is_deferred_localizable<ViewParams>(),
         notifier_ptr, cb
        )...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by ephemeral paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(tg_callback const& cb)
  {
    vs_map(lazy_ref_release_func(cb), m_views);
    vs_map(lazy_ref_release_func(cb), m_orig_vs);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by persistent paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(void)
  {
    vs_map(lazy_ref_release_persistent_func(), m_views);
    vs_map(lazy_ref_release_persistent_func(), m_orig_vs);
  }

  base_view_set_type& get_orig_vs(void)
  { return m_orig_vs; }

  view_set_type& views(void)
  { return m_views; }
}; // class view_wrapper


//////////////////////////////////////////////////////////////////////
/// @brief View storage used for tasks when per-view localization is
///   performed and migration is disabled. Only store the transformed views.
///
/// @tparam FastVec   Boost.MPL vector of bools indicating whether each
/// view in the tuple is local (true) or non-local (false)
/// @tparam Views     Tuple of the views to be used to create a task
///
/// @ingroup pgViewOps
//////////////////////////////////////////////////////////////////////
template<typename FastTuple, typename ...Views>
class mixed_no_mg_storage;


template<typename... Fast, typename ...Views>
class mixed_no_mg_storage<tuple<Fast...>, Views...>
  : public compute_mixed_viewset<
      sizeof...(Views) <= 3, tuple<Fast...>, Views...>::type
{
  static constexpr bool b_localize_views = sizeof...(Views) <= 3;

  using FastVec = tuple<Fast...>;

public:
  using base_view_set_type = std::tuple<Views...>;

  using view_set_type =
    typename compute_mixed_viewset<b_localize_views, FastVec, Views...>::type;

  template<typename ...ViewParams>
  mixed_no_mg_storage(tg_callback const& cb,
                      edge_local_notifier_base* notifier_ptr,
                      ViewParams&&... views)
    : view_set_type(
        compute_view_params::apply<
          typename get_fast_view_type<Views>::type
        >(
         std::forward<ViewParams>(views),
         std::integral_constant<bool, Fast::value && b_localize_views>(),
         is_deferred_localizable<ViewParams>(),
         notifier_ptr, cb
        )...)
  { }

  view_set_type& views(void)
  { return static_cast<view_set_type&>(*this); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by ephemeral paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(tg_callback const& cb, bool b_migrating = false)
  {
    vs_map(lazy_ref_release_func(cb), this->views());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature called by persistent paragraphs
  //////////////////////////////////////////////////////////////////////
  void release(void)
  {
    vs_map(lazy_ref_release_persistent_func(), this->views());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to compute the storage class template used
///  by a task in the paragraph.
/// @ingroup pgViewOps
/// @tparam T enumerated type denoting whether localization should be performed.
/// @tparam Migratable Boolean denoting whether task migration is enabled.
/// @tparam Views The views for a task.
//////////////////////////////////////////////////////////////////////
template<view_wrapper_type T, bool Migratable, typename FastVec,
         typename ...Views>
struct compute_task_view_storage;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used for tasks when no localization is
///   performed. Only store one copy of original views.
//////////////////////////////////////////////////////////////////////
template<bool Migratable, typename FastVec, typename ...Views>
struct compute_task_view_storage<Standard, Migratable, FastVec, Views...>
{ using type = std_storage<Views...>; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used for tasks when localization is
///   performed and migration support is enabled.
//////////////////////////////////////////////////////////////////////
template<typename FastVec, typename ...Views>
struct compute_task_view_storage<Fast, true, FastVec, Views...>
{ using type = fast_mg_storage<Views...>; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used for tasks when localization is
///   performed and migration support is disabled.
//////////////////////////////////////////////////////////////////////
template<typename FastVec, typename ...Views>
struct compute_task_view_storage<Fast, false, FastVec, Views...>
{ using type = fast_no_mg_storage<Views...>; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used for tasks when localization
///   may be performed and migration support is enabled.
//////////////////////////////////////////////////////////////////////
template<typename FastVec, typename ...Views>
struct compute_task_view_storage<Mixed, true, FastVec, Views...>
{ using type = mixed_mg_storage<FastVec, Views...>; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used for tasks when localization
///   may be performed and migration support is disabled.
//////////////////////////////////////////////////////////////////////
template<typename FastVec, typename ...Views>
struct compute_task_view_storage<Mixed, false, FastVec, Views...>
{ using type = mixed_no_mg_storage<FastVec, Views...>; };


//////////////////////////////////////////////////////////////////////
/// @brief Initialize view set for task with no localization applied.
///   Construct @ref Task object on heap and return a pointer to it.
/// @ingroup pgTasks
/// @copydetails create_task(edge_local_notifier_base*, tg_callback const&, edge_entry_base*, SchedulerInfoParam&&, WFParam&&, ViewParams&&)
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename Migratable,
         typename Persistent,
         typename SchedulerInfoParam,
         typename WFParam,
         typename ...ViewParams>
paragraph_impl::task_base_intermediate<SchedulerEntry>*
standard_task_creator(edge_local_notifier_base* notifier_ptr,
                      tg_callback const& cb,
                      edge_entry_base* edge_entry_ptr,
                      SchedulerInfoParam&& scheduler_info,
                      WFParam&& wf, ViewParams&&... views)
{
  return new paragraph_impl::Task<
    SchedulerEntry, Migratable, Persistent,
    typename std::decay<WFParam>::type,             // Workfunction
    typename compute_task_view_storage<             // ViewSet Type
      Standard, Migratable::value, int,             // dummy passed for FastVec
      typename std::decay<ViewParams>::type...>::type
  >(cb, notifier_ptr, edge_entry_ptr,
    std::forward<SchedulerInfoParam>(scheduler_info),
    std::forward<WFParam>(wf),
    std::forward<ViewParams>(views)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Initialize view set for task with localization transformation
///   applied. Wraps the workfunction passed to the task with mechanism to
///   promote a return value from workfunction which is dependent on an
///   input localized view.  Construct @p Task object on heap and return
///   a pointer to it.
/// @ingroup pgTasks
/// @sa paragraph_impl::compute_promotion_wf
/// @copydetails create_task(edge_local_notifier_base*, tg_callback const&, edge_entry_base*, SchedulerInfoParam&&, WFParam&&, ViewParams&&)
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename Migratable,
         typename Persistent,
         typename SchedulerInfoParam,
         typename WFParam,
         typename ...ViewParams>
paragraph_impl::task_base_intermediate<SchedulerEntry>*
fast_task_creator(edge_local_notifier_base* notifier_ptr,
                  tg_callback const& cb,
                  edge_entry_base* edge_entry_ptr,
                  SchedulerInfoParam&& scheduler_info,
                  WFParam&& wf, ViewParams&&... views)
{
  typedef typename compute_task_view_storage<
    Fast, Migratable::value, int,                   // dummy passed for FastVec
    typename std::decay<ViewParams>::type...>::type      view_storage_type;

  // WF selector
  typedef paragraph_impl::compute_promotion_wf<
    typename std::decay<WFParam>::type,
    typename view_storage_type::base_view_set_type,
    typename view_storage_type::view_set_type>           wf_selector_type;

  return new paragraph_impl::Task<
    SchedulerEntry,                                // Scheduler Info
    Migratable, Persistent,                        // Migration/Persistent Bool
    typename wf_selector_type::result_type,        // Workfunction
    view_storage_type                              // ViewSet
  >(cb, notifier_ptr, edge_entry_ptr,
    std::forward<SchedulerInfoParam>(scheduler_info),
    wf_selector_type::apply(cb, wf, views...),
    std::forward<ViewParams>(views)...);
}


#ifdef STAPL_PER_VIEW_LOCALIZATION
//////////////////////////////////////////////////////////////////////
/// @brief Work function that is used to construct a task after the
/// per-view localization transformation has been performed on the views
/// for the task.
///
/// @ingroup pgTasks
/// @sa paragraph_impl::compute_promotion_wf
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename Migratable,
         typename Persistent,
         typename SchedulerInfo,
         typename WF>
struct mixed_task_creator
{
private:
  edge_local_notifier_base* m_notifier_ptr;
  tg_callback const&        m_cb;
  edge_entry_base*          m_edge_entry_ptr;
  SchedulerInfo const&      m_scheduler_info;
  WF const&                 m_wf;

public:
  template<typename SchedulerInfoParam, typename WFParam>
  mixed_task_creator(edge_local_notifier_base* notifier_ptr,
                     tg_callback const& cb,
                     edge_entry_base* edge_entry_ptr,
                     SchedulerInfoParam&& scheduler_info,
                     WFParam&& wf)
    : m_notifier_ptr(notifier_ptr), m_cb(cb), m_edge_entry_ptr(edge_entry_ptr),
      m_scheduler_info(std::forward<SchedulerInfoParam>(scheduler_info)),
      m_wf(std::forward<WFParam>(wf))
  { }

  template<typename FastVec, typename ...ViewParams>
  paragraph_impl::task_base_intermediate<SchedulerEntry>*
  operator()(ViewParams&&... views) const
  {
    using view_storage_type =
      typename compute_task_view_storage<
        Mixed, Migratable::value, FastVec,
        typename std::decay<ViewParams>::type...>::type;

    using wf_selector_type =
      paragraph_impl::compute_promotion_wf<
        WF,
        typename view_storage_type::base_view_set_type,
        typename view_storage_type::view_set_type>;

    return new paragraph_impl::Task<
      SchedulerEntry,                         // Scheduler Info
      Migratable, Persistent,                 // Migration/Persistent Bool
      typename wf_selector_type::result_type, // Workfunction
      view_storage_type                       // ViewSet
    >(m_cb, m_notifier_ptr, m_edge_entry_ptr,
      m_scheduler_info,
      wf_selector_type::apply(m_cb, m_wf, views...),
      std::forward<ViewParams>(views)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Construct the Boost.MPL vector of booleans that indicates
/// which views to be used in a task are local.
///
/// @param v Work function used to create task after the locality of
/// the views has been determined.
/// @param svs Tuple of views to be used to construct the task
/// @param is_last Indicates if we've processed the entire tuple of views.
/// @return Task with localization performed on a per-view basis
//////////////////////////////////////////////////////////////////////
template<int i, typename FastVec, typename SchedulerEntry,
         typename Visitor, typename SVS, typename ...ViewParams>
paragraph_impl::task_base_intermediate<SchedulerEntry>*
mixed_viewset_helper(Visitor const& v,
                     SVS const& svs,
                     std::false_type is_last,
                     ViewParams&&... views)
{
  if (localizer<typename std::remove_pointer<typename std::decay<
        typename std::tuple_element<i, SVS>::type>::type>::type>::apply(
          std::get<i>(svs)))
  {
    return mixed_viewset_helper<
      i+1,
      typename tuple_ops::result_of::push_back<FastVec, std::true_type>::type,
      SchedulerEntry
    >(v, svs,
      std::integral_constant<bool, tuple_size<SVS>::value-1 == i>(),
      std::forward<ViewParams>(views)...);
  }

  // else
  return mixed_viewset_helper<
    i+1,
    typename tuple_ops::result_of::push_back<FastVec, std::false_type>::type,
    SchedulerEntry
  >(v, svs,
    std::integral_constant<bool, tuple_size<SVS>::value-1 == i>(),
    std::forward<ViewParams>(views)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization used to stop the recursion and execute the
/// visitor functor to create the task.  Note the type of is_last is
/// different.
//////////////////////////////////////////////////////////////////////
template<int i, typename FastVec, typename SchedulerEntry,
         typename Visitor, typename SVS, typename ...ViewParams>
paragraph_impl::task_base_intermediate<SchedulerEntry>*
mixed_viewset_helper(Visitor const& v,
                     SVS const& svs,
                     std::true_type is_last,
                     ViewParams&&... views)
{
  return v.template operator()<FastVec>(views...);
}
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Called after a tasks' views have been created to make the
///   task object.  Statically guards localization with @ref is_localizable<
///   metafunction.
/// @ingroup pgTasks
/// @tparam Localize Statically denotes whether localization should be
///   performed.  Used to dispatch to proper specialization.
/// @bug per-view localization is currently disabled for lazy_edge_references
///   because the ref counting performed when they're copied results in
///   early attempts to destroy them. Eliminating view copies in the per-view
///   localization code path will resolve this issue.
/// @todo Guard mix_task_creator instantiation in a partial specialization
///   to reduce compile times.
//////////////////////////////////////////////////////////////////////
template<bool Localize>
struct create_task_impl
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for viewless task.  No need to
  ///   apply localization.
  //////////////////////////////////////////////////////////////////////
  template<typename SchedulerEntry,
           typename Migratable,
           typename Persistent,
           typename SchedulerInfoParam,
           typename WFParam>
  static paragraph_impl::task_base_intermediate<SchedulerEntry>*
  apply(edge_local_notifier_base* notifier_ptr,
        tg_callback const& cb, edge_entry_base* edge_entry_ptr,
        SchedulerInfoParam&& scheduler_info, WFParam&& wf)
  {
    return standard_task_creator<SchedulerEntry, Migratable, Persistent>(
      notifier_ptr, cb,
      edge_entry_ptr,
      std::forward<SchedulerInfoParam>(scheduler_info),
      std::forward<WFParam>(wf));
  }


  template<typename SchedulerEntry,
           typename Migratable,
           typename Persistent,
           typename SchedulerInfoParam,
           typename WFParam,
           typename ...ViewParams>
  static paragraph_impl::task_base_intermediate<SchedulerEntry>*
  apply(edge_local_notifier_base* notifier_ptr,
        tg_callback const& cb,
        edge_entry_base* edge_entry_ptr,
        SchedulerInfoParam&& scheduler_info,
        WFParam&& wf, ViewParams&&... views)
   {
     // Based on localizer test, create task with fast or slow views. Initialize
     // it with knowledge of succs so it passed it to tg::processed later.
     if (pack_ops::functional::and_(
           localizer<typename std::decay<ViewParams>::type>::apply(views)...))
     {
       return fast_task_creator<SchedulerEntry, Migratable, Persistent>(
                notifier_ptr, cb, edge_entry_ptr,
                std::forward<SchedulerInfoParam>(scheduler_info),
                std::forward<WFParam>(wf),
                std::forward<ViewParams>(views)...);
     }
#ifdef STAPL_PER_VIEW_LOCALIZATION
     // Currently lazy_edge_reference ref counting is causing problems.
     // See @bug above.
     else if (!pack_ops::functional::or_(
                is_lazy_edge_reference<typename std::decay<ViewParams>::type>
                  ::apply(views)...))
     {
       mixed_task_creator<SchedulerEntry, Migratable, Persistent,
                          typename std::decay<SchedulerInfoParam>::type,
                          typename std::decay<WFParam>::type>
       creator(
         notifier_ptr, cb, edge_entry_ptr,
         std::forward<SchedulerInfoParam>(scheduler_info),
         std::forward<WFParam>(wf));

       return mixed_viewset_helper<0, tuple<>, SchedulerEntry>(
         creator, std::forward_as_tuple(views...),
         std::integral_constant<bool, sizeof...(ViewParams) == 0>(),
         std::forward<ViewParams>(views)...);
     }
#endif
     else
     {
       return standard_task_creator<SchedulerEntry, Migratable, Persistent>(
         notifier_ptr, cb, edge_entry_ptr,
         std::forward<SchedulerInfoParam>(scheduler_info),
         std::forward<WFParam>(wf),
         std::forward<ViewParams>(views)...);
     }
  }
}; // struct create_task_impl


template<>
struct create_task_impl<false>
{
  template<typename SchedulerEntry,
           typename Migratable,
           typename Persistent,
           typename SchedulerInfoParam,
           typename WFParam,
           typename ...ViewParams>
  static paragraph_impl::task_base_intermediate<SchedulerEntry>*
  apply(edge_local_notifier_base* notifier_ptr,
        tg_callback const& cb,
        edge_entry_base* edge_entry_ptr,
        SchedulerInfoParam&& scheduler_info,
        WFParam&& wf, ViewParams&&... views)
  {
    return standard_task_creator<SchedulerEntry, Migratable, Persistent>(
      notifier_ptr, cb, edge_entry_ptr,
      std::forward<SchedulerInfoParam>(scheduler_info),
      std::forward<WFParam>(wf),
      std::forward<ViewParams>(views)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Checks static localizability of task based on views and
///  dispatches to appropriate task creation mechanism.
/// @ingroup pgTasks
/// @tparam SchedulerEntry Intrusive entry for scheduler which
///  stores scheduling metadata and container related fields.
/// @tparam Migratable Boolean type parameter denoting whether task
///   migration enabled for the associated PARAGRAPH.
/// @tparam Persistent Boolean type parameter denoting whether
///   persistency enabled for the associated PARAGRAPH.
/// @param notifier_ptr Pointer to task's predcessor notifier.  Allows
///   views that are deferred localizable to inform the paragraph that
///   their localization is complete (i.e., don't hold the task any further
///   because of dependence on data fetch).
/// @param edge_entry_ptr Edge entry that this task writes its results to.
/// @param sched_info Information used to determine relative priority of task.
/// @param wf The workfunction for the task.
/// @param views The pre-localization views for this task.
//////////////////////////////////////////////////////////////////////
template<typename SchedulerEntry,
         typename Migratable,
         typename Persistent,
         typename SchedulerInfoParam,
         typename WFParam,
         typename ...ViewParams>
paragraph_impl::task_base_intermediate<SchedulerEntry>*
create_task(edge_local_notifier_base* notifier_ptr,
            tg_callback const& cb,
            edge_entry_base* edge_entry_ptr,
            SchedulerInfoParam&& scheduler_info,
            WFParam&& wf, ViewParams&&... views)
{
  return create_task_impl<
    is_localizable<typename std::decay<ViewParams>::type...>::value
  >::template apply<SchedulerEntry, Migratable, Persistent>
    (notifier_ptr, cb, edge_entry_ptr,
     std::forward<SchedulerInfoParam>(scheduler_info),
     std::forward<WFParam>(wf),
     std::forward<ViewParams>(views)...);
}

} // namespace detail

} // namespace stapl

#endif // STAPL_PARAGRAPH_TASK_CREATION_HPP
