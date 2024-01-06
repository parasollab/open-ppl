/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_WF_INVOKE_HPP
#define STAPL_PARAGRAPH_WF_INVOKE_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/paragraph/edge_container/views/edge_accessor_fwd.hpp>
#include <stapl/views/type_traits/is_proxy.hpp>

namespace stapl {

namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that unwraps redundant proxy / view wrapping such as
///   with the edge_accessor backed proxy when the edge type is a proxy.
/// @ingroup pgUtility
///
/// @tparam View The view set element to inspect for redundant wrapping.
/// @tparam V Default assigned template parameter that employs SFINAE to select
///   a specific specialization if the view set element is a view.
///
/// The user level code need not see proxy<proxy<T>> or proxy<view<T>>,
/// strip_df_proxy unwraps redundant wrappers (from the user workfunction's
/// perspective) as both of the nested types in the case already represent a
/// lightweight reference to the underlying element type.
/// The PARAGRAPH framework adds this nesting internally, and here we remove it,
/// prior to invocation of the workfunction via @ref wf_invoke or
/// @ref wf_invoke_void.
///
/// We defer this unwrapping until this point, as we are not guaranteed that the
/// underlying type (either a proxy or another view) has been initialized prior
/// to the predecessor it represents being completed, a condition certainly met
/// if we are in Task::operator() where wf_invoke is used to call the user's
/// workfunction.
///
/// Primary template is default case, representing on an identity op.
///
/// @sa wf_invoke
/// @sa wf_invoke_void
//////////////////////////////////////////////////////////////////////
template<typename View, typename V = void>
struct strip_df_proxy
{
  using result_type = View;

  static View& apply(View& view)
  {
    return view;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for proxy<view<T>>.
/// @ingroup pgUtility
///
/// Remove the proxy by invoking proxy's conversion operator.
//////////////////////////////////////////////////////////////////////
template<typename T, typename A>
struct strip_df_proxy<proxy<T, A>, typename T::is_view_>
{
private:
  using input_t = proxy<T, A>;

public:
  // Force const so that we properly use the temporary object
  // returned by operator() here as parameter to the wf called in
  // wf_invoke...
  using result_type = const T;

  static result_type apply(input_t& prx)
  {
    return static_cast<result_type>(prx);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref edge_container backed values.
/// Creates proxy<edge_accessor<T>>.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct strip_df_proxy<lazy_edge_reference<T>>
{
  using result_type =
    typename std::conditional<
      is_proxy<T>::value,
      T,
      typename lazy_edge_reference<T>::reference_type
    >::type;

  static result_type apply(lazy_edge_reference<T>& ref)
  {
    return ref.get_reference();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref edge_container backed values for
/// inter-paragraph dataflow. Creates proxy<edge_accessor<T>>.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct strip_df_proxy<pg_lazy_edge_reference<T>>
{
  using result_type =
    typename std::conditional<
      is_proxy<T>::value,
      T,
      typename pg_lazy_edge_reference<T>::reference_type
    >::type;

  static result_type apply(pg_lazy_edge_reference<T>& ref)
  {
    return ref.get_reference();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object that invokes a workfunction by expanding a set
///   views stored in a tuple into a comma separated list of parameters.
///   This list defines the actual parameters to the workfunction.
/// @ingroup pgUtility
///
/// It is used for workfunctions returning non-void value types.
///
/// @tparam WF Workfunction type.
/// @tparam ViewSet A tuple where each element is a view that should be
///   passed to the workfunction invocation.
/// @tparam IdxList Integer pack of indices used for tuple argument expansion.
/// @return The return value of the workfunction invocation
//////////////////////////////////////////////////////////////////////
template<typename WF, typename ViewSet,
         typename IdxList = make_index_sequence<
           tuple_size<typename std::remove_const<ViewSet>::type>::value>>
struct wf_invoke;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of class for @p i number of views.
/// @ingroup pgUtility
/////////////////////////////////////////////////////////////////////
template<typename WF, typename... Args, std::size_t... Indices>
struct wf_invoke<WF, tuple<Args...>, index_sequence<Indices...>>
{
  using result_type =
    typename result_of<
      WF(typename strip_df_proxy<Args>::result_type...)>::type;

  using ViewSet = tuple<Args...>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for non const qualified view set.
  //////////////////////////////////////////////////////////////////////
  result_type
  operator()(WF& wf, ViewSet& vs) const
  {
    return wf(strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for const qualified view set.
  //////////////////////////////////////////////////////////////////////
  result_type
  operator()(WF& wf, ViewSet const& vs) const
  {
    return wf(strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for non const qualified view set as well as
  /// the @ref paragraph_view as one of the workfunction parameters.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV>
  result_type
  operator()(WF& wf, ViewSet& vs, TGV const& tgv) const
  {
    return wf(tgv, strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for const qualified view set as well as
  /// the @ref paragraph_view as one of the workfunction parameters.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV>
  result_type
  operator()(WF& wf, ViewSet const& vs, TGV const& tgv) const
  {
    return wf(tgv, strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }
}; // struct wf_invoke


//////////////////////////////////////////////////////////////////////
/// @brief Function object that invokes a workfunction by expanding a set
///   views stored in a tuple into a comma separated list of parameters.
///   This list defines the actual parameters to the workfunction.
/// @ingroup pgUtility
///
/// It is used for workfunctions returning void.
///
/// @tparam WF Workfunction type.
/// @tparam ViewSet A tuple where each element is a view that should be
///   passed to the workfunction invocation.
/// @tparam IdxList Integer pack of indices used for tuple argument expansion.
/// @return The return value of the workfunction invocation
//////////////////////////////////////////////////////////////////////
template<typename WF, typename ViewSet,
         typename IdxList = make_index_sequence<
           tuple_size<typename std::remove_const<ViewSet>::type>::value>>
struct wf_invoke_void;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of class for @p i number of views.
/// @ingroup pgUtility
/////////////////////////////////////////////////////////////////////
template<typename WF, typename... Args, std::size_t... Indices>
struct wf_invoke_void<WF, tuple<Args...>, index_sequence<Indices...>>
{
  using result_type = void;
  using ViewSet     = tuple<Args...>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for const qualified view set.
  //////////////////////////////////////////////////////////////////////
  void operator()(WF& wf, ViewSet const& vs) const
  {
    wf(strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for non const qualified view set.
  //////////////////////////////////////////////////////////////////////
  void operator()(WF& wf, ViewSet& vs) const
  {
    wf(strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for const qualified view set as well as
  /// the @ref paragraph_view as one of the workfunction parameters.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV>
  void operator()(WF& wf, ViewSet const& vs, TGV const& tgv) const
  {
    wf(tgv, strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization for non const qualified view set as well as
  /// the @ref paragraph_view as one of the workfunction parameters.
  //////////////////////////////////////////////////////////////////////
  template<typename TGV>
  result_type
  operator()(WF& wf, ViewSet& vs, TGV const& tgv) const
  {
    return wf(tgv, strip_df_proxy<Args>::apply(get<Indices>(vs))...);
  }
}; // struct wf_invoke_void

} // namespace paragraph_impl

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_WF_INVOKE_HPP

