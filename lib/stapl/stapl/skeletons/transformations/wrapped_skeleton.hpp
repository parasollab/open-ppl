/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_WRAPPED_SKELETON_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_WRAPPED_SKELETON_HPP

#include <type_traits>
#include <stapl/skeletons/utility/dynamic_wf.hpp>
#include <stapl/skeletons/utility/identity_helpers.hpp>
#include <stapl/skeletons/utility/is_nested_skeleton.hpp>
#include <stapl/views/metadata/mix_view.hpp>
#include <boost/utility/result_of.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/executors/execution_params.hpp>

namespace stapl {

template <typename V, typename I, typename CID>
class mix_view;

template <typename V>
struct localized_view;

namespace paragraph_impl {
template <typename Sched>
class paragraph_view;
}

namespace skeletons {

template <typename Op>
auto get_op_helper(std::true_type, Op&& op)
-> decltype(op.get_skeleton())
{
  return op.get_skeleton();
}

template <typename Op>
Op get_op_helper(std::false_type, Op&& op)
{
  return op;
}

template <typename Op>
auto get_op(Op&& op)
-> decltype(get_op_helper(is_nested_skeleton<
            typename std::decay<Op>::type>(),
            std::forward<Op>(op)))
{
  return get_op_helper(is_nested_skeleton<
                        typename std::decay<Op>::type>(),
                        std::forward<Op>(op));
}

namespace skeletons_impl {


//////////////////////////////////////////////////////////////////////
/// @brief The return type to be determined for an optimizer by passing
/// the fine-grain type.
///
/// @ingroup skeletonTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename O, typename T>
struct compute_return_type
{
  using type = typename boost::result_of<O(T)>::type;
};


template <typename O>
struct compute_return_type<O, void>
{
  using type = void;
};


//////////////////////////////////////////////////////////////////////
/// @brief Check if the given view is a @ref paragraph_view
///
/// @ingroup skeletonTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename V>
struct is_tgv
  : public std::false_type
{ };


template <typename Sched>
struct is_tgv<paragraph_impl::paragraph_view<Sched>>
  : public std::true_type
{ };


BOOST_MPL_HAS_XXX_TRAIT_DEF(value_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(domain_type)

template <typename V, bool has_domain = has_domain_type<V>::value>
struct find_domain_type
  : public indexed_domain<size_t, 1>
{ };

template <typename V>
struct find_domain_type<V, true>
  : public V::domain_type
{ };

template <typename V>
struct is_input_view
  : public is_view<V>
{ };

template < typename Sched>
struct is_input_view<paragraph_impl::paragraph_view<Sched>>
  : public std::true_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief A fake_flow is used for making the result_type computation
/// possible for wrapped skeletons (@ref wrapped_skeleton).
///
/// A fake_flow is basically specialized to extract the underlying
/// value type of a given coarsened input and providing it to the
/// @ref wrapped_skeleton @c result struct.
///
/// @ingroup skeletonTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename V,
          bool isView = has_value_type<V>::value || is_input_view<V>::value>
struct fake_flow
{
  using domain_type = find_domain_type<V>;
  using flow_value_type = typename V::value_type;
};


template <typename V>
struct fake_flow<V, false>
{
  using domain_type = find_domain_type<V>;
  using flow_value_type = V;
};

//////////////////////////////////////////////////////////////////////
/// @brief A fake_flow is used for making the result_type computation
/// possible for wrapped skeletons (@ref wrapped_skeleton).
///
/// This specialization handles the case in which the given input is
/// a coarsened view over the original input.
///
/// @ingroup skeletonTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename V, typename I, typename CID>
struct fake_flow<mix_view<V, I, CID>, true>
{
  using domain_type     = typename mix_view<V, I, CID>::domain_type;
  using flow_value_type = typename mix_view<V, I, CID>::reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief A fake_flow is used for making the result_type computation
/// possible for wrapped skeletons (@ref wrapped_skeleton).
///
/// This specialization handles the case in which the given input is
/// a fast view.
///
/// @ingroup skeletonTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename V>
struct fake_flow<localized_view<V>, true>
{
  using domain_type     = typename localized_view<V>::domain_type;
  using flow_value_type = typename localized_view<V>::reference;
};

//////////////////////////////////////////////////////////////////////
/// @brief A fake_flow is used for making the result_type computation
/// possible for @c wrapped_skeletons.
///
/// This specialization handles the case in which the given input is
/// a paragraph view. A paragraph view does not carry a value, hence,
/// typed as void.
///
/// @ingroup skeletonTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename V>
struct fake_flow<paragraph_impl::paragraph_view<V>, true >
{
  using domain_type = void;
  using flow_value_type = void;
};

template <typename ViewsTuple>
struct find_flows_type;

template <typename... Views>
struct find_flows_type<tuple<Views...>>
{
  using type = typename std::decay<std::tuple<fake_flow<Views>...>>::type;
};

template <typename Coarsener, typename V0, typename... V>
struct find_in_flow_type
{
  using views_tuple_t = stapl::tuple<V0, V...>;
  using coarsened_views_t =
    typename find_flows_type<
      decltype(std::declval<Coarsener>()(
        std::declval<views_tuple_t>()))
    >::type;

  using type =
    typename std::conditional<
      skeletons_impl::is_tgv<V0>::value,
      stapl::tuple<skeletons_impl::fake_flow<V>...>,
      coarsened_views_t
    >::type;
};

template <typename Coarsener, typename Optimizer, typename S,
          bool has_result_type = has_result_type<Optimizer>::value>
struct wrapped_skeleton_result_type
{
  using type = typename Optimizer::result_type;
};

template <typename Coarsener,
          typename Optimizer,
          typename S,
          typename V0,
          typename... V>
struct wrapped_skeleton_result_type<Coarsener, Optimizer, S(V0, V...), false>
{
private:
  using in_flow_t = typename find_in_flow_type<Coarsener, V0, V...>::type;

  using out_flow_t = typename S::template out_port_type<in_flow_t>::type;
  using value_t = typename tuple_element<0, out_flow_t>::type::flow_value_type;

public:
  using type = typename skeletons_impl::compute_return_type<
                 Optimizer, value_t>::type;
};

} // namespace skeletons_impl


//////////////////////////////////////////////////////////////////////
/// @brief A wrapped_skeleton is used in the nested execution of skeletons.
///
/// An @c ExecutionTag is used to customize the execution method. By
/// default, a @c PARAGRAPH will be created for a wrapped skeleton.
/// This behavior can be customized by providing a custom @c ExecutionTag
/// to this class. For example, passing a sequential execution tag
/// (tags::sequential_execution), avoids creating a PARAGRAPH for the
/// nested computation.
///
/// A @c wrapped_skeleton passes the skeleton to specializations of the
/// @ref optimizer. These specializations are defined over the skeleton
/// and @c ExecutionTag. For example, the specialization for the
/// @ref reduce skeleton (optimizer<tags::reduce, tags::sequential_execution>)
/// implements this skeleton using std::accumulate.
///
/// @see optimizers/nested.hpp
/// @see optimizers/reduce.hpp
///
/// @tparam S            the skeleton to be wrapped
/// @tparam ExecutionTag a tag defining the execution strategy for this
///                      skeleton
/// @tparam requiresTGV  a boolean variable to determine whether this
///                      wrapped skeleton needs @ref paragraph_view or not
///
/// @ingroup skeletonTransformations
///
/// @todo requiresTGV should be removed from here.
//////////////////////////////////////////////////////////////////////
template <typename S,
          typename ExecutionTag,
          typename ExecutionParams,
          bool requiresTGV>
class wrapped_skeleton
  : public std::conditional<requiresTGV, dynamic_wf, stapl::use_default>::type
{
  S m_skeleton;
  ExecutionParams m_exec_params;
public:
  using optimizer_t = optimizers::optimizer<
                        typename S::skeleton_tag_type,
                        ExecutionTag>;

  using wrapped_skeleton_type = S;
  using execution_params_t = ExecutionParams;

  explicit wrapped_skeleton(S const& skeleton,
                            ExecutionParams const& exec_params)
    : m_skeleton(skeleton),
      m_exec_params(exec_params)
  { }

  template <typename F>
  struct result;

  template <typename F, typename ...V>
  struct result<F(V...)>
  {
    using type = typename skeletons_impl::wrapped_skeleton_result_type<
      typename ExecutionParams::coarsener_type,
      optimizer_t,
      S(typename std::decay<V>::type...)>::type;
  };

  template <typename ...V>
  typename result<wrapped_skeleton(V...)>::type
  operator()(V&&... view) const
  {
    using return_t = typename result<wrapped_skeleton(V...)>::type;
    return optimizer_t::template execute<return_t>(
             m_skeleton, std::forward<V>(view)...);
  }

  template <typename ...V>
  typename result<wrapped_skeleton(V...)>::type
  operator()(V&&... view)
  {
    using return_t = typename result<wrapped_skeleton(V...)>::type;
    return optimizer_t::template execute<return_t>(
             m_skeleton, std::forward<V>(view)...);
  }

  S const& get_skeleton() const
  {
    return m_skeleton;
  }

  ExecutionParams get_exec_params() const
  {
    return m_exec_params;
  }

  void define_type(typer& t)
  {
    t.member(m_skeleton);
    t.member(m_exec_params);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wraps a skeleton with a suggested execution strategy.
///
/// @tparam S            the skeleton to be wrapped
/// @tparam ExecutionTag a tag defining the execution strategy for this
///                      skeleton
/// @tparam requiresTGV  a boolean variable to determine whether this
///                      skeleton needs @ref paragraph_view or not
///
/// @return a @c wrapped_skeleton
///
/// @ingroup skeletonTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename ExecutionTag = stapl::use_default,
          bool requiresTGV = false,
          typename S,
          typename ExecutionParams = skeletons_impl::default_execution_params,
          typename = typename std::enable_if<
            is_skeleton<typename std::decay<S>::type>::value>::type>
wrapped_skeleton<typename std::decay<S>::type,
                 ExecutionTag,
                 typename std::decay<ExecutionParams>::type,
                 requiresTGV>
wrap(S&& skeleton, ExecutionParams&& execution_params = ExecutionParams())
{
  return wrapped_skeleton<
          typename std::decay<S>::type,
          ExecutionTag,
          typename std::decay<ExecutionParams>::type,
          requiresTGV>(
            std::forward<S>(skeleton),
            std::forward<ExecutionParams>(execution_params));
}

template <typename ExecutionTag = stapl::use_default,
          bool requiresTGV      = false,
          typename Op,
          typename =
            typename std::enable_if<
              !is_skeleton<typename std::decay<Op>::type>::value>::type>
Op wrap(Op&& op)
{
  return op;
}


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_WRAPPED_SKELETON_HPP
