/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_MAP_REDUCE_HPP
#define STAPL_SKELETONS_MAP_REDUCE_HPP

#include <utility>
#include <type_traits>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/zip_reduce.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>
#include <stapl/skeletons/functional/sink_value.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/coarse/zip_reduce.hpp>
#include <stapl/views/metadata/coarseners/multiview.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace result_of {

template <typename Tag, typename MapOp, typename ReduceOp, typename ...V>
struct map_reduce
{
  typedef typename
    boost::result_of<
      ReduceOp(
        typename boost::result_of<
          MapOp(typename std::decay<V>::type::reference...)>::type,
        typename boost::result_of<
          MapOp(typename std::decay<V>::type::reference...)>::type)
    >::type type;
};


//////////////////////////////////////////////////////////////////////
/// @brief This specialization exists to accommodate the over eager
/// template instantiations of clang in a disabled map_reduce function
/// signature guarded by enable_if. It tries to compute the return type
/// even though it will never be used.
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename CoarseTag, typename ExecutionTag,
          typename ReduceOp, typename ...Args>
struct map_reduce<
  Tag, skeletons::tags::coarse<CoarseTag, ExecutionTag>,
  ReduceOp, Args...>
{
  using type = void;
};


template <typename MapOp, typename ReduceOp, typename ...V>
struct map_reduce<skeletons::tags::with_coarsened_wf, MapOp, ReduceOp, V...>
{
  typedef typename
    boost::result_of<
      ReduceOp(
        typename boost::result_of<MapOp(typename std::decay<V>::type...)>::type,
        typename boost::result_of<MapOp(typename std::decay<V>::type...)>::type)
    >::type type;
};

}

namespace map_reduce_helper {

struct empty_view
{
  typedef bool result_type;

  template<typename V>
  result_type operator()(V const& v) const
  {
    return v.empty();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief The core implementation of map_reduce.
///
/// Given the executor and the coarsened skeleton it creates the
/// auxiliary static_array to populate the final result of map-reduce
/// into it.
///
/// @tparam T       result type of the reduction operation
/// @tparam C       data coarsening method used
/// @param skeleton The skeleton containing the coarsened map and reduce
///                 operations to be applied to the input views.
/// @param v         the inputs
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename C, typename S, typename ...V>
inline T
map_reduce_impl(S const& skeleton, V&&... v)
{
#ifndef STAPL_NDEBUG
  stapl::tuple<typename std::decay<V>::type...> t(v...);
  stapl_assert(
    !vs_map_reduce(
      empty_view(), stapl::logical_or<bool>(), false, t
    ),
    "Invoking map_reduce on an empty view");
#endif

  using namespace skeletons;
  typedef typename df_stored_type<T>::type val_t;
  return execute(
           execution_params<val_t>(C()),
           skeletons::sink_value<val_t>(compose(skeleton, broadcast_to_locs())),
           std::forward<V>(v)...
         );
}


//////////////////////////////////////////////////////////////////////
/// @brief The core implementation of map_reduce.
///
/// Given the coarsened skeleton and a scheduler to use for its execution
/// create the executor and invoke it with a tuple of the input views.
///
/// @tparam T       result type of the reduction operation
/// @tparam C       data coarsening method used
/// @param skeleton The skeleton containing the coarsened map and reduce
///                 operations to be applied to the input views.
/// @param scheduler The scheduler to use in the execution of the skeleton.
/// @param v         the inputs
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename C, typename S, typename Scheduler, typename ...V>
inline typename std::enable_if<!is_view<Scheduler>::value, T>::type
map_reduce_impl(S const& skeleton, Scheduler const& scheduler, V&&... v)
{
#ifndef STAPL_NDEBUG
  stapl::tuple<typename std::decay<V>::type...> t(v...);
  stapl_assert(
    !vs_map_reduce(
      empty_view(), stapl::logical_or<bool>(), false, t
    ),
    "Invoking map_reduce on an empty view");
#endif

  using namespace skeletons;
  typedef typename df_stored_type<T>::type val_t;
  return execute(
           execution_params<val_t>(C(), stapl::use_default(), scheduler),
           skeletons::sink_value<val_t>(
            compose(skeleton, broadcast_to_locs())),
           std::forward<V>(v)...);

}


//////////////////////////////////////////////////////////////////////
/// @brief Applies map-reduce on a set of inputs by coarsening the
/// map-reduce skeleton and the inputs. This is the most commonly used
/// version of map-reduce in STAPL.
///
/// @tparam Tag use_default specifies that both data and skeleton need to be
///                  coarsened
/// @param map_op    the operation to be applied in the first step on
///                  the given input(s)
/// @param reduce_op the operation to be applied on the results of the
///                  the first map phase
/// @param v         the inputs
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename Tag, typename MapOp, typename ReduceOp, typename ...V>
inline
typename std::enable_if<
  std::is_same<Tag, stapl::use_default>::value,
  typename stapl::result_of::map_reduce<Tag, MapOp, ReduceOp, V...>::type>::type
map_reduce(Tag, MapOp const& map_op, ReduceOp const& reduce_op, V const&... v)
{
  typedef typename
    stapl::result_of::map_reduce<
      stapl::use_default, MapOp, ReduceOp, V...>::type val_t;
  using namespace skeletons;

  return map_reduce_helper::map_reduce_impl<val_t, default_coarsener>(
           skeletons::coarse(
             skeletons::zip_reduce<sizeof...(V)>(map_op, reduce_op)
           ),
           v...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Applies map-reduce on a set of inputs by coarsening the
/// map-reduce skeleton according to the @c CoarseTag and @c ExecutionTag
/// specified.
///
/// @param CoarseTag    the type of coarsening
/// @param ExecutionTag the type of execution for the specified coarsening
/// @param map_op       the operation to be applied in the first step on
///                     the given input(s)
/// @param reduce_op    the operation to be applied on the results of the
///                     the first map phase
/// @param v            the inputs
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template <typename CoarseTag, typename ExecutionTag, typename MapOp,
          typename ReduceOp, typename... V>
inline
typename stapl::result_of::map_reduce<CoarseTag, MapOp, ReduceOp, V...>::type
map_reduce(skeletons::tags::coarse<CoarseTag, ExecutionTag>,
           MapOp const& map_op, ReduceOp const& reduce_op, V const&... v)
{
  typedef typename
    stapl::result_of::map_reduce<
      CoarseTag, MapOp, ReduceOp, V...>::type val_t;
  using namespace skeletons;

  return map_reduce_helper::map_reduce_impl<val_t, default_coarsener>(
           skeletons::coarse<CoarseTag, ExecutionTag>(
             skeletons::zip_reduce<sizeof...(V)>(map_op, reduce_op)
           ),
           v...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Applies map-reduce on a set of inputs without coarsening the
/// map-reduce skeleton and the inputs.
///
/// @tparam Tag no_coarsening specifies that neither data nor skeleton need
///                  to be coarsened
/// @param map_op    the operation to be applied in the first step on
///                  the given input(s)
/// @param reduce_op the operation to be applied on the results of the
///                  the first map phase
/// @param v         the inputs
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename Tag, typename MapOp, typename ReduceOp, typename ...V>
inline
typename std::enable_if<
  std::is_same<Tag, skeletons::tags::no_coarsening>::value,
  typename stapl::result_of::map_reduce<Tag, MapOp, ReduceOp, V...>::type>::type
map_reduce(Tag, MapOp const& map_op, ReduceOp const& reduce_op, V&&... v)
{
  typedef typename
    stapl::result_of::map_reduce<
      skeletons::tags::no_coarsening, MapOp, ReduceOp, V...>::type val_t;
  using namespace skeletons;

  return map_reduce_helper::map_reduce_impl<val_t, null_coarsener>(
           skeletons::zip_reduce<sizeof...(V)>(map_op, reduce_op),
           std::forward<V>(v)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Applies map-reduce on a set of inputs assuming the the given
/// @c map_op can accept coarse-grain data.
///
/// @tparam Tag with_coarsened_wf specifies that only data needs to be
///                  coarsened
/// @param map_op    the operation to be applied in the first step on
///                  the given input(s)
/// @param reduce_op the operation to be applied on the results of the
///                  the first map phase
/// @param v         the inputs
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename Tag, typename MapOp, typename ReduceOp, typename ...V>
inline
typename std::enable_if<
  std::is_same<Tag, skeletons::tags::with_coarsened_wf>::value,
  typename stapl::result_of::map_reduce<Tag, MapOp, ReduceOp, V...>::type>::type
map_reduce(Tag, MapOp const& map_op, ReduceOp const& reduce_op, V&&... v)
{
  typedef typename
    stapl::result_of::map_reduce<
      skeletons::tags::with_coarsened_wf, MapOp, ReduceOp, V...>::type val_t;

  return map_reduce_helper::map_reduce_impl<val_t, default_coarsener>(
           skeletons::zip_reduce<sizeof...(V)>(map_op, reduce_op),
           std::forward<V>(v)...);
}

} // namespace map_reduce_helpers

//////////////////////////////////////////////////////////////////////
/// @brief Applies map-reduce on a set of inputs. In the first step
/// the given map operation (zip operation if there is more than one
/// view involved) on the given inputs and the results are reduce
/// using the given reduce operation (@c reduce_op)
///
/// @param map_op    the operation to be applied in the first step on
///                  the given input(s)
/// @param reduce_op the operation to be applied on the results of the
///                  the first map phase
/// @param v         the inputs
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename MapOp, typename ReduceOp, typename ...V>
inline
typename std::enable_if<
  !std::is_base_of<skeletons::tags::coarsening_tag, MapOp>::value,
  stapl::result_of::map_reduce<stapl::use_default, MapOp, ReduceOp, V...>
>::type::type
map_reduce(MapOp const& map_op, ReduceOp const& reduce_op, V&&... v)
{
  return map_reduce_helper::map_reduce(stapl::use_default(),
                                       map_op, reduce_op,
                                       std::forward<V>(v)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Applies map-reduce on the given input by using the specific
/// @c map_reduce implementation specified by the given tag. Current
/// set of tags which can be used for map_reduce are:
/// @li stapl::use_default - uses the default coarsened version of
///     map_reduce. This is the most-used version of map_reduce in STAPL.
/// @li with_coarsened_wf - in some cases it is desirable to avoid the
///     default workfunction used in the coarsened map phase. One can
///     use this tag to indicate that the given @c map_op can handle
///     coarse-grain inputs
/// @li no_coarsening - when the coarsening phase (data and skeletons)
///     needs to be avoided as a whole.
///
/// @param map_op    the operation to be applied in the first step on
///                  the given input(s)
/// @param reduce_op the operation to be applied on the results of the
///                  the first map phase
/// @param v         the inputs
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename Tag, typename MapOp, typename ReduceOp, typename ...V>
inline
typename stapl::result_of::map_reduce<Tag, MapOp, ReduceOp, V...>::type
map_reduce(MapOp const& map_op, ReduceOp const& reduce_op, V&&... v)
{
  return map_reduce_helper::map_reduce(Tag(), map_op, reduce_op,
                                       std::forward<V>(v)...);
}

} // namespace stapl

#endif // STAPL_SKELETONS_MAP_REDUCE_HPP
