/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_COARSE_OPTIMIZERS_ZIP_REDUCE_HPP
#define STAPL_SKELETONS_COARSE_OPTIMIZERS_ZIP_REDUCE_HPP

#include <vector>
#include <stapl/skeletons/transformations/optimizer.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/wf_iter_compare.hpp>
#include <stapl/skeletons/utility/dynamic_wf.hpp>
#include <stapl/skeletons/transformations/optimizers/utils.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief Helper recursive template that unrolls a zip_reduce
/// loop (iterator based) by the specified factor.
//////////////////////////////////////////////////////////////////////
template<std::size_t N>
struct zr_unroller
{
  template<typename Return, typename Zip, typename Reduce, typename ...Iters>
  static void apply(Return& ret, Zip& zip_op, Reduce& red_op, Iters&... iters)
  {
     helpers::reduce_assign(ret, red_op(ret, zip_op(*(++iters)...)));
     zr_unroller<N-1>::apply(ret, zip_op, red_op, iters...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Base case of the recursion.  Call zip and reduction one more time.
//////////////////////////////////////////////////////////////////////
template<>
struct zr_unroller<1>
{
  template<typename Return, typename Zip, typename Reduce, typename ...Iters>
  static void apply(Return& ret, Zip& zip_op, Reduce& red_op, Iters&... iters)
  {
    helpers::reduce_assign(ret, red_op(ret, zip_op(*(++iters)...)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A @c zip_reduce optimizer is used in the cases that both
/// views are local and their traversal would be fast which improves the
/// performance.  Tagged with @ref tags::sequential_unroll allows directs
/// the optimizer to unroll the sequential loop with the specified factor.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <int arity, std::size_t factor>
struct optimizer<tags::zip_reduce<arity>, tags::sequential_unroll<factor>>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using type = OutputValueType;
  };

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of sequential execution when @ref paragraph_view
  /// is required by the zip operation
  //////////////////////////////////////////////////////////////////////
  struct dynamic
  {
    template <typename R, typename S,
              typename TGV,
              typename IterComp, typename ...Iter>
    static R apply(S&& skeleton, TGV tgv,
                   IterComp& iter_compare, Iter... iter)
    {
      auto red_op = skeleton.get_reduce_op();
      auto zip_op = skeleton.get_zip_op();
      R result = zip_op(tgv, *(iter++)...);
      for (; iter_compare(iter...); helpers::no_op(++iter...)) {
        helpers::reduce_assign(
          result,
          red_op(result, zip_op(tgv, *(iter)...)));
      }
      return result;
    }

    template <typename R, typename S, typename TGV,
              typename V0, typename... V>
    static R execute(S&& skeleton, TGV tgv,
                     V0&& view0, V&&... view)
    {
      stapl_assert(view0.size() > 0, "empty views cannot be zip-reduced");
      wf_iter_compare<
        typename std::decay<V0>::type,
        typename std::decay<V>::type...> iter_compare(
          std::forward<V0>(view0),
          std::forward<V>(view)...);
      return apply<R>(std::forward<S>(skeleton), tgv, iter_compare,
                      view0.begin(), view.begin()...);
    }
  }; // nested struct dynamic


  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of sequential execution when @ref paragraph_view
  /// is not required by the zip operation
  //////////////////////////////////////////////////////////////////////
  struct non_dynamic
  {
    template <typename R, typename S,
              typename IterComp, typename ...Iter>
    static R apply_iterator(S&& skeleton, std::size_t size,
                            IterComp& iter_compare, Iter... iter)
    {
      auto red_op       = skeleton.get_reduce_op();
      auto zip_op       = skeleton.get_zip_op();

      R result = zip_op(*(iter)...);

      if (size > 1)
      {
        for (size_t trip_count = (size-1)/factor; trip_count > 0; --trip_count)
          zr_unroller<factor>::apply(result, zip_op, red_op, iter...);

        for (size_t leftovers = (size-1)%factor; leftovers > 0; --leftovers)
          helpers::reduce_assign(result, red_op(result, zip_op(*(++iter)...)));
      }

      return result;
    }


    template <typename R, typename S, typename... Tuple>
    static R apply_domain(S&& skeleton,
                          std::size_t size, Tuple... t)
    {
      auto red_op = skeleton.get_reduce_op();
      auto zip_op = skeleton.get_zip_op();
      R result = zip_op(
        helpers::referencer(std::get<0>(t), std::get<2>(t))...
      );

      helpers::no_op(
        helpers::advance_domain(std::get<1>(t), std::get<2>(t))...);

      for (std::size_t i = 0; i < size-1; ++i, helpers::no_op(
        helpers::advance_domain(std::get<1>(t), std::get<2>(t))...))
        helpers::reduce_assign(result, red_op(result,
          zip_op(helpers::referencer(std::get<0>(t), std::get<2>(t))...)
        ));

      return result;
    }

    template <typename R, typename S, typename... V>
    static R apply(std::false_type, S&& skeleton, V&&... v)
    {
      const std::size_t size = helpers::view_size(std::forward<V>(v)...);

      return apply_domain<R>(std::forward<S>(skeleton), size,
        stapl::make_tuple(v, v.domain(), v.domain().first())...
      );
    }

    template <typename R, typename S,
              typename V0, typename...V>
    static R apply(std::true_type, S&& skeleton,
                   V0&& view0, V&&... view)
    {
      stapl_assert(view0.size() > 0, "empty views cannot be zip-reduced");
      wf_iter_compare<
        typename std::decay<V0>::type,
        typename std::decay<V>::type...> iter_compare(
          std::forward<V0>(view0),
          std::forward<V>(view)...);

      return apply_iterator<R>(
        std::forward<S>(skeleton),
        helpers::view_size(std::forward<V0>(view0), std::forward<V>(view)...),
        iter_compare,
        view0.begin(), view.begin()...
      );
    }

    template <typename R, typename S, typename... V>
    static R execute(S&& skeleton, V&&... v)
    {
      return apply<R>(
        typename helpers::pack_has_iterator<
          typename std::decay<V>::type...
        >::type(),
        std::forward<S>(skeleton), std::forward<V>(v)...);
    }
  }; // nested struct non_dynamic

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sequential zip_reduce optimizer dispatches the execution to
  /// two different signatures based on the requirement of the given
  /// zip operation. If the zip operations needs the @ref paragraph_view
  /// the dynamic specialization is called, otherwise the non_dynamic
  /// one is called.
  ///
  /// @return the result of the zip_reduce operation (can be void)
  //////////////////////////////////////////////////////////////////////
  template <typename R, typename S, typename... Args>
  static R execute(S&& skeleton, Args&&... args)
  {
    using dispatcher_t = typename std::conditional<
                           std::is_base_of<
                             dynamic_wf,
                             typename std::decay<S>::type::zip_op_type>::value,
                           dynamic,
                           non_dynamic>::type;
    return dispatcher_t::template execute<R>(
             std::forward<S>(skeleton),
             std::forward<Args>(args)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A @c zip_reduce optimizer is used in the cases that both
/// views are local and their traversal would be fast which improves the
/// performance.
///
/// Calls the unrolled variant with factor = 1.
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <int arity>
struct optimizer<tags::zip_reduce<arity>, tags::sequential_execution>
  : optimizer<tags::zip_reduce<arity>, tags::sequential_unroll<1>>
{ };

} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPTIMIZERS_ZIP_REDUCE_HPP
