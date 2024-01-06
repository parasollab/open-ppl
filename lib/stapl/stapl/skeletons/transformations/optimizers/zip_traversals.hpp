/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPTIMIZERS_ZIP_TRAVERSALS_HPP
#define STAPL_SKELETONS_OPTIMIZERS_ZIP_TRAVERSALS_HPP

#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/wf_iter_compare.hpp>
#include <stapl/skeletons/transformations/optimizers/utils.hpp>
#include <stapl/utility/tuple/for_each.hpp>
#include <stapl/utility/tuple/index_of_max.hpp>
#include <stapl/utility/iterator.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/views/type_traits/underlying_container.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to check if the first element of the first
///        param is less than or equal to the first element of the
///        second param
//////////////////////////////////////////////////////////////////////
template<typename First, typename Last>
bool first_less_equal(First&& first, Last&& last) {
  return get<0>(first) <= get<0>(last);
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to increment a single element.
//////////////////////////////////////////////////////////////////////
struct increment_by_one
{
  template<typename T>
  void operator()(T& x) const
  {
    ++x;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to retrieve the traversal type of the container
///        that is at the bottom of a view composition.
//////////////////////////////////////////////////////////////////////
template<typename View>
using basic_traversal_type =
  typename underlying_container_t<
    typename std::decay<View>::type
  >::traversal_type;


//////////////////////////////////////////////////////////////////////
/// @brief Helper class to iterate over a set of views one dimension at
///        time, slicing at each dimension, and applying a function on
///        view elements.
//////////////////////////////////////////////////////////////////////
template<int Current, typename ViewsIndexSequence>
struct static_recurse_and_invoke;


//////////////////////////////////////////////////////////////////////
/// @tparam Current The dimension that is currently being iterated over
/// @tparam Indices A pack of (0 ... num_views-1)
//////////////////////////////////////////////////////////////////////
template<int Current, std::size_t... Indices>
struct static_recurse_and_invoke<Current, index_sequence<Indices...>>
{
  template<typename F, typename... Views>
  static void apply(F&& f, Views&&... view)
  {
    // The scalar indices representing the first index for each view
    // in the next dimension to slice
    auto firsts = std::make_tuple(
        get<tuple_ops::index_of_max<basic_traversal_type<Views>>::value>(
          std::forward<Views>(view).domain().first()
        )...
      );

    // The scalar indices representing the last index for each view
    // in the next dimension to slice
    auto lasts = std::make_tuple(
        get<tuple_ops::index_of_max<basic_traversal_type<Views>>::value>(
          std::forward<Views>(view).domain().last()
        )...
      );

    // Slice along the outermost dimension for each view and recursively call
    // for the next dimension
    for (; first_less_equal(firsts, lasts);
           tuple_ops::for_each(firsts, increment_by_one{}))
    {
      static_recurse_and_invoke<Current-1, index_sequence<Indices...>>::apply(
        std::forward<F>(f),
        std::forward<Views>(view).template slice<std::tuple<
          std::integral_constant<std::size_t,
            tuple_ops::index_of_max<basic_traversal_type<Views>>::value
          >>>(get<Indices>(firsts))...
      );
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for single dimension (last dimension)
//////////////////////////////////////////////////////////////////////
template<std::size_t... Indices>
struct static_recurse_and_invoke<0, index_sequence<Indices...>>
{
  template<typename F, typename... Views>
  static void apply(F&& f, Views&&... view)
  {
    auto firsts =
      std::make_tuple(std::forward<Views>(view).domain().first()...);
    auto lasts =
      std::make_tuple(std::forward<Views>(view).domain().last()...);

    for (; first_less_equal(firsts, lasts);
           tuple_ops::for_each(firsts, increment_by_one{}))
    {
      std::forward<F>(f)(std::forward<Views>(view)[get<Indices>(firsts)]...);
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Zip optimizer traversal methods for when the views have iterators
//////////////////////////////////////////////////////////////////////
struct zipper_iterator
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of sequential execution when @ref paragraph_view
  /// is required by the zip operation
  //////////////////////////////////////////////////////////////////////
  struct dynamic
  {
    template <typename R, typename S, typename TGV,
              typename IterComp, typename ...Iter>
    static R apply(std::false_type, S&& skeleton, TGV tgv,
                   std::size_t size, IterComp& iter_compare, Iter... iter)
    {
      R result;
      result.reserve(size);
      auto zip_op = skeleton.get_op();
      for (; iter_compare(iter...); helpers::no_op(++iter...)) {
        result.push_back(tgv, zip_op(*(iter)...));
      }
      return result;
    }

    template <typename R, typename S, typename TGV,
              typename IterComp, typename ...Iter>
    static R apply(std::true_type, S&& skeleton, TGV tgv,
                   std::size_t, IterComp& iter_compare, Iter... iter)
    {
      auto zip_op = skeleton.get_op();
      for (; iter_compare(iter...); helpers::no_op(++iter...)) {
        zip_op(tgv, *(iter)...);
      }
    }

    template <typename R, typename S, typename TGV,
              typename V0, typename... V>
    static R execute(S&& skeleton, TGV tgv,
                     V0&& view0, V&&... view)
    {
      wf_iter_compare<
        typename std::decay<V0>::type,
        typename std::decay<V>::type...> iter_compare(
          std::forward<V0>(view0),
          std::forward<V>(view)...);
      return apply<R>(typename std::is_same<R, void>::type(),
                      std::forward<S>(skeleton), tgv,
                      view0.size(), iter_compare,
                      stapl::begin(view0), stapl::begin(view)...);
    }
  }; // nested struct dynamic

  struct non_dynamic
  {
    template <typename R, typename S,
              typename IterComp, typename ...Iter>
    static R apply_iterator(std::false_type, S&& skeleton,
                            std::size_t size,
                            IterComp& iter_compare, Iter... iter)
    {
      R result;
      result.reserve(size);
      auto zip_op = skeleton.get_op();
      for (; iter_compare(iter...); helpers::no_op(++iter...)) {
        result.push_back(zip_op(*(iter)...));
      }
      return result;
    }

    template <typename R, typename S,
              typename IterComp, typename ...Iter>
    static R apply_iterator(std::true_type, S&& skeleton,
                            std::size_t size,
                            IterComp& iter_compare, Iter... iter)
    {
      auto zip_op = skeleton.get_op();
      for (; iter_compare(iter...); helpers::no_op(++iter...)) {
        zip_op(*(iter)...);
      }
    }

    template <typename R, typename S, typename V0, typename... V>
    static R execute(S&& skeleton, V0&& view0, V&&... view)
    {
      wf_iter_compare<typename std::decay<V0>::type,
                      typename std::decay<V>::type...>
        iter_compare(std::forward<V0>(view0), std::forward<V>(view)...);

      return apply_iterator<R>(typename std::is_same<R, void>::type(),
                               std::forward<S>(skeleton),
                               view0.size(), iter_compare,
                               stapl::begin(view0),
                               stapl::begin(view)...);
    }
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief Zip optimizer traversal methods for when the views do not
///        have iterators. Uses general domain traversal instead.
//////////////////////////////////////////////////////////////////////
struct zipper_domain
{
  struct dynamic { };

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of sequential execution when @ref paragraph_view
  /// is not required by the zip operation
  //////////////////////////////////////////////////////////////////////
  struct non_dynamic
  {
    template <typename R, typename S, typename... Tuple>
    static R apply_domain(std::true_type, S&& skeleton, std::size_t size,
                          Tuple&&... t)
    {
      auto zip_op = skeleton.get_op();

      for (std::size_t i = 0; i < size; ++i, helpers::no_op(
        helpers::advance_domain(std::get<1>(t), std::get<2>(t))...))
        zip_op(helpers::referencer(std::get<0>(t), std::get<2>(t))...);
    }

    template <typename R, typename S, typename... Tuple>
    static R apply_domain(std::false_type, S&& skeleton,
                          std::size_t size, Tuple... t)
    {
      R result;
      result.reserve(size);
      auto zip_op = skeleton.get_op();

      for (std::size_t i = 0; i < size; ++i, helpers::no_op(
        helpers::advance_domain(std::get<1>(t), std::get<2>(t))...))
      {
        result.push_back(
          zip_op(helpers::referencer(std::get<0>(t), std::get<2>(t))...)
        );
      }

      return result;
    }

    template <typename R, typename S, typename... V>
    static R execute(S&& skeleton, V&&... v)
    {
      const std::size_t size = helpers::view_size(std::forward<V>(v)...);

      return non_dynamic::apply_domain<R>(
        typename std::is_same<R, void>::type(),
        std::forward<S>(skeleton), size,
        std::forward_as_tuple(v, v.domain(), v.domain().first())...
      );
    }
  }; // nested struct non_dynamic
};


//////////////////////////////////////////////////////////////////////
/// @brief Zip optimizer traversal methods for when the views do not
///        have iterators, but are deep sliceable.
//////////////////////////////////////////////////////////////////////
struct zipper_slice_loop_nest
{
  struct dynamic { };

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of sequential execution when @ref paragraph_view
  /// is not required by the zip operation
  //////////////////////////////////////////////////////////////////////
  struct non_dynamic
  {
    template <typename R, int Dims, typename S, typename... Tuple>
    static R apply_loop_nest(std::true_type, S&& skeleton,
                             Tuple&&... t)
    {
      auto zip_op = skeleton.get_op();

      using recurser_t = static_recurse_and_invoke<
        Dims-1, make_index_sequence<sizeof...(Tuple)>
      >;

      stapl_assert(pack_ops::functional::and_(
        get<1>(t).size() == helpers::view_size(get<0>(t)...)...),
        "Coarsened views are not the same size");

      recurser_t::apply(zip_op, get<0>(t)...);
    }

    template <typename R, typename S, typename... V>
    static R execute(S&& skeleton, V&&... v)
    {
      constexpr std::size_t num_dims =
        helpers::dimensions_of_first_view<V...>::type::value;

      // Can't do slicing if this is a subview of a base container.
      // Fall back to old way.
      if (!pack_ops::functional::and_(
            v.domain().size() == v.container().size()...))
        return zipper_domain::non_dynamic::execute<R>(
          std::forward<S>(skeleton),
          std::forward<V>(v)...
        );

      return non_dynamic::apply_loop_nest<R, num_dims>(
        typename std::is_same<R, void>::type(),
        std::forward<S>(skeleton),
        std::forward_as_tuple(v, v.domain())...
      );
    }
  }; // nested struct non_dynamic
};

} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPTIMIZERS_ZIP_HPP
