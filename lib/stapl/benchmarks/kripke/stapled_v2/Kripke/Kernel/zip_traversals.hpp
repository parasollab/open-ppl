/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef KRIPKE_ZIP_TRAVERSALS_HPP
#define KRIPKE_ZIP_TRAVERSALS_HPP

#include <stapl/skeletons/transformations/optimizers/zip_traversals.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {


//////////////////////////////////////////////////////////////////////
/// @brief Zip optimizer traversal methods for when the views have iterators
//////////////////////////////////////////////////////////////////////
struct kripke_zipper_iterator
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
      auto&& zip_op = skeleton.get_op();
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
      auto&& zip_op = skeleton.get_op();
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
      using plane_t = typename R::value_type;

      plane_t result0(size);
      plane_t result1(size);
      plane_t result2(size);

      auto&& zip_op = skeleton.get_op();

      for (size_t i = 0; iter_compare(iter...); helpers::no_op(++iter...), ++i)
      {
        auto res = zip_op(*(iter)...);

        result0[i] = std::move(std::get<0>(res));
        result1[i] = std::move(std::get<1>(res));
        result2[i] = std::move(std::get<2>(res));
      }

      return {result0, result1, result2};
    }

    template <typename R, typename S,
              typename IterComp, typename ...Iter>
    static R apply_iterator(std::true_type, S&& skeleton,
                            std::size_t size,
                            IterComp& iter_compare, Iter... iter)
    {
      auto&& zip_op = skeleton.get_op();
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
struct kripke_zipper_domain
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
      auto&& zip_op = skeleton.get_op();

      for (std::size_t i = 0; i < size; ++i, helpers::no_op(
        helpers::advance_domain(std::get<1>(t), std::get<2>(t))...))
        zip_op(helpers::referencer(std::get<0>(t), std::get<2>(t))...);
    }

    template <typename R, typename S, typename... Tuple>
    static R apply_domain(std::false_type, S&& skeleton,
                          std::size_t size, Tuple... t)
    {
      using plane_t = typename R::value_type;
      plane_t result0(size);
      plane_t result1(size);
      plane_t result2(size);

      auto&& zip_op = skeleton.get_op();

      for (std::size_t i = 0; i < size; ++i, helpers::no_op(
        helpers::advance_domain(std::get<1>(t), std::get<2>(t))...))
      {
        auto&& res =
          zip_op(helpers::referencer(std::get<0>(t), std::get<2>(t))...);

        result0[i] = std::move(std::get<0>(res));
        result1[i] = std::move(std::get<1>(res));
        result2[i] = std::move(std::get<2>(res));
      }

      return {result0, result1, result2};
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
struct kripke_zipper_slice_loop_nest
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
      auto&& zip_op = skeleton.get_op();

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
        return kripke_zipper_domain::non_dynamic::execute<R>(
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

#endif // KRIPKE_ZIP_TRAVERSALS_HPP
