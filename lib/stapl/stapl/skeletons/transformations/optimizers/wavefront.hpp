/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_WAVEFRONT_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_WAVEFRONT_HPP

#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>
#include <stapl/skeletons/transformations/optimizers/nested.hpp>

#include "is_deep_sliceable.hpp"

namespace stapl {
namespace skeletons {


template <typename SkeletonTag, typename ExecutionTag>
struct skeleton_execution_traits;


template <int dims>
struct skeleton_execution_traits<
         tags::wavefront<dims>, tags::sequential_execution>
{
  template <typename OutputValueType>
  using result_type = lightweight_multiarray<OutputValueType, dims>;
};


namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A sequential optimizer of a 2D wavefront is used in the cases
/// that all the arguments are local.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <>
struct optimizer<tags::wavefront<2>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using type = lightweight_multiarray<OutputValueType, 2>;
  };

  template <typename R, typename S,
            typename... Views>
  static R execute(S&& skeleton, Views&&... views)
  {
    return apply_reorder<R>(std::forward<S>(skeleton),
                            std::forward_as_tuple(views...),
                            make_index_sequence<
                              std::decay<S>::type::number_of_inputs>());
  }

private:
  template <typename R, typename S,
            typename ViewsTuple, std::size_t... Indices>
  static R apply_reorder(S&& skeleton, ViewsTuple&& views,
                         index_sequence<Indices...>&&)
  {
    return apply_execute<R>(std::forward<S>(skeleton),
                            get<sizeof...(Indices)>(views),
                            get<sizeof...(Indices)+1>(views),
                            get<Indices>(views)...);
  }

  template <typename R, typename S,
            typename BoundaryView0,
            typename BoundaryView1,
            typename View0,
            typename... Views>
  static R apply_execute(S&& skeleton,
                         BoundaryView0&& bv0,
                         BoundaryView1&& bv1,
                         View0&& view0,
                         Views&&... views)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();
    auto f_d0 = get_wavefront_filter(skeleton.get_filter(),
                                     direction::direction0);
    auto f_d1 = get_wavefront_filter(skeleton.get_filter(),
                                     direction::direction1);

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first  = view0.domain().first();

    auto&& bv0_first = bv0.domain().first();
    auto&& bv1_first = bv1.domain().first();

    auto const bv0_first_i = get<0>(bv0_first);
    auto const bv1_first_i = get<0>(bv1_first);
    auto const bv0_first_j = get<1>(bv0_first);
    auto const bv1_first_j = get<1>(bv1_first);

    int d0, d1;
    std::size_t start_i, start_j,
                  end_i,   end_j;
    std::tie(     d0,      d1) = wavefront_direction(start_corner);
    std::tie(start_i, start_j) = first_index(start_corner, dimensions);
    std::tie(  end_i,   end_j) =  last_index(start_corner, dimensions);

    // the end indices are inclusive. We need to adjust them a priori
    end_i -= d0;
    end_j -= d1;

    using filters::apply_filter;
    R result(dimensions);
    // the rest of the domain
    for (std::size_t i = start_i; i != end_i; i -= d0)
    {
      for (std::size_t j = start_j; j != end_j; j -= d1)
      {
        auto dep0 = (i == start_i) ?
                         bv0(bv0_first_i, bv0_first_j + j)
                       : apply_filter(f_d0, result(i + d0, j));
        auto dep1 = (j == start_j) ?
                         bv1(bv1_first_i + i, bv1_first_j)
                       : apply_filter(f_d1, result(i, j + d1));
        result(i, j) =
          op( view0( i + get<0>(v0_first),
                     j + get<1>(v0_first) ),
              views( i + get<0>(views.domain().first()),
                     j + get<1>(views.domain().first()) )...,
              dep0, dep1 );
      }
    }

    return result;
  }

};

//////////////////////////////////////////////////////////////////////
/// @brief A sequential optimizer of a 3D wavefront is used in the cases
/// that all the arguments are local.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <>
struct optimizer<tags::wavefront<3>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using type = lightweight_multiarray<OutputValueType, 3>;
  };

  template <typename R, typename S,
            typename... Views>
  static R execute(S&& skeleton, Views&&... views)
  {
    return apply_reorder<R>(std::forward<S>(skeleton),
                            std::forward_as_tuple(views...),
                            make_index_sequence<
                              std::decay<S>::type::number_of_inputs>());
  }

private:
  template <typename R, typename S,
            typename ViewsTuple, std::size_t... Indices>
  static R apply_reorder(S&& skeleton, ViewsTuple&& views,
                         index_sequence<Indices...>&&)
  {
    return apply_execute<R>(std::forward<S>(skeleton),
                            get<sizeof...(Indices)>(views),
                            get<sizeof...(Indices)+1>(views),
                            get<sizeof...(Indices)+2>(views),
                            get<Indices>(views)...);
  }

  template <typename R, typename S,
            typename BoundaryView0,
            typename BoundaryView1,
            typename BoundaryView2,
            typename View0,
            typename... Views>
  static R apply_execute(S&& skeleton,
                         BoundaryView0&& bv0,
                         BoundaryView1&& bv1,
                         BoundaryView2&& bv2,
                         View0&& view0,
                         Views&&... views)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();
    auto f_d0 = get_wavefront_filter(skeleton.get_filter(),
                                     direction::direction0);
    auto f_d1 = get_wavefront_filter(skeleton.get_filter(),
                                     direction::direction1);
    auto f_d2 = get_wavefront_filter(skeleton.get_filter(),
                                     direction::direction2);

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first  = view0.domain().first();

    auto&& bv0_first = bv0.domain().first();
    auto&& bv1_first = bv1.domain().first();
    auto&& bv2_first = bv2.domain().first();

    auto const bv0_first_i = get<0>(bv0_first);
    auto const bv1_first_i = get<0>(bv1_first);
    auto const bv2_first_i = get<0>(bv2_first);

    auto const bv0_first_j = get<1>(bv0_first);
    auto const bv1_first_j = get<1>(bv1_first);
    auto const bv2_first_j = get<1>(bv2_first);

    auto const bv0_first_k = get<2>(bv0_first);
    auto const bv1_first_k = get<2>(bv1_first);
    auto const bv2_first_k = get<2>(bv2_first);

    int d0, d1, d2;
    std::size_t start_i, start_j, start_k,
                  end_i,   end_j,   end_k;
    std::tie(     d0,      d1,      d2) = wavefront_direction(start_corner);
    std::tie(start_i, start_j, start_k) = first_index(start_corner, dimensions);
    std::tie(  end_i,   end_j,   end_k) =  last_index(start_corner, dimensions);

    // the end indices are inclusive. We need to adjust them a priori
    end_i -= d0;
    end_j -= d1;
    end_k -= d2;

    using filters::apply_filter;
    R result(dimensions);
    // the rest of the domain
    for (std::size_t i = start_i; i != end_i; i -= d0)
    {
      for (std::size_t j = start_j; j != end_j; j -= d1)
      {
        for (std::size_t k = start_k; k != end_k; k -= d2)
        {
          auto dep0 =
            (i == start_i) ?
              bv0(bv0_first_i, bv0_first_j + j, bv0_first_k + k)
            : apply_filter(f_d0, result(i+d0,    j,    k));
          auto dep1 =
            (j == start_j) ?
              bv1(bv1_first_i + i, bv1_first_j, bv1_first_k + k)
            : apply_filter(f_d1, result(   i, j+d1,    k));
          auto dep2 =
            (k == start_k) ?
              bv2(bv2_first_i + i, bv2_first_j + j, bv2_first_k)
            : apply_filter(f_d2, result(   i,    j, k+d2));

          result(i, j, k) =
            op( view0( i + get<0>(v0_first),
                       j + get<1>(v0_first),
                       k + get<2>(v0_first) ),
                views( i + get<0>(views.domain().first()),
                       j + get<1>(views.domain().first()),
                       k + get<2>(views.domain().first()) )...,
                dep0, dep1, dep2 );
        }
      }
    }

    return result;
  }
};


template <typename T>
struct compute_bndary_t
{
  using type = T;
};

template <typename T, typename A>
struct compute_bndary_t<proxy<T, A>>
{
  using type = T;
};

template <size_t dir, typename BndaryView>
struct get_bndary
{
  static BndaryView apply(BndaryView const& view)
  {
    return view;
  }
};

template <size_t dir, size_t dim, typename T>
struct get_bndary<dir, std::array<T, dim>>
{
  static T apply(std::array<T, dim> const& view)
  {
    return view[dir];
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Slices the @c current dimension of the @c views passed using the
///        @c cur_idx for the @current dimension.
///
//////////////////////////////////////////////////////////////////////
template <size_t current>
struct slice_views
{
  template <typename FirstsTuple, typename ViewsTuple, std::size_t... Indices>
  static auto apply(size_t cur_idx, FirstsTuple&& firsts, ViewsTuple&& views,
                    index_sequence<Indices...> const&)
    -> decltype(std::make_tuple(
      get<Indices>(views)
        .template slice<std::tuple<std::integral_constant<std::size_t, 0>>>(
          cur_idx + std::get<current>(get<Indices>(firsts)))...))
  {
    return std::make_tuple(
      get<Indices>(views)
        .template slice<std::tuple<std::integral_constant<std::size_t, 0>>>(
          cur_idx + std::get<current>(get<Indices>(firsts)))...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A sequential optimizer of a 3D wavefront is used in the cases
/// when only the boundary planes are needed as output and the input views
/// are not slice-able.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <>
struct optimizer<tags::wavefront<2>, tags::sequential_execution_partial>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using value_t = OutputValueType;
    using type    = std::array<lightweight_multiarray<value_t, 2>, 2>;
  };


  template <typename R, typename S,
            typename... Views>
  static R execute(S&& skeleton, Views&&... views)
  {
    return apply_reorder<R>(std::forward<S>(skeleton),
                            std::forward_as_tuple(views...),
                            make_index_sequence<
                              std::decay<S>::type::number_of_inputs>());
  }

private:
  template <typename R, typename S,
            typename ViewsTuple, std::size_t... Indices>
  static R apply_reorder(S&& skeleton, ViewsTuple&& views,
                         index_sequence<Indices...>&&)
  {
    return apply_execute<R>(
      pack_is_deep_sliceable<
        typename tuple_element<Indices, ViewsTuple>::type...>(),
      make_index_sequence<std::decay<S>::type::number_of_inputs - 1>(),
      std::forward<S>(skeleton),
      get<sizeof...(Indices)>(views),
      get<sizeof...(Indices) + 1>(views),
      get<Indices>(views)...);
  }

  template <typename R,
            std::size_t... Indices,
            typename S,
            typename BoundaryView0,
            typename BoundaryView1,
            typename View0,
            typename... Views
            >
  static R apply_execute(std::false_type,
                         index_sequence<Indices...>&&,
                         S&& skeleton,
                         BoundaryView0&& bndary_v0,
                         BoundaryView1&& bndary_v1,
                         View0&& view0,
                         Views&&... views)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first   = view0.domain().first();

    using bndary0_t =
      typename compute_bndary_t<typename std::decay<BoundaryView0>::type>::type;
    using bndary1_t =
      typename compute_bndary_t<typename std::decay<BoundaryView1>::type>::type;

    bndary0_t bndary0 = bndary_v0;
    bndary1_t bndary1 = bndary_v1;

    auto&& bv0 = get_bndary<0, bndary0_t>::apply(bndary0);
    auto&& bv1 = get_bndary<1, bndary1_t>::apply(bndary1);

    auto&& bv0_first = bv0.domain().first();
    auto&& bv1_first = bv1.domain().first();

    auto const bv0_first_i = get<0>(bv0_first);
    auto const bv1_first_i = get<0>(bv1_first);

    auto const bv0_first_j = get<1>(bv0_first);
    auto const bv1_first_j = get<1>(bv1_first);

    int d0, d1;
    std::size_t start_i, start_j,
                  end_i,   end_j;
    std::tie(     d0,      d1) = wavefront_direction(start_corner);
    std::tie(start_i, start_j) = first_index(start_corner, dimensions);
    std::tie(  end_i,   end_j) =  last_index(start_corner, dimensions);

    // the end indices are inclusive. We need to adjust them a priori
    end_i -= d0;
    end_j -= d1;

    using filters::apply_filter;

    using plane_t = typename R::value_type;
    plane_t i_plane_result(1, get<1>(dimensions));
    plane_t j_plane_result(get<0>(dimensions), 1);

    auto i_plane_slice_a = i_plane_result.template slice<0>(0);

    // the rest of the domain
    for (std::size_t i = start_i; i != end_i; i -= d0)
    {
      auto& j_point = j_plane_result(i, 0);

      for (std::size_t j = start_j; j != end_j; j -= d1)
      {
        auto ret =
          op(view0(i + get<0>(v0_first),
                   j + get<1>(v0_first)),
             views(i + get<0>(views.domain().first()),
                   j + get<1>(views.domain().first()))...,
             (i == start_i) ?
                bv0(bv0_first_i, bv0_first_j + j)
              : i_plane_slice_a(j),
             (j == start_j) ?
                bv1(bv1_first_i + i, bv1_first_j)
              : j_point);

          /// @todo: this line needs to be changed for the case when the
          ///       work-function returns different results for each direction
          i_plane_slice_a(j) = j_point = ret;
      }
    }

    // clang generates warning without double curly braces initialization
    return {{std::move(i_plane_result), std::move(j_plane_result)}};
  }

  template <typename R,
            std::size_t... Indices,
            typename S,
            typename BoundaryView0,
            typename BoundaryView1,
            typename View0,
            typename... Views
            >
  static R apply_execute(std::true_type,
                         index_sequence<Indices...>&& idx_seq,
                         S&& skeleton,
                         BoundaryView0&& bndary_v0,
                         BoundaryView1&& bndary_v1,
                         View0&& view0,
                         Views&&... views)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first   = view0.domain().first();
    auto&& vs_first   = std::make_tuple(views.domain().first()...);

    using bndary0_t =
      typename compute_bndary_t<typename std::decay<BoundaryView0>::type>::type;
    using bndary1_t =
      typename compute_bndary_t<typename std::decay<BoundaryView1>::type>::type;

    bndary0_t bndary0 = bndary_v0;
    bndary1_t bndary1 = bndary_v1;

    auto&& bv0 = get_bndary<0, bndary0_t>::apply(bndary0);
    auto&& bv1 = get_bndary<1, bndary1_t>::apply(bndary1);

    auto&& bv0_first = bv0.domain().first();
    auto&& bv1_first = bv1.domain().first();

    auto const bv0_first_i = get<0>(bv0_first);
    auto const bv1_first_i = get<0>(bv1_first);

    auto const bv0_first_j = get<1>(bv0_first);
    auto const bv1_first_j = get<1>(bv1_first);

    int d0, d1;
    std::size_t start_i, start_j,
                  end_i,   end_j;
    std::tie(     d0,      d1) = wavefront_direction(start_corner);
    std::tie(start_i, start_j) = first_index(start_corner, dimensions);
    std::tie(  end_i,   end_j) =  last_index(start_corner, dimensions);

    // the end indices are inclusive. We need to adjust them a priori
    end_i -= d0;
    end_j -= d1;

    using filters::apply_filter;

    using plane_t = typename R::value_type;
    plane_t i_plane_result(1, get<1>(dimensions));
    plane_t j_plane_result(get<0>(dimensions), 1);

    auto i_plane_slice_a = i_plane_result.template slice<0>(0);

    // the rest of the domain
    for (std::size_t i = start_i; i != end_i; i -= d0)
    {
      auto& j_point = j_plane_result(i, 0);

      auto view0_sliced_i =
        view0.template slice<
          std::tuple<std::integral_constant<std::size_t, 0>>
        >(i + std::get<0>(v0_first));

      auto&& views_sliced_i =
        slice_views<0>::apply(i, vs_first, std::make_tuple(views...), idx_seq);

      for (std::size_t j = start_j; j != end_j; j -= d1)
      {
        auto ret = op(
          view0_sliced_i[j + get<1>(v0_first)],
          get<Indices>(views_sliced_i)[j + get<1>(views.domain().first())]...,
          (i == start_i) ? bv0(bv0_first_i, bv0_first_j + j)
                         : i_plane_slice_a(j),
          (j == start_j) ? bv1(bv1_first_i + i, bv1_first_j)
                         : j_point);

          /// @todo: this line needs to be changed for the case when the
          ///       work-function returns different results for each direction
          i_plane_slice_a(j) = j_point = ret;
      }
    }

    // clang generates warning without double curly braces initialization
    return {{std::move(i_plane_result), std::move(j_plane_result)}};
  }

}; // optimizer<tags::wavefront<2>, tags::sequential_execution_partial>


//////////////////////////////////////////////////////////////////////
/// @brief A sequential optimizer of a 3D wavefront is used in the cases
/// when only the boundary planes are needed as output and the input views
/// are not slice-able.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <>
struct optimizer<tags::wavefront<3>, tags::sequential_execution_partial>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using value_t = OutputValueType;
    using type    = std::array<lightweight_multiarray<value_t, 3>, 3>;
  };

  template <typename R, typename S,
            typename... Views>
  static R execute(S&& skeleton, Views&&... views)
  {
    return apply_reorder<R>(std::forward<S>(skeleton),
                            std::forward_as_tuple(views...),
                            make_index_sequence<
                              std::decay<S>::type::number_of_inputs>());
  }

private:
  template <typename R, typename S,
            typename ViewsTuple, std::size_t... Indices>
  static R apply_reorder(S&& skeleton, ViewsTuple&& views,
                         index_sequence<Indices...>&& input_indices)
  {
    return apply_execute<R>(
      pack_is_deep_sliceable<
        typename tuple_element<Indices, ViewsTuple>::type...>(),
      make_index_sequence<std::decay<S>::type::number_of_inputs - 1>(),
      std::forward<S>(skeleton),
      get<sizeof...(Indices)>(views),
      get<sizeof...(Indices) + 1>(views),
      get<sizeof...(Indices) + 2>(views),
      get<Indices>(views)...);
  }

  template <typename R,
            std::size_t... Indices,
            typename S,
            typename BoundaryView0,
            typename BoundaryView1,
            typename BoundaryView2,
            typename View0,
            typename... Views
            >
  static R apply_execute(std::false_type,
                         index_sequence<Indices...>&&,
                         S&& skeleton,
                         BoundaryView0&& bndary_v0,
                         BoundaryView1&& bndary_v1,
                         BoundaryView2&& bndary_v2,
                         View0&& view0,
                         Views&&... views)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first   = view0.domain().first();

    using bndary0_t =
      typename compute_bndary_t<typename std::decay<BoundaryView0>::type>::type;
    using bndary1_t =
      typename compute_bndary_t<typename std::decay<BoundaryView1>::type>::type;
    using bndary2_t =
      typename compute_bndary_t<typename std::decay<BoundaryView2>::type>::type;

    bndary0_t bndary0 = bndary_v0;
    bndary1_t bndary1 = bndary_v1;
    bndary2_t bndary2 = bndary_v2;

    auto&& bv0 = get_bndary<0, bndary0_t>::apply(bndary0);
    auto&& bv1 = get_bndary<1, bndary1_t>::apply(bndary1);
    auto&& bv2 = get_bndary<2, bndary2_t>::apply(bndary2);

    auto&& bv0_first = bv0.domain().first();
    auto&& bv1_first = bv1.domain().first();
    auto&& bv2_first = bv2.domain().first();

    auto const bv0_first_i = get<0>(bv0_first);
    auto const bv1_first_i = get<0>(bv1_first);
    auto const bv2_first_i = get<0>(bv2_first);

    auto const bv0_first_j = get<1>(bv0_first);
    auto const bv1_first_j = get<1>(bv1_first);
    auto const bv2_first_j = get<1>(bv2_first);

    auto const bv0_first_k = get<2>(bv0_first);
    auto const bv1_first_k = get<2>(bv1_first);
    auto const bv2_first_k = get<2>(bv2_first);

    int d0, d1, d2;
    std::size_t start_i, start_j, start_k,
                  end_i,   end_j,   end_k;
    std::tie(     d0,      d1,      d2) = wavefront_direction(start_corner);
    std::tie(start_i, start_j, start_k) = first_index(start_corner, dimensions);
    std::tie(  end_i,   end_j,   end_k) =  last_index(start_corner, dimensions);

    // the end indices are inclusive. We need to adjust them a priori
    end_i -= d0;
    end_j -= d1;
    end_k -= d2;

    using filters::apply_filter;

    using plane_t = typename R::value_type;
    plane_t i_plane_result(1, get<1>(dimensions), get<2>(dimensions));
    plane_t j_plane_result(get<0>(dimensions), 1, get<2>(dimensions));
    plane_t k_plane_result(get<0>(dimensions), get<1>(dimensions), 1);

    auto i_plane_slice_a = i_plane_result.template slice<0>(0);

    // the rest of the domain
    for (std::size_t i = start_i; i != end_i; i -= d0)
    {
      auto j_plane_slice_a = j_plane_result.template slice<0>(i);
      auto k_plane_slice_a = k_plane_result.template slice<0>(i);

      for (std::size_t j = start_j; j != end_j; j -= d1)
      {
        auto i_plane_slice_b = i_plane_slice_a.template slice<0>(j);
        auto j_plane_slice_b = j_plane_slice_a.template slice<0>(0);
        auto& k_point        = k_plane_slice_a(j, 0);

        for (std::size_t k = start_k; k != end_k; k -= d2)
        {
          auto ret =
            op(view0(i + get<0>(v0_first),
                     j + get<1>(v0_first),
                     k + get<2>(v0_first)),
               views(i + get<0>(views.domain().first()),
                     j + get<1>(views.domain().first()),
                     k + get<2>(views.domain().first()) )...,
               (i == start_i) ?
                  bv0(bv0_first_i, bv0_first_j + j, bv0_first_k + k)
                   : i_plane_slice_b(k),
               (j == start_j) ?
                 bv1(bv1_first_i + i, bv1_first_j, bv1_first_k + k)
                  : j_plane_slice_b(k),
               (k == start_k) ?
                 bv2(bv2_first_i + i, bv2_first_j + j, bv2_first_k)
                  : k_point);

          /// @todo: this line needs to be changed for the case when the
          ///       work-function returns different results for each direction
          i_plane_slice_b(k) = j_plane_slice_b(k) = k_point = ret;
        }
      }
    }

    // clang generates warning without double curly braces initialization
    return {{std::move(i_plane_result),
             std::move(j_plane_result),
             std::move(k_plane_result)}};
  }

  template <typename R,
            std::size_t... Indices,
            typename S,
            typename BoundaryView0,
            typename BoundaryView1,
            typename BoundaryView2,
            typename View0,
            typename... Views>
  static R apply_execute(std::true_type,
                         index_sequence<Indices...>&& idx_seq,
                         S&& skeleton,
                         BoundaryView0&& bndary_v0,
                         BoundaryView1&& bndary_v1,
                         BoundaryView2&& bndary_v2,
                         View0&& view0,
                         Views&&... views)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first   = view0.domain().first();
    auto&& vs_first   = std::make_tuple(views.domain().first()...);

    using bndary0_t =
      typename compute_bndary_t<typename std::decay<BoundaryView0>::type>::type;
    using bndary1_t =
      typename compute_bndary_t<typename std::decay<BoundaryView1>::type>::type;
    using bndary2_t =
      typename compute_bndary_t<typename std::decay<BoundaryView2>::type>::type;

    bndary0_t bndary0 = bndary_v0;
    bndary1_t bndary1 = bndary_v1;
    bndary2_t bndary2 = bndary_v2;

    auto&& bv0 = get_bndary<0, bndary0_t>::apply(bndary0);
    auto&& bv1 = get_bndary<1, bndary1_t>::apply(bndary1);
    auto&& bv2 = get_bndary<2, bndary2_t>::apply(bndary2);

    auto&& bv0_first = bv0.domain().first();
    auto&& bv1_first = bv1.domain().first();
    auto&& bv2_first = bv2.domain().first();

    auto const bv0_first_i = get<0>(bv0_first);
    auto const bv1_first_i = get<0>(bv1_first);
    auto const bv2_first_i = get<0>(bv2_first);

    auto const bv0_first_j = get<1>(bv0_first);
    auto const bv1_first_j = get<1>(bv1_first);
    auto const bv2_first_j = get<1>(bv2_first);

    auto const bv0_first_k = get<2>(bv0_first);
    auto const bv1_first_k = get<2>(bv1_first);
    auto const bv2_first_k = get<2>(bv2_first);

    int d0, d1, d2;
    std::size_t start_i, start_j, start_k,
                  end_i,   end_j,   end_k;
    std::tie(     d0,      d1,      d2) = wavefront_direction(start_corner);
    std::tie(start_i, start_j, start_k) = first_index(start_corner, dimensions);
    std::tie(  end_i,   end_j,   end_k) =  last_index(start_corner, dimensions);

    // the end indices are inclusive. We need to adjust them a priori
    end_i -= d0;
    end_j -= d1;
    end_k -= d2;

    using filters::apply_filter;

    using plane_t = typename R::value_type;
    plane_t i_plane_result(1, get<1>(dimensions), get<2>(dimensions));
    plane_t j_plane_result(get<0>(dimensions), 1, get<2>(dimensions));
    plane_t k_plane_result(get<0>(dimensions), get<1>(dimensions), 1);

    auto i_plane_slice_a = i_plane_result.template slice<0>(0);

    // the rest of the domain
    for (std::size_t i = start_i; i != end_i; i -= d0)
    {
      auto j_plane_slice_a = j_plane_result.template slice<0>(i);
      auto k_plane_slice_a = k_plane_result.template slice<0>(i);

      auto view0_sliced_i =
        view0.template slice<
          std::tuple<std::integral_constant<std::size_t, 0>>
        >(i + std::get<0>(v0_first));

      auto&& views_sliced_i =
        slice_views<0>::apply(i, vs_first, std::make_tuple(views...), idx_seq);

      for (std::size_t j = start_j; j != end_j; j -= d1)
      {
        auto i_plane_slice_b = i_plane_slice_a.template slice<0>(j);
        auto j_plane_slice_b = j_plane_slice_a.template slice<0>(0);
        auto& k_point        = k_plane_slice_a(j, 0);

        auto view0_sliced_i_j =
          view0_sliced_i.template slice<
            std::tuple<std::integral_constant<std::size_t, 0>>
          >(j + std::get<1>(v0_first));

        auto&& views_sliced_i_j =
          slice_views<1>::apply(j, vs_first, views_sliced_i, idx_seq);

        for (std::size_t k = start_k; k != end_k; k -= d2)
        {
          auto ret = op(
            view0_sliced_i_j[k + get<2>(v0_first)],
            get<Indices>(
              views_sliced_i_j)[k + get<2>(views.domain().first())]...,
            (i == start_i) ? bv0(bv0_first_i, bv0_first_j + j, bv0_first_k + k)
                           : i_plane_slice_b(k),
            (j == start_j) ? bv1(bv1_first_i + i, bv1_first_j, bv1_first_k + k)
                           : j_plane_slice_b(k),
            (k == start_k) ? bv2(bv2_first_i + i, bv2_first_j + j, bv2_first_k)
                           : k_point);

          /// @todo: this line needs to be changed for the case when the
          ///       work-function returns different results for each direction
          i_plane_slice_b(k) = j_plane_slice_b(k) = k_point = ret;
        }
      }
    }

    // clang generates warning without double curly braces initialization
    return {{std::move(i_plane_result),
             std::move(j_plane_result),
             std::move(k_plane_result)}};
  }
}; // optimizer<tags::wavefront<3>, tags::sequential_execution_partial>


} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_WAVEFRONT_hpp
