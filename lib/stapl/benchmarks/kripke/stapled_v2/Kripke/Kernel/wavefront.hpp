
#ifndef KRIPKE_OPTIMIZERS_WAVEFRONT_HPP
#define KRIPKE_OPTIMIZERS_WAVEFRONT_HPP


#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>
#include <stapl/skeletons/transformations/optimizers/nested.hpp>

namespace stapl {
namespace skeletons {
namespace tags {

struct seq_exec_only_boundary {};

struct seq_exec_only_boundary_sliced {};

} // namespace tags

namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A sequential optimizer of a 3D wavefront is used in the cases
/// when only the boundary planes are needed as output and the input views
/// are slice-able.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <>
struct optimizer<tags::wavefront<3>,
                 tags::seq_exec_only_boundary_sliced>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using value_t = typename OutputValueType::value_type;
    using type =
      std::array<lightweight_multiarray<value_t, 3>, 3>;
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
            typename View1,
            typename View2,
            typename View3>
  static R apply_execute(S&& skeleton,
                         BoundaryView0&& bndary_v0,
                         BoundaryView1&& bndary_v1,
                         BoundaryView2&& bndary_v2,
                         View0&& view0,
                         View1&& view1,
                         View2&& view2,
                         View3&& view3)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first   = view0.domain().first();
    auto&& v1_first   = view1.domain().first();
    auto&& v2_first   = view2.domain().first();
    auto&& v3_first   = view3.domain().first();

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

      auto slice0a =
        view0.template slice<
          std::tuple<std::integral_constant<std::size_t, 0>>
        >(i + std::get<0>(v0_first));

      auto slice1a =
        view1.template slice<
          std::tuple<std::integral_constant<std::size_t, 0>>
        >(i + std::get<0>(v1_first));

      auto slice2a =
        view2.template slice<
          std::tuple<std::integral_constant<std::size_t, 0>>
        >(i + std::get<0>(v2_first));

      for (std::size_t j = start_j; j != end_j; j -= d1)
      {
        auto i_plane_slice_b = i_plane_slice_a.template slice<0>(j);
        auto j_plane_slice_b = j_plane_slice_a.template slice<0>(0);
        auto& k_point        = k_plane_slice_a(j, 0);

        auto slice0b =
          slice0a.template slice<
            std::tuple<std::integral_constant<std::size_t, 0>>
          >(j + std::get<1>(v0_first));

        auto slice1b =
          slice1a.template slice<
            std::tuple<std::integral_constant<std::size_t, 0>>
          >(j + std::get<1>(v1_first));

        auto slice2b =
          slice2a.template slice<
            std::tuple<std::integral_constant<std::size_t, 0>>
          >(j + std::get<1>(v2_first));

        for (std::size_t k = start_k; k != end_k; k -= d2)
        {
          auto ret =
            op(slice0b[k + std::get<2>(v0_first)],
               slice1b[k + std::get<2>(v1_first)],
               slice2b[k + std::get<2>(v2_first)],
               view3(i + std::get<0>(v3_first),
                     j + std::get<1>(v3_first),
                     k + std::get<2>(v3_first)),
               (i == start_i) ?
                 bv0(bv0_first_i, bv0_first_j + j, bv0_first_k + k)
                  : i_plane_slice_b(k),
               (j == start_j) ?
                 bv1(bv1_first_i + i, bv1_first_j, bv1_first_k + k)
                  : j_plane_slice_b(k),
               (k == start_k) ?
                 bv2(bv2_first_i + i, bv2_first_j + j, bv2_first_k)
                  : k_point);

          i_plane_slice_b(k) = get<0>(ret);
          j_plane_slice_b(k) = get<1>(ret);
          k_point            = get<2>(ret);
        }
      }
    }

    return {std::move(i_plane_result),
            std::move(j_plane_result),
            std::move(k_plane_result)};
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
struct optimizer<tags::wavefront<3>, tags::seq_exec_only_boundary>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using value_t = typename OutputValueType::value_type;
    using type =
      std::array<lightweight_multiarray<value_t, 3>, 3>;
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
            typename View1,
            typename View2,
            typename View3>
  static R apply_execute(S&& skeleton,
                         BoundaryView0&& bndary_v0,
                         BoundaryView1&& bndary_v1,
                         BoundaryView2&& bndary_v2,
                         View0&& view0,
                         View1&& view1,
                         View2&& view2,
                         View3&& view3)
  {
    using namespace skeletons;
    using namespace wavefront_utils;
    using index_t = typename std::decay<View0>::type::index_type;

    auto&& op = skeleton.get_op();

    auto&& start_corner = skeleton.get_start_corner();

    auto&& dimensions = view0.domain().dimensions();
    auto&& v0_first   = view0.domain().first();
    auto&& v1_first   = view1.domain().first();
    auto&& v2_first   = view2.domain().first();
    auto&& v3_first   = view3.domain().first();

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
               view1(i + get<0>(v1_first),
                     j + get<1>(v1_first),
                     k + get<2>(v1_first)),
               view2(i + get<0>(v2_first),
                     j + get<1>(v2_first),
                     k + get<2>(v2_first)),
               view3(i + std::get<0>(v3_first),
                     j + std::get<1>(v3_first),
                     k + std::get<2>(v3_first)),
               (i == start_i) ?
                 bv0(bv0_first_i, bv0_first_j + j, bv0_first_k + k)
                  : i_plane_slice_b(k),
               (j == start_j) ?
                 bv1(bv1_first_i + i, bv1_first_j, bv1_first_k + k)
                  : j_plane_slice_b(k),
               (k == start_k) ?
                 bv2(bv2_first_i + i, bv2_first_j + j, bv2_first_k)
                  : k_point);

          i_plane_slice_b(k) = get<0>(ret);
          j_plane_slice_b(k) = get<1>(ret);
          k_point            = get<2>(ret);
        }
      }
    }

    return {std::move(i_plane_result),
            std::move(j_plane_result),
            std::move(k_plane_result)};
  }
};

} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // KRIPKE_OPTIMIZERS_WAVEFRONT_HPP
