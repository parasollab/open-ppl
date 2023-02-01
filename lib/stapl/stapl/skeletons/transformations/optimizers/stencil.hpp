/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_STENCIL_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_STENCIL_HPP

#include <numeric>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/transformations/optimizers/nested.hpp>

namespace stapl {
namespace skeletons {

template <typename SkeletonTag, typename ExecutionTag>
struct skeleton_execution_traits;

template <int NP>
struct skeleton_execution_traits<tags::stencil<1, NP, false>,
                                 tags::nested_execution<false>>
{
  template <typename OutputValueType>
  using result_type = std::vector<OutputValueType>;
};

template <>
struct skeleton_execution_traits<tags::stencil<2, 5, false>,
                                 tags::nested_execution<false>>
{
  template <typename OutputValueType>
  using result_type = lightweight_multiarray<OutputValueType, 2>;
};

namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

/////////////////////////////////////////////////////////////////////
/// @brief A @c stencil_optimizer executes the non-periodic stencil
/// 1D, 3 point stencil skeleton on given input and boundary values.
/// This specialization is for executing the nested stencil
/// sequentially.
///
/// @ingroup skeletonsTransformationsCoarse
/////////////////////////////////////////////////////////////////////
template <>
struct optimizer<tags::stencil<1, 3, false>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Skeleton, typename FlowValueType>
  struct result<Skeleton(FlowValueType)>
  {
    typedef std::vector<FlowValueType> type;
  };

  template <typename ReturnType, typename S,
            typename Boundary, typename Center>
  static ReturnType
  execute(S&& skeleton,
          Center&& center,
          Boundary&& l, Boundary&& r)
  {
    const std::size_t n = center.size();

    using filters::apply_filter;

    auto f_left  =
      stencil_utils::get_stencil_filter<
        tags::direction<-1>>(skeleton.get_filter());
    auto f_right =
      stencil_utils::get_stencil_filter<
        tags::direction<+1>>(skeleton.get_filter());

    auto stencil_op = skeleton.get_op();

    if (n == 1)
    {
      return { stencil_op(l[0], center[0], r[0]) };
    }

    ReturnType result(n);
    // left and right boundary values
    result[0]   = stencil_op(l[0],
                             center[0],
                             apply_filter(f_right, center[1]));

    result[n-1] = stencil_op(apply_filter(f_left, center[n-2]),
                             center[n-1],
                             r[0]);

    for (std::size_t i = 1; i < n-1; ++i)
    {
      result[i] = stencil_op(apply_filter(f_left,  center[i-1]),
                             center[i],
                             apply_filter(f_right, center[i+1]));
    }

    return result;
  }

};

template<int Points>
struct optimizer<tags::stencil<1, Points, false>, tags::sequential_execution>
{
  static constexpr size_t P = (Points - 1) / 2;

  template <typename R>
  struct result;

  template <typename Skeleton, typename FlowValueType>
  struct result<Skeleton(FlowValueType)>
  {
    typedef std::vector<FlowValueType> type;
  };

  template <typename ReturnType, typename S,
            typename Center, typename... Boundaries>
  static ReturnType
  execute(S&& skeleton,
          Center&& center,
          Boundaries&&... b) {
    return execute_impl<ReturnType>(skeleton, center,
        std::forward_as_tuple(b...), make_index_sequence<P>{});
  }

private:
  template<size_t I, typename Filters, typename Center>
  static auto read_center_left(size_t i, Filters&& f, Center&& c)
  STAPL_AUTO_RETURN((
    filters::apply_filter(get<I>(std::forward<Filters>(f)),
        c[i-P+I])
  ))

  template<size_t I, typename Filters, typename Center>
  static auto read_center_right(size_t i, Filters&& f, Center&& c)
  STAPL_AUTO_RETURN((
    filters::apply_filter(get<I>(std::forward<Filters>(f)),
        c[i+1+I])
  ))

  template<typename ReturnType, typename S, typename Center,
    typename Boundaries, size_t... Is>
  static ReturnType
  execute_impl(S&& skeleton,
          Center&& center, Boundaries b,
          index_sequence<Is...>) {

    const std::size_t n = center.size();

    using filters::apply_filter;

    auto f_lefts = make_tuple(
      stencil_utils::get_stencil_filter<
        tags::direction<P-Is>>(skeleton.get_filter())...
    );
    auto f_rights = make_tuple(
      stencil_utils::get_stencil_filter<
        tags::direction<Is>>(skeleton.get_filter())...
    );

    auto stencil_op = skeleton.get_op();
    if (n == 1) {
      return {stencil_op(get<Is>(b)[0]..., center[0], get<Is+P>(b)[0]...)};
    }
    else {
      stapl_assert(n >= Points, "Assuming center is big enough that we read"
          " from only one boundary at a time");

      ReturnType result(n);
      for (size_t i = 0; i < P; ++i) {
        result[i] = stencil_op(
          (i + Is < P ?
            get<Is>(b)[i+Is] : read_center_left<Is>(i, f_lefts, center))...,
          center[i],
          read_center_right<Is>(i, f_rights, center)...
        );

        //The index on the right we are reading from
        const auto r = n-i-1;
        result[n-i-1] = stencil_op(
          read_center_left<Is>(r, f_lefts, center)...,
          center[r],
          (i <= Is ? //from boundary, as r is i away from it
            get<Is+P>(b)[Is-i] : read_center_right<Is>(r, f_rights, center))...
        );
      }

      for (std::size_t i = P; i < n-P; ++i)
      {
        result[i] = stencil_op(
            read_center_left<Is>(i, f_lefts, center)...,
            center[i],
            read_center_right<Is>(i, f_rights, center)...
         );
      }
      return result;
    }


  }

};

/////////////////////////////////////////////////////////////////////
/// @brief A @c stencil_optimizer executes the non-periodic
/// stencil 2D, 5 point skeleton on given input and boundary values.
/// This specialization is for executing the nested stencil sequentially.
///
///
///   (first, first)........(first, middle)..........(first, last)
///   ...........................................................
///   ...........................................................
///   (middle, first).......(middle, middle).........(middle, last)
///   ...........................................................
///   ...........................................................
///   (last, first).........(last, middle)...........(last, last)
///
/// @ingroup skeletonsTransformationsCoarse
/////////////////////////////////////////////////////////////////////
template <>
struct optimizer<tags::stencil<2, 5, false>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Skeleton, typename FlowValueType>
  struct result<Skeleton(FlowValueType)>
  {
    typedef lightweight_multiarray<FlowValueType, 2> type;
  };

  template <typename ReturnType, typename S,
            typename Boundary, typename Center>
  static ReturnType
  execute(S&& skeleton,
          Center && center,
          Boundary && u, Boundary && l,
          Boundary && d, Boundary && r)
  {

    auto dims = center.dimensions();
    auto stencil_op = skeleton.get_op();

    ReturnType out_container(dims);

    const std::size_t nx = std::get<0>(dims);
    const std::size_t ny = std::get<1>(dims);

    auto up_first    = u.domain().first();
    auto left_first  = l.domain().first();
    auto down_first  = d.domain().first();
    auto right_first = r.domain().first();

    using index_t = typename std::decay<Boundary>::type::index_type;
    auto&& read_view = [](Boundary const& b, std::size_t i, std::size_t j,
                           index_t const& first)
                        { return b(i + std::get<0>(first),
                                   j + std::get<1>(first)); };

    auto f_up    =
      stencil_utils::get_stencil_filter<
        tags::direction<-1, 0>>(skeleton.get_filter());
    auto f_down  =
      stencil_utils::get_stencil_filter<
        tags::direction<+1, 0>>(skeleton.get_filter());
    auto f_left  =
      stencil_utils::get_stencil_filter<
        tags::direction<0, -1>>(skeleton.get_filter());
    auto f_right =
      stencil_utils::get_stencil_filter<
        tags::direction<0, +1>>(skeleton.get_filter());

    using filters::apply_filter;


    // (first, first) point
    out_container(0, 0) =
      stencil_op(
        apply_filter(f_up,   read_view(u, 0, 0, up_first)),
        apply_filter(f_left, read_view(l, nx-1, 0, left_first)),
        center(0, 0),
        center(1, 0),
        center(0, 1));

    // (last, first) point
    out_container(nx-1,0) =
      stencil_op(
        center(nx-2, 0),
        apply_filter(f_left, read_view(l, nx-1, 0, left_first)),
        center(nx-1, 0),
        apply_filter(f_down, d(0, 0)),
        center(nx-1, 1));

    // (last, last) point
    out_container(nx-1,ny-1) =
      stencil_op(
        center(nx-2, ny-1),
        center(nx-1, ny-2),
        center(nx-1, ny-1),
        apply_filter(f_down, read_view(d, 0, ny-1, down_first)),
        apply_filter(f_right, read_view(r, 0, 0, right_first)));

    // (first, last) point
    out_container(0,ny-1) =
      stencil_op(
        apply_filter(f_up, read_view(u, 0, ny-1, up_first)),
        center(0, ny-2),
        center(0, ny-1),
        center(1, ny-1),
        apply_filter(f_right, read_view(r, 0, 0, right_first)));

    // (middle, first) , (middle, last) points
    for (std::size_t i = 1; i < nx-1; ++i)
    {
      out_container(i,0) =
        stencil_op(
          center(i-1, 0),
          apply_filter(f_left, read_view(l, i , 0, left_first)),
          center(i,0),
          center(i+1, 0),
          center(i, 1));

      out_container(i,ny-1) =
        stencil_op(
          center(i-1, ny-1),
          center(i, ny-2),
          center(i, ny-1),
          center(i+1, ny-1),
          apply_filter(f_right, read_view(r, i, 0, right_first)));
    }

    // (first, middle) , (last, middle) points
    for (std::size_t j = 1; j < ny-1; ++j)
    {
      out_container(0, j) =
        stencil_op(
          apply_filter(f_up, read_view(u, 0, 0, up_first)),
          center(0, j-1),
          center(0, j),
          center(1, j),
          center(0, j+1));

      out_container(nx-1, j) =
        stencil_op(
          center(nx-2, j),
          center(nx-1, j-1),
          center(nx-1, j),
          apply_filter(f_down, read_view(d, 0, j, down_first)),
          center(nx-1, j+1));
    }

    // (middle, middle) points
    for (std::size_t i = 1; i < nx-1; ++i)
    {
      for (std::size_t j = 1; j < ny-1; ++j)
      {
        out_container(i,j) =
          stencil_op(
            center(i-1, j),
            center(  i, j-1),
            center(  i, j  ),
            center(i+1, j),
            center(  i, j+1));
      }
    }
    return out_container;
  }
};


/////////////////////////////////////////////////////////////////////
/// @brief A @c stencil_optimizer executes the non-periodic
/// stencil n-D, 2np+1 point skeleton on given input and boundary values.
/// This specialization is for executing the nested stencil sequentially.
/// @ingroup skeletonsTransformationsCoarse
/////////////////////////////////////////////////////////////////////
template <int N, int Points>
struct optimizer<tags::stencil<N, Points, false>, tags::sequential_execution>
{
private:
  //the number of points in for each dimension per direction
  static constexpr size_t P = (Points - 1)/N/2;

  static_assert((Points - 1) % (2*N) == 0,
      "Only Nd, 2*N*p+1 point stencils currently supported");

  using dimensions  = make_index_sequence<N>;
  using offsets     = make_index_sequence<P>;
  using points      = stencil_impl::traversal_order<dimensions, offsets>;

  using nd          = nd_traits<N>;

  using out_index_t = typename homogeneous_tuple_type<N, size_t>::type;

public:

  template <typename R>
  struct result;

  template <typename Skeleton, typename FlowValueType>
  struct result<Skeleton(FlowValueType)>
  {
    typedef lightweight_multiarray<FlowValueType, N> type;
  };


  template <typename ReturnType, typename S,
            typename Center, typename... Boundaries>
  static ReturnType
  execute(S const& skeleton, Center const& center, Boundaries const&... b)
  {
    auto stencil_op = skeleton.get_op();
    const auto dims = nd::dimensions(center);

    ReturnType out_container(dims);
    auto boundaries = tie(b...);

    out_index_t idx;

    loop(stencil_op, center, boundaries, out_container,
      skeleton.get_filter(), idx, std::integral_constant<int, 0>());

    return out_container;
  }

private:

  //Helper that returns a copy of m_filter_function with the direction set
  template<typename F, typename Pt>
  static F filter_for(Pt, F const& f)
  {
    return stencil_utils::get_stencil_filter(Pt(), f, dimensions());
  }

  template<size_t Dim, int SignedOffset, size_t... Is>
  static out_index_t index_of(out_index_t const& idx, index_sequence<Is...>&&)
  {
    return out_index_t((
      Is == Dim ? get<Is>(idx) + SignedOffset : get<Is>(idx)
    )...);
  }

  template<typename Boundary, size_t... Is>
  static typename Boundary::value_type
  value_at(Boundary const& b, out_index_t const& idx,
    index_sequence<Is...>)
  {
    auto first = b.domain().first();

    auto i = typename Boundary::index_type((get<Is>(first) + get<Is>(idx))...);

    return b[i];
  }


  //Loop over all points in the input view
  template<typename Op, typename Center, typename Boundaries, typename Output,
    typename Filter, int I>
  static void loop(Op& op, Center const& c, Boundaries const& b, Output& out,
    Filter const& f, out_index_t& idx,
    std::integral_constant<int,I>)
  {
    const auto dims = nd::dimensions(c);
    for (size_t x = 0; x < nd::template nd_get<I>(dims); ++x)
    {
      nd::template nd_get<I>(idx) = x;
      loop(op, c, b, out, f, idx, std::integral_constant<int, I+1>());
    }
  }

  //Base case: now read this index's dependencies and store it
  template<typename Op, typename Center, typename Boundaries, typename Output,
    typename Filter>
  static void loop(Op& op, Center const& c, Boundaries const& b, Output& out,
    Filter const& f, out_index_t const idx,
    std::integral_constant<int,N>)
  {
    visit_left(op, c, b, out, f, idx, dimensions(), offsets(), c[idx]);
  }


  template<typename Op, typename Center, typename Boundaries,
    typename Output, typename Filter, typename... Args>
  static void visit_left(Op& op, Center const& c, Boundaries& b,
    Output& out, Filter const& f, out_index_t const& idx,
    index_sequence<>, offsets, Args&&... args)
  {
    visit_right(op, c, b, out, f, idx, dimensions(), offsets(),
        std::forward<Args>(args)...);
  }
  //base case: no other points to read, apply visitor
  template<typename Op, typename Center, typename Boundaries,
    typename Output, typename Filter, typename... Args>
  static void visit_right(Op& op, Center const&, Boundaries&,
    Output& out, Filter const&, out_index_t const& idx,
    index_sequence<>, offsets, Args&&... args)
  {
    out[idx] = op(std::forward<Args>(args)...);
  }

  //no more points to read on the left for this dim, go to next dim
  template <typename Op, typename Center, typename Boundaries, typename Output,
    typename Filter, size_t Dim, size_t... Dims, typename... Args>
  static void visit_left(Op& op, Center const& c, Boundaries& b, Output& out,
    Filter const& f, out_index_t const& idx,
    index_sequence<Dim, Dims...> dims, index_sequence<>, Args&&... args) {
    using NewDims = index_sequence<Dims...>;
    visit_left(op, c, b, out, f, idx, NewDims{}, offsets(),
        std::forward<Args>(args)...);
  }

  template<typename Op, typename Center, typename Boundaries, typename Output,
    typename Filter, size_t Dim, size_t... Dims, typename... Args>
  static void visit_right(Op& op, Center const& c, Boundaries& b,
    Output& out, Filter const& f, out_index_t const& idx,
    index_sequence<Dim, Dims...> dims, index_sequence<>, Args&&... args)
  {
    // Move to the left side for the next dim, resetting the offsets
    using NewDims = index_sequence<Dims...>;

    visit_right(op, c, b, out, f, idx, NewDims{}, offsets(),
        std::forward<Args>(args)...);
  }

  template<typename Op, typename Center, typename Boundaries,
    typename Output, typename Filter, size_t Dim, size_t... Dims,
    size_t Off0, size_t... Offs, typename... Args>
  static void visit_left(Op& op, Center const& c, Boundaries& b,
    Output& out, Filter const& f, out_index_t const& idx,
    index_sequence<Dim, Dims...> dims,
    index_sequence<Off0, Offs...>, Args&&... args)
  {
    constexpr auto Off = Off0 + 1;

    using filters::apply_filter;
    using stencil_utils::left_boundary_index;

    //past boundary as are rest of offs for this dim
    if (nd_get<Dim>(idx) < Off)
    {
      auto&& boundary = get<sizeof...(Args) - 1>(b);

      using NewDims = index_sequence<Dims...>;

      visit_left(op, c, b, out, f, idx, NewDims{}, offsets(),
        std::forward<Args>(args)...,
        value_at(boundary,
          left_boundary_index<P, Dim, Off>(idx, dimensions()), dimensions()),
        value_at(boundary,
          left_boundary_index<P, Dim, Offs + 1>(idx, dimensions()),
          dimensions())...
       );
    }
    else
    {
      constexpr int SignedOff = -int(Off);
      stencil_impl::point<-1, Dim, Off> p;
      Filter f2 = filter_for(p, f);
      visit_left(op, c, b, out, f, idx, dims, index_sequence<Offs...>{},
        std::forward<Args>(args)...,
        apply_filter(f2,
          c[optimizer::index_of<Dim,SignedOff>(idx, dimensions())]));
    }

  }
  template<typename Op, typename Center, typename Boundaries,
    typename Output, typename Filter, size_t Dim, size_t... Dims,
    size_t Off0, size_t... Offs, typename... Args>
  static void visit_right(Op& op, Center const& c, Boundaries& b,
    Output& out, Filter const& f, out_index_t const& idx,
    index_sequence<Dim, Dims...> dims,
    index_sequence<Off0, Offs...>, Args&&... args)
  {
    constexpr auto Off = Off0 + 1;

    const auto m = get<Dim>(c.dimensions());

    using filters::apply_filter;
    using stencil_utils::right_boundary_index;

    //past boundary as are rest of offs for this dim
    if (nd_get<Dim>(idx) + Off >= m)
    {
      auto&& boundary = get<sizeof...(Args) - 1>(b);

      using NewDims = index_sequence<Dims...>;

      visit_right(op, c, b, out, f, idx, NewDims{}, offsets(),
        std::forward<Args>(args)...,
        value_at(boundary,
          right_boundary_index<Dim, Off>(idx, m, dimensions()), dimensions()),
        value_at(boundary,
          right_boundary_index<Dim, Offs + 1>(idx, m, dimensions()),
          dimensions())...
       );
    }
    else
    {
      stencil_impl::point<+1, Dim, Off> p;
      Filter f2 = filter_for(p, f);
      visit_right(op, c, b, out, f, idx, dims, index_sequence<Offs...>{},
        std::forward<Args>(args)...,
        apply_filter(f2, c[optimizer::index_of<Dim,Off>(idx, dimensions())]));
    }

  }


};

} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_STENCIL_HPP
