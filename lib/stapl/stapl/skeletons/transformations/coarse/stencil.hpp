/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_STENCIL_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_STENCIL_HPP

#include <type_traits>
#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/functional/stencil.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/optimizers/stencil.hpp>
#include <stapl/skeletons/utility/nd_traits.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

namespace stencil_utils {

/////////////////////////////////////////////////////////////////////
/// @brief The filter function used for accessing the boundary values
/// for coarse-grain stencil.
///
/// @tparam skeleton_tag skeleton tag that filter is specialized
///                      based on that
/////////////////////////////////////////////////////////////////////
template <typename skeleton_tag, typename Filter>
struct stencil_filter;

/////////////////////////////////////////////////////////////////////
/// @brief The filter function used for accessing the boundary values
/// for coarse-grain stencil.
/////////////////////////////////////////////////////////////////////
template <int Points, typename Filter>
struct stencil_filter<tags::stencil<1, Points>, Filter>
{
private:
  static constexpr int P = (Points-1)/2;

  int m_direction;
  Filter m_filter;

public:

  template <typename F>
  struct result;

  template <typename V>
  struct result<stencil_filter(V)>
  {
    using value_t = typename std::decay<V>::type::value_type;
    using filtered_t = typename filters::result_of<Filter,value_t>::type;
    using type    = std::vector<filtered_t>;
  };

  stencil_filter(Filter filter)
    : m_filter(std::move(filter))
  { }

  template <typename V>
  typename result<stencil_filter(V)>::type
  operator()(V const& v) const
  {
    return impl(v, make_index_sequence<size_t{P}>{});
  }

  template <int i>
  void set_direction(tags::direction<i>)
  {
    m_direction = i;
    filters::set_direction(m_filter, i);
  }

  bool operator==(stencil_filter const& other) const
  {
    return m_direction == other.m_direction;
  }

  void define_type(typer& t)
  {
    t.member(m_direction);
    t.member(m_filter);
  }
private:
  template <typename V, size_t... Is>
  typename result<stencil_filter(V)>::type
  impl(V const& v, index_sequence<Is...>) const
  {
    return {
      filters::apply_filter(m_filter,
                            v[m_direction > 0 ? Is : v.size() + Is - P])...};
  }


};


/////////////////////////////////////////////////////////////////////
/// @brief The filter function used for accessing the boundary values
/// for coarse-grain stencil.
///
/////////////////////////////////////////////////////////////////////
template <int Points, typename Filter>
struct stencil_filter<tags::stencil<2, Points>, Filter>
{
private:
  static constexpr int P = (Points-1)/4;

  int m_direction0;
  int m_direction1;
  Filter m_filter;

  template<class V>
  using index_type = typename nd_traits<2>::template index_type<V>;

public:
  template <typename F>
  struct result;

  template <typename V>
  struct result<stencil_filter(V)>
  {
    using value_t = typename std::decay<V>::type::value_type;
    using filtered_t = typename filters::result_of<Filter,value_t>::type;
    using type    = lightweight_multiarray<filtered_t, 2>;
  };

  stencil_filter(Filter filter)
    : m_filter(std::move(filter))
  { }

  template <typename V>
  typename result<stencil_filter(V)>::type
  operator()(V const& v) const
  {
    using result_type = typename result<stencil_filter(V)>::type;
    using index_type  = index_type<V>;

    auto dims = v.dimensions();
    const std::size_t m = std::get<0>(dims);
    const std::size_t n = std::get<1>(dims);


    std::size_t start_i, start_j;
    std::size_t end_i, end_j;
    std::size_t nx, ny;

    start_i = m_direction0 < 0 ? m - P : 0 ;
    start_j = m_direction1 < 0 ? n - P : 0 ;

    end_i = m_direction0 > 0 ? P : m ;
    end_j = m_direction1 > 0 ? P : n ;

    nx = end_i - start_i;
    ny = end_j - start_j;

    result_type result(nx, ny);

    for (size_t i = start_i; i < end_i ; ++i)
    {
      for (size_t j = start_j; j < end_j ; ++j)
      {
        result(i - start_i, j - start_j) =
          filters::apply_filter(m_filter, v(i,j));
      }
    }

    return result;
  }

  template <int i, int j>
  void set_direction(tags::direction<i, j> t)
  {
    m_direction0 = i;
    m_direction1 = j;
    filters::set_direction(m_filter, t);
  }

  bool operator==(stencil_filter const& other) const
  {
    return (m_direction0 == other.m_direction0)
            and
           (m_direction1 == other.m_direction1) ;
  }

  void define_type(typer& t)
  {
    t.member(m_direction0);
    t.member(m_direction1);
    t.member(m_filter);
  }
};

/////////////////////////////////////////////////////////////////////
/// @brief The filter function used for accessing the boundary values
/// for coarse-grain stencil.
///
/////////////////////////////////////////////////////////////////////
template<int N, int Points, typename Filter>
struct stencil_filter<tags::stencil<N, Points>, Filter>
{
private:
  static constexpr int P = (Points-1)/2/N;

  using nd = nd_traits<N>;

  using directions = typename nd::template make_nd<int>;
  using dimensions = make_index_sequence<N>;

  template<class V>
  using index_type = typename nd::template index_type<V>;

  directions m_dirs;
  Filter m_filter;

public:
  template <typename F>
  struct result;

  template <typename V>
  struct result<stencil_filter(V)>
  {
    using value_t = typename std::decay<V>::type::value_type;
    using filtered_t = typename filters::result_of<Filter,value_t>::type;
    using type    = lightweight_multiarray<filtered_t, N>;
  };


  stencil_filter(Filter filter)
    : m_filter(std::move(filter))
  { }


  template<typename V>
  typename result<stencil_filter(V)>::type
  operator()(V const& v) const
  {
    return filter(v, dimensions());
  };

  template<int... Is>
  void set_direction(tags::direction<Is...>)
  {
    m_dirs = directions{Is...};
  }

  bool operator==(stencil_filter const& other) const
  {
    return m_dirs == other.m_dirs;
  }

  void define_type(typer& t)
  {
    t.member(m_dirs);
    t.member(m_filter);
  }

private:
  //store and return the filtered values
  template<typename V, size_t... Is>
  typename result<stencil_filter(V)>::type
  filter(V const& v, index_sequence<Is...>) const
  {
    using result_type = typename result<stencil_filter(V)>::type;

    const auto dims = nd::dimensions(v);

    result_type result(
      (get<Is>(m_dirs) == 0 ? get<Is>(dims) : P)...
    );

    index_type<V> v_idx, r_idx;

    set(v, result, v_idx, r_idx, std::integral_constant<int, 0>());

    return result;
  }

  //recursively iterate over all the indices by dimension
  template<typename V, int I>
  void set(V const& v, typename result<stencil_filter(V)>::type& r,
    index_type<V>& v_idx, index_type<V>& r_idx,
    std::integral_constant<int, I>) const
  {
    const auto m     = nd_get<I>(nd::dimensions(v));
    const auto dir   = nd_get<I>(m_dirs);

    stapl_assert(m >= P, "View not big enough to read boundaries");

    const auto start = dir < 0 ? m - P : 0;
    const auto end   = dir > 0 ? P     : m;


    for (size_t x = start; x < end; ++x)
    {
      //set the corresponding index components and recurse to higher dimensions
      get<I>(v_idx) = x;
      get<I>(r_idx) = x - start;
      set(v, r, v_idx, r_idx, integral_constant<int, I+1>());
    }
  }

  //base case: indices are completely filled, set r[r_idx]
  template<typename V>
  void set(V const& v, typename result<stencil_filter(V)>::type& r,
    index_type<V>& v_idx, index_type<V>& r_idx,
    std::integral_constant<int, N>) const
  {
    r[r_idx] = filters::apply_filter(m_filter, v[v_idx]);
  }

};

} // namespace stencil_utils


template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;


/////////////////////////////////////////////////////////////////////
/// @brief Coarse-grain stencil can be created by creating a
/// @c stencil of @c stencils. The nested stencils are non-periodic even
/// if the outer one is periodic.
///
/// @tparam S          the stencil skeleton to be coarsened
/// @tparam dimensions specifies the dimension of stencil skeleton inputs
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
/// @ingroup skeletonsTransformationsCoarse
/////////////////////////////////////////////////////////////////////
template<typename S, int dim, int numPoints,
         typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::stencil<dim, numPoints, true>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
public:
  static auto call (S const& skeleton)
  STAPL_AUTO_RETURN((
    stencil<tags::stencil<dim, numPoints>>(
      skeletons::wrap<ExecutionTag>(
        stencil<tags::stencil<dim, numPoints, false>>(
          skeleton.get_op(), skeleton.get_filter())),
      stencil_utils::stencil_filter<typename S::skeleton_tag_type,
        typename S::filter_type>(skeleton.get_filter()))
  ))
};

template<typename S, int dim, int numPoints,
         typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::stencil<dim, numPoints, false>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
public:
  static auto call (S const& skeleton)
  STAPL_AUTO_RETURN((
    stencil<tags::stencil<dim, numPoints, false>>(
      skeletons::wrap<ExecutionTag>(
        stencil<tags::stencil<dim, numPoints, false>>(
          skeleton.get_op(), skeleton.get_filter()))
    )
  ))
};


} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_STENCIL_HPP
