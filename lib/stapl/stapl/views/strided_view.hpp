/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING

#ifndef STAPL_VIEWS_STRIDED_VIEW_HPP
#define STAPL_VIEWS_STRIDED_VIEW_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>

#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/iteration/local.hpp>
#include <boost/preprocessor/iteration/iterate.hpp>
#include <boost/preprocessor/arithmetic/sub.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>

#include <iostream>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that generates a domain from (0,0,..0) to
///        (size_1/step_1-1, .. size_n/step_n-1).
//////////////////////////////////////////////////////////////////////
template<int N, typename S, typename D>
struct strided_domain_helper
{};

//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction that computes the mapping function type
///        needed for an n-dimensional strided view.
//////////////////////////////////////////////////////////////////////
template<int N, typename T>
struct f_stride_helper
{};

//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction that computes the type of a
///        multi-dimensional strided view.
//////////////////////////////////////////////////////////////////////
template<typename BV, int N>
struct strided_view_type_helper
{
  typedef compose_func<
    typename view_traits<BV>::map_function,
    typename view_impl::f_stride_helper<
      dimension_traits<BV>::type::value, typename BV::index_type
    >::type
  >  compose_func_type;
  typedef typename compose_func_type::type map_func_type;

  typedef multiarray_view<typename BV::view_container_type,
    typename BV::domain_type, map_func_type> type;

};


//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction that computes the type of a
///        1-dimensional strided view.
//////////////////////////////////////////////////////////////////////
template<typename BV>
struct strided_view_type_helper<BV, 1>
{
  typedef compose_func<
    typename view_traits<BV>::map_function,
    typename view_impl::f_stride_helper<
      dimension_traits<BV>::type::value, typename BV::index_type
    >::type
  >  compose_func_type;
  typedef typename compose_func_type::type map_func_type;

  typedef array_view<typename view_traits<BV>::container,
    typename view_traits<BV>::domain_type, map_func_type> type;

};

#ifndef MAX_NUM_DIM
#define MAX_NUM_DIM 10
#endif

#define BOOST_PP_ITERATION_LIMITS (1, MAX_NUM_DIM)
#define BOOST_PP_FILENAME_1       "stapl/views/strided_view.hpp"
#include BOOST_PP_ITERATE()

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief A view over a container where elements are chosen based on a
///   specified stride.
/// @tparam BV The view on which the stride is based.
/// @ingroup strided_view
//////////////////////////////////////////////////////////////////////
template<typename BV>
class strided_view
  : public view_impl::strided_view_type_helper<BV,
             dimension_traits<BV>::type::value>
{
public:
  typedef typename view_impl::strided_view_type_helper<
    BV, dimension_traits<BV>::type::value>                   base_type;
  typedef typename base_type::type                           type;
  typedef typename base_type::compose_func_type              compose_func_type;

};

//////////////////////////////////////////////////////////////////////
/// @brief Takes an input view and creates a strided view over the
///        original view's container.
/// @param view The view on which to base the strided view.
/// @param step The step size.
/// @param start The index at which to start striding.
/// @ingroup strided_view
//////////////////////////////////////////////////////////////////////
template<typename BV>
typename strided_view<BV>::type
make_strided_view(BV const& view,
                  typename BV::index_type const& step,
                  typename BV::index_type const& start)
{
  typedef view_impl::strided_domain_helper<
    dimension_traits<BV>::type::value,
    typename BV::index_type, typename BV::domain_type
  > domain_helper_type;

  assert(view.domain().contains(start));

  typedef typename strided_view<BV>::type              view_type;
  typedef typename strided_view<BV>::compose_func_type compose_func_type;

  typedef typename view_impl::f_stride_helper<
      dimension_traits<BV>::type::value, typename BV::index_type
    >::type stride_map_fun_type;

  return view_type(
    view.container(),
    domain_helper_type()(view.domain(), start, step),
    compose_func_type::apply(
      view.mapfunc(), stride_map_fun_type(step, start)
    )
  );
}
} // stapl namespace

#endif /* STAPL_VIEWS_STRIDED_VIEW_HPP */

#else // BOOST_PP_IS_ITERATING

#define GET_elem(z,i,var) get<i>(var)
#define CALC_offset(z,i,var) get<i>(m_first_gid) + get<i>(idx) * get<i>(m_step)
#define CALC_inv_offset(z,i,var) \
  (get<i>(idx) - get<i>(m_first_gid)) / get<i>(m_step)
#define CALC_last_1 \
  ((dom.size() - start + dom.first()) / step) + \
  ((dom.size() - start + dom.first()) % step ? 1 : 0) - 1
#define CALC_last(z,i,var) \
  ((get<i>(dom.dimensions()) - get<i>(start) + get<i>(dom.first())) / \
    get<i>(step)) + \
  ((get<i>(dom.dimensions()) - get<i>(start) + get<i>(dom.first())) % \
    get<i>(step) ? 1 : 0) - 1
#define ZERO_dom(z,i,var) (0)
#define CHECK_inv_offset_idx(z,i,var) BOOST_PP_IF(i,&&,) \
  get<i>(idx) >= get<i>(m_first_gid)
#define DEFINED_stride_op(z,i,var) BOOST_PP_IF(i,&&,) \
  (get<i>(idx) - get<i>(m_first_gid)) % get<i>(m_step) == 0

#define this_iteration BOOST_PP_ITERATION()

//////////////////////////////////////////////////////////////////////
/// @brief Function object that generates a domain from (0,0,..0) to
///        ((size_1-start_1)/step_1-1, .. (size_n-start_n)/step_n-1).
//////////////////////////////////////////////////////////////////////
template<typename S, typename D>
struct strided_domain_helper<this_iteration, S, D>
{
  strided_domain_helper() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs and returns a strided domain
  /// @param dom The original domain of the view.
  /// @param step The step size.
  //////////////////////////////////////////////////////////////////////
  D operator()(D const& dom, S start, S step)
  {
    using index_type = typename D::index_type;

#if this_iteration == 1
    return D(index_type(0), index_type(CALC_last_1));
#else
    return D( index_type( BOOST_PP_ENUM(this_iteration, ZERO_dom, ~) ),
              index_type( BOOST_PP_ENUM(this_iteration, CALC_last, ~) ) );
#endif
  }
};

template<typename T>
class BOOST_PP_CAT(f_stride_inv,this_iteration);

//////////////////////////////////////////////////////////////////////
/// @brief A mapping function that calculates strided offsets.
/// @ingroup strided_view
//////////////////////////////////////////////////////////////////////
template<typename T>
class BOOST_PP_CAT(f_stride,this_iteration)
{
public:
  typedef T                                            index_type;
  typedef index_type                                   gid_type;
  typedef BOOST_PP_CAT(f_stride_inv,this_iteration)<T> inverse;
  typedef std::true_type is_injective;

  index_type  m_step;
  gid_type    m_first_gid;

  BOOST_PP_CAT(f_stride,this_iteration)() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a strided mapping function.
  /// @param step The number of GIDs to step over.
  /// @param first_gid The first GID to start mapping.
  //////////////////////////////////////////////////////////////////////
  BOOST_PP_CAT(f_stride,this_iteration)(index_type step, gid_type first_gid)
    : m_step(step), m_first_gid(first_gid)
  { }

  template<typename MF>
  BOOST_PP_CAT(f_stride,this_iteration)(MF const& other)
    : m_step(other.m_step), m_first_gid(other.m_first_gid)
  { }

  gid_type operator()(index_type const& idx) const
  {
#if this_iteration == 1
    return gid_type(m_first_gid + m_step * idx);
#else
    return gid_type(BOOST_PP_ENUM(this_iteration,CALC_offset,~));
#endif
  }

  index_type step() const { return m_step; }
  index_type offset() const { return m_first_gid; }

  void define_type(typer& t)
  {
    t.member(m_step);
    t.member(m_first_gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "STRIDED_VIEW::F_STRIDE " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    std::cerr << " m_step " << m_step;
    std::cerr << " m_first_gid " << m_first_gid << std::endl;
  }

};

//////////////////////////////////////////////////////////////////////
/// @brief A mapping function that maps several indices to the same
///        gid based on a stride. Inverse of the f_stride mapping
///        function.
//////////////////////////////////////////////////////////////////////
template<typename T>
class BOOST_PP_CAT(f_stride_inv,this_iteration)
{
public:
  typedef T                                        index_type;
  typedef index_type                               gid_type;
  typedef BOOST_PP_CAT(f_stride,this_iteration)<T> inverse;

  index_type  m_step;
  gid_type    m_first_gid;

  BOOST_PP_CAT(f_stride_inv,this_iteration)() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an inverse strided mapping function.
  /// @param step The amount of indices to map to the same GID.
  /// @param first_gid The first GID to start mapping.
  //////////////////////////////////////////////////////////////////////
  BOOST_PP_CAT(f_stride_inv,this_iteration)(index_type step, gid_type first_gid)
    : m_step(step), m_first_gid(first_gid)
  { }

  template<typename MF>
  BOOST_PP_CAT(f_stride_inv,this_iteration)(MF const& other)
    : m_step(other.m_step), m_first_gid(other.m_first_gid)
  { }

  bool defined(index_type const& idx) const
  {
#if this_iteration == 1
    return idx >= m_first_gid && (idx - m_first_gid) % m_step == 0;
#else
    return BOOST_PP_REPEAT(this_iteration ,CHECK_inv_offset_idx, ~) &&
           BOOST_PP_REPEAT(this_iteration ,DEFINED_stride_op, ~);
#endif

  }


  gid_type operator()(index_type const& idx) const
  {
#if this_iteration == 1
    assert(idx >= m_first_gid);
    return gid_type((idx - m_first_gid) / m_step);
#else
    assert(BOOST_PP_REPEAT(this_iteration, CHECK_inv_offset_idx, ~));
    return gid_type(BOOST_PP_ENUM(this_iteration,CALC_inv_offset,~));
#endif
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that computes the type of an n-dimensional
///        stride mapping function.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct f_stride_helper<this_iteration, T>
{
  typedef BOOST_PP_CAT(f_stride,this_iteration)<T> type;
};

#undef this_iteration
#undef GET_elem
#undef CALC_offset
#undef CALC_inv_offset
#undef CALC_size
#undef ZERO_dom

#endif //STAPL_VIEWS_STRIDED_VIEW_HPP
