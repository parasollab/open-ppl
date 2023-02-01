/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_UTILITY_VS_MAP_HPP
#    define STAPL_UTILITY_VS_MAP_HPP

#include <boost/preprocessor/iteration/iterate.hpp>
#include <boost/preprocessor/iteration/local.hpp>
#include <boost/preprocessor/repetition/enum.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/enum_binary_params.hpp>
#include <stapl/utility/tuple.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/has_xxx.hpp>
#include <boost/utility/result_of.hpp>
#include <type_traits>

namespace stapl {

// Number of viewsets
#    ifndef PARAGRAPH_VS_MAP_MAX_VIEWSETS
#      define PARAGRAPH_VS_MAP_MAX_VIEWSETS 8
#    endif

// Elements per viewset dimensions
#    ifndef PARAGRAPH_VS_MAP_MAX_VIEWS
#      define PARAGRAPH_VS_MAP_MAX_VIEWS 11
#    endif

namespace utility_impl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(result_type)

template<typename WF>
struct result_type_void
  : std::is_same<typename WF::result_type, void>
{ };

} // namespace utility_impl


//////////////////////////////////////////////////////////////////////
/// @brief Helper method for @p vs_map that does most of the actual work.
///   Is passed the size of the tuples and this is used by the preprocessing
///   below to create size specific specialization of this class template. Each
///   specialization defines a static function with corresponding arity and
///   repetition of operations.
/// @ingroup utility
///
/// @tparam Map  Map functor to apply.
/// @tparam  Size Size of each tuple in viewsets.
//////////////////////////////////////////////////////////////////////
template<typename Map, int Size,
         bool = boost::mpl::eval_if<
           utility_impl::has_result_type<Map>,
           utility_impl::result_type_void<Map>,
           boost::mpl::false_
         >::type::value>
struct vs_map_helper;


//////////////////////////////////////////////////////////////////////
/// @brief Helper method for @p vs_map_reduce that does most of the actual work.
/// Is passed the size of the tuples and this is used by preprocessing below
/// to create size specific specialization of this class template. Each
/// specialization defines a static function with corresponding arity and
/// repetition of operations.
///
/// @tparam Map    Map functor to apply.
/// @tparam Reduce Reduce functor to apply.
/// @tparam  Size Size of each tuple in viewsets.
//////////////////////////////////////////////////////////////////////
template<typename Map, typename Reduce, int Size>
struct vs_map_reduce_helper;

//////////////////////////////////////////////////////////////////////
/// @brief Invokes the input functor tuple_size<VS0> times, using the ith
///   element of each viewset as argument.
/// @ingroup utility
///
/// @param fmap Functor to apply.  Arity defined by tuple_size of @p vs.
/// @param vs0  First tuple of arguments to be passed to incations of @p fmap.
/// @param viewsets Variadic number of tuples from which arguments to successive
/// invocation of @p fmap are obtained.
///
/// Example behavior for vs_map(MapFunctor, ViewSet0, ViewSet1):
/// <code>
///   fmap(get<0>ViewSet0, get<0>ViewSet1);
///   fmap(get<1>ViewSet0, get<1>ViewSet1);
/// </code>
///
/// @note fmap may mutate the views, however the functor itself is immutable.
/// As the name implies, no order should be assumed on the invocations of fmap,
/// and it is a candidate for future parallelism.
///
/// @todo Need to generalize to support all possibilities of & and
///    const& VS arguments (perhaps not until C++11).
//////////////////////////////////////////////////////////////////////
template<typename Map, typename VS0, typename ...ViewSets>
inline auto
vs_map(Map const& fmap, VS0&& vs0, ViewSets&... viewsets)
  -> decltype(vs_map_helper<
       Map, tuple_size<typename std::decay<VS0>::type>::value
    >()(fmap, std::forward<VS0>(vs0), viewsets...))
{
  return vs_map_helper<
    Map, tuple_size<typename std::decay<VS0>::type>::value>()
    (fmap, std::forward<VS0>(vs0), viewsets...);
}


#ifdef STAPL_DOCUMENTATION_ONLY
//////////////////////////////////////////////////////////////////////
/// @brief Invokes @p fmap tuple_size<ViewSet0> times, using the ith
/// element of each viewset as argument.  The return values of these invocations
/// are then reduced to a single value using @p freduce and returned from the
/// function.
///
/// @ingroup utility
///
/// @param fmap    Functor to apply to tuple list.  Arity defined by
/// tuple_size of @p vs.
///
/// @param freduce Functor to apply to result of @p fmap applications.
///
/// @param init value with which the return is initialized before the reduction
/// begins.
///
/// @param vs   Variadic number of tuples from which arguments to successive
/// invocation of @p fmap are obtained.
///
/// @result Result of map_reduce operation applied by function.
///
///
///  Example behavior for
/// vs_map_reduce(MapFunctor, ReduceFunctor, init, ViewSet0, ViewSet1):
///
///     init = freduce(init, fmap(get<0>ViewSet0, get<0>ViewSet1));
///     init = freduce(init, fmap(get<1>ViewSet0, get<1>ViewSet1));
///     ...
///     return init;
//////////////////////////////////////////////////////////////////////
template <typename Map, typename Reduce, typename... VS>
typename Reduce::result_type
vs_map_reduce(Map const& fmap, Reduce const& freduce,
              typename Reduce::result_type const& init,
              VS... vs);
#endif

#      define STAPL_VS_MAP_REDUCE(z, n, nothing)                              \
template<typename Map, typename Reduce, BOOST_PP_ENUM_PARAMS(n, typename VS)> \
inline typename Reduce::result_type                                           \
vs_map_reduce(Map const& fmap,                                                \
              Reduce const& freduce,                                          \
              typename Reduce::result_type const& init,                       \
              BOOST_PP_ENUM_BINARY_PARAMS(n, VS, & vs))                       \
{                                                                             \
  typedef typename std::remove_const<VS0>::type tuple_t;                      \
  return vs_map_reduce_helper<Map, Reduce, tuple_size<tuple_t>::value>::      \
    apply(fmap, freduce, init, BOOST_PP_ENUM_PARAMS(n, vs));                  \
}

namespace result_of {

template<typename Map, typename VS0>
struct vs_map
  : boost::result_of<vs_map_helper<
      Map, tuple_size<typename std::decay<VS0>::type>::value
    >(VS0&&)>
{ };

} // namespace result_of

#      define BOOST_PP_LOCAL_MACRO(n) STAPL_VS_MAP_REDUCE(~, n, ~)
#      define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_VS_MAP_MAX_VIEWSETS)
#      include BOOST_PP_LOCAL_ITERATE()

#      undef STAPL_VS_MAP_REDUCE

#      define BOOST_PP_ITERATION_LIMITS (0, PARAGRAPH_VS_MAP_MAX_VIEWS)
#      define BOOST_PP_FILENAME_1 "stapl/utility/vs_map.hpp"
#      include BOOST_PP_ITERATE()

#  endif // STAPL_UTILITY_VS_MAP_HPP

#else // BOOST_PP_IS_ITERATING

#  define i BOOST_PP_ITERATION()

#  define PARAGRAPH_TUPLE_ARG(z, viewset, element)                            \
    get<element>(BOOST_PP_CAT(vs, viewset))

#  define PARAGRAPH_TUPLE_ARG_TYPE(z, viewset, element)                       \
    typename tuple_element<                                                   \
      element, typename std::decay<BOOST_PP_CAT(VS, viewset)>::type           \
    >::type

#if i<=PARAGRAPH_VS_MAP_MAX_VIEWS
template<typename Map>
struct vs_map_helper<Map, i, false>
{
#  define STAPL_VM_MAP(z, element, num_viewsets)                              \
     fmap(BOOST_PP_ENUM(num_viewsets, PARAGRAPH_TUPLE_ARG, element))

#  define STAPL_VM_MAP_RESULT(z, element, num_viewsets)                       \
     typename boost::result_of<                                               \
       Map(BOOST_PP_ENUM(num_viewsets, PARAGRAPH_TUPLE_ARG_TYPE, element))    \
     >::type

  template<typename Signature>
  struct result;

#  define STAPL_VM_MAP2(z, n, nothing)                                        \
  template<BOOST_PP_ENUM_PARAMS(n, typename VS)>                              \
  struct result<vs_map_helper(BOOST_PP_ENUM_PARAMS(n, VS))>                   \
  {                                                                           \
    typedef tuple<BOOST_PP_ENUM(i, STAPL_VM_MAP_RESULT, n)> type;             \
  };                                                                          \
                                                                              \
  template<BOOST_PP_ENUM_PARAMS(n, typename VS)>                              \
  typename result<vs_map_helper(BOOST_PP_ENUM_PARAMS(n, VS))>::type           \
  operator()(Map const& fmap,                                                 \
        BOOST_PP_ENUM_BINARY_PARAMS(n, VS, && vs)) const                      \
  {                                                                           \
    return typename result<vs_map_helper(BOOST_PP_ENUM_PARAMS(n, VS))>::type( \
      BOOST_PP_ENUM(i, STAPL_VM_MAP, n)                                       \
    );                                                                        \
  }

#  define BOOST_PP_LOCAL_MACRO(n) STAPL_VM_MAP2(~, n, ~)
#  define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_VS_MAP_MAX_VIEWSETS)
#  include BOOST_PP_LOCAL_ITERATE()

#  undef STAPL_VM_MAP
#  undef STAPL_VM_MAP_RESULT
#  undef STAPL_VM_MAP2
};
#endif


template<typename Map>
struct vs_map_helper<Map, i, true>
{
#  define STAPL_VM_MAP(z, element, num_viewsets) \
     fmap(BOOST_PP_ENUM(num_viewsets, PARAGRAPH_TUPLE_ARG, element));

#  define STAPL_VM_MAP2(z, n, nothing)                                   \
  template<BOOST_PP_ENUM_PARAMS(n, typename VS)>                         \
  void operator()(Map const& fmap,                                       \
                    BOOST_PP_ENUM_BINARY_PARAMS(n, VS, & vs)) const      \
  {                                                                      \
    BOOST_PP_REPEAT(i, STAPL_VM_MAP, n)                                  \
  }

  typedef void result_type;

#  define BOOST_PP_LOCAL_MACRO(n) STAPL_VM_MAP2(~, n, ~)
#  define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_VS_MAP_MAX_VIEWSETS)
#  include BOOST_PP_LOCAL_ITERATE()

#  undef STAPL_VM_MAP
#  undef STAPL_VM_MAP2
};


template<typename Map, typename Reduce>
struct vs_map_reduce_helper<Map, Reduce, i>
{
#  define STAPL_VM_MAP(z, element, num_viewsets) \
     init = freduce( \
        init, fmap(BOOST_PP_ENUM(num_viewsets, PARAGRAPH_TUPLE_ARG, element)) \
      );

#  define STAPL_VM_MAP2(z, n, const_param)           \
  template<BOOST_PP_ENUM_PARAMS(n, typename VS)>     \
  static typename Reduce::result_type                \
  apply(Map const& fmap,                             \
        Reduce const& freduce,                       \
        typename Reduce::result_type init,           \
        BOOST_PP_ENUM_BINARY_PARAMS(n, VS, & vs))    \
  {                                                  \
    BOOST_PP_REPEAT(i, STAPL_VM_MAP, n)              \
    return init;                                     \
  }

#  define BOOST_PP_LOCAL_MACRO(n) STAPL_VM_MAP2(~, n, BOOST_PP_EMPTY())
#  define BOOST_PP_LOCAL_LIMITS (1, PARAGRAPH_VS_MAP_MAX_VIEWSETS)
#  include BOOST_PP_LOCAL_ITERATE()

#  undef STAPL_VM_MAP
#  undef STAPL_VM_MAP2
};

#  undef PARAGRAPH_TUPLE_ARG
#  undef PARAGRAPH_TUPLE_ARG_TYPE

#undef i
#endif // BOOST_PP_IS_ITERATING

#ifndef BOOST_PP_IS_ITERATING
#  ifndef STAPL_UTILITY_VS_MAP_FOOTER_H
#    define STAPL_UTILITY_VS_MAP_FOOTER_H

} // namespace stapl

#  endif // ifndef STAPL_UTILITY_VS_MAP_FOOTER_H
#endif // ifndef BOOST_PP_IS_ITERATING


