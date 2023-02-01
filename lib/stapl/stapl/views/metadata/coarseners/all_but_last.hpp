/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_COARSEN_ALL_BUT_LAST_HPP
#define STAPL_VIEWS_METADATA_COARSEN_ALL_BUT_LAST_HPP

#include <stapl/utility/tuple.hpp>
#include <type_traits>
#include <boost/utility/result_of.hpp>
#include <boost/mpl/eval_if.hpp>
#include "null.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to coarsen a set of given views, where the last
///        view in the given set of views is coarsened using the
///        null_coarsener.
//////////////////////////////////////////////////////////////////////
template<typename Coarsener>
struct coarsen_all_but_last
{
  template<typename>
  struct result;

  template<typename F, typename Views>
  struct result<F(Views)>
  {
    typedef const typename tuple_ops::result_of::pop_back<Views>::type views1_t;

    typedef typename stapl::tuple<
              typename std::remove_reference<
                typename tuple_ops::result_of::back<Views>::type>::type
            > views2_t;

    typedef typename boost::mpl::eval_if_c<
      stapl::tuple_size<Views>::value == 1,
      typename std::result_of<null_coarsener(Views)>,
      typename result_of::tuple_cat<
        typename std::result_of<Coarsener(views1_t)>::type,
        typename std::result_of<null_coarsener(views2_t)>::type
      >
      >::type type;
  };

  template <typename Views>
  auto apply(Views const& views, integral_constant<std::size_t, 1>) const
  STAPL_AUTO_RETURN(null_coarsener()(views))

  template <typename Views, typename VSize>
  auto apply(Views const& views, VSize) const
  STAPL_AUTO_RETURN(
    tuple_cat(
      Coarsener()(tuple_ops::pop_back(views)),
      null_coarsener()(stapl::make_tuple(tuple_ops::back(views)))))

  template <typename Views>
  typename boost::result_of<coarsen_all_but_last(Views)>::type
  operator()(Views const& views) const
  {
    return apply(views, stapl::tuple_size<Views>());
  }
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_COARSEN_ALL_BUT_LAST_HPP
