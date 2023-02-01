/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GID_OF_FWD_HPP
#define STAPL_CONTAINERS_GID_OF_FWD_HPP

#include <boost/version.hpp>

namespace stapl{

template<typename Iter>
typename
  boost::unordered::iterator_detail::iterator<Iter>::value_type::first_type
gid_of(boost::unordered::iterator_detail::iterator<Iter> it);

template<typename Iter, typename Iter2>
typename
#if BOOST_VERSION < 105800
  boost::unordered::iterator_detail::c_iterator<Iter, Iter2>::value_type
gid_of(boost::unordered::iterator_detail::c_iterator<Iter, Iter2> it);
#else
  boost::unordered::iterator_detail::c_iterator<Iter>::value_type
gid_of(boost::unordered::iterator_detail::c_iterator<Iter> it);
#endif

} // namespace stapl

#endif // STAPL_CONTAINERS_GID_OF_FWD_HPP
