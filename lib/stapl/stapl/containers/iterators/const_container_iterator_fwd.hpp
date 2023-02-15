/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONST_CONTAINER_ITERATOR_FWD_HPP
#define STAPL_CONTAINERS_CONST_CONTAINER_ITERATOR_FWD_HPP

namespace stapl {

template<typename Container,
         typename Accessor,
         typename Category = std::forward_iterator_tag>
class const_container_iterator;

template <typename C,typename A, typename Cat>
typename container_traits<C>::gid_type
gid_of(const_container_iterator<C,A,Cat> const& it);

} // stapl namespace

#endif /* STAPL_CONTAINERS_CONST_CONTAINER_ITERATOR_FWD_HPP */
