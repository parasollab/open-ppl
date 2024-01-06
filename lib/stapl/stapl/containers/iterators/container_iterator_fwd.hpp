/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_ITERATOR_FWD_HPP
#define STAPL_CONTAINERS_CONTAINER_ITERATOR_FWD_HPP

namespace stapl {

template<typename Container,
         typename Accessor,
         typename ...OptionalParams>
class container_iterator;

template <typename C,typename A, typename Cat>
typename container_traits<C>::gid_type
gid_of(const container_iterator<C,A,Cat>& it);

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINER_ITERATOR_FWD_HPP
