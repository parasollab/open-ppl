/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HEAP_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_HEAP_BASE_CONTAINER_TRAITS_HPP

#include <stapl/containers/heap/seq_heap.hpp>
#include <stapl/domains/iterator_domain.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for a sequential
///         heap container. These traits can be changed to customization.
/// @ingroup pheapTraits
/// @tparam T Type of the stored elements in the base container.
/// @tparam Comp The comparator used to maintain ordering of the elements.
////////////////////////////////////////////////////////////////////////
template<typename T, typename Comp>
struct container_traits<seq_heap<T,Comp> >
{
  typedef T                                       value_type;
  typedef Comp                                    comp_type;
  typedef typename seq_heap<T,Comp>::iterator     gid_type;
  typedef gid_type                                index_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the heap base container. These traits
///        can be changed to customize base containers.
/// @ingroup pheapTraits
/// @tparam T Type of the stored elements in the base container.
/// @tparam Comp The comparator used to maintain ordering of the elements.
////////////////////////////////////////////////////////////////////////
template <typename T, typename Comp>
struct heap_base_container_traits
{
  typedef T                           value_type;
  typedef Comp                        comp_type;
  typedef seq_heap<T,Comp>            container_type;
};

} // namespace stapl

#endif
