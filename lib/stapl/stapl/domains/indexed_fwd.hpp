/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_INDEXED_FWD_HPP
#define STAPL_DOMAINS_INDEXED_FWD_HPP

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Defines a multi-dimensional domain where the indexes are
///        consecutive and enumerable in each dimension.
///
/// The domain is defined as a range of indexes between first index
/// and last index (included) @$ [first..last]@$.  This defines a convex
/// region in the multi-dimensional space.
///
/// @tparam T Index type.
/// @note @c T must be and integral type.
//////////////////////////////////////////////////////////////////////
template<typename T, int N = 1, typename Traversal = use_default>
class indexed_domain;

#else

template<typename T, int N = 1, typename ...OptionalTraversal>
class indexed_domain;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // STAPL_DOMAINS_INDEXED_FWD_HPP
