/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_SPANS_FWD_HPP
#define STAPL_SKELETONS_SPANS_SPANS_FWD_HPP

namespace stapl {
namespace skeletons {
namespace spans {

template <int i>
class balanced;

template <int i>
class blocked;

template <typename OnSpan, typename Phase>
struct binomial_tree;

class only_once;

template <typename OnSpan>
struct nearest_pow_two;

class per_location;

template <typename OnSpan>
struct reduce_to_pow_two;

template<int i>
class summa;

template <typename OnSpan, typename Alignment>
struct tree;

template <typename OnSpan, typename Alignment>
struct reverse_tree;

}
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_SPANS_FWD_hpp
