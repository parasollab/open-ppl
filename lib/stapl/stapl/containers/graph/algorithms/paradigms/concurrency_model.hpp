/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_CONCURRENCY_MODEL_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_CONCURRENCY_MODEL_HPP

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

namespace sgl {

//////////////////////////////////////////////////////////////////////
/// @brief Tag to specify a strong concurrency model.
///
///        In a strong SGL concurrency model, a vertex operator and a neighbor
///        operator cannot be executed concurrently on the same vertex. This
///        model allows for shared state between the two operators to be
///        operated on with mutual exclusion.
//////////////////////////////////////////////////////////////////////
struct strong_concurrency {};

//////////////////////////////////////////////////////////////////////
/// @brief Tag to specify a weak concurrency model.
///
///        In a weak SGL concurrency model, a vertex operator and a neighbor
///        operator can be executed concurrently on the same vertex. This
///        model should only be used under the following conditions:
///          1. There is no shared state between the two operators
///          2. Visitations / graph methods only occur as the very last
///             statement of the vertex operator
//////////////////////////////////////////////////////////////////////
struct weak_concurrency {};

BOOST_MPL_HAS_XXX_TRAIT_DEF(concurrency_model)

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to extract the concurrency model of an SGL algorithm
///        based on its vertex operator.
//////////////////////////////////////////////////////////////////////
template<typename VertexOp, bool = has_concurrency_model<VertexOp>::type::value>
struct operator_concurrency_model
{
  using type = typename VertexOp::concurrency_model;
};

//////////////////////////////////////////////////////////////////////
/// @brief Default value for when there is no concurrency model specifed
//////////////////////////////////////////////////////////////////////
template<typename VertexOp>
struct operator_concurrency_model<VertexOp, false>
{
  using type = strong_concurrency;
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine whether an SGL algorithm has a strong
///        concurrency model based on its vertex operator.
//////////////////////////////////////////////////////////////////////
template <typename VertexOp>
struct has_strong_concurrency_model
  : std::is_same<typename operator_concurrency_model<VertexOp>::type,
                 strong_concurrency>
{};

} // namespace sgl

} // namespace stapl

#endif
