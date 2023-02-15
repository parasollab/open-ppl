/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_MESH_GEOM_VECTOR_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_GEOM_VECTOR_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief n-dimensional geometric vector
/// @tparam DIM dimension of the vector.
/// @tparam ELEMENT type of elements stored in vector.
///
/// Base templated class. Template specializations are required
/// for each dimension supported.
//////////////////////////////////////////////////////////////////////
template<int DIM, typename ELEMENT>
class geom_vector
{ };

} //namespace stapl

#include <stapl/containers/graph/mesh/geom_vector2.hpp>
#include <stapl/containers/graph/mesh/geom_vector3.hpp>

#endif
