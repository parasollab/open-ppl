/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_COM
#define STAPL_BENCHMARK_LONESTAR_BH_COM

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Visitor for the center of mass traversal that simply accumulates
///        the center of mass from the child vertex, and marks the parent
///        as being active in the traversal.
//////////////////////////////////////////////////////////////////////
class compute_center_of_mass_visitor
{
  point m_coord;
  double m_mass;

public:
  compute_center_of_mass_visitor(point const& coord, double mass)
    : m_coord(coord), m_mass(mass)
  { }

  template<typename Vertex>
  bool operator()(Vertex&& v) const
  {
    v.property().update_center_of_mass(m_coord, m_mass);
    v.property().activate();

    return true;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_coord);
    t.member(m_mass);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function used in the asynchronous traversal that visits
///        each vertex's reverse edges and applies the center of mass
///        visitor.
//////////////////////////////////////////////////////////////////////
struct compute_center_of_mass
{
  typedef bool result_type;

  template<typename OctreeNode, typename Visitor>
  bool operator()(OctreeNode&& node, Visitor&& vis) const
  {
    // if this node is being traversed
    if (node.property().is_active())
    {
      // mark this node as no longer being part of the traversal
      node.property().deactivate();

      // visit the backward edges of this node and apply
      // the center of mass visitor to the targets
      vis.visit_all_edges_if(node,
        compute_center_of_mass_visitor(
          node.property().coord(),
          node.property().mass()
        ),
        [=](typename OctreeNode::adj_edges_type::edge_type const& e) {
          return !e.property().first;
        }
      );

      return true;
    }

    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that sets an octree node to be a source for
///        a traversal if that node is a leaf
//////////////////////////////////////////////////////////////////////
struct set_leaf_to_active
{
  typedef void result_type;

  template<typename OctreeNode>
  void operator()(OctreeNode node)
  {
    if (node.property().is_leaf())
      node.property().activate();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Given an octree, compute the centers of mass of the internal
///        nodes by traversing the tree asynchronously from the leaves
///        to the root
/// @param oct The octree view
//////////////////////////////////////////////////////////////////////
template<typename OctreeView>
void update_centers_of_mass(OctreeView oct)
{
  // set the sources of the traversal to be
  // all the leaves in the octree
  stapl::for_each(oct, set_leaf_to_active());

  // traverse the octree from the leaves to the root,
  // accumulating the centers of mass along the way
  kla_paradigm(compute_center_of_mass(), 0, oct,
    std::numeric_limits<std::size_t>::max()-1
  );
}

#endif
