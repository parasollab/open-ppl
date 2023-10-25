/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_NODE_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_NODE_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

namespace ordering_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a connection between base containers (ordering pointer).
///
/// A connection is modeled as a pointer, composed of a location where
/// the pointed element is located and a local id to identify the
/// local component inside the location.
//////////////////////////////////////////////////////////////////////
struct ptr_bcontainer
{
  affinity_tag                affinity;
  rmi_handle::reference       handle;
  location_type               location;
  size_t                      id;

  ptr_bcontainer()
    : affinity(invalid_affinity_tag),
      location(index_bounds<location_type>::invalid()),
      id(index_bounds<size_t>::invalid())
  { }

  ptr_bcontainer(rmi_handle::reference h, location_type l, size_t i)
    : affinity(invalid_affinity_tag),
      handle(h),
      location(l),
      id(i)
  { }

  ptr_bcontainer(p_object& p, location_type l, size_t i)
    : affinity(invalid_affinity_tag),
      handle(p.get_rmi_handle()),
      location(l),
      id(i)
  { }

  void define_type(typer& t)
  {
    t.member(affinity);
    t.member(handle);
    t.member(location);
    t.member(id);
  }

  bool operator==(ptr_bcontainer const& other) const
  {
    return location == other.location && id == other.id;
  }

  bool operator!=(ptr_bcontainer const& other) const
  {
    return !(*this == other);
  }

  friend std::ostream&
  operator<<(std::ostream &os, ptr_bcontainer const& d);

}; // struct ptr_bcontainer


inline
std::ostream& operator<<(std::ostream &os, ptr_bcontainer const& d)
{
  os << "(loc:" << d.location << "," << "id:" << d.id << ")";
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Defines an entry node in the list of base containers.
///
/// This node keeps information about forward and backward connections
/// and the rank associated to the node.
//////////////////////////////////////////////////////////////////////
struct node
{
  typedef ptr_bcontainer ptr_type;

  ptr_bcontainer      prev;
  ptr_bcontainer      next;
  size_t              rank;

  node()
    : prev(),
      next(),
      rank()
  { }

  node(ptr_bcontainer const& p, ptr_bcontainer const& n, size_t id = 0)
    : prev(p), next(n), rank(id)
  { }

  void define_type(typer& t)
  {
    t.member(prev);
    t.member(next);
    t.member(rank);
  }

  friend std::ostream& operator<<(std::ostream &os, node const& d);
}; // struct node


inline
std::ostream& operator<<(std::ostream &os, node const& d)
{
  os << "(prev:" << d.prev << ", "
     << "next:"<< d.next << ", "
     << "rank:" << d.rank << ") ";

  return os;
}

} // namespace ordering_impl

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_NODE_HPP
