/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LIST_LIST_GID_HPP
#define STAPL_CONTAINERS_LIST_LIST_GID_HPP

#include <stapl/utility/hash_fwd.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the gid type used by @ref stapl::list.
/// @ingroup plistDistObj
///
/// @tparam Ptr Base container iterator type.
/// @tparam BC Base container type.
/// @todo Make data member private and provide methods for operations
///  to guard against misuse (i.e., base container equivalence without
///  location check).
//////////////////////////////////////////////////////////////////////
template<typename Ptr, typename BC>
struct list_gid
{
  typedef Ptr pointer_type;

  Ptr           m_pointer;
  BC*           m_base_container;
  location_type m_location;

  list_gid(void)
    : m_pointer(),
      m_base_container(nullptr),
      m_location(index_bounds<location_type>::invalid())
  { }

  list_gid(Ptr const& p, BC* bc, location_type loc)
    : m_pointer(p),
      m_base_container(bc),
      m_location(loc)
  { }

  bool base_container_equal(list_gid const& other) const
  {
    return m_location == other.m_location
           && m_base_container == other.m_base_container;
  }

  bool base_container_equal(BC* other_bc, location_type other_loc) const
  {
    return m_location == other_loc && m_base_container == other_bc;
  }

  bool operator==(list_gid const& other) const
  {
    stapl_assert(
      !(m_location == other.m_location &&m_pointer == other.m_pointer)
      || m_base_container == other.m_base_container,
      "gid pointers are equivalent but base container aren't");

    return m_location == other.m_location
           && m_pointer == other.m_pointer;
  }

  bool operator!=(list_gid const& other) const
  {
    return !(*this == other);
  }

  bool valid(void) const
  {
    return m_base_container != nullptr
           && m_location != index_bounds<location_type>::invalid();
  }

  void define_type(typer& t)
  {
    t.member(bitwise(m_pointer));
    t.member(bitwise(m_base_container));
    t.member(m_location);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization to define an invalid value of a @ref list_gid.
/// @ingroup plistDistObj
//////////////////////////////////////////////////////////////////////
template<typename Ptr, typename BC>
struct index_bounds<list_gid<Ptr, BC> >
{
  static list_gid<Ptr, BC> invalid(void)
  {
    return list_gid<Ptr, BC>();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to generate a @ref list_gid based on a given
///        local iterator (base container iterator), and using the
///        base container and location defined during creation.
/// @ingroup plistDistObj
//////////////////////////////////////////////////////////////////////
template <typename GID, typename BC>
class gid_generator
{
public:
  typedef GID gid_type;

private:
  BC*           m_base_container;
  location_type m_location;

public:
  gid_generator(void)
    : m_base_container(0),
      m_location(index_bounds<location_type>::invalid())
  { }

  gid_generator(BC* bc, location_type loc)
    : m_base_container(bc),
      m_location(loc)
  { }

  gid_type operator()(typename gid_type::pointer_type ptr) const
  {
    return gid_type(ptr, m_base_container, m_location);
  }

  template<typename It>
  gid_type operator()(It const& it) const
  {
    return it.index();
  }

  location_type location(void) const
  {
    return m_location;
  }

  void define_type(typer& t)
  {
    t.member(bitwise(m_base_container));
    t.member(m_location);
  }
};


template<typename Ptr, typename BC>
std::ostream& operator<<(std::ostream& os, list_gid<Ptr,BC> const& gid)
{
  if (gid.m_location == get_location_id())
    os << "(" << gid.m_pointer << ","
       << gid.m_base_container << ","
       << gid.m_location << ")";
  else
    os << "( remote ptr,"
       << gid.m_base_container << ","
       << gid.m_location << ")";
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for hash_value over a @ref list_gid
/// @ingroup plistDistObj
//////////////////////////////////////////////////////////////////////
template<typename Ptr, typename BC>
std::size_t hash_value(list_gid<Ptr, BC> const& gid)
{
  boost::hash<void*> hasher;
  // extracts the pointer to the memory referenced by gid
  void* raw_ptr = static_cast<void*>(&(*(gid.m_pointer.operator->())));
  return hasher(raw_ptr);
}

} // namespace stapl

#endif // STAPL_CONTAINERS_LIST_LIST_GID_HPP
