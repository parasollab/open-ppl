/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_BASE_COMP_SPEC_PTR_HPP
#define STAPL_CONTAINERS_BASE_COMP_SPEC_PTR_HPP

#include <stapl/runtime.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for a composed distribution specification.  This struct
/// is used when a nested container is being created on a lower level of the
/// system.  It is passed into the communication group for the container and
/// restores the pointer to the composed distribution specification.  This is
/// needed because the runtime currently does not support sending p_object
/// instances between locations.
//////////////////////////////////////////////////////////////////////
template <typename CompSpec, typename GID>
struct comp_spec_ptr
{
  typedef void is_composed_dist_spec;

  typedef typename CompSpec::distribution_spec distribution_spec;

private:
  p_object_pointer_wrapper<const CompSpec> m_comp_spec;
  GID                                      m_gid;
  mutable bool                             m_ptr_reset;

public:
  comp_spec_ptr(CompSpec const& comp_spec,
                GID const& gid)
    : m_comp_spec(&comp_spec),
      m_gid(gid), m_ptr_reset(false)
  { }

  ~comp_spec_ptr(void)
  {
    if (m_ptr_reset)
      delete m_comp_spec;
  }

  void define_type(typer& t)
  {
    t.member(m_comp_spec);
    t.member(m_gid);
    t.member(m_ptr_reset);
    stapl_assert(m_comp_spec != nullptr,
      "Failed to resolve handle of composed specification.");
  }

  size_t size(void) const
  {
#if 0
    // Code will be used if the destructor of the class will be invoked when
    // args passed to construct<>() are destroyed.
    if (!m_ptr_reset)
    {
      m_comp_spec = new CompSpec((*m_comp_spec)[m_gid]);
      m_ptr_reset = true;
    }
    return m_comp_spec->size();
#endif
    return (*m_comp_spec)[m_gid].size();
  }

  operator distribution_spec(void) const
  {
#if 0
    // Code will be used if the destructor of the class will be invoked when
    // args passed to construct<>() are destroyed.
    if (!m_ptr_reset)
    {
      m_comp_spec = new CompSpec((*m_comp_spec)[m_gid]);
      m_ptr_reset = true;
    }
    return (distribution_spec)(*m_comp_spec);
#endif
    return (distribution_spec)((*m_comp_spec)[m_gid]);
  }

  operator CompSpec(void) const
  {
#if 0
    // Code will be used if the destructor of the class will be invoked when
    // args passed to construct<>() are destroyed.
    if (!m_ptr_reset)
    {
      m_comp_spec = new CompSpec((*m_comp_spec)[m_gid]);
      m_ptr_reset = true;
    }
    return *m_comp_spec;
#endif
    return (*m_comp_spec)[m_gid];
  }

  distribution_spec spec(void) const
  {
#if 0
    // Code will be used if the destructor of the class will be invoked when
    // args passed to construct<>() are destroyed.
    if (!m_ptr_reset)
    {
      m_comp_spec = new CompSpec((*m_comp_spec)[m_gid]);
      m_ptr_reset = true;
    }
    return (distribution_spec)(*m_comp_spec);
#endif
    return (distribution_spec)((*m_comp_spec)[m_gid].spec());
  }

  CompSpec operator[](size_t i) const
  {
#if 0
    // Code will be used if the destructor of the class will be invoked when
    // args passed to construct<>() are destroyed.
    if (!m_ptr_reset)
    {
      m_comp_spec = new CompSpec((*m_comp_spec)[m_gid]);
      m_ptr_reset = true;
    }
    return CompSpec(*m_comp_spec, i);
#endif
    return CompSpec((*m_comp_spec)[m_gid], i);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper struct to remove the comp_spec_ptr type from the
/// composed distribution specification.  This allows the comp_spec_ptr
/// to be used without all containers providing a constructor to accept it.
//////////////////////////////////////////////////////////////////////
template <typename ComposedSpec>
struct strip_comp_spec_ptr
{
  typedef ComposedSpec type;
};

template <typename ComposedSpec, typename GID>
struct strip_comp_spec_ptr<detail::comp_spec_ptr<ComposedSpec, GID>>
{
  typedef ComposedSpec type;
};

} // namspace detail

} // namespace stapl

#endif // STAPL_CONTAINERS_BASE_CONTAINER_HPP
