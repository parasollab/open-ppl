/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


/*!
  \file Cell.h

   This file defines a 3D discretized spatial unit.  It is used as the vertex
   property of the pGraph representing the grid.
*/

#ifndef STAPL_BENCHMARKS_KRIPKE_CELL_H
#define STAPL_BENCHMARKS_KRIPKE_CELL_H

#include <vector>
#include <boost/multi_array.hpp>
#include <GroupDirSet.hpp>
#include <stapl/views/proxy_macros.hpp>
#include <stapl/runtime.hpp>

// Class stored as vertex property of graph used to represent zonal grid.
// The dimension of the data structures is groupsxdirections, as there is an
// instance of the class per zone.
class Cell3D
{
public:
  typedef boost::multi_array<double, 2> storage_type;

private:
  // Keep up with nesting orders specified.
  Nesting_Order m_nest;
  Nesting_Order m_nest_phi;

  // These variables are total_num_groups * total_num_angles.
  storage_type m_sigt;
  storage_type m_phi;
  storage_type m_phi_out;
  storage_type m_ell;
  storage_type m_ell_plus;

  // Variables relocated from Group_Set_Dir
  // Dimensionality is # GroupSets * # Direction Sets.
  // Each element has dimensionality of
  // # groups per groupset * # directions per direction set

  // Solution
  std::vector<std::vector<storage_type>> m_psi;

  // RHS, source term
  std::vector<std::vector<storage_type>> m_rhs;

  int m_groups;

public:
  void allocate(Nesting_Order nest, Nesting_Order nest_phi,
         int total_num_groups, int total_moments, int total_dirs,
         std::vector<std::vector<Group_Dir_Set>> const& gd_sets)
  {
    m_nest = nest;
    m_nest_phi = nest_phi;

    m_groups = total_num_groups;

    // TGS : determine how to use m_nesting_phi to set layout of
    // m_phi, m_phi_out
    m_phi.resize(boost::extents[total_num_groups][total_moments]);
    m_phi_out.resize(boost::extents[total_num_groups][total_moments]);

    if (nest == NEST_GDZ || nest == NEST_DZG || nest == NEST_DGZ){
      // Use NEST_ZGD for ell
      // TGS: Z dim was specified as 1 instead of num_zones.  Move to Grid?
      m_ell.resize(boost::extents[total_moments][total_dirs]);

      // Use NEST_ZDG for ell_plus.
      // TGS: Z dim was specified as 1 instead of num_zones.  Move to Grid?
      m_ell_plus.resize(boost::extents[total_moments][total_dirs]);
    }
    else{
      // Use NEST_ZDG for ell
      // TGS: Z dim was specified as 1 instead of num_zones.  Move to Grid?
      m_ell.resize(boost::extents[total_moments][total_dirs]);

      // Use NEST_ZDG for ell_plus
      // TGS: Z dim was specified as 1 instead of num_zones.  Move to Grid?
      m_ell_plus.resize(boost::extents[total_moments][total_dirs]);
    }

    // allocate sigt  1xGxZ if groups come before zones
    if(nest == NEST_GDZ || nest ==  NEST_DGZ || nest == NEST_GZD){
      // Use NEST_DGZ for sigt
      m_sigt.resize(boost::extents[total_num_groups][1]);
    }
    // otherwise, 1xZxG
    else{
      // Use NEST_DZG for sigt
      m_sigt.resize(boost::extents[total_num_groups][1]);
    }

    int num_groupsets = gd_sets.size();
    m_psi.resize(num_groupsets);
    m_rhs.resize(num_groupsets);
    for (int gs = 0; gs != num_groupsets; ++gs)
    {
      int num_dirsets = gd_sets[gs].size();
      m_psi[gs].resize(num_dirsets);
      m_rhs[gs].resize(num_dirsets);
      for (int ds = 0; ds != num_dirsets; ++ds)
      {
        Group_Dir_Set const& gd_set = gd_sets[gs][ds];
        m_psi[gs][ds].resize(
          boost::extents[gd_set.num_groups][gd_set.num_directions]);
        m_rhs[gs][ds].resize(
          boost::extents[gd_set.num_groups][gd_set.num_directions]);
      }
    }
  }

  void copy(Cell3D const& b)
  {
    m_groups   = b.groups();
    m_sigt     = b.sigt();
    m_phi      = b.phi();
    m_phi_out  = b.phi_out();
    m_ell      = b.ell();
    m_ell_plus = b.ell_plus();
    m_psi      = b.psi();
    m_rhs      = b.rhs();
  }

  int groups(void) const
  { return m_groups; }

  storage_type const& sigt(void) const
  { return m_sigt; }

  storage_type& sigt(void)
  { return m_sigt; }

  storage_type const& phi(void) const
  { return m_phi; }

  storage_type& phi(void)
  { return m_phi; }

  storage_type const& phi_out(void) const
  { return m_phi_out; }

  storage_type& phi_out(void)
  { return m_phi_out; }

  storage_type const& ell(void) const
  { return m_ell; }

  storage_type& ell(void)
  { return m_ell; }

  storage_type const& ell_plus(void) const
  { return m_ell_plus; }

  storage_type& ell_plus(void)
  { return m_ell_plus; }

  std::vector<std::vector<storage_type>> const& psi(void) const
  { return m_psi; }

  std::vector<std::vector<storage_type>>& psi(void)
  { return m_psi; }

  std::vector<std::vector<storage_type>> const& rhs(void) const
  { return m_rhs; }

  std::vector<std::vector<storage_type>>& rhs(void)
  { return m_rhs; }

  void define_type(stapl::typer& t)
  {
    stapl::abort("boost::multi_array::define_type needed to serialize Cell3D");
  }
};

bool operator==(Cell3D const& lhs, Cell3D const& rhs);

namespace stapl {

template <typename Accessor>
class proxy<Cell3D, Accessor>
  : public Accessor
{
  friend class proxy_core_access;

  typedef Cell3D target_t;

public:
  typedef boost::multi_array<double, 2> storage_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  void copy(target_t const& b)
  { Accessor::invoke(&target_t::copy, b); }

  void allocate(Nesting_Order nest, Nesting_Order nest_phi,
         int total_num_groups, int total_moments, int total_dirs,
         std::vector<std::vector<Group_Dir_Set>> const& gd_sets)
  {
    Accessor::invoke(&target_t::allocate, nest, nest_phi, total_num_groups,
      total_moments, total_dirs, gd_sets);
  }

  STAPL_PROXY_REFERENCE_METHOD_0(sigt, storage_type)
  STAPL_PROXY_REFERENCE_METHOD_0(phi, storage_type)
  STAPL_PROXY_REFERENCE_METHOD_0(phi_out, storage_type)
  STAPL_PROXY_REFERENCE_METHOD_0(ell, storage_type)
  STAPL_PROXY_REFERENCE_METHOD_0(ell_plus, storage_type)

  STAPL_PROXY_REFERENCE_METHOD_0(psi, std::vector<std::vector<storage_type>>)

  STAPL_PROXY_REFERENCE_METHOD_0(rhs, std::vector<std::vector<storage_type>>)

  int groups(void) const
  { return Accessor::const_invoke(&target_t::groups); }
};


template <typename T, std::size_t Dim, typename Accessor>
class proxy<boost::multi_array<T, Dim>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef boost::multi_array<T, Dim> target_t;

  STAPL_PROXY_CONSTRUCTOR(target_t, Accessor)
  STAPL_PROXY_METHODS(target_t, Accessor)

public:
  STAPL_PROXY_REFLECT_TYPE(index)
  typedef std::array<index, Dim> tuple_t;

public:
  STAPL_PROXY_REFERENCE_METHOD_1(inner, operator(), T, tuple_t)

}; // class proxy<boost::multi_array<>

} // namespace stapl
#endif
