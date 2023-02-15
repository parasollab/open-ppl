/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_KERNEL_WORK_FUNCTIONS_HPP
#define STAPL_BENCHMARKS_KERNEL_WORK_FUNCTIONS_HPP

#include <GroupDirSet.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Functor used in Kernel::LTimes to clear phi in each vertex.
//////////////////////////////////////////////////////////////////////
struct clear_phi
{
private:
  double m_value;
  int m_total_num_groups;
  int m_total_moments;

public:
  typedef void result_type;

  clear_phi(double const& value, int total_num_groups, int total_moments)
    : m_value(value), m_total_num_groups(total_num_groups),
      m_total_moments(total_moments)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Calls assign on the phi vector with the value provided
  /// at construction.
  /// @param v Proxy over the vertex representing a spatial zone
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    auto phi_ref = v.property().phi();
    std::array<typename Vertex::property_type::storage_type::index, 2>
      index{{0, 0}};
    for (int i = 0; i != m_total_num_groups; ++i)
      for (int j = 0; j != m_total_moments; ++j)
      {
        index[0] = i;
        index[1] = j;
        phi_ref(index) = m_value;
      }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_value);
    t.member(m_total_num_groups);
    t.member(m_total_moments);
  }
};

struct allocate_vertex_members
{
private:
  Nesting_Order                           m_nest;
  Nesting_Order                           m_nest_phi;
  int                                     m_total_num_groups;
  int                                     m_total_moments;
  int                                     m_total_dirs;
  std::vector<std::vector<Group_Dir_Set>> m_gd_sets;

public:
  typedef void result_type;

  allocate_vertex_members(Nesting_Order nest, Nesting_Order nest_phi,
    int total_num_groups, int total_moments, int total_dirs,
    std::vector<std::vector<Group_Dir_Set>> const& gd_sets)
    : m_nest(nest), m_nest_phi(nest_phi), m_total_num_groups(total_num_groups),
      m_total_moments(total_moments), m_total_dirs(total_dirs),
      m_gd_sets(gd_sets)
  {}

  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    v.property().allocate(m_nest, m_nest_phi,
                          m_total_num_groups, m_total_moments, m_total_dirs,
                          m_gd_sets);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_nest);
    t.member(m_nest_phi);
    t.member(m_total_num_groups);
    t.member(m_total_moments);
    t.member(m_total_dirs);
    t.member(m_gd_sets);
  }
};
#endif
