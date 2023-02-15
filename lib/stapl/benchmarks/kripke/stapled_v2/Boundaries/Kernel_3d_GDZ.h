/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef KRIPKE_BOUNDARY_KERNEL_3D_GDZ_H__
#define KRIPKE_BOUNDARY_KERNEL_3D_GDZ_H__

#include "common_headers.hpp"


template<int OuterIdx, int InnerIdx>
class gdz_z_boundary_gen
{
private:
  using plane_t = std::vector<double>;

  plane_t * m_plane;
  int       m_idx_max;
  int       m_num_directions;
  int       m_num_groups;
  int       m_direction;
  int       m_group;

public:
  using index_type  = std::tuple<std::size_t, std::size_t, std::size_t>;
  using result_type = double;

  gdz_z_boundary_gen(plane_t * plane, int idx_max,
                     int num_directions, int num_groups,
                     int direction, int group)
    : m_plane(plane), m_idx_max(idx_max),
      m_num_directions(num_directions), m_num_groups(num_groups),
      m_direction(direction), m_group(group)
  { }

  result_type operator()(index_type const& idx) const
  {
    const int plane_idx =
     ((std::get<InnerIdx>(idx) * (m_idx_max)) + std::get<OuterIdx>(idx))
       * m_num_directions * m_num_groups
     + (m_direction * m_num_groups)
     + m_group;

    return (*m_plane)[plane_idx];
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_plane);
    t.member(m_idx_max);
    t.member(m_num_directions);
    t.member(m_num_groups);
    t.member(m_direction);
    t.member(m_group);
  }
}; // class gdz_z_boundary_gen


template<int OuterIdx, int InnerIdx>
class gdz_d_boundary_gen
{
private:
  using plane_t     = std::vector<double>;
  using dimension_t = std::tuple<std::size_t, std::size_t, std::size_t>;

  plane_t *   m_plane;
  int         m_idx_max;
  dimension_t m_dimensions;
  int         m_num_directions;
  int         m_num_groups;
  int         m_group;

public:
  // index is the direction
  using index_type  = std::size_t;
  using value_type =
    typename functor_view_type<
      functor_container<gdz_z_boundary_gen<OuterIdx, InnerIdx>, 3>>::type;

  using result_type = std::array<value_type, 3>;

  gdz_d_boundary_gen(plane_t * plane, int idx_max,
                     dimension_t const& dimensions,
                     int num_directions, int num_groups, int group)
    : m_plane(plane), m_idx_max(idx_max), m_dimensions(dimensions),
      m_num_directions(num_directions), m_num_groups(num_groups),
      m_group(group)
  { }

  result_type operator()(index_type const& idx) const
  {
    value_type view_gen =
      functor_view(
        m_dimensions,
        gdz_z_boundary_gen<OuterIdx, InnerIdx>(
          m_plane, m_idx_max, m_num_directions, m_num_groups, idx, m_group));

    result_type res = {view_gen, view_gen, view_gen};
    return res;
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_plane);
    t.member(m_idx_max);
    t.member(m_dimensions);
    t.member(m_num_directions);
    t.member(m_num_groups);
    t.member(m_group);
  }
}; // class gdz_d_boundary_gen


template<int OuterIdx, int InnerIdx>
class gdz_g_boundary_gen
{
private:
  using plane_t     = std::vector<double>;
  using dimension_t = std::tuple<std::size_t, std::size_t, std::size_t>;

  plane_t *   m_plane;
  int         m_idx_max;
  dimension_t m_dimensions;
  int         m_num_directions;
  int         m_num_groups;

public:
  // index is the group
  using index_type  = std::size_t;
  using result_type =
    typename functor_view_type<
      functor_container<gdz_d_boundary_gen<OuterIdx, InnerIdx>, 1>>::type;

  gdz_g_boundary_gen(plane_t * plane,
                     dimension_t const& dimensions,
                     int num_directions, int num_groups, int idx_max)
    : m_plane(plane),  m_idx_max(idx_max), m_dimensions(dimensions),
      m_num_directions(num_directions), m_num_groups(num_groups)
  {  }

  result_type operator()(index_type const& idx) const
  {
    return functor_view(
      m_num_directions,
      gdz_d_boundary_gen<OuterIdx, InnerIdx>
        (m_plane, m_idx_max, m_dimensions, m_num_directions, m_num_groups, idx)
    );
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_plane);
    t.member(m_idx_max);
    t.member(m_dimensions);
    t.member(m_num_directions);
    t.member(m_num_groups);
  }
}; // class gdz_g_boundary_gen


template <std::size_t axis_num>
struct gdz_g_boundary_gen_type
{
  static constexpr direction dir = (direction)axis_num;
  using type = gdz_g_boundary_gen<1, 2>;
};

template<>
struct gdz_g_boundary_gen_type<1>
{
  static constexpr direction dir = (direction)1;
  using type = gdz_g_boundary_gen<0, 2>;
};

template<>
struct gdz_g_boundary_gen_type<2>
{
  static constexpr direction dir = (direction)2;
  using type = gdz_g_boundary_gen<0, 1>;
};


template <size_t dir, Nesting_Order>
struct grid_boundary_traits;

template <size_t dir>
struct grid_boundary_traits<dir, NEST_GDZ>
{
  using D3_traversal = stapl::index_sequence<2, 1, 0>;
  using D3_spec = stapl::md_distribution_spec<D3_traversal>::type;

  using D1_traversal = stapl::index_sequence<0>;
  using D1_spec = stapl::distribution_spec<>;

  using dist_t = typename stapl::array<double>::distribution_type;

  using boundary_gen_t   = typename gdz_g_boundary_gen_type<dir>::type;
  using zoneset_boundary_cont_t = functor_container<boundary_gen_t, 1, dist_t>;

  using boundary_cont_t  = multiarray<3, zoneset_boundary_cont_t, D3_spec>;
  using boundary_view_t  = multiarray_view<boundary_cont_t>;
  using variable_type  = boundary_view_t;

  using spec_boundary_gen_t =
    get_boundary_spec_wf_1D<D3_spec, D1_spec, D3_traversal, D1_traversal>;


  template <typename Outer, typename Inner>
  static boundary_gen_t boundary_gen(Outer&& outer,
                                   Inner&& inner,
                                   int num_groups,
                                   int num_dirs,
                                   int idx_max)
  {
    return boundary_gen_t(
      gen_bdry_ones::get_plane(
        std::forward<Inner>(inner), num_groups, num_dirs, dir),
      std::forward<Inner>(inner), num_dirs, num_groups, idx_max);
  }

  template <typename Inner, typename BndaryGen>
  static auto boundary_factory(Inner&& inner, BndaryGen&& boundary_gen)
  STAPL_AUTO_RETURN((
    boost::bind(
      boost::factory<zoneset_boundary_cont_t*>(),
      std::forward<Inner>(inner),
      std::forward<BndaryGen>(boundary_gen),
      _1)
  ))

  template <typename Outer, typename Inner, typename Location>
  static auto make_boundary_spec(Outer const& outer, Inner const& inner,
                               Location const& locations)
  STAPL_AUTO_RETURN((
    stapl::make_heterogeneous_composed_dist_spec<
      spec_boundary_gen_t, D3_spec, D1_spec>(
        spec_boundary_gen_t(outer, inner, locations))
  ))


  template <typename Location, typename NumZones, typename ZoneSize>
  static boundary_view_t * make_view(Location&& locations, NumZones&& num_zones,
                                   ZoneSize&& zone_size, size_t num_groups,
                                   size_t num_directions)
  {
    auto&& value_factory =
      boundary_factory(
        num_groups,
        boundary_gen(std::forward<NumZones>(num_zones),
                     std::forward<ZoneSize>(zone_size),
                     num_groups, num_directions, std::get<dir>(zone_size)));

    auto&& boundary_spec = make_boundary_spec(num_zones, num_groups, locations);

    boundary_cont_t* boundary_ct =
      new boundary_cont_t(boundary_spec, value_factory);

    return new boundary_view_t(boundary_ct);
  }
};

#endif // KRIPKE_BOUNDARY_KERNEL_3D_GDZ_H__
