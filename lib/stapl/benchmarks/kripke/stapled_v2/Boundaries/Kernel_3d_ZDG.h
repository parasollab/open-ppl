/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef KRIPKE_BOUNDARY_KERNEL_3D_ZDG_H__
#define KRIPKE_BOUNDARY_KERNEL_3D_ZDG_H__

#include "common_headers.hpp"


template <int OuterIdx, int InnerIdx>
class zdg_boundary_gen
{
private:
  using plane_t = std::vector<double>;

  plane_t * m_plane;
  int       m_num_directions;
  int       m_num_groups;
  int       m_idx_max;

public:
  constexpr static size_t dims_num = 3;
  using index_type  = std::tuple<std::size_t, std::size_t, std::size_t>;
  using result_type = lightweight_vector<lightweight_vector<double>>;

  zdg_boundary_gen(plane_t * plane,
                   int num_groups, int num_directions, int idx_max)
    : m_plane(plane),
      m_num_directions(num_directions),
      m_num_groups(num_groups),
      m_idx_max(idx_max)
  { }

  result_type operator()(index_type const& idx) const
  {
    using domain_t = indexed_domain<size_t, 1>;

    result_type result;
    result.resize(m_num_directions);

    // Calculating the offset outside of the loop in order to stride by
    // number of directions * number of groups from idx
    const int offset = ((std::get<InnerIdx>(idx) * m_idx_max)
        + std::get<OuterIdx>(idx)) * m_num_directions * m_num_groups;

    for (int direction = 0; direction < m_num_directions; ++direction)
    {
      result[direction].resize(m_num_groups);

      for (int group = 0; group < m_num_groups; ++group)
      {
        result[direction][group]
          = (*m_plane)[(direction * m_num_groups) + group + offset];
      }
    }

    return result;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_plane);
    t.member(m_num_directions);
    t.member(m_num_groups);
    t.member(m_idx_max);
  }
}; // class zdg_boundary_gen


template <std::size_t axis_num>
struct zdg_boundary_gen_type
{
  static constexpr direction dir = (direction)axis_num;
  using type = zdg_boundary_gen<1, 2>;
};

template<>
struct zdg_boundary_gen_type<1>
{
  static constexpr direction dir = (direction)1;
  using type = zdg_boundary_gen<0, 2>;
};

template<>
struct zdg_boundary_gen_type<2>
{
  static constexpr direction dir = (direction)2;
  using type = zdg_boundary_gen<0, 1>;
};



template <size_t dir, Nesting_Order>
struct grid_boundary_traits;

template <size_t dir>
struct grid_boundary_traits<dir, NEST_ZDG>
{
  using D3_traversal = stapl::index_sequence<2, 1, 0>;
  using D3_spec = stapl::md_distribution_spec<D3_traversal>::type;

  using boundary_gen_t = typename zdg_boundary_gen_type<dir>::type;

  using dist_t =
    typename stapl::multiarray<3, double, D3_spec>::distribution_type;

  using zoneset_boundary_cont_t = functor_container<boundary_gen_t, 3, dist_t>;

  using boundary_cont_t  = multiarray<3, zoneset_boundary_cont_t, D3_spec>;
  using boundary_view_t  = multiarray_view<boundary_cont_t>;
  using variable_type  = boundary_view_t;

  using spec_boundary_gen_t =
    get_boundary_spec_wf<D3_spec, D3_spec, D3_traversal, D3_traversal>;

  template <typename Inner>
  static boundary_gen_t boundary_gen(Inner&& inner,
                                   int num_groups,
                                   int num_dirs,
                                   int idx_max)
  {
    return boundary_gen_t(
      gen_bdry_ones::get_plane(
        std::forward<Inner>(inner), num_groups, num_dirs, dir),
      num_groups, num_dirs, idx_max);
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
      spec_boundary_gen_t, D3_spec, D3_spec>(
        spec_boundary_gen_t(outer, inner, locations))
  ))

  template <typename Location, typename NumZones, typename ZoneSize>
  static boundary_view_t * make_view(Location&& locations, NumZones&& num_zones,
                                   ZoneSize&& zone_size, size_t num_groups,
                                   size_t num_directions)
  {
    auto&& value_factory =
      boundary_factory(
        std::forward<ZoneSize>(zone_size),
        boundary_gen(std::forward<ZoneSize>(zone_size), num_groups,
                     num_directions, std::get<dir>(zone_size)));

    auto&& boundary_spec = make_boundary_spec(num_zones, zone_size, locations);

    boundary_cont_t* boundary_ct =
      new boundary_cont_t(boundary_spec, value_factory);

    return new boundary_view_t(boundary_ct);
  }
};

#endif // KRIPKE_BOUNDARY_KERNEL_3D_ZDG_H__
