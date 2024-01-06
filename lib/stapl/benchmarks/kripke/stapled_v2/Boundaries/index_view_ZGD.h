#ifndef BOUNDARIES_INDEX_VIEW_ZGD_H
#define BOUNDARIES_INDEX_VIEW_ZGD_H

#include "common_headers.hpp"

class zgd_d_idx
{
private:
  using zone_idx_t = stapl::homogeneous_tuple_type_t<3, std::size_t>;

  zone_idx_t  m_zone_idx;
  int         m_group;

public:
  using index_type   = size_t;
  using result_type  = stapl::homogeneous_tuple_type_t<5, std::size_t>;;

  zgd_d_idx(zone_idx_t const& zone_idx, int group)
    : m_zone_idx(zone_idx),
      m_group(group)
  { }

  result_type operator()(size_t idx) const
  {
    return tuple_cat(m_zone_idx, stapl::make_tuple(idx, m_group));;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zone_idx);
    t.member(m_group);
  }
}; // class zgd_d_idx


class zgd_g_idx
{
private:
  using zone_idx_t = stapl::homogeneous_tuple_type_t<3, std::size_t>;

  zone_idx_t  m_zone_idx;
  int         m_num_directions;

public:
  using index_type   = size_t;
  using result_type =
    typename functor_view_type<functor_container<zgd_d_idx, 1>>::type;

  zgd_g_idx(zone_idx_t const& zone_idx, int num_directions)
    : m_zone_idx(zone_idx),
      m_num_directions(num_directions)
  { }

  result_type operator()(size_t idx) const
  {
    return functor_view(m_num_directions, zgd_d_idx(m_zone_idx, idx));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zone_idx);
    t.member(m_num_directions);
  }
}; // class zgd_idx


class zgd_z_idx
{
private:
  int       m_num_directions;
  int       m_num_groups;

public:
  using index_type  = stapl::homogeneous_tuple_type_t<3, std::size_t>;
  using result_type =
    typename functor_view_type<functor_container<zgd_g_idx, 1>>::type;

  zgd_z_idx(int num_groups, int num_directions)
    : m_num_directions(num_directions),
      m_num_groups(num_groups)
  { }

  result_type operator()(index_type const& idx) const
  {
    return functor_view(m_num_groups, zgd_g_idx(idx, m_num_directions));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_directions);
    t.member(m_num_groups);
  }
}; // class zgd_idx


template <Nesting_Order>
struct index_view_traits;

template <>
struct index_view_traits<NEST_ZGD>
{
  using D3_traversal = stapl::index_sequence<2, 1, 0>;
  using D3_spec = stapl::md_distribution_spec<D3_traversal>::type;

  // using index_gen_t = zgd_idx;
  using index_gen_t = zgd_z_idx;

  using dist_t =
    typename stapl::multiarray<3, double, D3_spec>::distribution_type;

  using zoneset_index_cont_t = functor_container<index_gen_t, 3, dist_t>;

  using index_cont_t     = multiarray<3, zoneset_index_cont_t, D3_spec>;
  using index_view_t     = multiarray_view<index_cont_t>;
  using variable_type    = index_view_t;

  using spec_index_gen_t =
    get_boundary_spec_wf<D3_spec, D3_spec, D3_traversal, D3_traversal>;


  template <typename Outer, typename Inner>
  static index_gen_t index_gen(Outer&& outer,
                               Inner&& inner,
                               int num_groups,
                               int num_dirs)
  {
    return index_gen_t(num_groups, num_dirs);
  }

  template <typename Inner, typename IndexGen>
  static auto index_factory(Inner&& inner, IndexGen&& index_gen)
  STAPL_AUTO_RETURN((
    boost::bind(
      boost::factory<zoneset_index_cont_t*>(),
      std::forward<Inner>(inner),
      std::forward<IndexGen>(index_gen),
      _1)
  ))

  template <typename Outer, typename Inner, typename Location>
  static auto make_index_spec(Outer const& outer, Inner const& inner,
                               Location const& locations)
  STAPL_AUTO_RETURN((
    stapl::make_heterogeneous_composed_dist_spec<
      spec_index_gen_t, D3_spec, D3_spec>(
        spec_index_gen_t(outer, inner, locations))
  ))


  template <typename Location, typename NumZones, typename ZoneSize>
  static index_view_t *
  make_view(Location&& locations, NumZones&& num_zones,
            ZoneSize&& zone_size, size_t num_groups,
            size_t num_directions)
  {
    auto&& value_factory =
      index_factory(
        std::forward<ZoneSize>(zone_size),
        index_gen(std::forward<NumZones>(num_zones),
                  std::forward<ZoneSize>(zone_size),
                  num_groups, num_directions));

    auto&& index_spec =
      make_index_spec(num_zones, std::forward<ZoneSize>(zone_size), locations);

    index_cont_t * index_ct = new index_cont_t(index_spec, value_factory);
    stapl::rmi_fence();
    return new index_view_t(index_ct);
  }
};

#endif // BOUNDARIES_INDEX_VIEW_ZGD_H
