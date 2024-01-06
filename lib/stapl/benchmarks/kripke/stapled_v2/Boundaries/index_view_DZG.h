#ifndef BOUNDARIES_INDEX_VIEW_DZG_H
#define BOUNDARIES_INDEX_VIEW_DZG_H

#include "common_headers.hpp"

class dzg_zg_idx
{
private:
  int       m_num_groups;
  int       m_direction;

public:
  using index_type  = stapl::homogeneous_tuple_type_t<3, std::size_t>;
  using value_type  = stapl::homogeneous_tuple_type_t<5, std::size_t>;
  using result_type = lightweight_vector<value_type>;

  dzg_zg_idx(int num_groups, int direction)
    : m_num_groups(num_groups), m_direction(direction)
  { }

  result_type operator()(index_type const& idx) const
  {
    result_type result(m_num_groups);

    for (int group = 0; group < m_num_groups; ++group)
    {
      result[group] = tuple_cat(idx, stapl::make_tuple(m_direction, group));
    }

    return result;
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_num_groups);
    t.member(m_direction);
  }
}; // class dzg_zd_idx


class dzg_d_idx
{
private:
  using dimension_t  = stapl::homogeneous_tuple_type_t<3, std::size_t>;

  dimension_t m_dimensions;
  int          m_num_groups;

public:
  // index is the group
  using index_type  = std::size_t;
  using result_type =
    typename functor_view_type<functor_container<dzg_zg_idx, 3>>::type;

  dzg_d_idx(dimension_t const& dimensions, int num_groups)
    : m_dimensions(dimensions),
      m_num_groups(num_groups)
  { }

  result_type operator()(index_type const& idx) const
  {
    return functor_view(
      m_dimensions,
      dzg_zg_idx(m_num_groups, idx)
    );
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_dimensions);
    t.member(m_num_groups);
  }
}; // class dzg_d_idx


template <Nesting_Order>
struct index_view_traits;

template <>
struct index_view_traits<NEST_DZG>
{
  using D3_traversal = stapl::index_sequence<2, 1, 0>;
  using D3_spec = stapl::md_distribution_spec<D3_traversal>::type;

  using D1_traversal = stapl::index_sequence<0>;
  using D1_spec = stapl::distribution_spec<>;;

  using dist_t = typename stapl::array<double>::distribution_type;

  using index_gen_t   = dzg_d_idx;
  using zoneset_index_cont_t = functor_container<index_gen_t, 1, dist_t>;

  using index_cont_t  = multiarray<3, zoneset_index_cont_t, D3_spec>;
  using index_view_t  = multiarray_view<index_cont_t>;

  using spec_index_gen_t =
    get_boundary_spec_wf_1D<D3_spec, D1_spec, D3_traversal, D1_traversal>;


  template <typename Outer, typename Inner>
  static index_gen_t index_gen(Outer&& outer,
                               Inner&& inner,
                               int num_groups,
                               int num_dirs)
  {
    return index_gen_t(std::forward<Inner>(inner), num_groups);
  }

  template <typename Inner, typename BndaryGen>
  static auto index_factory(Inner&& inner, BndaryGen&& index_gen)
  STAPL_AUTO_RETURN((
    boost::bind(
      boost::factory<zoneset_index_cont_t*>(),
      std::forward<Inner>(inner),
      std::forward<BndaryGen>(index_gen),
      _1)
  ))

  template <typename Outer, typename Inner, typename Location>
  static auto make_index_spec(Outer const& outer, Inner const& inner,
                               Location const& locations)
  STAPL_AUTO_RETURN((
    stapl::make_heterogeneous_composed_dist_spec<
      spec_index_gen_t, D3_spec, D1_spec>(
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
        num_directions,
        index_gen(std::forward<NumZones>(num_zones),
                  std::forward<ZoneSize>(zone_size),
                  num_groups, num_directions));

    auto&& index_spec = make_index_spec(num_zones, num_directions, locations);

    index_cont_t * index_ct = new index_cont_t(index_spec, value_factory);
    return new index_view_t(index_ct);
  }
};

#endif // BOUNDARIES_INDEX_VIEW_DZG_H
