#ifndef BOUNDARIES_INDEX_VIEW_ZDG_H
#define BOUNDARIES_INDEX_VIEW_ZDG_H

#include "common_headers.hpp"

class zdg_idx
{
private:

  int       m_num_directions;
  int       m_num_groups;

public:
  using index_type  = stapl::homogeneous_tuple_type_t<3, std::size_t>;
  using value_type  = stapl::homogeneous_tuple_type_t<5, std::size_t>;;
  using result_type = lightweight_vector<lightweight_vector<value_type>>;

  zdg_idx(int num_groups, int num_directions)
    : m_num_directions(num_directions),
      m_num_groups(num_groups)
  { }

  result_type operator()(index_type const& idx) const
  {
    result_type result;
    result.resize(m_num_directions);

    for (int direction = 0; direction < m_num_directions; ++direction)
    {
      result[direction].resize(m_num_groups);

      for (int group = 0; group < m_num_groups; ++group)
      {
        result[direction][group]
          = tuple_cat(idx, stapl::make_tuple(direction, group));
      }
    }

    return result;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_directions);
    t.member(m_num_groups);
  }
}; // class zdg_idx



template <Nesting_Order>
struct index_view_traits;

template <>
struct index_view_traits<NEST_ZDG>
{
  using D3_traversal = stapl::index_sequence<2, 1, 0>;
  using D3_spec = stapl::md_distribution_spec<D3_traversal>::type;

  using index_gen_t = zdg_idx;

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
    return new index_view_t(index_ct);
  }
};

#endif // BOUNDARIES_INDEX_VIEW_ZDG_H
