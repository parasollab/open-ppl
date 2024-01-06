#ifndef KRIPKE_KERNEL_3D_ZDG_H__
#define KRIPKE_KERNEL_3D_ZDG_H__

#include "common_headers.hpp"
#include "common_kernel_functions.h"

class Kernel_3d_ZDG
  : public Kernel
{
public:
  static constexpr Nesting_Order nesting_val = NEST_ZDG;
  // Grid is needed to access metadata (e.g. gd_sets) stored on it.
  using grid_data_t = Grid_Data<nesting_val>;
  grid_data_t* grid_data;

  Kernel_3d_ZDG(Grid_Data_Base*);
  virtual ~Kernel_3d_ZDG();

  void LTimes(void);
  void LPlusTimes(void);

  void define_type(stapl::typer& t)
  {
    t.base<Kernel>(*this);
    t.member(grid_data);
  }
};


struct ZDG_filter
{
  using dir_t = stapl::skeletons::direction;

  dir_t m_direction;

  template <typename F>
  struct result;

  template <typename V>
  struct result<ZDG_filter(V)>
  {
    using type = typename std::decay<V>::type;
  };

  void set_direction(dir_t direction)
  {
    m_direction = direction;
  }

  template <typename V>
  typename result<ZDG_filter(V)>::type
  operator()(V&& v) const
  {
    using result_t = typename result<ZDG_filter(V)>::type;
    result_t res;

    res[(size_t)m_direction] = v[(size_t)m_direction];

    return res;
  }

  void define_type(typer& t)
  {
    t.member(m_direction);
  }
};


template <Nesting_Order>
struct kernel_skeleton;

template <>
struct kernel_skeleton<NEST_ZDG>
{
  constexpr static size_t ndims0 = 3;
  constexpr static size_t ndims1 = 3;
  constexpr static size_t ndims2 = 1;
  constexpr static size_t ndims3 = 1;

  using coarse_tag     = stapl::skeletons::tags::only_boundary;
  using wf_exec_tag    = stapl::skeletons::tags::seq_exec_only_boundary;
  using kripke_seq_tag = stapl::skeletons::tags::kripke_sequential_execution;

  constexpr static size_t num_inputs = 4;

  template <typename Corner, typename Op>
  static auto make(Corner&& corner, Op&& op)
    STAPL_AUTO_RETURN((wavefront<num_inputs>(
      coarse<coarse_tag, wf_exec_tag>(wavefront<num_inputs>(
        wrap<kripke_seq_tag>(zip<ndims0 + num_inputs>(
          wrap<kripke_seq_tag>(zip<ndims0 + num_inputs>(
            std::forward<Op>(op),
            skeleton_traits<blocked<ndims3>, true>())),
          skeleton_traits<blocked<ndims2>, true>())),
        corner,
        skeleton_traits<blocked<ndims1>, true>())),
      corner,
      skeleton_traits<blocked<ndims0>>(ZDG_filter()),
      execution_params(pg_aware_multiview_coarsener()))))

  template <typename Locs, typename Zonesets, typename Skeleton>
  static void set_level_dims(
    Locs&& locs, Zonesets&& zone_sets,
    std::tuple<size_t, size_t, size_t> const& zone_size,
    size_t num_groups, size_t num_directions, Skeleton&& sk)
  {
    using namespace stapl::tuple_ops;

    using nested_size_t = std::tuple<size_t, size_t, size_t>;
    std::array<nested_size_t, 2> level_dims;
    level_dims[0] = zone_sets;
    level_dims[1] = stapl::tuple_ops::transform(locs, zone_sets,
                                                stapl::divides<std::size_t>());

    size_t num_locs = fold(level_dims[1], 1, stapl::multiplies<size_t>());
    size_t zone_sz  = fold(zone_size, 1, stapl::multiplies<size_t>());

    level_dims[1] = num_locs <= zone_sz ? level_dims[1] : zone_size;

    sk.span().set_level_dims(level_dims);
  }
};

#endif
