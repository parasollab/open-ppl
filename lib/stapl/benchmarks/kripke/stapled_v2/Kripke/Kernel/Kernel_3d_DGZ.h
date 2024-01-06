
#ifndef KRIPKE_KERNEL_KERNEL_3D_DGZ_H__
#define KRIPKE_KERNEL_KERNEL_3D_DGZ_H__

#include "common_headers.hpp"
#include "common_kernel_functions.h"


class Kernel_3d_DGZ
  : public Kernel
{
public:
  static constexpr Nesting_Order nesting_val = NEST_DGZ;
  // Grid is needed to access metadata (e.g. gd_sets) stored on it.
  using grid_data_t = Grid_Data<NEST_DGZ>;
  grid_data_t* grid_data;

  Kernel_3d_DGZ(Grid_Data_Base*);
  virtual ~Kernel_3d_DGZ();

  void LTimes(void);
  void LPlusTimes(void);

  void define_type(stapl::typer& t)
  {
    t.base<Kernel>(*this);
    t.member(grid_data);
  }
};


struct DGZ_filter
{
  using dir_t = stapl::skeletons::direction;

  dir_t m_direction;

  template <typename F>
  struct result;

  template <typename V>
  struct result<DGZ_filter(V)>
  {
    using type       = typename std::decay<V>::type;
  };

  void set_direction(dir_t direction)
  {
    m_direction = direction;
  }

  template <typename V>
  typename result<DGZ_filter(V)>::type
  operator()(V&& v) const
  {
    using result_t = typename result<DGZ_filter(V)>::type;
    result_t res;
    res.resize(v.size());

    for (size_t d = 0; d < v.size() ; ++d)
    {
      res[d].resize(v[d].size());
        for (size_t g = 0; g < v[d].size() ; ++g)
          res[d][g][(size_t)m_direction] =
            std::move(v[d][g][(size_t)m_direction]);
    }

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
struct kernel_skeleton<NEST_DGZ>
{
  constexpr static size_t ndims0 = 3;
  constexpr static size_t ndims1 = 1;
  constexpr static size_t ndims2 = 1;
  constexpr static size_t ndims3 = 3;

  using wf_seq_tag_t = stapl::skeletons::tags::seq_exec_only_boundary_sliced;
  using kripke_seq_tag_t = stapl::skeletons::tags::lightweight_seq_execution;

  constexpr static size_t num_inputs = 4;

  template <typename Corner, typename Op>
  static auto make(Corner&& corner, Op&& op)
  STAPL_AUTO_RETURN((
    wavefront<num_inputs>(
      coarse<use_default, kripke_seq_tag_t>(zip<ndims0 + num_inputs>(
        wrap<kripke_seq_tag_t>(
          zip<ndims0 + num_inputs>(
            wrap<wf_seq_tag_t>(wavefront<num_inputs>(
              std::forward<Op>(op),
              corner,
              skeleton_traits<blocked<ndims3>, true>())),
            skeleton_traits<blocked<ndims2>, true>())),
        skeleton_traits<blocked<ndims1>, true>())),
      corner, skeleton_traits<blocked<ndims0>>(DGZ_filter()),
      execution_params(pg_aware_multiview_coarsener()))
  ))


  template <typename Locs, typename Zonesets, typename ZoneSize,
            typename Skeleton>
  static void set_level_dims(Locs&& locs, Zonesets&& zone_sets,
                             ZoneSize&& zone_size, size_t num_groups,
                             size_t num_directions, Skeleton&& sk)
  {
    using namespace stapl::tuple_ops;

    using nested_size_t = std::tuple<size_t, size_t, size_t>;
    std::array<nested_size_t, 2> level_dims;
    level_dims[0] = zone_sets;
    level_dims[1] = stapl::tuple_ops::transform(locs, zone_sets,
                                                stapl::divides<std::size_t>());

    nested_size_t dir_size = pad_tuple<3>(std::make_tuple(num_directions), 1);
    size_t num_locs = fold(level_dims[1], 1, stapl::multiplies<size_t>());

    level_dims[1] = num_locs <= num_directions ? level_dims[1] : dir_size;

    sk.span().set_level_dims(level_dims);
  }
};

#endif // KRIPKE_KERNEL_3D_DGZ_H__
