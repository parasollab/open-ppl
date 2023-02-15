/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_MAPPERS_HPP
#define STAPL_SKELETONS_UTILITY_MAPPERS_HPP

#include <stapl/skeletons/param_deps/wavefront_utils.hpp>
#include <stapl/skeletons/utility/should_flow.hpp>

namespace stapl {
namespace skeletons {

/////////////////////////////////////////////////////////////////////////
/// @brief Given the coordination of task, decides whether or not
///        this task produces result in wavefront skeleton.
///        the result id is just linearization of the coordination.
///
/// @tparam dims dimension of skeleton
/////////////////////////////////////////////////////////////////////////
template <int dims>
class wf_result_mapping
{
  using corner_t = skeletons::position;
  using corner_type = std::array<corner_t, dims>;

  using domain_type = indexed_domain<std::size_t, dims>;
  using index_type  = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearize_t = nd_linearize<index_type, traversal_t>;

  corner_type m_corner;
  index_type m_last_index;
  linearize_t m_linear_mf;

public:
  wf_result_mapping(corner_type corner, index_type const& dimension)
    : m_corner(corner),
      m_last_index(wavefront_utils::last_index(corner, dimension)),
      m_linear_mf(dimension)
  { }

  template <std::size_t... Idx>
  bool check_border(index_type const& coord, index_sequence<Idx...>&&) const
  {
    auto on_border =
      stapl::make_tuple((get<Idx>(coord) == get<Idx>(m_last_index))...);
    return tuple_ops::fold(on_border, false, logical_or<bool>());
  }

  std::size_t operator()(index_type const& coord) const
  {
    if (check_border(coord, make_index_sequence<dims>()))
      return m_linear_mf(coord);
    else
      return std::numeric_limits<std::size_t>::max();
  }

  void define_type(typer& t)
  {
    t.member(m_corner);
    t.member(m_last_index);
    t.member(m_linear_mf);
  }
}; // wf_result_mapping

/////////////////////////////////////////////////////////////////////////
/// @brief maps the linearized index of the consumer task to the
///        corresponding linearized index of the producer task
///        for wavefront skeleton.
///
/// @tparam nested_dims maximum dimension of all skeletons
/// @tparam dims dimension which skeleton operate on it.
/////////////////////////////////////////////////////////////////////////
template <int nested_dims, int dims>
class wf_output_to_input_mapper
{
  constexpr static int diff_dims = nested_dims - dims;
  using corner_t    = skeletons::position;
  using corner_type = std::array<corner_t, nested_dims>;
  using dir_t       = direction;

  using domain_type = indexed_domain<std::size_t, nested_dims>;
  using index_type  = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearize_t = nd_linearize<index_type, traversal_t>;

  dir_t m_dir;
  corner_type m_corner;
  index_type  m_last_index;
  linearize_t m_linear_mf;

public:
  wf_output_to_input_mapper(corner_type const& corners)
    : m_corner(corners)
  { }

  void set_direction(dir_t dir)
  {
    m_dir = dir;
  }

  void set_dimensions(index_type const& dimension)
  {
    m_last_index = wavefront_utils::last_index(m_corner, dimension);
    m_linear_mf = linearize_t(dimension);
  }

  template <std::size_t... Indices>
  index_type compute_index(index_type coord, index_sequence<Indices...>&&) const
  {
    return stapl::make_tuple(Indices == (size_t)m_dir + diff_dims
                               ? std::get<Indices>(m_last_index)
                               : std::get<Indices>(coord)...);
  }

  std::size_t operator()(index_type coord) const
  {
    return
      m_linear_mf(compute_index(coord, make_index_sequence<nested_dims>()));
  }

  void define_type(typer& t)
  {
    t.member(m_dir);
    t.member(m_corner);
    t.member(m_last_index);
    t.member(m_linear_mf);
  }
}; // wf_output_to_input_mapper

/////////////////////////////////////////////////////////////////////////
/// @brief default mapper that maps the linearized index
///        the consumer task to the corresponding linearized index
///        of the producer task, basically just linearization.
///
/// @tparam dims dimension which skeleton operate on it.
/////////////////////////////////////////////////////////////////////////
template <int dims>
class default_output_to_input_mapper
{
  using domain_type = indexed_domain<std::size_t, dims>;
  using index_type = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearize_t = nd_linearize<index_type, traversal_t>;

  linearize_t m_linear_mf;

public:
  void set_direction(direction dir)
  { }

  void set_dimensions(index_type const& dimension)
  {
    m_linear_mf = linearize_t(dimension);
  }

  std::size_t operator()(index_type coord) const
  {
    return m_linear_mf(coord);
  }

  void define_type(typer& t)
  {
    t.member(m_linear_mf);
  }
}; // default_output_to_input_mapper

/////////////////////////////////////////////////////////////////////////
/// @brief A mapper for mapping the result ids from the child skeleton
///        to parent skeleton, for wavefront it checks if the producer
///        skeletons are on the boundary of wavefront or not and from
///        which direction.
/// @tparam nested_dims maximum dimension of all skeletons
/// @tparam dims dimension which skeleton operate on
/////////////////////////////////////////////////////////////////////////
template <int nested_dims, int dims>
class wf_output_to_output_mapper
{
  constexpr static int diff_dims = nested_dims - dims;
  using should_flow_t = recursive_should_flow<nested_dims>;
  using corner_t = skeletons::position;
  using corner_type = std::array<corner_t, nested_dims>;
  using dir_t = direction;

  using domain_type = indexed_domain<std::size_t, nested_dims>;
  using index_type = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearizer_t = nd_linearize<index_type, traversal_t>;
  using rev_linearizer_t = nd_reverse_linearize<index_type, traversal_t>;

  corner_type m_corner;
  index_type m_cur_coord;
  index_type m_cur_dimension;
  index_type m_parent_coord;
  index_type m_parent_dimension;
  index_type m_task_dimension;
  index_type m_last_index;
  should_flow_t m_should_flow;

  std::vector<dir_t> m_sides;
  std::vector<std::size_t> m_res_ids;

public:
  wf_output_to_output_mapper(corner_type const& corners)
    : m_corner(corners)
  { }

  void set_dimensions(index_type const& cur_coord,
                      index_type const& cur_dimension,
                      index_type const& parent_dimension,
                      index_type const& task_dimension)
  {
    using namespace tuple_ops;

    m_cur_coord = cur_coord;
    m_cur_dimension = cur_dimension;
    m_parent_dimension = parent_dimension;
    m_task_dimension = task_dimension;
    m_parent_coord = tuple_ops::transform(
      m_cur_coord, m_task_dimension, stapl::multiplies<std::size_t>());
    m_last_index = wavefront_utils::last_index(m_corner, cur_dimension);
    m_should_flow.set_dimension(task_dimension);

    m_sides.clear();
    find_should_flow_dir<dims - 1>();
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    m_res_ids.clear();

    for (auto dir : m_sides)
    {
      m_should_flow.set_direction(dir);
      m_should_flow.compute_result_ids(std::forward<Op>(op),
                                       std::forward<LevelDims>(level_dims));

      for (auto res_ids : m_should_flow.get_result_ids())
      {
        if (std::end(m_res_ids) ==
            std::find(m_res_ids.begin(), m_res_ids.end(), res_ids))
          m_res_ids.push_back(res_ids);
      }
    }
  }

  std::size_t operator()(std::size_t idx) const
  {
    using namespace tuple_ops;

    index_type child_coord = rev_linearizer_t(m_task_dimension)(idx);
    index_type parent_coord = tuple_ops::transform(
      m_parent_coord, child_coord, stapl::plus<std::size_t>());

    return linearizer_t(m_parent_dimension)(parent_coord);
  }

  template <std::size_t i>
  typename std::enable_if<(i == 0), void>::type
  find_should_flow_dir(void)
  {
    if (std::get<i + diff_dims>(m_cur_coord) ==
        std::get<i + diff_dims>(m_last_index))
      m_sides.push_back(static_cast<dir_t>(i));
  }

  template <std::size_t i>
  typename std::enable_if<(i > 0), void>::type
  find_should_flow_dir(void)
  {
    if (std::get<i + diff_dims>(m_cur_coord) ==
        std::get<i + diff_dims>(m_last_index))
      m_sides.push_back(static_cast<dir_t>(i));

    find_should_flow_dir<i - 1>();
  }

  bool should_flow(std::size_t result_id) const
  {
    return std::end(m_res_ids) !=
           std::find(m_res_ids.begin(), m_res_ids.end(), result_id);
  }

  should_flow_t get_should_flow() const
  {
    return m_should_flow;
  }

  std::vector<std::size_t> get_result_ids() const
  {
    return m_res_ids;
  }

  void define_type(typer& t)
  {
    t.member(m_corner);
    t.member(m_cur_coord);
    t.member(m_parent_coord);
    t.member(m_cur_dimension);
    t.member(m_parent_dimension);
    t.member(m_task_dimension);
    t.member(m_last_index);
    t.member(m_should_flow);
    t.member(m_sides);
    t.member(m_res_ids);
  }
}; // wf_output_to_output_mapper

/////////////////////////////////////////////////////////////////////////
/// @brief A mapper for mapping the result ids from the child skeleton
///        to parent skeleton, for default case given the coordination
///        of producer task, it will map to the new coordination
///        in parent dimension
/// @tparam dims dimension which skeleton operate on it.
/////////////////////////////////////////////////////////////////////////
template <int dims>
class default_output_to_output_mapper
{
  using should_flow_t = recursive_should_flow<dims>;
  using corner_t = skeletons::position;
  using corner_type = std::array<corner_t, dims>;
  using dir_t = direction;

  using domain_type = indexed_domain<std::size_t, dims>;
  using index_type = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearizer_t = nd_linearize<index_type, traversal_t>;
  using rev_linearizer_t = nd_reverse_linearize<index_type, traversal_t>;

  corner_type m_corner;
  index_type m_cur_coord;
  index_type m_cur_dimension;
  index_type m_parent_coord;
  index_type m_parent_dimension;
  index_type m_task_dimension;
  should_flow_t m_should_flow;

  std::array<dir_t, dims>  m_sides;
  std::vector<std::size_t> m_res_ids;

public:
  void set_direction(dir_t dir)
  { }

  void set_dimensions(index_type const& cur_coord,
                      index_type const& cur_dimension,
                      index_type const& parent_dimension,
                      index_type const& task_dimension)
  {
    m_cur_coord = cur_coord;
    m_cur_dimension = cur_dimension;
    m_parent_dimension = parent_dimension;
    m_task_dimension = task_dimension;
    m_parent_coord = tuple_ops::transform(
      m_cur_coord, m_task_dimension, stapl::multiplies<std::size_t>());
    m_should_flow.set_dimension(task_dimension);

    for (std::size_t dir = 0; dir < dims; ++dir)
      m_sides[dir] = (dir_t) dir;

  }

  std::size_t operator()(std::size_t idx) const
  {
    using namespace tuple_ops;

    index_type child_coord = rev_linearizer_t(m_task_dimension)(idx);
    index_type parent_coord = tuple_ops::transform(
      m_parent_coord, child_coord, stapl::plus<std::size_t>());
    return linearizer_t(m_parent_dimension)(parent_coord);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    m_res_ids.clear();
    for (auto dir : m_sides) {
      m_should_flow.set_direction(dir);
      m_should_flow.compute_result_ids(std::forward<Op>(op),
                                       std::forward<LevelDims>(level_dims));
      for (auto res_ids : m_should_flow.get_result_ids()) {
        if (std::end(m_res_ids) ==
            std::find(m_res_ids.begin(), m_res_ids.end(), res_ids))
          m_res_ids.push_back(res_ids);
      }
    }
  }

  bool should_flow(std::size_t result_id) const
  {
    return std::end(m_res_ids) !=
           std::find(m_res_ids.begin(), m_res_ids.end(), result_id);
  }

  should_flow_t get_should_flow() const
  {
    return m_should_flow;
  }

  std::vector<std::size_t> get_result_ids() const
  {
    return m_res_ids;
  }

  void define_type(typer& t)
  {
    t.member(m_corner);
    t.member(m_cur_coord);
    t.member(m_parent_coord);
    t.member(m_cur_dimension);
    t.member(m_parent_dimension);
    t.member(m_task_dimension);
    t.member(m_should_flow);
    t.member(m_sides);
    t.member(m_res_ids);
  }
}; // default_output_to_output_mapper


/////////////////////////////////////////////////////////////////////////
/// @brief  Specialization for 1D dimension
/////////////////////////////////////////////////////////////////////////
template <>
class default_output_to_output_mapper<1>
{

  using should_flow_t = recursive_should_flow<1>;
  using corner_t = skeletons::position;
  using corner_type = std::array<corner_t, 1>;
  using dir_t = direction;

  using domain_type = indexed_domain<std::size_t, 1>;
  using index_type = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearizer_t = nd_linearize<index_type, traversal_t>;
  using rev_linearizer_t = nd_reverse_linearize<index_type, traversal_t>;

  corner_type m_corner;
  index_type m_cur_coord;
  index_type m_cur_dimension;
  index_type m_parent_coord;
  index_type m_parent_dimension;
  index_type m_task_dimension;
  should_flow_t m_should_flow;

  std::array<dir_t, 1>  m_sides;
  std::vector<std::size_t> m_res_ids;

public:
  void set_direction(dir_t dir)
  { }

  void set_dimensions(index_type const& cur_coord,
                      index_type const& cur_dimension,
                      index_type const& parent_dimension,
                      index_type const& task_dimension)
  {
    m_cur_coord = cur_coord;
    m_cur_dimension = cur_dimension;
    m_parent_dimension = parent_dimension;
    m_task_dimension = task_dimension;
    m_parent_coord = m_cur_coord * m_task_dimension;
    m_should_flow.set_dimension(task_dimension);

    for (std::size_t dir = 0; dir < 1; ++dir)
      m_sides[dir] = (dir_t) dir;

  }

  std::size_t operator()(std::size_t idx) const
  {
    using namespace tuple_ops;

    index_type child_coord  = rev_linearizer_t(m_task_dimension)(idx);
    index_type parent_coord = m_parent_coord + child_coord;
    return linearizer_t(m_parent_dimension)(parent_coord);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    m_res_ids.clear();
    for (auto dir : m_sides) {
      m_should_flow.set_direction(dir);
      m_should_flow.compute_result_ids(std::forward<Op>(op),
                                       std::forward<LevelDims>(level_dims));
      for (auto res_ids : m_should_flow.get_result_ids()) {
        if (std::end(m_res_ids) ==
            std::find(m_res_ids.begin(), m_res_ids.end(), res_ids))
          m_res_ids.push_back(res_ids);
      }
    }
  }

  bool should_flow(std::size_t result_id) const
  {
    return std::end(m_res_ids) !=
           std::find(m_res_ids.begin(), m_res_ids.end(), result_id);
  }

  should_flow_t get_should_flow() const
  {
    return m_should_flow;
  }

  std::vector<std::size_t> get_result_ids() const
  {
    return m_res_ids;
  }

  void define_type(typer& t)
  {
    t.member(m_corner);
    t.member(m_cur_coord);
    t.member(m_parent_coord);
    t.member(m_cur_dimension);
    t.member(m_parent_dimension);
    t.member(m_task_dimension);
    t.member(m_should_flow);
    t.member(m_sides);
    t.member(m_res_ids);
  }
}; // default_output_to_output_mapper

/////////////////////////////////////////////////////////////////////////
/// @brief wavefront mapper when the wavefront is consuming from the
///        the parent paragraph. The difference between default one
///        is that it first computes the corresponding producer task
///        index given the consumer index.
///
/// @tparam nested_dims maximum dimension of all skeletons
/// @tparam dims dimension which skeleton operate on
/////////////////////////////////////////////////////////////////////////
template <int nested_dims, int dims>
class wf_input_to_input_mapper
{
  constexpr static int diff_dims = nested_dims - dims;
  using should_flow_t = recursive_should_flow<nested_dims>;
  using corner_t = skeletons::position;
  using corner_type = std::array<corner_t, nested_dims>;
  using dir_t = direction;
  using domain_type = indexed_domain<std::size_t, nested_dims>;
  using index_type = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearizer_t = nd_linearize<index_type, traversal_t>;
  using rev_linearizer_t = nd_reverse_linearize<index_type, traversal_t>;

  corner_type m_corner;
  should_flow_t m_should_flow;

  index_type m_consumer_coord;
  index_type m_consumer_dimension;
  index_type m_total_dimension;
  index_type m_task_dimension;
  index_type m_last_index;
  dir_t m_dir;

  std::vector<std::size_t> m_res_ids;
  std::map<std::size_t, std::size_t> m_result_map;

public:
  wf_input_to_input_mapper(corner_type corner)
    : m_corner(corner)
  { }

  void set_direction(dir_t dir)
  {
    stapl_assert((int)dir >= 0 && (int)dir < dims, "wrong mapper direction");
    m_dir = dir;
    m_res_ids.clear();
    m_result_map.clear();
    m_should_flow.set_direction(dir);
  }

  void set_dimensions(index_type const& consumer_coord,
                      index_type const& consumer_dimension,
                      index_type const& total_dimension,
                      index_type const& task_dimension)
  {
    using namespace tuple_ops;

    m_consumer_coord = consumer_coord;
    m_consumer_dimension = consumer_dimension;
    m_total_dimension = total_dimension;
    m_task_dimension = task_dimension;
    m_last_index = wavefront_utils::last_index(m_corner, consumer_dimension);
    m_should_flow.set_dimension(task_dimension);
  }

  template <std::size_t... Indices>
  index_type compute_producer_index(index_sequence<Indices...>&&) const
  {
    // fix here, I guess fixed now
    return stapl::make_tuple(Indices == (size_t)m_dir + diff_dims
                               ? std::get<Indices>(m_last_index)
                               : std::get<Indices>(m_consumer_coord)...);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    rev_linearizer_t rev_linearizer(m_task_dimension);
    linearizer_t linearizer(m_total_dimension);

    index_type p_coord =
      compute_producer_index(make_index_sequence<nested_dims>());

    m_should_flow.compute_result_ids(std::forward<Op>(op),
                                     std::forward<LevelDims>(level_dims));
    for (auto res_id : m_should_flow.get_result_ids()) {
      index_type res_child_coord = rev_linearizer(res_id);

      index_type parent_res_coord = tuple_ops::transform(
        p_coord, m_task_dimension, stapl::multiplies<std::size_t>());

      parent_res_coord = tuple_ops::transform(
        parent_res_coord, res_child_coord, stapl::plus<std::size_t>());

      std::size_t upper_level_idx = linearizer(parent_res_coord);
      m_res_ids.push_back(upper_level_idx);
      m_result_map[upper_level_idx] = res_id;
    }
  }

  std::size_t operator()(const std::size_t idx) const
  {
    return const_cast<wf_input_to_input_mapper*>(this)->m_result_map[idx];
  }

  should_flow_t get_should_flow(void) const
  {
    return m_should_flow;
  }

  std::vector<size_t> get_result_ids() const
  {
    return m_res_ids;
  }

  void define_type(typer& t)
  {
    t.member(m_corner);
    t.member(m_should_flow);
    t.member(m_consumer_coord);
    t.member(m_consumer_dimension);
    t.member(m_total_dimension);
    t.member(m_task_dimension);
    t.member(m_dir);
    t.member(m_last_index);
    t.member(m_res_ids);
    t.member(m_result_map);
  }
}; // wf_input_to_input_mapper

/////////////////////////////////////////////////////////////////////////
/// @brief  default mapper when skeleton is consuming from the
///         the parent paragraph results. it maps the
///         result ids requested by the child, from the parent dimension
///         (upper level) to child dimension.
/// @tparam dims dimension which skeleton operate on
/////////////////////////////////////////////////////////////////////////
template <int dims>
class default_input_to_input_mapper
{
  using should_flow_t = recursive_should_flow<dims>;
  using corner_t = skeletons::position;
  using corner_type = std::array<corner_t, dims>;
  using dir_t = direction;
  using domain_type = indexed_domain<std::size_t, dims>;
  using index_type = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearizer_t = nd_linearize<index_type, traversal_t>;
  using rev_linearizer_t = nd_reverse_linearize<index_type, traversal_t>;

  should_flow_t m_should_flow;
  index_type m_consumer_coord;
  index_type m_consumer_dimension;
  index_type m_total_dimension;
  index_type m_task_dimension;
  dir_t m_dir;

  std::vector<std::size_t> m_res_ids;
  std::map<std::size_t, std::size_t> m_result_map;

public:
  void set_direction(dir_t dir)
  {
    m_dir = dir;
    m_res_ids.clear();
    m_result_map.clear();
    m_should_flow.set_direction(dir);
  }

  void set_dimensions(index_type const& consumer_coord,
                      index_type const& consumer_dimension,
                      index_type const& total_dimension,
                      index_type const& task_dimension)
  {
    using namespace tuple_ops;

    m_consumer_coord = consumer_coord;
    m_consumer_dimension = consumer_dimension;
    m_total_dimension = total_dimension;
    m_task_dimension = task_dimension;
    m_should_flow.set_dimension(task_dimension);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    rev_linearizer_t rev_linearizer(m_task_dimension);
    linearizer_t linearizer(m_total_dimension);

    m_should_flow.compute_result_ids(std::forward<Op>(op),
                                     std::forward<LevelDims>(level_dims));

    for (auto res_id : m_should_flow.get_result_ids()) {
      index_type res_child_coord = rev_linearizer(res_id);

      index_type parent_res_coord = tuple_ops::transform(
        m_consumer_coord, m_task_dimension, stapl::multiplies<std::size_t>());

      parent_res_coord = tuple_ops::transform(
        parent_res_coord, res_child_coord, stapl::plus<std::size_t>());

      std::size_t upper_level_idx = linearizer(parent_res_coord);
      m_res_ids.push_back(upper_level_idx);
      m_result_map[upper_level_idx] = res_id;
    }
  }

  std::size_t operator()(const std::size_t idx) const
  {
    return const_cast<default_input_to_input_mapper*>(this)->m_result_map[idx];
  }

  should_flow_t get_should_flow(void) const
  {
    return m_should_flow;
  }

  std::vector<size_t> get_result_ids() const
  {
    return m_res_ids;
  }

  void define_type(typer& t)
  {
    t.member(m_should_flow);
    t.member(m_consumer_coord);
    t.member(m_consumer_dimension);
    t.member(m_total_dimension);
    t.member(m_task_dimension);
    t.member(m_dir);
    t.member(m_res_ids);
    t.member(m_result_map);
  }
}; // default_input_to_input_mapper

/////////////////////////////////////////////////////////////////////////
/// @brief  Specialization for 1D dimension
/////////////////////////////////////////////////////////////////////////
template <>
class default_input_to_input_mapper<1>
{
  using should_flow_t = recursive_should_flow<1>;
  using corner_t = skeletons::position;
  using corner_type = std::array<corner_t, 1>;
  using dir_t = direction;
  using domain_type = indexed_domain<std::size_t, 1>;
  using index_type = typename domain_type::index_type;
  using traversal_t = typename domain_type::traversal_type;
  using linearizer_t = nd_linearize<index_type, traversal_t>;
  using rev_linearizer_t = nd_reverse_linearize<index_type, traversal_t>;

  should_flow_t m_should_flow;
  index_type m_consumer_coord;
  index_type m_consumer_dimension;
  index_type m_total_dimension;
  index_type m_task_dimension;
  dir_t m_dir;

  std::vector<std::size_t> m_res_ids;
  std::map<std::size_t, std::size_t> m_result_map;

public:
  void set_direction(dir_t dir)
  {
    m_dir = dir;
    m_res_ids.clear();
    m_result_map.clear();
    m_should_flow.set_direction(dir);
  }

  void set_dimensions(index_type const& consumer_coord,
                      index_type const& consumer_dimension,
                      index_type const& total_dimension,
                      index_type const& task_dimension)
  {
    using namespace tuple_ops;

    m_consumer_coord = consumer_coord;
    m_consumer_dimension = consumer_dimension;
    m_total_dimension = total_dimension;
    m_task_dimension = task_dimension;
    m_should_flow.set_dimension(task_dimension);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    rev_linearizer_t rev_linearizer(m_task_dimension);
    linearizer_t linearizer(m_total_dimension);

    m_should_flow.compute_result_ids(std::forward<Op>(op),
                                     std::forward<LevelDims>(level_dims));
    for (auto res_id : m_should_flow.get_result_ids()) {

      index_type res_child_coord = rev_linearizer(res_id);
      index_type parent_res_coord = m_consumer_coord * m_task_dimension;
      parent_res_coord += res_child_coord;

      std::size_t upper_level_idx = linearizer(parent_res_coord);
      m_res_ids.push_back(upper_level_idx);
      m_result_map[upper_level_idx] = res_id;
    }
  }

  std::size_t operator()(const std::size_t idx) const
  {
    return const_cast<default_input_to_input_mapper*>(this)->m_result_map[idx];
  }

  should_flow_t get_should_flow(void) const
  {
    return m_should_flow;
  }

  std::vector<size_t> get_result_ids() const
  {
    return m_res_ids;
  }

  void define_type(typer& t)
  {
    t.member(m_should_flow);
    t.member(m_consumer_coord);
    t.member(m_consumer_dimension);
    t.member(m_total_dimension);
    t.member(m_task_dimension);
    t.member(m_dir);
    t.member(m_res_ids);
    t.member(m_result_map);
  }
}; // default_input_to_input_mapper<1>

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_MAPPERS_HPP
