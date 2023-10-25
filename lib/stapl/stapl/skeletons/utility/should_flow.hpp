/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_SKELETONS_UTILITY_SHOULD_FLOW_HPP
#define STAPL_SKELETONS_UTILITY_SHOULD_FLOW_HPP

#include <stapl/skeletons/param_deps/wavefront_utils.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/position.hpp>

namespace stapl {
namespace skeletons {

/////////////////////////////////////////////////////////////////////////
/// @brief The default should flow that flows all the elements to
///        to the consumers
/// @tparam SkeletonTag the skeleton tag of skeleton
/////////////////////////////////////////////////////////////////////////
template <int dims, typename SkeletonTag>
class should_flow
{
public:
  template<typename Sk>
  should_flow(Sk&& skeleton)
  { }

  template <typename Dir>
  void set_direction(Dir&& dir)
  { }

  template <typename Dimension>
  void set_dimension(Dimension&& dimension)
  { }

  bool operator()(std::size_t result_id) const
  {
    return true;
  }
};

/////////////////////////////////////////////////////////////////////////
/// @brief The specialized should flow for wavefront skeleton that only
///        flows results belonged to tasks on the boundary of
///        wavefront.
/// @tparam SkeletonTag the skeleton tag of skeleton
/////////////////////////////////////////////////////////////////////////
template <int nested_dims, int dims>
class should_flow<nested_dims, tags::wavefront<dims>>
{
  constexpr static int diff_dims = nested_dims - dims;
  using corner_t        = skeletons::position;
  using corner_type     = std::array<corner_t, nested_dims>;
  using dir_t           = direction;

  using domain_type     = indexed_domain<std::size_t, nested_dims>;
  using index_type      = typename domain_type::index_type;

  using boundary_domain_type = indexed_domain<std::size_t, nested_dims>;
  using boundary_index_type  = typename boundary_domain_type::index_type;

  using traversal_t     = typename domain_type::traversal_type;
  using linearize_t     = nd_linearize<index_type, traversal_t>;

  corner_type m_corner;
  dir_t       m_dir;
  index_type  m_dimension;
  index_type  m_last_index;

  std::vector<std::size_t> m_res_ids;
public:
  template<typename Sk>
  should_flow(Sk&& skeleton)
    // : m_corner(skeleton.get_start_corner())
    : m_corner(
        pad_corner<nested_dims, dims>::apply(skeleton.get_start_corner()))
  { }

  void set_direction(dir_t dir)
  {
    m_dir = dir;
    m_res_ids.clear();
    find_side<dims - 1>();
  }

  void set_dimension(index_type dimension)
  {
    m_dimension  = dimension;
    m_last_index = wavefront_utils::last_index(m_corner, dimension);
  }

  template <std::size_t side>
  void calculate_ids()
  {
    // std::cout << "calculate_ids side: " << side << std::endl;
    linearize_t linear_mf(m_dimension);
    auto dom = domain_type(m_dimension);
    auto cur = dom.first();
    auto sz  = dom.size();
    for (std::size_t i = 0; i < sz; ++i)
    {
      if (std::get<side>(cur) == std::get<side>(m_last_index))
        m_res_ids.push_back(linear_mf(cur));
      cur = dom.advance(cur, 1);
    }
  }

  template <int i>
  typename std::enable_if< (i == 0), void>::type
  find_side()
  {
    if ((int)m_dir == i)
      calculate_ids<i + diff_dims>();
  }

  template <int i>
  typename std::enable_if< (i > 0), void>::type
  find_side()
  {
    if ((int)m_dir == i)
       calculate_ids<i + diff_dims>();
    else
      find_side<i - 1>();
  }

  bool operator()(std::size_t result_id) const
  {
    return std::end(m_res_ids) !=
           std::find(m_res_ids.begin(), m_res_ids.end(), result_id);
  }

  std::vector<std::size_t> get_result_ids(void) const
  {
    return m_res_ids;
  }

 void define_type(typer& t)
  {
    t.member(m_corner);
    t.member(m_dir);
    t.member(m_dimension);
    t.member(m_last_index);
    t.member(m_res_ids);
  }

};

/////////////////////////////////////////////////////////////////////////
/// @brief Goes recursively into each nested level and uses the
///        corresponding should flow for the skeleton on that level.
///
/// @tparam SkeletonTag the skeleton tag of skeleton
/////////////////////////////////////////////////////////////////////////
template <int dims>
class recursive_should_flow
{
  using dir_t                = direction;
  using domain_type          = indexed_domain<std::size_t, dims>;
  using index_type           = typename domain_type::index_type;
  using traversal_t          = typename domain_type::traversal_type;
  using linearizer_t         = nd_linearize<index_type, traversal_t>;

  dir_t       m_dir;
  index_type  m_total_dims;
  std::vector<std::size_t> m_res_ids;

public:

  void set_direction(dir_t dir)
  {
    m_dir = dir;
  }

  void set_dimension(index_type total_dims)
  {
    m_total_dims = total_dims;
  }

  template <size_t level, typename SkeletonTag, typename Sk, typename Op,
            typename LevelDims, typename Dimension, typename Coord>
  void compute_result_ids(std::true_type,
                          Sk&& sk,
                          Op&& op,
                          Dimension&& total_dims,
                          LevelDims&& level_dims,
                          Coord&& first_coord)
  {
    using op_sk_t    = typename std::decay<Op>::type::wrapped_skeleton_type;
    using op_sk_tag  = typename op_sk_t::skeleton_tag_type;
    using inner_op_t = typename op_sk_t::op_type;

    should_flow<dims, SkeletonTag> cur_should_flow(sk);
    cur_should_flow.set_dimension(level_dims[level]);
    cur_should_flow.set_direction(m_dir);

    auto&& dom = domain_type(level_dims[level]);
    auto&& cur = dom.first();
    auto&& sz  = dom.size();
    auto&& task_dims =
      tuple_ops::transform(total_dims,
                           level_dims[level],
                           stapl::divides<std::size_t>());

    for (std::size_t i = 0; i < sz; ++i)
    {
      auto new_cur_coord   =
        tuple_ops::transform(task_dims, cur, stapl::multiplies<std::size_t>());
      auto new_first_coord =
        tuple_ops::transform(first_coord,
                             new_cur_coord,
                             stapl::plus<std::size_t>());

      if (cur_should_flow(i))
      {
        compute_result_ids<level + 1, op_sk_tag>(
          is_nested_skeleton<inner_op_t>(),
          op.get_skeleton(),
          op.get_skeleton().get_op(),
          task_dims,
          std::forward<LevelDims>(level_dims),
          new_first_coord);
      }
      cur = dom.advance(cur, 1);
    }
  }

  template <size_t level, typename SkeletonTag, typename Sk, typename Op,
            typename Dimension, typename LevelDims, typename Coord>
  void compute_result_ids(std::false_type,
                          Sk&& sk,
                          Op&& op,
                          Dimension&& total_dims,
                          LevelDims&& level_dims,
                          Coord&& first_coord)
  {
    should_flow<dims, SkeletonTag> cur_should_flow(sk);
    cur_should_flow.set_dimension(level_dims[level]);
    cur_should_flow.set_direction(m_dir);

    auto&& dom = domain_type(level_dims[level]);
    auto&& cur = dom.first();
    auto&& sz  = dom.size();

    linearizer_t linearizer(m_total_dims);

    for (std::size_t i = 0; i < sz; ++i)
    {
      auto&& new_cur_coord =
        tuple_ops::transform(first_coord, cur, stapl::plus<std::size_t>());
      if (cur_should_flow(i))
      {
        m_res_ids.push_back(linearizer(new_cur_coord));
      }
      cur = dom.advance(cur, 1);
    }

  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(std::true_type, Op&& op, LevelDims&& level_dims)
  {
    using op_sk_t    = typename std::decay<Op>::type::wrapped_skeleton_type;
    using op_sk_tag  = typename op_sk_t::skeleton_tag_type;
    using inner_op_t = typename op_sk_t::op_type;

    m_res_ids.clear();
    compute_result_ids<0, op_sk_tag>(is_nested_skeleton<inner_op_t>(),
                                     op.get_skeleton(),
                                     op.get_skeleton().get_op(),
                                     m_total_dims,
                                     std::forward<LevelDims>(level_dims),
                                     homogeneous_tuple<dims>(0));
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(std::false_type, Op&&, LevelDims&&)
  { }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    compute_result_ids(
      is_nested_skeleton<typename std::decay<Op>::type>(),
      std::forward<Op>(op),
      std::forward<LevelDims>(level_dims));
  }

  std::vector<std::size_t> get_result_ids(void) const
  {
    return m_res_ids;
  }

  bool operator()(std::size_t result_id) const
  {
    return std::end(m_res_ids) !=
           std::find(m_res_ids.begin(), m_res_ids.end(), result_id);
  }

  void define_type(typer& t)
  {
    t.member(m_dir);
    t.member(m_total_dims);
    t.member(m_res_ids);
  }

};

/////////////////////////////////////////////////////////////////////////
/// @brief  Specialization for 1D dimension
/////////////////////////////////////////////////////////////////////////
template <>
class recursive_should_flow<1>
{
  using dir_t                = direction;
  using domain_type          = indexed_domain<std::size_t, 1>;
  using index_type           = typename domain_type::index_type;
  using traversal_t          = typename domain_type::traversal_type;
  using linearizer_t         = nd_linearize<index_type, traversal_t>;

  dir_t       m_dir;
  index_type  m_total_dims;
  std::vector<std::size_t> m_res_ids;

public:

  void set_direction(dir_t dir)
  {
    m_dir = dir;
  }

  void set_dimension(index_type total_dims)
  {
    m_total_dims = total_dims;
  }

  template <size_t level, typename SkeletonTag, typename Sk, typename Op,
            typename LevelDims, typename Dimension, typename Coord>
  void compute_result_ids(std::true_type,
                          Sk&& sk,
                          Op&& op,
                          Dimension&& total_dims,
                          LevelDims&& level_dims,
                          Coord&& first_coord)
  {
    using op_sk_t    = typename std::decay<Op>::type::wrapped_skeleton_type;
    using op_sk_tag  = typename op_sk_t::skeleton_tag_type;
    using inner_op_t = typename op_sk_t::op_type;

    should_flow<1, SkeletonTag> cur_should_flow(sk);
    cur_should_flow.set_dimension(level_dims[level]);
    cur_should_flow.set_direction(m_dir);

    auto&& dom = domain_type(level_dims[level]);
    auto&& cur = dom.first();
    auto&& sz  = dom.size();
    auto&& task_dims = total_dims/level_dims[level];

    for (std::size_t i = 0; i < sz; ++i)
    {
      auto new_cur_coord   = task_dims * cur;
      auto new_first_coord = first_coord + new_cur_coord;

      if (cur_should_flow(i))
      {
        compute_result_ids<level + 1, op_sk_tag>(
          is_nested_skeleton<inner_op_t>(),
          op.get_skeleton(),
          op.get_skeleton().get_op(),
          task_dims,
          std::forward<LevelDims>(level_dims),
          new_first_coord);
      }
      cur = dom.advance(cur, 1);
    }
  }

  template <size_t level, typename SkeletonTag, typename Sk, typename Op,
            typename Dimension, typename LevelDims, typename Coord>
  void compute_result_ids(std::false_type,
                          Sk&& sk,
                          Op&& op,
                          Dimension&& total_dims,
                          LevelDims&& level_dims,
                          Coord&& first_coord)
  {
    should_flow<1, SkeletonTag> cur_should_flow(sk);
    cur_should_flow.set_dimension(level_dims[level]);
    cur_should_flow.set_direction(m_dir);

    auto&& dom = domain_type(level_dims[level]);
    auto&& cur = dom.first();
    auto&& sz  = dom.size();

    linearizer_t linearizer(m_total_dims);

    for (std::size_t i = 0; i < sz; ++i)
    {
      auto&& new_cur_coord = first_coord + cur;
      if (cur_should_flow(i))
      {
        m_res_ids.push_back(linearizer(new_cur_coord));
      }
      cur = dom.advance(cur, 1);
    }

  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(std::true_type, Op&& op, LevelDims&& level_dims)
  {
    using op_sk_t    = typename std::decay<Op>::type::wrapped_skeleton_type;
    using op_sk_tag  = typename op_sk_t::skeleton_tag_type;
    using inner_op_t = typename op_sk_t::op_type;

    m_res_ids.clear();
    compute_result_ids<0, op_sk_tag>(is_nested_skeleton<inner_op_t>(),
                                     op.get_skeleton(),
                                     op.get_skeleton().get_op(),
                                     m_total_dims,
                                     std::forward<LevelDims>(level_dims),
                                     0);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(std::false_type, Op&&, LevelDims&&)
  { }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    compute_result_ids(
      is_nested_skeleton<typename std::decay<Op>::type>(),
      std::forward<Op>(op),
      std::forward<LevelDims>(level_dims));
  }

  std::vector<std::size_t> get_result_ids(void) const
  {
    return m_res_ids;
  }

  bool operator()(std::size_t result_id) const
  {
    return std::end(m_res_ids) !=
           std::find(m_res_ids.begin(), m_res_ids.end(), result_id);
  }

  void define_type(typer& t)
  {
    t.member(m_dir);
    t.member(m_total_dims);
    t.member(m_res_ids);
  }

};


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_SHOULD_FLOW_HPP
