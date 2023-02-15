/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_MAPPER_UTILS_HPP
#define STAPL_SKELETONS_UTILITY_MAPPER_UTILS_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/skeletons/utility/position.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>
#include <stapl/skeletons/utility/sink_traits.hpp>
#include <stapl/skeletons/param_deps/wavefront_utils.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief A wrapper class for mappers
///
/// @tparam dims        dimension of skeleton
/// @tparam SkeletonTag the tag to specify skeleton tag
//////////////////////////////////////////////////////////////////////
template <int dims, typename SkeletonTag>
struct mappers
{
  using out_to_out_mapper_t =
    typename sink_traits<SkeletonTag, dims>::output_to_output_mapper_type;
  using in_to_in_mapper_t =
    typename sink_traits<SkeletonTag, dims>::input_to_input_mapper_type;
  using out_to_in_mapper_t =
    typename sink_traits<SkeletonTag, dims>::output_to_input_mapper_type;

  out_to_in_mapper_t m_out_to_in_mapper;
  in_to_in_mapper_t m_in_to_in_mapper;
  out_to_out_mapper_t m_out_to_out_mapper;

public:
  mappers() = default;

  void set_direction(skeletons::direction dir)
  {
    skeletons::filters::set_direction(m_out_to_in_mapper, dir);
    skeletons::filters::set_direction(m_in_to_in_mapper, dir);
  }

  template <typename Dimension>
  void set_dimensions(Dimension&& cur_coord,
                      Dimension&& cur_dims,
                      Dimension&& total_dims,
                      Dimension&& task_dims)
  {
    m_out_to_in_mapper.set_dimensions(cur_dims);
    m_in_to_in_mapper.set_dimensions(
      cur_coord, cur_dims, total_dims, task_dims);
    m_out_to_out_mapper.set_dimensions(
      cur_coord, cur_dims, total_dims, task_dims);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    m_in_to_in_mapper.compute_result_ids(std::forward<Op>(op),
                                         std::forward<LevelDims>(level_dims));
    m_out_to_out_mapper.compute_result_ids(std::forward<Op>(op),
                                           std::forward<LevelDims>(level_dims));
  }

  template <typename Dir, typename Dimension, typename LevelDims, typename Op>
  int configure_mapper(Dir&& dir,
                       Dimension&& cur_coord,
                       Dimension&& cur_dims,
                       Dimension&& total_dims,
                       Dimension&& task_dims,
                       LevelDims&& level_dims,
                       Op&& op)
  {
    this->set_dimensions(cur_coord, cur_dims, total_dims, task_dims);
    this->set_direction(dir);
    this->compute_result_ids(std::forward<Op>(op),
                             std::forward<LevelDims>(level_dims));
    return 0;
  }

  out_to_in_mapper_t get_out_to_in_mapper(void) const
  {
    return m_out_to_in_mapper;
  }

  in_to_in_mapper_t get_in_to_in_mapper(void) const
  {
    return m_in_to_in_mapper;
  }

  out_to_out_mapper_t get_out_to_out_mapper(void) const
  {
    return m_out_to_out_mapper;
  }

  void define_type(typer& t)
  {
    t.member(m_out_to_in_mapper);
    t.member(m_in_to_in_mapper);
    t.member(m_out_to_out_mapper);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief A wrapper class for mappers
///
/// @tparam dims        dimension of skeleton
//////////////////////////////////////////////////////////////////////
template <int nested_dims, int dims>
struct mappers<nested_dims, tags::wavefront<dims>>
{
  using skeleton_tag_t = tags::wavefront<dims>;

  using out_to_out_mapper_t =
    typename sink_traits<skeleton_tag_t,
                         nested_dims>::output_to_output_mapper_type;
  using in_to_in_mapper_t =
    typename sink_traits<skeleton_tag_t,
                         nested_dims>::input_to_input_mapper_type;
  using out_to_in_mapper_t =
    typename sink_traits<skeleton_tag_t,
                         nested_dims>::output_to_input_mapper_type;

  out_to_in_mapper_t  m_out_to_in_mapper;
  in_to_in_mapper_t   m_in_to_in_mapper;
  out_to_out_mapper_t m_out_to_out_mapper;

public:
  template <typename Corner>
  mappers(Corner const& corner)
    : m_out_to_in_mapper(pad_corner<nested_dims, dims>::apply(corner)),
      m_in_to_in_mapper(pad_corner<nested_dims, dims>::apply(corner)),
      m_out_to_out_mapper(pad_corner<nested_dims, dims>::apply(corner))
  { }

  void set_direction(skeletons::direction dir)
  {
    skeletons::filters::set_direction(m_out_to_in_mapper, dir);
    skeletons::filters::set_direction(m_in_to_in_mapper, dir);
  }

  template <typename Dimension>
  void set_dimensions(Dimension&& cur_coord,
                      Dimension&& cur_dims,
                      Dimension&& total_dims,
                      Dimension&& task_dims)
  {
    m_out_to_in_mapper.set_dimensions(cur_dims);
    m_in_to_in_mapper.set_dimensions(
      cur_coord, cur_dims, total_dims, task_dims);
    m_out_to_out_mapper.set_dimensions(
      cur_coord, cur_dims, total_dims, task_dims);
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    m_in_to_in_mapper.compute_result_ids(std::forward<Op>(op),
                                         std::forward<LevelDims>(level_dims));
    m_out_to_out_mapper.compute_result_ids(std::forward<Op>(op),
                                           std::forward<LevelDims>(level_dims));
  }

  template <typename Dir, typename Dimension, typename LevelDims, typename Op>
  int configure_mapper(Dir&& dir,
                       Dimension&& cur_coord,
                       Dimension&& cur_dims,
                       Dimension&& total_dims,
                       Dimension&& task_dims,
                       LevelDims&& level_dims,
                       Op&& op)
  {
    this->set_dimensions(cur_coord, cur_dims, total_dims, task_dims);
    this->set_direction(dir);
    this->compute_result_ids(std::forward<Op>(op),
                             std::forward<LevelDims>(level_dims));
    return 0;
  }

  out_to_in_mapper_t get_out_to_in_mapper(void) const
  {
    return m_out_to_in_mapper;
  }

  in_to_in_mapper_t get_in_to_in_mapper(void) const
  {
    return m_in_to_in_mapper;
  }

  out_to_out_mapper_t get_out_to_out_mapper(void) const
  {
    return m_out_to_out_mapper;
  }

  void define_type(typer& t)
  {
    t.member(m_out_to_in_mapper);
    t.member(m_in_to_in_mapper);
    t.member(m_out_to_out_mapper);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief returns a wrapper class for all the mappers needed for
///        nested execution(e.g. input to input, output to output)
///
/// @tparam Dim         dimension of the skeleton mapper is used for
/// @tparam SkeletonTag the tag to specify skeleton tag
/// @param  Args        parameters need to be passed to the
///                     mappers
//////////////////////////////////////////////////////////////////////
template <std::size_t Dim, typename SkeletonTag = stapl::use_default,
          typename... Args>
auto get_nested_mappers(Args&&... args)
  -> decltype(mappers<Dim, SkeletonTag>(std::forward<Args>(args)...))
{
  return mappers<Dim, SkeletonTag>(std::forward<Args>(args)...);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_MAPPER_UTILS_hpp
