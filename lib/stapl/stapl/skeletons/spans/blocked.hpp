/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_BLOCKED_HPP
#define STAPL_SKELETONS_SPANS_BLOCKED_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/spans/span_helpers.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/views/operations/local_domain.hpp>
#include <stapl/containers/partitions/block_partition.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

//////////////////////////////////////////////////////////////////////
/// @brief A blocked span which is used the most across the
/// skeletons library. It assigns a blocked <b>spawn</b> load to each
/// location. The load on each location is roughly \f$\frac{n}{p}\f$.
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <int i>
class blocked
{
public:
  using dims_num           = std::integral_constant<int, i>;
  /// @todo this type should be removed, the only use right now is in nest
  ///       transformation
  using nested_dims_num    = dims_num;
  using traversal_type     = typename default_traversal<i>::type;
  using partition_type     = multiarray_impl::block_partition<
                               typename default_traversal<i>::type>;
  using domain_type        = typename partition_type::domain_type;
  using index_type         = typename domain_type::index_type;
  using size_type          = typename domain_type::size_type;
  using dimension_type     = std::integral_constant<int, i>;
  using linearization_type = nd_linearize<index_type, traversal_type>;

protected:
  std::vector<domain_type>   m_local_domain;
  domain_type                m_global_domain;
  linearization_type         m_linear_mf;

  template<typename Spawner, typename View>
  void set_size_impl(Spawner const& spawner, View const& view)
  {
    m_global_domain = view.domain();
    m_local_domain  =
      view.local_domain(spawner.get_PE_id(), spawner.get_num_PEs());
    m_linear_mf = linearization_type(this->dimensions());
  }

public:
  blocked(void) = default;

  template <typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    using tuple_t =
      tuple<
        skeletons::domain_type<
          typename Views::domain_type, has_finite_domain<Views>::value
        >...
      >;

    constexpr size_t idx = skeletons::first_finite_domain_index<tuple_t>::value;

    set_size_impl(spawner, get<idx>(tuple<Views const&...>(views...)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Change view and container implementation to pass iterator
  /// or collection of intervals in a more generic, lightweight manner
  /// than std::vector.
  //////////////////////////////////////////////////////////////////////
  std::vector<domain_type> const&
  local_domain()
  {
    return m_local_domain;
  }

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    return true;
  }

  size_type dimensions() const
  {
    return m_global_domain.dimensions();
  }

  size_type total_dimension(void) const
  {
    return this->dimensions();
  }

  size_type task_dimension(void) const
  {
    return homogeneous_tuple<i>(size_t(1));
  }

  std::vector<size_type> level_dimensions(void) const
  {
    return std::vector<size_type>();
  }

  std::size_t size() const
  {
    return m_global_domain.size();
  }

  std::size_t linearize(index_type const& coord) const
  {
    return m_linear_mf(coord);
  }

  linearization_type get_linear_mf(void) const
  {
    return m_linear_mf;
  }

  void define_type(typer& t)
  {
    t.member(m_global_domain);
    t.member(m_local_domain);
    t.member(m_linear_mf);
  }
};


template <>
class blocked<1>
{
public:
  using dims_num        = std::integral_constant<int, 1>;
  /// @todo this type should be removed, the only use right now is in nest
  ///       transformation
  using nested_dims_num = dims_num;
  using domain_type     = indexed_domain<std::size_t, 1>;
  using index_type      = typename domain_type::index_type;
  using size_type       = typename domain_type::size_type;
  using dimension_type  = std::integral_constant<int, 1>;

protected:
  std::vector<domain_type>   m_local_domain;
  domain_type                m_global_domain;

public:
  blocked() = default;

  template<typename Spawner, typename View>
  void set_size_impl(Spawner const& spawner, View const& view)
  {
    m_global_domain = view.domain();
    m_local_domain  =
      view.local_domain(spawner.get_PE_id(), spawner.get_num_PEs());
  }

  template <typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    using tuple_t =
      tuple<
        skeletons::domain_type<
          typename Views::domain_type, has_finite_domain<Views>::value
        >...
      >;

    constexpr size_t idx = skeletons::first_finite_domain_index<tuple_t>::value;

    set_size_impl(spawner, get<idx>(tuple<Views const&...>(views...)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Change view and container implementation to pass iterator
  /// or collection of intervals in a more generic, lightweight manner
  /// than std::vector.
  //////////////////////////////////////////////////////////////////////
  std::vector<domain_type> const&
  local_domain() const
  {
    return m_local_domain;
  }

  template <typename Coord>
  bool should_spawn (Coord const& skeleton_size, Coord const& coord) const
  {
    return true;
  }

  size_type dimensions(void) const
  {
    return m_global_domain.dimensions();
  }

  size_type total_dimension(void) const
  {
    return this->dimensions();
  }

  size_type task_dimension(void) const
  {
    return 1;
  }

  std::vector<size_type> level_dimensions(void) const
  {
    return std::vector<size_type>();
  }

  std::size_t size(void) const
  {
    return m_global_domain.size();
  }

  std::size_t linearize(index_type const& coord) const
  {
    return coord;
  }

  void define_type(typer& t)
  {
    t.member(m_local_domain);
    t.member(m_global_domain);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief A span which is used for skeletons with nested execution
///
/// @todo Ideally there shouldn't be a new span type for nested execution,
///       but in current implementation for nested skeletons, we need
///       some more information about the dimensions( e.g. each nested level
///       dimension) that makes the span type a little heavy.
///       The span type is separated so performance of skeletons with
///       non-nested execution is not hurt.
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <int dims, int nested_dims, typename Op, size_t level>
class nest_blocked
  : public blocked<dims>
{
public:
  using base_t          = blocked<dims>;
  using size_type       = typename base_t::size_type;
  using dims_num = std::integral_constant<size_t, dims>;

  using nested_size_t  = homogeneous_tuple_type_t<nested_dims, size_t>;
  using nested_dims_num = std::integral_constant<int, nested_dims>;

private:
  nested_size_t                    m_total_dimension;
  std::array<nested_size_t, level> m_level_dims;
  bool                             m_levels_set = false;

public:

  void set_level_dims(std::array<nested_size_t, level + 1> level_dims)
  {
    for (size_t i = 1; i < level_dims.size(); ++i)
    {
      m_level_dims[i-1] = level_dims[i];
    }

    m_total_dimension = homogeneous_tuple<nested_dims>(1);

    for (auto&& dim : level_dims)
      m_total_dimension =
        tuple_ops::transform(
          dim, m_total_dimension, stapl::multiplies<std::size_t>());

    m_levels_set = true;
  }

  template <bool forced = false, typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    base_t::set_size(spawner, views...);

    using tuple_t =
      tuple<skeletons::domain_type<typename Views::domain_type,
                                   has_finite_domain<Views>::value>...>;

    constexpr size_t idx = skeletons::first_finite_domain_index<tuple_t>::value;

    if (!m_levels_set)
    {
      std::array<nested_size_t, level + 1> level_dims;
      compute_total_dimension<level, Op>(
        get<idx>(tuple<Views const&...>(views...)), level_dims);

      this->set_level_dims(level_dims);
    }
  }

  nested_size_t total_dimension(void) const
  {
    return m_total_dimension;
  }

  nested_size_t task_dimension(void) const
  {
    return tuple_ops::transform(
      m_total_dimension,
      tuple_ops::pad_tuple<nested_dims>(base_t::dimensions(), 1),
      stapl::divides<std::size_t>());
  }

  std::array<nested_size_t, level>
  level_dimensions(void) const
  {
    return m_level_dims;
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
    t.member(m_total_dimension);
    t.member(m_level_dims);
    t.member(m_levels_set);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for 1D
///
/// @todo Ideally there shouldn't be a new span type for nested execution,
///       but in current implementation for nested skeletons, we need
///       some more information about the dimensions( e.g. each nested level
///       dimension) that makes the span type a little heavy.
///       The span type is separated so performance of skeletons with
///       non-nested execution is not hurt.
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <int dims, size_t level, typename Op>
class nest_blocked<dims, 1, Op, level>
  : public blocked<dims>
{
public:
  using base_t          = blocked<dims>;
  using size_type       = typename base_t::size_type;
  using dims_num = std::integral_constant<int, dims>;

  using nested_size_t  = size_t;
  using nested_dims_num = std::integral_constant<int, 1>;

private:
  nested_size_t                    m_total_dimension;
  std::array<nested_size_t, level> m_level_dims;
  bool                             m_levels_set = false;

public:

  void set_level_dims(std::array<nested_size_t, level + 1> level_dims)
  {
    m_total_dimension = 1;

    for (auto&& dim : level_dims)
      m_total_dimension *= dim;

    for (size_t i = 1; i < level_dims.size(); ++i)
      m_level_dims[i-1] = level_dims[i];

    m_levels_set = true;
  }

  template <bool forced = false, typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    base_t::set_size(spawner, views...);

    using tuple_t =
      tuple<skeletons::domain_type<typename Views::domain_type,
                                   has_finite_domain<Views>::value>...>;

    constexpr size_t idx = skeletons::first_finite_domain_index<tuple_t>::value;

    if (!m_levels_set)
    {
      using tuple1_t = homogeneous_tuple_type_t<1, size_t>;
      std::array<tuple1_t, level + 1> tuple_level_dims;

      compute_total_dimension<level, Op>(
        get<idx>(tuple<Views const&...>(views...)), tuple_level_dims);

      std::array<nested_size_t, level + 1> level_dims;

      for (size_t i = 0; i < level_dims.size(); ++i)
        level_dims[i] = get<0>(tuple_level_dims[i]);

      this->set_level_dims(level_dims);
    }
  }

  nested_size_t total_dimension(void) const
  {
    return m_total_dimension;
  }

  nested_size_t task_dimension(void) const
  {
    return m_total_dimension/base_t::dimensions();
  }

  std::array<nested_size_t, level>
  level_dimensions(void) const
  {
    return m_level_dims;
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
    t.member(m_total_dimension);
    t.member(m_level_dims);
    t.member(m_levels_set);
  }
};

} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_BLOCKED_HPP
