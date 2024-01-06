/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_ELEM_IMPL_HPP
#define STAPL_SKELETONS_OPERATORS_ELEM_IMPL_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/consumer_count.hpp>
#include <stapl/skeletons/operators/elem_helpers.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template<typename Span, bool b = has_metadata_type<Span>::value>
struct dimensions_metadata_type
{
  using type          = typename Span::metadata_type;
  using metadata_type = type;

  static type call(Span const& span)
  {
    return span.dimensions_metadata();
  }
};

template<typename Span>
struct dimensions_metadata_type<Span, false>
{
  using type = typename Span::size_type;

  static type call(Span const& span)
  {
    return span.dimensions();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Every parametric dependency is wrapped by this class in order
/// to represent an executable, expandable skeleton. This class has a
/// generic interface for spawning parametric dependencies,
/// and for accessing the flows connected to each node.
///
/// An expanded parametric dependency is a skeleton for which the dimensions
/// are set based on the given span and can be spawned in a given environment.
///
/// @tparam Skeleton the enclosed skeleton that should be spawned
/// @tparam Span    the span (iteration space) of the enclosed
///                 parametric dependency
/// @tparam Flows   the flow that should be used for the expanded
///                 parametric dependency. @c flows::forked is usually used
///
/// @ingroup skeletonsOperatorsInternal
//////////////////////////////////////////////////////////////////////
template <typename Skeleton, typename Span, typename Flows>
class elem
  : public Skeleton,
    public dimensions_metadata_type<typename std::decay<Span>::type>
{
public:
  using span_type           = Span;
  using dims_type           = typename Span::size_type;
  using index_type          = typename Span::index_type;
  using op_type             = typename Skeleton::op_type;
private:
  dims_type m_dims;
  span_type m_span;

public:
  using nested_p_type       = Skeleton;
  using ports_t             = typename Flows::template port_types<elem>;
  using in_port_type        = typename ports_t::in_port_type;
  using skeleton_tag_type   = tags::unnamed_skeleton;

  static constexpr std::size_t in_port_size = ports_t::in_port_size;

  template <typename In>
  struct out_port_type
  {
    using type = typename ports_t::template out_port_type<In>::type;
  };

  elem(Skeleton skeleton, Span span = Span())
    : Skeleton(std::move(skeleton)), m_span(std::move(span))
  { }

public:
  auto dimensions_metadata(void) const
  STAPL_AUTO_RETURN((
    dimensions_metadata_type<span_type>::call(m_span)
  ))

  dims_type dimensions(void) const
  {
    return m_dims;
  }

  template <typename Spawner, typename... Views>
  void set_dimensions(Spawner const& spawner, Views const&... views)
  {
    m_span.set_size(spawner, views...);
    m_dims = m_span.dimensions();
  }

  span_type& span()
  {
    return m_span;
  }

  span_type const& span() const
  {
    return m_span;
  }

  std::size_t last_id() const
  {
    return m_span.size();
  }

  Skeleton const& nested_skeleton() const
  {
    return static_cast<Skeleton const&>(*this);
  }

  Skeleton& nested_skeleton()
  {
    return static_cast<Skeleton&>(*this);
  }

  in_port_type
  in_port(std::size_t lid_offset) const
  {
    return ports_t(*this).in_port(lid_offset);
  }

  template <typename In>
  typename out_port_type<In>::type
  out_port(In const& in, std::size_t lid_offset) const
  {
    return ports_t(*this).out_port(in, lid_offset);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief spawns elements of the expanded parametric dependency using
  /// the given spawner. It does this by hiding @c spawn_element details
  /// from the parametric dependency and passing a @c spawn_visitor to the
  /// parametric dependencies.
  ///
  /// @param spawner      the spawner that is going to be used to spawn
  ///                     elements of the enclosed parametric dependency.
  ///                     Typically, the given spawner is associated with
  ///                     @c taskgraph_env and would create a task per
  ///                     @c spawn_element request
  /// @param lid_offset   the offset from which the underlying elementary
  ///                     skeleton should spawn
  /// @param skeleton_size the size of the skeleton as a tuple of dimensions
  /// @param coord        the coordinate for the parametric dependency. The
  ///                     given coordinate will be extended with one new
  ///                     dimension in this level and will traverse the
  ///                     iteration space specified by the @c Span
  /// @param in           the input flow to the underlying elementary.
  /// @param out          the output flow from this elementary and it is
  ///                     used to determine the number of successors of
  ///                     each spawned element in the final dependence
  ///                     graph
  /// @param cur_stage    current stage of the spawning process of this
  ///                     skeleton
  //////////////////////////////////////////////////////////////////////
  template<typename Spawner, typename Coord, typename In, typename Out>
  bool spawn(Spawner& spawner, std::size_t lid_offset,
             Coord const& skeleton_size, Coord const& coord,
             In& in, Out& out,
             std::size_t cur_stage = 0)
  {
    auto cur_skeleton_size = stapl::tuple_cat(
                               stapl::make_tuple(m_dims), skeleton_size);

    // then we have to iterate over the local portions of the view that this
    // processor owns
    auto&& p       = ports_t(*this);
    auto&& cur_in  = p.in_flow(in, lid_offset);
    auto&& cur_out = p.out_flow(out, lid_offset);

    spawn_visitor<Spawner> visitor(spawner);

    // Spans return a vector of subdomains that this location is
    // responsible for spawning.
    auto domains = m_span.local_domain();

    for (auto&& dom : domains)
    {
      auto cur = dom.first();
      auto sz  = dom.size();

      for (std::size_t i = 0; i < sz; ++i) {
        auto cur_coord = stapl::tuple_cat(stapl::make_tuple(cur), coord);

        if (m_span.should_spawn(cur_skeleton_size, cur_coord)) {

          auto&& result_id = this->get_result_id(cur_skeleton_size, cur_coord);

          visitor.set_info(lid_offset + m_span.linearize(cur),
                         result_id,
                           skeletons::consumer_count(cur_out, cur_coord));

          Skeleton::configure(cur, m_span);
          Skeleton::case_of(cur_skeleton_size, cur_coord, visitor, cur_in);
        }
        cur = dom.advance(cur, 1);
      }
    }

    return true;
  }

  void define_type(typer& t)
  {
    t.base<Skeleton>(*this);
    t.member(m_dims);
    t.member(m_span);
  }
};

} // namespace skeletons_impl
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_ELEM_IMPL_HPP
