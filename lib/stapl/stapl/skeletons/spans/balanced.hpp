/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_BALANCED_HPP
#define STAPL_SKELETONS_SPANS_BALANCED_HPP

#include <stapl/skeletons/spans/span_helpers.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

template <int i = 1>
class balanced;

//////////////////////////////////////////////////////////////////////
/// @brief A balanced span which is used the most across the
/// skeletons library. It assigns a balanced <b>spawn</b> load to each
/// location. The load on each location is roughly \f$\frac{n}{p}\f$.
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <>
class balanced<1>
{
public:
  using dims_num       = std::integral_constant<int, 1>;
  using domain_type    = indexed_domain<std::size_t>;
  using index_type     = domain_type::index_type;
  using size_type      = domain_type::size_type;
  using dimension_type = std::integral_constant<int, 1>;

protected:
  balanced_partition<domain_type>     m_partition;
  std::size_t                         m_cur_pid;

public:
  template <bool forced = false, typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    using VDomains = stapl::tuple<skeletons::domain_type<
                       typename Views::domain_type,
                       has_finite_domain<Views>::value>...>;

    auto dom = stapl::get<
                 skeletons::first_finite_domain_index<VDomains>::value
               >(make_tuple(
                 skeletons::domain_type<
                   typename Views::domain_type, has_finite_domain<Views>::value
                 >(views.domain())...));

    m_partition = balanced_partition<domain_type>(
                    domain_type(dom.dimensions()), spawner.get_num_PEs());

    m_cur_pid = spawner.get_PE_id();
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Change view and container implementation to pass iterator
  /// or collection of intervals in a more generic, lightweight manner
  /// than std::vector.
  //////////////////////////////////////////////////////////////////////
  std::vector<domain_type> local_domain() const
  {
    std::vector<domain_type> v;
    v.push_back(m_partition[m_cur_pid]);
    return v;
  }

  template <typename Coord>
  bool should_spawn (Coord const&, Coord const&) const
  {
    return true;
  }

  size_type dimensions() const
  {
    return m_partition.global_domain().dimensions();
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

  std::size_t size() const
  {
    return m_partition.global_domain().size();
  }

  std::size_t linearize(index_type const& coord) const
  {
    return coord;
  }

  void define_type(typer& t)
  {
    t.member(m_partition);
    t.member(m_cur_pid);
  }
};


} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_BALANCED_HPP
