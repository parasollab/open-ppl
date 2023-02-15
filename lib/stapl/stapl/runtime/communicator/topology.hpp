/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COMMUNICATOR_TOPOLOGY_HPP
#define STAPL_RUNTIME_COMMUNICATOR_TOPOLOGY_HPP

#include "../config.hpp"
#include "../runtime_fwd.hpp"
#include "../gang_description.hpp"
#include "../utility/tree.hpp"
#include <algorithm>
#include <iterator>
#include <tuple>
#include <vector>
#include <boost/range/counting_range.hpp>
#include <boost/range/join.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Describes the topology of the processes of a gang.
///
/// This class offers a partial view of the topology. Each process knows only
/// the root of the topology, its parent and its children to keep it scalable.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class topology
{
public:
  using id             = process_id;
  using size_type      = gang_description::size_type;
  using container_type = std::vector<id>;

private:
  id             m_id;
  id             m_root;
  id             m_parent;
  container_type m_children;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref topology when there is only one process.
  //////////////////////////////////////////////////////////////////////
  topology(void)
  : m_id(get_process_id()),
    m_root(m_id),
    m_parent(m_id)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref topology based on the @ref gang_description and
  ///        the current process id.
  ///
  /// This function calls @ref gang_description::get_process_id_index() to
  /// figure out the index of @p myid in the set of processes that define the
  /// gang.
  ///
  /// @todo Building a @ref topology might be expensive since
  ///       @ref gang_description::get_process_id_index() has \f$O(n)\f$ time
  ///       complexity.
  //////////////////////////////////////////////////////////////////////
  topology(gang_description const& desc)
  : m_id(get_process_id()),
    m_root(m_id),
    m_parent(m_id)
  {
    if (desc.is_on_shmem())
      return;

    // find my index and size
    const size_type idx = desc.get_process_id_index(m_id);

    // create tree based on indices
    const auto r = make_binomial_tree(idx, desc.get_num_processes());

    // find actual ids
    m_root        = desc.get_nth_process_id(std::get<0>(r));
    m_parent      = desc.get_nth_process_id(std::get<1>(r));
    auto const& v = std::get<2>(r);
    std::transform(std::begin(v), std::end(v),
                   std::back_inserter(m_children),
                   [&](size_type i) { return desc.get_nth_process_id(i); });
  }

  id get_id(void) const noexcept
  { return m_id; }

  id root_id(void) const noexcept
  { return m_root; }

  id parent_id(void) const noexcept
  { return m_parent; }

  container_type const& children(void) const noexcept
  { return m_children; }

  auto children_and_root(void) const noexcept
    -> decltype(
         boost::join(m_children, boost::counting_range(m_root, (m_root + 1))))
  {
    return boost::join(m_children, boost::counting_range(m_root, (m_root + 1)));
  }

  bool is_root(void) const noexcept
  { return (m_id==m_parent); }

  bool is_leaf(void) const noexcept
  { return m_children.empty(); }
};

} // namespace runtime

} // namespace stapl

#endif
