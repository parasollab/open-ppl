/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_GANG_DESCRIPTION_HPP
#define STAPL_RUNTIME_GANG_DESCRIPTION_HPP

#include "config.hpp"
#include "constants.hpp"
#include "exception.hpp"
#include "utility/algorithm.hpp"
#include "utility/functional.hpp"
#include <algorithm>
#include <functional>
#include <iosfwd>
#include <iterator>
#include <unordered_set>
#include <utility>
#include <vector>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Contains information about the locations of a gang.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class gang_description
{
public:
  using size_type = std::size_t;

  //////////////////////////////////////////////////////////////////////
  /// @brief Iterator over locations ids of a specific process.
  //////////////////////////////////////////////////////////////////////
  class process_location_iterator
  : public boost::iterator_facade<
             process_location_iterator,
             const location_id,
             boost::forward_traversal_tag,
             location_id
           >
  {
  private:
    friend class boost::iterator_core_access;

    gang_description const* m_gd;
    size_type               m_index;
    process_id              m_pid;

    void find_next_on_process(void)
    {
      while (m_index < m_gd->get_num_locations() &&
             m_gd->get_process_id(m_index) != m_pid)
        ++m_index;
    }

  public:
    explicit constexpr process_location_iterator(const size_type i = 0) noexcept
    : m_gd(nullptr),
      m_index(i),
      m_pid(invalid_process_id)
    { }

    explicit process_location_iterator(gang_description const& gd,
                                       const process_id pid) noexcept
    : m_gd(&gd),
      m_index(0),
      m_pid(pid)
    {
      if (!m_gd->is_on_shmem())
        find_next_on_process();
    }

    location_id dereference(void) const noexcept
    { return m_index; }

    bool equal(process_location_iterator const& other) const noexcept
    { return (m_index==other.m_index); }

    void increment(void) noexcept
    {
      if (m_gd->is_on_shmem()) {
        // one process, all locations on it
        ++m_index;
      }
      else if (m_gd->is_on_distmem()) {
        // one location per process
        m_index = m_gd->get_num_locations();
      }
      else {
        // multiple locations per process
        ++m_index;
        find_next_on_process();
      }
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Iterator over process ids.
  //////////////////////////////////////////////////////////////////////
  class process_iterator
  : public boost::iterator_facade<
             process_iterator,
             const process_id,
             boost::random_access_traversal_tag,
             process_id
           >
  {
  private:
    friend class boost::iterator_core_access;

    gang_description const* m_gd;
    size_type               m_index;

  public:
    explicit constexpr process_iterator(const size_type i = 0) noexcept
    : m_gd(nullptr),
      m_index(i)
    { }

    explicit constexpr process_iterator(gang_description const& gd) noexcept
    : m_gd(&gd),
      m_index(0)
    { }

    process_id dereference(void) const noexcept
    { return m_gd->get_nth_process_id(m_index); }

    bool equal(process_iterator const& other) const noexcept
    { return (m_index==other.m_index); }

    void increment(void) noexcept
    { ++m_index; }

    void decrement(void) noexcept
    { --m_index; }

    void advance(difference_type n) noexcept
    { m_index += n; }

    difference_type distance_to(process_iterator const& other) const noexcept
    {
      return ((m_index<=other.m_index)
                ? difference_type(other.m_index - m_index)
                : -difference_type(m_index - other.m_index));
    }
  };

private:
  using resolution_function_type = std::function<process_id(location_id)>;
  using index_function_type      = std::function<process_id(size_type)>;

  /// Location to process mapping.
  resolution_function_type m_map;
  /// Gang size.
  size_type                m_size;
  /// Index to process mapping.
  index_function_type      m_proc_map;
  /// Number of processes.
  size_type                m_nprocs;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref gang_description object.
  ///
  /// @param rf     Function to return on which process a location is on.
  /// @param size   Size of the new gang.
  /// @param pf     Function to return the process id based on its index.
  /// @param nprocs Number of processes.
  //////////////////////////////////////////////////////////////////////
  template<typename ResolutionFunction, typename PhysicalMappingFunction>
  gang_description(ResolutionFunction&& rf,
                   const size_type size,
                   PhysicalMappingFunction&& pf,
                   const size_type nprocs)
  : m_map(std::forward<ResolutionFunction>(rf)),
    m_size(size),
    m_proc_map(std::forward<PhysicalMappingFunction>(pf)),
    m_nprocs(nprocs)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref gang_description object that has @p size number
  ///        of locations on the same process.
  ///
  /// @param size Size of the new gang.
  /// @param pid  Process id.
  //////////////////////////////////////////////////////////////////////
  gang_description(const size_type size,
                   const process_id pid)
  : m_map([pid](location_id) { return pid; }),
    m_size(size),
    m_proc_map([pid](size_type) { return pid; }),
    m_nprocs(1)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref gang_description object from the given one and
  ///        the mapping function.
  ///
  /// @warning This function will perform a linear search of the location id
  ///          space to find which of the locations are on the same process and
  ///          build the necessary mapping functions.
  ///
  /// @param other @ref gang_description to base the created one on.
  /// @param rf    Function that translates location ids of the created gang to
  ///              location ids of @p other.
  /// @param size  Size of the new gang.
  //////////////////////////////////////////////////////////////////////
  template<typename ResolutionFunction>
  gang_description(gang_description const& other,
                   ResolutionFunction&& rf,
                   const size_type size)
  : m_size(size),
    m_nprocs(0)
  {
    STAPL_RUNTIME_ASSERT(size<=other.get_num_locations());
    if (other.is_on_shmem()) {
      // one process
      const auto pid = other.get_process_id(0);
      m_map          = [pid](location_id) { return pid; };
      m_proc_map     = [pid](size_type) { return pid; };
      m_nprocs       = 1;
    }
    else if (other.is_on_distmem()) {
      // one location per process, compose resolution functions
      auto const& other_f = other.m_map;
      auto nrf   = [other_f, rf](location_id lid) { return other_f(rf(lid)); };
      m_map      = nrf;
      m_proc_map = std::move(nrf); // maps [0, size) index to process id
      m_nprocs   = m_size;
    }
    else {
      // collect the unique processes, O(num_locs)
      std::vector<process_id> pids;
      std::unordered_set<process_id> added;
      for (size_type i = 0;
             i<m_size && pids.size() < other.get_num_processes();
               ++i) {
        const auto pid = other.m_map(rf(i));
        if (added.insert(pid).second)
          pids.push_back(pid);
      }

      if (pids.size()==1) {
        // one process
        const auto pid = *std::begin(pids);
        m_map          = [pid](location_id) { return pid; };
        m_proc_map     = [pid](size_type) { return pid; };
        m_nprocs       = 1;
      }
      else {
        // one or more locations per process, compose resolution functions
        auto const& other_f = other.m_map;
        auto nrf = [other_f, rf](location_id lid) { return other_f(rf(lid)); };
        m_map      = std::move(nrf);
        m_nprocs   = pids.size();
        m_proc_map = arbitrary<size_type, process_id>(std::move(pids));
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of locations of the gang.
  //////////////////////////////////////////////////////////////////////
  size_type get_num_locations(void) const noexcept
  { return m_size; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a range of the locations that are on process @p pid.
  //////////////////////////////////////////////////////////////////////
  boost::iterator_range<process_location_iterator>
  get_location_ids(const process_id pid) const noexcept
  {
    return boost::make_iterator_range(
             process_location_iterator{*this, pid},
             process_location_iterator{get_num_locations()});
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of processes the gang is on.
  //////////////////////////////////////////////////////////////////////
  size_type get_num_processes(void) const noexcept
  { return m_nprocs; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a range of the processes the gang is on.
  //////////////////////////////////////////////////////////////////////
  boost::iterator_range<process_iterator> get_processes(void) const noexcept
  {
    return boost::make_iterator_range(process_iterator{*this},
                                      process_iterator{get_num_processes()});
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if all the locations are on the same process.
  //////////////////////////////////////////////////////////////////////
  bool is_on_shmem(void) const noexcept
  { return (m_nprocs==1); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if no two locations are on the same process.
  //////////////////////////////////////////////////////////////////////
  bool is_on_distmem(void) const noexcept
  { return (get_num_locations()==get_num_processes()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the id of the process that the location id is on.
  //////////////////////////////////////////////////////////////////////
  process_id get_process_id(const location_id lid) const noexcept
  {
    STAPL_RUNTIME_ASSERT(lid<get_num_locations());
    return m_map(lid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the nth process of the gang.
  //////////////////////////////////////////////////////////////////////
  process_id get_nth_process_id(const size_type index) const noexcept
  {
    STAPL_RUNTIME_ASSERT(index<get_num_processes());
    return m_proc_map(index);
  }

// ----------------------------------------------------------------------------
// Functions relying on iteration on mapping functions
// ----------------------------------------------------------------------------

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the local index of the location @p lid.
  ///
  /// @warning This function will perform a linear search of the location id
  ///          space to find which locations are on the same process.
  //////////////////////////////////////////////////////////////////////
  size_type get_location_local_index(const location_id lid) const noexcept
  {
    if (is_on_shmem()) {
      // all locations on one process, location id is the index
      return lid;
    }

    if (is_on_distmem()) {
      // one location locally managed, index always 0
      return 0;
    }

    // multiple locations on one process, O(num_locs)
    const auto pid = get_process_id(lid);
    size_type cnt = 0;
    for (size_type i=0; i<lid; ++i) {
      if (get_process_id(i)==pid)
        ++cnt;
    }
    return cnt;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of locations that are on the process @p pid.
  ///
  /// @warning This function will perform a linear search of the location id
  ///          space to find which locations are on the same process.
  //////////////////////////////////////////////////////////////////////
  size_type get_num_locations(const process_id pid) const noexcept
  {
    if (is_on_shmem()) {
      // one process, all locations on it
      return get_num_locations();
    }

    if (is_on_distmem()) {
      // one location per process
      return 1;
    }

    // multiple locations per process, O(num_locs)
    size_type cnt = 0;
    for (size_type i=0; i<get_num_locations(); ++i)
      if (get_process_id(i)==pid)
        ++cnt;
    return cnt;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index in the process set of the process with the given
  ///        id.
  ///
  /// @warning This function will perform a linear search of the process id
  ///          space to find the index.
  //////////////////////////////////////////////////////////////////////
  size_type get_process_id_index(const process_id pid) const noexcept
  {
    // first test if it is an ordered range [0, p)
    if (size_type(pid)<get_num_processes() && get_nth_process_id(pid)==pid)
      return pid;

    // search over all processes, O(num_procs)
    size_type i = 0;
    for (; i<get_num_processes() && (get_nth_process_id(i)!=pid); ++i);
    STAPL_RUNTIME_ASSERT(i!=get_num_processes());
    return i;
  }
};

std::ostream& operator<<(std::ostream&, gang_description const&);

} // namespace runtime

} // namespace stapl

#endif
