/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_SCHEDULER_STEAL_POLICIES_HPP
#define STAPL_RUNTIME_EXECUTOR_SCHEDULER_STEAL_POLICIES_HPP

#include <stapl/runtime/random_location_generator.hpp>
#include <stapl/runtime/stapl_assert.hpp>
#include <cmath>
#include <iosfwd>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object to generate neighbors of a location id in a 2D space.
///
/// @ingroup executorsImpl
///
/// @todo It returns a @c std::set which is not very efficient. It has to be
///       changed to a @c std::vector or a @c std::array.
//////////////////////////////////////////////////////////////////////
template<typename T>
class processor_map_2D
{
public:
  typedef std::size_t size_type;
  typedef std::set<T> result_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Converts a 2D id to a 1D id.
  //////////////////////////////////////////////////////////////////////
  static constexpr T convert_to_1D(std::pair<size_type, size_type> const& p,
                                   size_type ncols)
  { return (p.first * ncols + p.second); }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the neighbors of @p proc_id.
  ///
  /// Since no duplicate ids are allowed, the number of returned neighbors may
  /// be less than @p num_procs.
  //////////////////////////////////////////////////////////////////////
  static result_type apply(size_type proc_id, size_type num_procs)
  {
    stapl_assert(proc_id < num_procs, "Id greater than number of locations");

    // calculate the most square layout
    size_type num_rows = std::sqrt(double(num_procs));
    while ((num_procs % num_rows) != 0)
      --num_rows;

    const size_type num_cols = (num_procs/num_rows);

    // compute id in the 2D space
    const size_type row = (proc_id / num_cols);
    const size_type col = (proc_id % num_cols);

    // compute the neighbors in the 2D space
    const std::pair<size_type, size_type>
      left_neighbor(  row, ((col>0) ? (col-1) : (num_cols-1)) ),
      up_neighbor(    ((row>0) ? (row-1) : (num_rows-1)), col ),
      right_neighbor( row, ((col+1) % num_cols)               ),
      down_neighbor(  ((row+1) % num_rows), col               );

    result_type neighbors;

    const T left_neighbor_id  = convert_to_1D(left_neighbor, num_cols);
    const T up_neighbor_id    = convert_to_1D(up_neighbor, num_cols);
    const T right_neighbor_id = convert_to_1D(right_neighbor, num_cols);
    const T down_neighbor_id  = convert_to_1D(down_neighbor, num_cols);

    // do not mark yourself as neighbor
    if (left_neighbor_id != proc_id)
      neighbors.insert(left_neighbor_id);
    if (up_neighbor_id != proc_id)
      neighbors.insert(up_neighbor_id);
    if (right_neighbor_id != proc_id)
      neighbors.insert(right_neighbor_id);
    if (down_neighbor_id != proc_id)
      neighbors.insert(down_neighbor_id);

    return neighbors;
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Base class for stealing policies.
///
/// @ingroup workstealing
///
/// @bug @p fraction is not used anywhere.
//////////////////////////////////////////////////////////////////////
class steal_policy_base
{
public:
  typedef std::size_t size_type;

private:
  const size_type m_chunk_size;
  const size_type m_fraction;
  size_type       m_migrated;

  //////////////////////////////////////////////////////////////////////
  /// @brief Expected work.
  ///
  /// There is no ordering between the notifications of steal completion and the
  /// actual work messages. So you may get all the notifications first and no
  /// work, or you may get all the work but no notifications. Therefore,
  /// @ref m_expected_work is a signed counter. The -ve side represents the
  /// received work. The +ve side represents the total expected work. So in
  /// equilibrium, the sum should be 0.
  //////////////////////////////////////////////////////////////////////
  int             m_expected_work;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref steal_policy_base.
  ///
  /// @param chunk_size Number of entries to steal.
  /// @param fraction   Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  steal_policy_base(size_type chunk_size, size_type fraction) noexcept
    : m_chunk_size(chunk_size),
      m_fraction(fraction),
      m_migrated(0),
      m_expected_work(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the scheduler has available work to be stolen.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  bool has_work(Scheduler const& scheduler) const noexcept
  { return (scheduler.migratable_entries().size() > m_chunk_size); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Processes steal requests that it got from thieves.
  ///
  /// This function migrates work to the thieves. It will distribute work
  /// according to the steal requests that were received.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void process_steal_requests(Scheduler& scheduler, task_graph* tg_ptr)
  {
    // Do not use iterators, as iterators can be invalidated as a result of
    // processing RMIs that request additional steals.
    auto& thieves = scheduler.thieves();
    while (!thieves.empty()) {
      // get the current thief
      const location_type thief = thieves.front();
      thieves.pop_front();

      // process the steal request for this thief
      if (has_work(scheduler))
        migrate_work(scheduler, thief, m_chunk_size, tg_ptr);
      else
        migrate_work(scheduler, thief, 0, tg_ptr);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Migrate some work to a thief.
  ///
  /// @param scheduler      Scheduler to migrate work from.
  /// @param loc            Thief to migrate work to.
  /// @param num_to_migrate How much work to migrate.
  ///
  /// @todo This generates @p num_to_migrate migration requests. It might be
  ///       more efficient to do it in one shot, rather wait from the runtime
  ///       to aggregate requests.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void migrate_work(Scheduler& scheduler,
                    location_type loc,
                    size_type num_to_migrate,
                    task_graph* tg_ptr)
  {
    // Do not use iterators, as iterators can be invalidated as a result of
    // processing RMIs that request additional steals.
    auto& entries = scheduler.migratable_entries();
    size_type loot = 0;
    for (; !entries.empty() && loot<num_to_migrate; ++loot) {
      auto& e = entries.back();
      entries.pop_back();
      e.migrate(loc, tg_ptr);
    }
    m_migrated += loot;

    // signal completion of steal request so that the thief can steal again
    scheduler.notify_steal_completion(loc, loot);
  }

  int expected_work(void) const noexcept
  { return m_expected_work; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a received work request.
  //////////////////////////////////////////////////////////////////////
  void receive_work_notification(void) noexcept
  { --m_expected_work; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(const size_type loot) noexcept
  { m_expected_work += loot; }

  friend std::ostream& operator<<(std::ostream& os, steal_policy_base const& p)
  {
    return os << " m_migrated = "      << p.m_migrated
              << " m_fraction = "      << p.m_fraction
              << " m_expected_work = " << p.m_expected_work;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Random-k random-k stealing policy.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class rk_rk_steal
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;

private:
  /// Random location id generator.
  random_location_generator m_gen;
  /// Pending steal requests.
  size_type                 m_pending;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref rk_rk_steal object.
  ///
  /// @param id            Current location id.
  /// @param num_locations Number of locations.
  /// @param chunk_size    Number of entries to steal.
  /// @param fraction      Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  rk_rk_steal(location_type id,
              location_type num_locations,
              size_type chunk_size = 1,
              size_type fraction = 2)
    : steal_policy_base(chunk_size, fraction),
      m_gen(id, num_locations),
      m_pending(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type lid           = scheduler.get_location_id();
    const location_type num_locations = scheduler.get_num_locations();

    if (m_pending > 0 || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    // random steal
    typedef std::unordered_set<location_type> victims_list_type;

    const size_type random_k = m_gen();
    victims_list_type victims;
    victims.reserve(random_k);

    for (size_type i=0; i<random_k; ++i) {
      const location_type victim = m_gen();
      if (victim == lid)
        continue;

      std::pair<victims_list_type::iterator, bool> ret = victims.insert(victim);

      // prevent one more iteration over the set of victims and post the steal
      // request right away
      if (ret.second == true) {
        ++m_pending;
        scheduler.request_steal(victim);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type, size_type loot) noexcept
  {
    stapl_assert(m_pending > 0, "no pending steals");
    --m_pending;
    steal_policy_base::receive_steal_completion(loot);
  }

  friend std::ostream& operator<<(std::ostream& os, rk_rk_steal const& p)
  {
    return os << " rk_rk_steal:"
              << static_cast<steal_policy_base const&>(p);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Random-k stealing policy.
///
/// @ingroup workstealing
///
/// @todo @c k is not customizable.
//////////////////////////////////////////////////////////////////////
class rk_steal
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;

private:
  /// Number of random requests.
  const size_type           m_k;
  /// Random location id generator.
  random_location_generator m_gen;
  /// Pending steal requests.
  size_type                 m_pending;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref rk_steal object.
  ///
  /// @param id            Current location id.
  /// @param num_locations Number of locations.
  /// @param chunk_size    Number of entries to steal.
  /// @param fraction      Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  rk_steal(location_type id,
           location_type num_locations,
           size_type chunk_size = 1,
           size_type fraction = 2)
    : steal_policy_base(chunk_size, fraction),
      m_k(8),
      m_gen(id, num_locations),
      m_pending(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type lid           = scheduler.get_location_id();
    const location_type num_locations = scheduler.get_num_locations();

    if (m_pending > 0 || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    // random steal
    typedef std::unordered_set<location_type> victims_list_type;

    const size_type n_steals = std::min(size_type(num_locations-1), m_k);
    victims_list_type victims;
    victims.reserve(n_steals);
    m_pending = n_steals;

    while (victims.size() < n_steals) {
      location_type victim = m_gen();
      while (victim == lid)
        victim = m_gen();

      std::pair<victims_list_type::iterator, bool> ret = victims.insert(victim);

      // prevent one more iteration over the set of victims and post the steal
      // request right away
      if (ret.second == true)
        scheduler.request_steal(victim);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type, size_type loot) noexcept
  {
    stapl_assert(m_pending > 0, "no pending steals");
    --m_pending;
    steal_policy_base::receive_steal_completion(loot);
  }

  friend std::ostream& operator<<(std::ostream& os, rk_steal const& p)
  {
    return os << " r" << p.m_k << "_steal:"
              << static_cast<steal_policy_base const&>(p);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Random neighbor stealing policy.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class neighbor_random_hybrid_steal
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;

private:
  typedef std::unordered_map<location_type, size_type> map_type;
  typedef std::vector<location_type>                   neighbor_list_type;

  /// Neighbor locations.
  const neighbor_list_type  m_neighbors;
  /// Random location id generator.
  random_location_generator m_gen;
  /// Map of the sources.
  map_type                  m_sources;
  /// Pending steal requests.
  size_type                 m_pending;
  /// @c true when it is ok to steal from neighbors.
  bool                      m_steal_from_neighbors;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a @ref neighbor_list_type from the given range.
  //////////////////////////////////////////////////////////////////////
  template<typename Range>
  static neighbor_list_type make_neighbor_list(Range&& r)
  { return neighbor_list_type{std::begin(r), std::end(r)}; }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref neighbor_random_hybrid_steal object.
  ///
  /// @param id            Current location id.
  /// @param num_locations Number of locations.
  /// @param chunk_size    Number of entries to steal.
  /// @param fraction      Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  neighbor_random_hybrid_steal(location_type id,
                               location_type num_locations,
                               size_type chunk_size = 1,
                               size_type fraction = 2)
    : steal_policy_base(chunk_size, fraction),
      m_neighbors(
        make_neighbor_list(
          detail::processor_map_2D<location_type>::apply(id, num_locations))),
      m_gen(id, num_locations),
      m_pending(0),
      m_steal_from_neighbors(true)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type num_locations = scheduler.get_num_locations();

    if (m_pending > 0 || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    // neighborhood steal
    if (m_steal_from_neighbors) {
      m_pending = m_neighbors.size();
      for (neighbor_list_type::const_iterator it=m_neighbors.begin();
           it!=m_neighbors.end();
           ++it)
        scheduler.request_steal(*it);

      // disable future neighborhood steals
      m_steal_from_neighbors = false;
    }
    else if (m_sources.size() > 0) {
      // steal from some known sources
      m_pending = m_sources.size();
      for (auto const& it : m_sources)
        scheduler.request_steal(it.first);
    }
    else {
      // random steal
      const location_type lid = scheduler.get_location_id();
      m_pending = 1;
      location_type victim = m_gen();
      while (victim == lid)
        victim = m_gen();
      scheduler.request_steal(victim);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type loc, size_type loot)
  {
    stapl_assert(m_pending > 0, "no pending steals");

    --m_pending;
    steal_policy_base::receive_steal_completion(loot);

    // update the map of sources
    map_type::iterator iter = m_sources.find(loc);

    // did we receive anything?
    if (loot > 0) {
      if (iter != m_sources.end())
        iter->second = loot;
      else
        m_sources.emplace(loc, loot);
    }
    else {
      if (iter != m_sources.end())
        m_sources.erase(iter);
    }
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  neighbor_random_hybrid_steal const& p)
  {
    return os << " neighbor_random_hybrid_steal:"
              << static_cast<steal_policy_base const&>(p);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Steal from left neighbor stealing policy.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class steal_from_previous
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;

private:
  /// Pending steal requests.
  bool                m_pending_steal;
  const location_type m_left_neighbor;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref steal_from_previous object.
  ///
  /// @param id            Current location id.
  /// @param num_locations Number of locations.
  /// @param chunk_size    Number of entries to steal.
  /// @param fraction      Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  steal_from_previous(location_type id,
                      location_type num_locations,
                      size_type chunk_size = 1,
                      size_type fraction = 2) noexcept
    : steal_policy_base(chunk_size, fraction),
      m_pending_steal(false),
      m_left_neighbor((id==0) ? (num_locations-1) : (id-1))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type num_locations = scheduler.get_num_locations();

    if (m_pending_steal || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    m_pending_steal = true;

    scheduler.request_steal(m_left_neighbor);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type, size_type loot) noexcept
  {
    stapl_assert(m_pending_steal, "no pending steals");
    m_pending_steal = false;
    steal_policy_base::receive_steal_completion(loot);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  steal_from_previous const& p)
  {
    return os << " steal_from_previous:"
              << static_cast<steal_policy_base const&>(p)
              << " m_pending_steal = " << p.m_pending_steal;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Stealing policy from one random neighbor.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class random_lifeline_steal
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;

private:
  /// Random location id generator.
  random_location_generator m_gen;
  /// Pending steal requests.
  bool                      m_pending_steal;
  /// Last steal location.
  location_type             m_last_steal;
  /// Last loot received.
  size_type                 m_last_loot;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref random_lifeline_steal object.
  ///
  /// @param id            Current location id.
  /// @param num_locations Number of locations.
  /// @param chunk_size    Number of entries to steal.
  /// @param fraction      Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  random_lifeline_steal(location_type id,
                        location_type num_locations,
                        size_type chunk_size = 1,
                        size_type fraction = 2)
    : steal_policy_base(chunk_size, fraction),
      m_gen(id, num_locations),
      m_pending_steal(false),
      m_last_steal(id),
      m_last_loot(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type lid           = scheduler.get_location_id();
    const location_type num_locations = scheduler.get_num_locations();

    // has the previous steal completed?
    if (m_pending_steal || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    m_pending_steal = true;

    // do we steal from the same location?
    if (m_last_loot == 0) {
      m_last_steal = m_gen();
      while (m_last_steal == lid)
        m_last_steal = m_gen();
    }

    scheduler.request_steal(m_last_steal);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type, size_type loot) noexcept
  {
    stapl_assert(m_pending_steal, "no pending steals");
    m_pending_steal = false;
    m_last_loot = loot;
    steal_policy_base::receive_steal_completion(loot);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  random_lifeline_steal const& p)
  {
    return os << " random_lifeline_steal:"
              << static_cast<steal_policy_base const&>(p)
              << " m_pending_steal = " << p.m_pending_steal
              << " m_last_steal = "    << p.m_last_steal
              << " m_last_loot = "     << p.m_last_loot;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Steal in a circular way (always advance to the right neighbor of the
///        last victim stealing policy.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class circular_steal
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;

private:
  /// Pending steal requests.
  bool          m_pending_steal;
  /// Last steal location.
  location_type m_last_steal;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref circular_steal object.
  ///
  /// @param id         Current location id.
  /// @param chunk_size Number of entries to steal.
  /// @param fraction   Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  circular_steal(location_type id,
                 location_type,
                 size_type chunk_size = 1,
                 size_type fraction = 2) noexcept
    : steal_policy_base(chunk_size, fraction),
      m_pending_steal(false),
      m_last_steal(id)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type lid           = scheduler.get_location_id();
    const location_type num_locations = scheduler.get_num_locations();

    // has the previous steal completed?
    if (m_pending_steal || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    m_pending_steal = true;

    m_last_steal = (m_last_steal + 1) % num_locations;
    if (m_last_steal == lid)
      m_last_steal = (m_last_steal + 1) % num_locations;

    scheduler.request_steal(m_last_steal);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type, size_type loot) noexcept
  {
    stapl_assert(m_pending_steal, "no pending steals");
    m_pending_steal = false;
    steal_policy_base::receive_steal_completion(loot);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  circular_steal const& p)
  {
    return os << " circular_steal:"
              << static_cast<steal_policy_base const&>(p)
              << " m_pending_steal = " << p.m_pending_steal
              << " m_last_steal = "    << p.m_last_steal;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Steal in a circular way (always advance to the right neighbor of the
///        last victim stealing policy. Avoids stealing from the same location
///        if it didn't have anything the last time.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class circular_lifeline_steal
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;

private:
  /// Pending steal requests.
  bool          m_pending_steal;
  /// Last steal location.
  location_type m_last_steal;
  /// Last loot received.
  size_type     m_last_loot;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref circular_lifeline_steal object.
  ///
  /// @param id         Current location id.
  /// @param chunk_size Number of entries to steal.
  /// @param fraction   Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  circular_lifeline_steal(location_type id,
                          location_type,
                          size_type chunk_size = 1,
                          size_type fraction = 2) noexcept
    : steal_policy_base(chunk_size, fraction),
      m_pending_steal(false),
      m_last_steal(id),
      m_last_loot(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type lid           = scheduler.get_location_id();
    const location_type num_locations = scheduler.get_num_locations();

    // has the previous steal completed?
    if (m_pending_steal || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    m_pending_steal = true;

    if (m_last_loot == 0) {
      // avoid stealing from the same location if there was no loot
      m_last_steal = (m_last_steal + 1) % num_locations;
      if (m_last_steal == lid)
        m_last_steal = (m_last_steal + 1) % num_locations;
    }

    scheduler.request_steal(m_last_steal);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type, size_type loot) noexcept
  {
    stapl_assert(m_pending_steal, "no pending steals");
    m_pending_steal = false;
    m_last_loot = loot;
    steal_policy_base::receive_steal_completion(loot);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  circular_lifeline_steal const& p)
  {
    return os << " circular_lifeline_steal:"
              << static_cast<steal_policy_base const&>(p)
              << " m_pending_steal = " << p.m_pending_steal
              << " m_last_steal = "    << p.m_last_steal
              << " m_last_loot = "     << p.m_last_loot;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Diffusive stealing policy.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class diffusive_steal
  : public steal_policy_base
{
public:
  using steal_policy_base::size_type;
private:
  typedef std::vector<location_type> neighbor_list_type;

  /// Neighbor locations.
  const neighbor_list_type m_neighbors;
  /// Pending steal requests.
  size_type                m_pending;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a @ref neighbor_list_type from the given range.
  //////////////////////////////////////////////////////////////////////
  template<typename Range>
  static neighbor_list_type make_neighbor_list(Range&& r)
  { return neighbor_list_type{std::begin(r), std::end(r)}; }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref circular_steal object.
  ///
  /// @param id            Current location id.
  /// @param num_locations Number of locations.
  /// @param chunk_size    Number of entries to steal.
  /// @param fraction      Fraction of the total entries to steal.
  //////////////////////////////////////////////////////////////////////
  diffusive_steal(location_type id,
                  location_type num_locations,
                  size_type chunk_size = 1,
                  size_type fraction = 2)
    : steal_policy_base(chunk_size, fraction),
      m_neighbors(
        make_neighbor_list(
          detail::processor_map_2D<location_type>::apply(id, num_locations))),
      m_pending(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Steal some work.
  ///
  /// @param scheduler Scheduler to steal entries from.
  //////////////////////////////////////////////////////////////////////
  template<typename Scheduler>
  void steal(Scheduler& scheduler)
  {
    const location_type num_locations = scheduler.get_num_locations();

    if (m_pending > 0 || num_locations == 1)
      return;

    // This means that we have received all the notifications.
    // But have we received all the work ? Also at this point
    // expected_work() >= 0 (since all the expected loot has
    // been added to it)
    stapl_assert(expected_work() >= 0, "more than expected work received");

    // are we waiting on some more work?
    if (expected_work() > 0)
      return;

    // neighborhood steal
    m_pending = m_neighbors.size();

    for (neighbor_list_type::const_iterator it=m_neighbors.begin();
         it!=m_neighbors.end();
         ++it)
      scheduler.request_steal(*it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive notification of a steal request completion.
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type, size_type loot) noexcept
  {
    stapl_assert(m_pending > 0, "no pending steals");
    --m_pending;
    steal_policy_base::receive_steal_completion(loot);
  }

  friend std::ostream& operator<<(std::ostream& os, diffusive_steal const& p)
  {
    return os << " diffusive_steal:"
              << static_cast<steal_policy_base const&>(p);
  }
};

} // namespace stapl

#endif
