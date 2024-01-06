/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/gang.hpp>
#include <stapl/runtime/gang_md.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/non_rmi/rpc.hpp>
#include <functional>
#include <queue>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <stapl/utility/hash_fwd.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Gang id and metadata distributor.
///
/// Used to transport gang ids and metadata by waiting on information from a
/// parent location and then sending information to children locations.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
class distributor
{
public:
  using size_type = gang_md::size_type;
  /// Distributed memory key.
  using key_type  = std::tuple<gang_md::id, location_md::id, comparable_proxy>;
  /// Shared memory key.
  using shmem_key_type = std::tuple<key_type, size_type>;

private:
  // if multimap/unordered_multimap preserves ordering, then both can become
  // unordered_multimap<key_type, T>
  using id_map_type = std::unordered_map<
                        key_type, std::queue<gang_md::id>, boost::hash<key_type>
                      >;

  using md_map_type = std::unordered_map<
                        shmem_key_type,
                        std::queue<std::pair<gang_md*, size_type>>,
                        boost::hash<shmem_key_type>>;

  /// Gang id registry.
  static id_map_type s_gid;
  static std::mutex  s_gid_mtx;

  /// Gang metadata registry.
  static md_map_type s_gang;
  static std::mutex  s_gang_mtx;

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the key and the gang id to the map.
  //////////////////////////////////////////////////////////////////////
  static void add(key_type const& key, const gang_md::id gid)
  {
    std::lock_guard<std::mutex> lock{s_gid_mtx};
    auto& q = s_gid[key];
    q.push(gid);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the gang id associated with the key, otherwise
  ///        @ref invalid_gang_id.
  //////////////////////////////////////////////////////////////////////
  static gang_md::id try_get_id(key_type const& key)
  {
    std::lock_guard<std::mutex> lock{s_gid_mtx};

    auto it = s_gid.find(key);
    if (it==s_gid.end())
      return invalid_gang_id;

    // gang id indexed with the key found
    auto& q = it->second;
    STAPL_RUNTIME_ASSERT(!q.empty());

    // remove entry from the queue and remove the queue if empty
    const gang_md::id gid = q.front();
    q.pop();
    if (q.empty())
      s_gid.erase(it);
    return gid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Releases locations waiting on a gang id.
  //////////////////////////////////////////////////////////////////////
  static void release_id(topology::container_type const& r,
                         key_type const& key, const gang_md::id gid)
  {
    rpc(&add, r, key, gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the gang metadata associated with the key, otherwise
  ///        @c nullptr.
  //////////////////////////////////////////////////////////////////////
  static gang_md* try_get_metadata(shmem_key_type const& key)
  {
    std::unique_lock<std::mutex> lock{s_gang_mtx, std::try_to_lock};

    if (!lock.owns_lock())
      return nullptr;

    auto it = s_gang.find(key);
    if (it==s_gang.end())
      return nullptr;

    // gang metadata indexed with the key found
    auto& q = it->second;
    STAPL_RUNTIME_ASSERT(!q.empty());

    // remove entry from the queue if ref count=0 and remove the queue if empty
    gang_md* const g = q.front().first;
    if ( (--q.front().second)==0 ) {
      q.pop();
      if (q.empty())
        s_gang.erase(it);
    }
    return g;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Release locally managed locations waiting on gang metadata.
  //////////////////////////////////////////////////////////////////////
  static void release_metadata(shmem_key_type const& key,
                               std::pair<gang_md*, size_type> const& v)
  {
    STAPL_RUNTIME_ASSERT(v.second>0);
    std::lock_guard<std::mutex> lock{s_gang_mtx};
    auto& q = s_gang[key];
    q.push(v);
  }
};
distributor::id_map_type distributor::s_gid;
std::mutex               distributor::s_gid_mtx;
distributor::md_map_type distributor::s_gang;
std::mutex               distributor::s_gang_mtx;

} // namespace runtime


using namespace runtime;


// Creates a new gang from an existing without a hint id
location_md& gang::create(const gang_md::id parent_gid,
                          const location_md::id root,
                          gang_description ngd,
                          const location_md::id nlid,
                          comparable_proxy p)
{
  const size_type local_index = ngd.get_location_local_index(nlid);

  const distributor::key_type key{parent_gid, root, std::move(p)};
  const bool leader = (local_index==0);
  gang_md* ng       = nullptr;
  if (nlid==0) {
    // root of the gang; creates the gang id and propagates it to local leaders
    STAPL_RUNTIME_ASSERT(leader);
    ng = new gang_md{parent_gid, std::move(ngd)};

    // distribute id to remote leaders
    auto const& leaders = ng->get_topology().children();
    if (!leaders.empty())
      distributor::release_id(leaders, key, ng->get_id());

    // distribute metadata to locally managed locations
    const size_type nmanaged = (ng->local_size() - 1);
    if (nmanaged>0) {
      const distributor::shmem_key_type skey{key, ng->size()};
      distributor::release_metadata(skey, std::make_pair(ng, nmanaged));
    }
  }
  else if (leader) {
    // local leader; waits for the root to give the id
    ng = new gang_md{deferred, parent_gid, std::move(ngd)};
    gang_md::id ngid = invalid_gang_id;
    yield_until(no_context,
                [&]
                {
                  ngid = distributor::try_get_id(key);
                  return (ngid != invalid_gang_id);
                });
    ng->set_id(ngid);

    // distribute id to remote leaders
    auto const& leaders = ng->get_topology().children();
    if (!leaders.empty())
      distributor::release_id(leaders, key, ngid);

    // distribute metadata to locally managed locations
    const size_type nmanaged = (ng->local_size() - 1);
    if (nmanaged>0) {
      const distributor::shmem_key_type skey{key, ng->size()};
      distributor::release_metadata(skey, std::make_pair(ng, nmanaged));
    }
  }
  else {
    // child; waits for the local leader to give the gang metadata
    const distributor::shmem_key_type skey{key, ngd.get_num_locations()};
    yield_until(no_context,
                [&]
                {
                  ng = distributor::try_get_metadata(skey);
                  return (ng != nullptr);
                });
  }
  location_md* const nl = new location_md{nlid, local_index, *ng};
  return *nl;
}


// Creates a new gang from an existing with a hint id
location_md& gang::create(const std::size_t id,
                          const gang_md::id parent_gid,
                          const location_md::id root,
                          gang_description ngd,
                          const location_md::id nlid,
                          comparable_proxy p)
{
  const size_type local_index = ngd.get_location_local_index(nlid);

  // generate id
  const gang_md::id ngid = id; // FIXME works for directly set correct ids

  const distributor::key_type key{parent_gid, root, std::move(p)};
  const bool leader = (local_index==0);
  gang_md* ng       = nullptr;
  if (leader) {
    // local leader; id is known
    ng = new gang_md{ngid, parent_gid, std::move(ngd)};

    // distribute metadata to locally managed locations
    const size_type nmanaged = (ng->local_size() - 1);
    if (nmanaged>0) {
      const distributor::shmem_key_type skey{key, ng->size()};
      distributor::release_metadata(skey, std::make_pair(ng, nmanaged));
    }
  }
  else {
    // child; waits for the local leader to give the gang metadata
    const distributor::shmem_key_type skey{key, ngd.get_num_locations()};
    yield_until(no_context,
                [&]
                {
                  ng = distributor::try_get_metadata(skey);
                  return (ng != nullptr);
                });
  }
  location_md* const nl = new location_md{nlid, local_index, *ng};
  return *nl;
}

} // namespace stapl
