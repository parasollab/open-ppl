/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/gang_md_registry.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/gang_md.hpp>
#include <stapl/runtime/yield.hpp>
#include <stapl/runtime/non_rmi/rpc.hpp>
#include <stapl/runtime/utility/block_registry.hpp>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <boost/serialization/unordered_map.hpp>

namespace stapl {

namespace runtime {

/// Global gang metadata (gang with id @c 0)
static std::atomic<gang_md*> global_gang_md{nullptr};

/// Gang metadata registry.
static block_registry<gang_md::id, gang_md*> registry;

/// Fence metadata registry.
static std::unordered_map<gang_md::id, fence_md*> fence_md_registry;

/// Fence metadata registry mutex.
static std::mutex fence_md_registry_mtx;


//////////////////////////////////////////////////////////////////////
/// @brief Releases the @ref gang_md object associated with @p gid.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void release_gang_md(const gang_id gid)
{
  STAPL_RUNTIME_ASSERT(gid!=invalid_gang_id);
  gang_md& g = gang_md_registry::get(gid);
  g.release();
}


// Initializes the registry
void gang_md_registry::initialize(const process_id pid,
                                  const unsigned int npids)
{
  STAPL_RUNTIME_ASSERT(!global_gang_md);
  registry = block_registry<gang_md::id, gang_md*>(pid, npids);
}


// Finalizes the registry
void gang_md_registry::finalize(void)
{
  yield_until(no_context, [] { return registry.empty(); });

  if (global_gang_md || !registry.empty())
    STAPL_RUNTIME_ERROR("Gang metadata still present during finalization.");
  if (!fence_md_registry.empty())
    STAPL_RUNTIME_ERROR("Fence metadata still present during finalization.");
}


// Returns the process that owns the id
process_id gang_md_registry::id_owner(const gang_id gid) noexcept
{
  STAPL_RUNTIME_ASSERT(gid!=invalid_gang_id);
  if (gid==0)
    return 0; // process 0 is the owner of gang id 0
  return process_id(registry.find_id(gid));
}


// Reserves and returns an id
gang_id gang_md_registry::reserve_id(void)
{
  const auto gid = registry.reserve_id();
  if (gid==invalid_gang_id)
    STAPL_RUNTIME_ERROR("Gang IDs depleted.");
  return gid;
}


// Registers the gang with the given id
void gang_md_registry::register_gang_md(gang_md& g, const gang_id gid)
{
  STAPL_RUNTIME_ASSERT(gid!=invalid_gang_id);
  if (gid==0) {
    STAPL_RUNTIME_ASSERT(!global_gang_md);
    global_gang_md = &g;
  }
  else {
    registry.insert(gid, &g);
  }
}


// Unregisters the gang
void gang_md_registry::unregister_gang_md(gang_md const& g)
{
  const auto gid = g.get_id();
  STAPL_RUNTIME_ASSERT(gid!=invalid_gang_id);
  if (gid==0) {
    global_gang_md = nullptr;
  }
  else {
    registry.erase(gid);
    topology const& t = g.get_topology();
    if (!t.is_root()) {
      // send gang_md deletion request to my parent in the topology
      rpc(&release_gang_md, t.parent_id(), gid);
    }
  }
}


// Returns the gang associated with the id
gang_md& gang_md_registry::get(const gang_id gid)
{
  gang_md* const g = try_get(gid);
  if (!g)
    STAPL_RUNTIME_ERROR("Gang metadata expected but not found.");
  return *g;
}


// Tries to return the gang associated with the id
gang_md* gang_md_registry::try_get(const gang_id gid)
{
  STAPL_RUNTIME_ASSERT(gid!=invalid_gang_id);
  if (gid==0)
    return global_gang_md.load(std::memory_order_relaxed);
  gang_md* g = nullptr;
  registry.try_retrieve(gid, g);
  return g;
}


// Returns the fence metadata for gang gid, constructing it if needed
boost::intrusive_ptr<fence_md>
gang_md_registry::get_fence_md(const gang_md::id gid)
{
  std::lock_guard<std::mutex> lock{fence_md_registry_mtx};
  auto it = fence_md_registry.find(gid);
  if (it!=fence_md_registry.end())
    return it->second;
  auto* p = new fence_md{gid};
  fence_md_registry.emplace(gid, p);
  return p;
}


// Deletes the fence metadata
void gang_md_registry::erase_fence_md(fence_md* const p)
{
  STAPL_RUNTIME_ASSERT(!p->none_pending());

  const auto gid   = p->get_gang_id();
  auto* const g    = try_get(gid);
  const auto owner = id_owner(gid);

  if (!g && (owner==runqueue::get_process_id()))
    STAPL_RUNTIME_ERROR("Outstanding coherence traffic for destroyed gang "
                        "detected.");

  // remove from the registry
  {
    std::lock_guard<std::mutex> lock{fence_md_registry_mtx};
    if (p->use_count()>0)
      return; // ref count increased since deletion
    fence_md_registry.erase(gid);
  }

  // update the gang_md metadata
  if (g) {
    g->get_fence_md().update(p->retrieve());
  }
  else {
    rpc(&gang_md::update_fence_md, owner, gid, p->retrieve());
  }

  delete p;
}

} // namespace runtime

} // namespace stapl
