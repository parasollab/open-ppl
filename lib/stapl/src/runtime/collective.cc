/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime/communicator/collective.hpp>
#include <stapl/runtime/gang_md.hpp>
#include <stapl/runtime/gang_md_registry.hpp>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <boost/functional/hash.hpp>

namespace stapl {

namespace runtime {

/// @ref collective object registry.
static std::unordered_map<
         collective::id, collective, boost::hash<collective::id>
       > collective_registry;

/// @ref collective object registry mutex.
static std::mutex collective_registry_mtx;


// Creates a new collective handle or returns an already existing one
collective& collective::get(collective::id const& id, topology const& t)
{
  std::lock_guard<std::mutex> lock{collective_registry_mtx};
  return collective_registry.emplace(
           std::piecewise_construct,
           std::forward_as_tuple(std::cref(id)),
           std::forward_as_tuple(std::cref(id), std::cref(t))).first->second;
}


// Creates a new collective handle or returns an already existing one
collective& collective::get(collective::id const& id)
{
  std::lock_guard<std::mutex> lock{collective_registry_mtx};
  auto it = collective_registry.find(id);
  if (it!=collective_registry.end())
    return it->second;

  // collective does not exist, create a new one
  auto const& t = gang_md_registry::get(id.first).get_topology();
  return collective_registry.emplace(
           std::piecewise_construct,
           std::forward_as_tuple(std::cref(id)),
           std::forward_as_tuple(std::cref(id), std::cref(t))).first->second;
}


// Attempts to destroy the handle
void collective::try_destroy(void) noexcept
{
  std::lock_guard<std::mutex> lock1{collective_registry_mtx};
  STAPL_RUNTIME_ASSERT(collective_registry.count(m_id)!=0);

  {
    std::lock_guard<std::mutex> lock2{m_mtx};
    STAPL_RUNTIME_ASSERT(!bool(m_notify_arrived));
    // if not empty, a new collective started with the same id
    if (!m_arrived.empty())
      return;
  }

  // remove the object from the registry
  collective_registry.erase(m_id);
}

} // namespace runtime

} // namespace stapl
