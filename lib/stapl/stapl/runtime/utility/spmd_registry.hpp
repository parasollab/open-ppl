/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_SPMD_REGISTRY_HPP
#define STAPL_RUNTIME_UTILITY_SPMD_REGISTRY_HPP

#include "../config.hpp"
#include "logical_clock.hpp"
#include "../exception.hpp"
#include <algorithm>
#include <memory>
#include <vector>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Registry for SPMD-registered objects.
///
/// The @ref spmd_registry is an array of @c void* that point to the registered
/// objects. Handles are indices into the array, making translation an
/// \f$O(1)\f$ time operation.
///
/// The distributed object has to be registered in the same order in all
/// locations of the gang, so that the handles will be created in the same order
/// in each location.
///
/// Unregistration can happen in any order, having the drawback that all
/// unregistrations have to be committed in the same point in the SPMD section,
/// e.g., during an @ref rmi_fence() call.
///
/// Although the registry will grow as needed, it is more efficient to allocate
/// it beforehand if a known, constant number of objects will need to be
/// registered. The size may be defined at compile time through the
/// @c STAPL_RUNTIME_SPMD_REGISTRY_SIZE macro (e.g.,
/// @c -DSTAPL_RUNTIME_SPMD_REGISTRY_SIZE=20).
///
/// @ingroup runtimeUtility
///
/// @todo If the registry runs out of elements, a new one has to be allocated
///       with double the space and memcpy all the handles. A more efficient
///       efficient approach could be to use a trick like @c std::deque, which
///       is a list of vectors.
//////////////////////////////////////////////////////////////////////
class spmd_registry
{
public:
  using handle_type         = object_id;
  using epoch_type          = logical_clock::time_type;
  using size_type           = std::size_t;
private:
  using reorder_buffer_type = std::vector<handle_type>;

  static_assert(sizeof(size_type)>=sizeof(handle_type),
                "Insufficient space for storing handles.");

  //////////////////////////////////////////////////////////////////////
  /// @brief Stores a pointer to an object and its registration epoch.
  ///
  /// If the epoch is invalid, then it stores a pointer to the next available
  /// entry.
  //////////////////////////////////////////////////////////////////////
  class data_type
  {
  private:
    union
    {
      void*     m_data;
      size_type m_index;
    };
    epoch_type  m_epoch;

  public:
    constexpr data_type(const size_type index = 0) noexcept
    : m_index(index),
      m_epoch(logical_clock::no_time)
    { }

    bool valid(void) const noexcept
    { return (m_epoch!=logical_clock::no_time); }

    void set_next(const size_type i) noexcept
    {
      STAPL_RUNTIME_ASSERT(!valid());
      m_index = i;
    }

    void set_data(void* const p, const epoch_type e) noexcept
    {
      STAPL_RUNTIME_ASSERT(!valid());
      m_data  = p;
      m_epoch = e;
    }

    size_type get_next(void) const noexcept
    {
      STAPL_RUNTIME_ASSERT(!valid());
      return m_index;
    }

    void* get_data(void) const noexcept
    {
      STAPL_RUNTIME_ASSERT(valid());
      return m_data;
    }

    epoch_type get_epoch(void) const noexcept
    { return m_epoch; }
  };

  std::unique_ptr<data_type[]> m_registry;
  size_type                    m_capacity;
  size_type                    m_index;
  size_type                    m_size;
  /// Unregistration reorder buffer.
  reorder_buffer_type          m_urb;

  void init(const size_type index) noexcept
  {
    for (auto i = index; i < (m_capacity-1); ++i) {
      m_registry[i].set_next(i+1);
    }
    m_registry[m_capacity-1].set_next(0);
  }

public:
  static const size_type default_size = STAPL_RUNTIME_SPMD_REGISTRY_SIZE;

  spmd_registry(void) noexcept
  : m_capacity(0),
    m_index(0),
    m_size(0)
  { }

  bool empty(void) const noexcept
  { return (m_size==0); }

  size_type size(void) const noexcept
  { return m_size; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a handle associated with @p p in epoch @p e.
  //////////////////////////////////////////////////////////////////////
  handle_type register_object(void* const p, const epoch_type e)
  {
    if (m_index==0) {
      if (!m_registry) {
        // first initialization - create registry
        m_capacity = default_size;
        m_registry = std::unique_ptr<data_type[]>(new data_type[m_capacity]);
        m_index    = 1;
        m_registry[0].set_next(0);
      }
      else {
        // registry expansion
        m_index     = m_capacity;
        m_capacity *= 2;
        std::unique_ptr<data_type[]> nregistry{new data_type[m_capacity]};
        std::copy(&m_registry[0], &m_registry[0] + m_index, &nregistry[0]);
        m_registry = std::move(nregistry);
      }
      init(m_index);
    }

    ++m_size;

    // create handle for object
    const handle_type h = m_index;
    m_index             = m_registry[h].get_next();
    m_registry[h].set_data(p, e);
    return h;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unregisters the object associated with handle @p h.
  ///
  /// If @p immediate is @c false, the handle remains unavailable until the next
  /// call to @ref commit_unregistrations().
  //////////////////////////////////////////////////////////////////////
  void unregister_object(const handle_type h, const bool immediate = false)
  {
    STAPL_RUNTIME_ASSERT_MSG(h!=invalid_object_id, "Invalid handle.");
    STAPL_RUNTIME_ASSERT_MSG((h<m_capacity) && m_registry[h].valid(),
                             "p_object is not registered.");
    if (immediate) {
      m_registry[h] = data_type{m_index};
      m_index       = h;
    }
    else {
      m_registry[h] = data_type{};
      m_urb.push_back(h);
    }
    --m_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of unregistrations that will be committed
  /// on the location during the next invocation of @ref rmi_fence.
  //////////////////////////////////////////////////////////////////////
  unsigned int num_pending_unregistrations(void) const
  { return m_urb.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Makes all unregistered handles available.
  //////////////////////////////////////////////////////////////////////
  void commit_unregistrations(void)
  {
    if (m_urb.empty())
      return;
    std::sort(m_urb.begin(), m_urb.end());
    for (auto const& h : m_urb) {
      m_registry[h].set_next(m_index);
      m_index = h;
    }
    m_urb.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer to the object associated with handle @p h,
  ///        registered in epoch @p e.
  ///
  /// If @p e is @ref logical_clock::no_time, then the registration epoch of
  /// @p h is not checked.
  //////////////////////////////////////////////////////////////////////
  void* get_object(const handle_type h, const epoch_type e) const noexcept
  {
    STAPL_RUNTIME_ASSERT_MSG(h!=invalid_object_id, "Invalid handle.");
    if (h>=m_capacity)
      return nullptr;
    auto& t = m_registry[h];
    if (!t.valid())
      return nullptr;
    if (e!=logical_clock::no_time && (t.get_epoch()!=e))
      return nullptr;
    return t.get_data();
  }
};

} // namespace runtime

} // namespace stapl

#endif
