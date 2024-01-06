/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_BLOCK_REGISTRY_HPP
#define STAPL_RUNTIME_UTILITY_BLOCK_REGISTRY_HPP

#include "../config.hpp"
#include "../exception.hpp"
#include "../concurrency/queue.hpp"
#include "../concurrency/mutex.hpp"
#include <atomic>
#include <limits>
#include <mutex>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Implements a registry that allows registration of ranges (blocks)
///        of keys to be used by an entity that has a unique id.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Mapped>
class block_registry
{
public:
  typedef Key                                      key_type;
  typedef Mapped                                   value_type;
  typedef std::size_t                              size_type;
private:
  typedef std::unordered_map<key_type, value_type> registry_type;

  static_assert(std::is_integral<Key>::value, "Only works for integral types.");

  /// Local id.
  size_type                m_id;
  /// Number of ids.
  size_type                m_num;
  /// Block size.
  size_type                m_block_size;
  /// Block offset.
  size_type                m_block_offset;
  /// Registry.
  registry_type            m_registry;
  /// Queue of unused keys.
  queue<key_type>          m_unused;
  /// Current index.
  std::atomic<size_type>   m_cur;
  mutable read_write_mutex m_mtx;

public:
  block_registry(void)
  : m_id(0),
    m_num(0),
    m_block_size(0),
    m_block_offset(0),
    m_cur(0)
  { }

  block_registry(const size_type id, const size_type num_ids)
  : m_id(id),
    m_num(num_ids),
    m_block_size(std::numeric_limits<key_type>::max()/num_ids),
    m_block_offset(id * m_block_size),
    m_cur(0)
  { }

  block_registry& operator=(block_registry&& other)
  {
    std::lock_guard<read_write_mutex> lock{m_mtx};
    m_id           = std::move(other.m_id);
    m_num          = std::move(other.m_num);
    m_block_size   = std::move(other.m_block_size);
    m_block_offset = std::move(other.m_block_offset);
    m_registry     = std::move(other.m_registry);
    m_unused       = std::move(other.m_unused);
    m_cur          = size_type(other.m_cur);
    return *this;
  }

  bool empty(void) const
  {
    read_lock_guard<read_write_mutex> lock{m_mtx};
    return m_registry.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the id that the key belongs to.
  //////////////////////////////////////////////////////////////////////
  size_type find_id(const key_type key) const noexcept
  {
    return (key/m_block_size);
  }

  key_type reserve_id(void)
  {
    key_type key{};
    if (!m_unused.try_pop(key)) {
      // generate new key
      const size_type cur = ++m_cur;
      if (cur>=m_block_size)
        STAPL_RUNTIME_ERROR("Gang IDs depleted.");
      key = key_type(cur + m_block_offset);
    }
    return key;
  }

  void insert(const key_type key, value_type const& value)
  {
    std::lock_guard<read_write_mutex> lock{m_mtx};
    m_registry.emplace(key, value);
  }

  void erase(const key_type key)
  {
    {
      std::lock_guard<read_write_mutex> lock{m_mtx};
      STAPL_RUNTIME_ASSERT(m_registry.find(key)!=m_registry.end());
      m_registry.erase(key);
    }

    if (m_id==find_id(key)) {
      m_unused.push(key);
    }
  }

  value_type const& retrieve(const key_type key) const
  {
    read_lock_guard<read_write_mutex> lock{m_mtx};
    auto it = m_registry.find(key);
    STAPL_RUNTIME_ASSERT(it!=m_registry.end());
    return it->second;
  }

  bool try_retrieve(const key_type key, value_type& value) const
  {
    read_lock_guard<read_write_mutex> lock{m_mtx};
    auto it = m_registry.find(key);
    if (it==m_registry.end())
      return false;
    value = it->second;
    return true;
  }
};

} // namespace runtime

} // namespace stapl

#endif
