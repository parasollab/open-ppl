/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_POOL_HPP
#define STAPL_RUNTIME_UTILITY_POOL_HPP

#include "../exception.hpp"
#include "../concurrency/queue.hpp"
#include <deque>
#include <memory>
#include <mutex>
#include <new>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Pool of memory chunks of a specific size.
///
/// @ingroup runtimeUtility
///
/// @todo A function that forces the pool to shrink is required.
//////////////////////////////////////////////////////////////////////
class pool
{
public:
  typedef std::size_t size_type;
  typedef void*       value_type;

private:
  /// Chunk size.
  size_type                            m_chunk_size;
  /// Number of chunks in one allocation chunk.
  size_type                            m_next_size;
  /// Maximum number of allocated chunks.
  size_type                            m_max_size;
  /// Allocated chunks.
  std::vector<std::unique_ptr<char[]>> m_alloc;
  /// Available chunks.
  queue<void*>                         m_free;
  mutable std::mutex                   m_mtx;

public:
  pool(void)
  : m_chunk_size(0),
    m_next_size(0),
    m_max_size(0)
  { }

  pool(pool const&) = delete;
  pool& operator=(pool const&) = delete;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of allocated chunks.
  //////////////////////////////////////////////////////////////////////
  size_type allocated_chunks(void) const noexcept
  { return (m_alloc.size() * m_next_size); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Attempts to allocate more chunks and returns @c true if the
  ///        allocation was successful.
  //////////////////////////////////////////////////////////////////////
  bool allocate_chunks(void)
  {
    if ((allocated_chunks() + m_next_size) > m_max_size)
      return false; // exceeded maximum size
    char* p = ::new(std::nothrow) char[m_chunk_size * m_next_size];
    if (!p)
      return false; // could not allocate
    m_alloc.emplace_back(p);
    for (size_type i = 0; i < m_next_size; ++i) {
      m_free.push(&p[i * m_chunk_size]);
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Releases all the chunks.
  ///
  /// @warning It will assert if there are chunks still in use.
  //////////////////////////////////////////////////////////////////////
  void release_chunks(void)
  {
    if (m_alloc.empty())
      return;
    if (allocated_chunks()!=m_free.size())
      STAPL_RUNTIME_ERROR("Objects are still in use");
    m_alloc.clear();
    m_free.clear();
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Resets this pool, releasing all allocated chunks.
  ///
  /// After this function is called, the pool cannot be used.
  //////////////////////////////////////////////////////////////////////
  void reset(void)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    release_chunks();
    m_chunk_size = 0;
    m_next_size  = 0;
    m_max_size   = 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resets this pool, releasing any memory.
  ///
  /// It initializes it to return chunks of @p chunk_size. A number of
  /// @p next_size chunks are allocated when the pool is empty, up to
  /// @p max_size.
  //////////////////////////////////////////////////////////////////////
  void reset(size_type chunk_size, size_type next_size, size_type max_size)
  {
    STAPL_RUNTIME_ASSERT(chunk_size>0 && next_size>0 && max_size>0);
    std::lock_guard<std::mutex> lock{m_mtx};
    release_chunks();
    m_chunk_size = chunk_size;
    m_next_size  = next_size;
    m_max_size   = max_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Purges the pool, releasing all chunks.
  ///
  /// @warning It will assert if there are chunks still in use.
  //////////////////////////////////////////////////////////////////////
  void purge(void)
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    release_chunks();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an available chunk or @c nullptr if there are none.
  //////////////////////////////////////////////////////////////////////
  void* allocate(void)
  {
    void* p = nullptr;

    // try to grab something available
    if (m_free.try_pop(p))
      return p;

    // nothing is available, try to allocate
    {
      std::unique_lock<std::mutex> lock{m_mtx, std::try_to_lock};

      if (!lock.owns_lock())
        return nullptr; // other thread has the lock

      if (!allocate_chunks())
        return nullptr; // couldn't allocate more
    }

    m_free.try_pop(p); // try to grab from the pool
    return p;          // might return nullptr
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Releases the given object back to the pool.
  //////////////////////////////////////////////////////////////////////
  void release(void* p)
  {
    m_free.push(p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if this @ref pool is empty.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return (size()==0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of allocated chunks.
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    std::lock_guard<std::mutex> lock{m_mtx};
    return allocated_chunks();
  }
};

} // namespace runtime

} // namespace stapl

#endif

