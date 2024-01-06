/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/new.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/system.hpp>
#include <stapl/runtime/concurrency/thread_local_storage.hpp>
#include <stapl/runtime/non_rmi/abort.hpp>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <boost/pool/pool.hpp>

namespace stapl {

namespace runtime {

void free_memory(void) throw (std::bad_alloc);


/// @c std::new_handler before @ref memory_allocator::initialize().
static std::new_handler old_new_handler = nullptr;

/// @c true if debugging information for pool allocations enabled.
static bool pool_debug = false;

/// @c true if run-time statistics for pool allocations enabled.
static bool count_allocs = false;


//////////////////////////////////////////////////////////////////////
/// @brief Thread local memory chunk pools.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class tl_pools
{
private:
  using size_type = std::size_t;

  //////////////////////////////////////////////////////////////////////
  /// @brief Allocation information.
  //////////////////////////////////////////////////////////////////////
  struct alloc_info
  {
    size_type mallocs;
    size_type frees;
    size_type high_watermark;

    constexpr alloc_info(void) noexcept
    : mallocs(0),
      frees(0),
      high_watermark(0)
    { }

    void malloced(void) noexcept
    {
      ++mallocs;
      const size_type diff = (mallocs - frees);
      if (diff > high_watermark)
        high_watermark = diff;
    }

    void freed(void) noexcept
    { ++frees; }
  };

  using pool_map  = std::unordered_map<std::size_t, boost::pool<>>;
  using info_map  = std::unordered_map<std::size_t, alloc_info>;
  using chunk_map = std::unordered_map<void*, std::size_t>;

  pool_map  m_pools;
  info_map  m_pools_info;
  chunk_map m_chunks;

public:
  tl_pools(void) = default;

  tl_pools(tl_pools const&) = delete;
  tl_pools& operator=(tl_pools const&) = delete;

  ~tl_pools(void)
  { clear(); }

  void clear(void)
  {
    // report memory leaks
    if (!m_chunks.empty()) {
      std::ostringstream oss;
      oss << "Memory leaked:\n";
      for (auto const& t : m_chunks) {
        oss << "\t(addr: " << t.first << ", size: " << t.second << ")\n";
      }
      abort(oss.str());
    }

    // report memory usage
    if (!m_pools_info.empty()) {
      std::ostringstream oss;
      oss << '[' << getpid() << ',' << std::this_thread::get_id() << "]:\n";
      for (auto const& t : m_pools_info) {
        oss << "\tpool(" << t.first << ") (mallocs/frees/max allocs): "
            << t.second.mallocs << ' '
            << t.second.frees << ' '
            << t.second.high_watermark << std::endl;
      }
      std::cout << oss.str();
      m_pools_info.clear();
    }

    m_pools.clear();
  }

  void* malloc(const std::size_t s)
  {
    // real size; required size plus space for pointer to pool
    const std::size_t size = (s + sizeof(boost::pool<>*));

    // find pool
    auto it = m_pools.find(size);
    if (it==m_pools.end()) {
      auto r = m_pools.emplace(std::piecewise_construct,
                               std::forward_as_tuple(size),
                               std::forward_as_tuple(size));
      STAPL_RUNTIME_ASSERT(r.second);
      it = r.first;
    }
    auto& pool = it->second;

    // allocate
    for (;;) {
      void* const p = pool.malloc();
      if (p) {
        if (pool_debug) {
          if (!m_chunks.emplace(p, size).second)
            STAPL_RUNTIME_ERROR("Memory already allocated");
        }
        if (count_allocs) {
          m_pools_info[s].malloced();
        }
        // save pointer to pool at the end of the allocated space
        auto* pool_ptr = &pool;
        std::memcpy((static_cast<char*>(p) + s), &pool_ptr, sizeof(pool_ptr));
        return p;
      }
      free_memory();
    }
  }

  void free(void* const p, const std::size_t s)
  {
    if (!p)
      return;

    // real size; required size plus space for pointer to pool
    const std::size_t size = (s + sizeof(boost::pool<>*));

    // retrieve pointer to pool from the end of the allocated space
    boost::pool<>* pool_ptr = nullptr;
    std::memcpy(&pool_ptr, (static_cast<char*>(p) + s), sizeof(pool_ptr));

    STAPL_RUNTIME_ASSERT((m_pools.count(size)==1) &&
                         (&(m_pools.find(size)->second)==pool_ptr));

    STAPL_RUNTIME_ASSERT_MSG(pool_ptr->is_from(p),
                             "Memory not allocated from this pool");

    // deallocate
    if (pool_debug) {
      if (m_chunks.erase(p)!=1)
        STAPL_RUNTIME_ERROR("Memory double freed or not allocated from this "
                            "pool");
      std::memset(p, 0xDEADBEEF, size);
    }
    if (count_allocs) {
      m_pools_info[s].freed();
    }
    pool_ptr->free(p);
  }

  bool release_memory(void)
  {
    for (auto& t : m_pools) {
      if (t.second.release_memory())
        return true; // some memory released successfully
    }
    return false;
  }
};


/// Thread-local pools.
static STAPL_RUNTIME_THREAD_LOCAL(tl_pools, pools)


// Frees memory
void free_memory(void) throw (std::bad_alloc)
{
  // ::operator new() is typically implemented as in TCPL
  // for (;;) {
  //   if (p=malloc()) return p;
  //   if (new_handler==0) throw std::bad_alloc();
  //   new_handler();
  // }

  // release memory managed by the runtime only for the calling thread
  auto& my_pools = pools.get();
  if (my_pools.release_memory())
    return;

  // could not release any memory
  // call old_new_handler, it will throw std::bad_alloc if no memory is freed
  if (!old_new_handler)
    throw std::bad_alloc();
  old_new_handler();
}


// Initializes the memory_allocator. Replaces the new_handler and initializes
// the heap collecting information if needed.
void memory_allocator::initialize(option const& opts)
{
  old_new_handler = std::set_new_handler(free_memory);
  count_allocs = (opts.get<int>("STAPL_ENABLE_MANAGED_ALLOC_STATS", 0)!=0);
  pool_debug =
    (opts.get<int>("STAPL_ENABLE_MANAGED_ALLOC_DEBUG", get_debug_level())!=0);
}


// Finalizes the memory_allocator
void memory_allocator::finalize(void)
{
  std::set_new_handler(old_new_handler);
  old_new_handler = nullptr;
}


// Allocates the requested memory chunk
void* memory_allocator::allocate(std::size_t size) throw(std::bad_alloc)
{
  return pools.get().malloc(size);
}


// Frees the given chunk
void memory_allocator::deallocate(void* p, std::size_t size)
{
  pools.get().free(p, size);
}

} // namespace runtime

} // namespace stapl
