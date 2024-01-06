/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/config.hpp>

#ifdef STAPL_ENABLE_HEAP_TRACKING
# include <stapl/runtime/heap_tracker.hpp>
# include <stapl/runtime/exception.hpp>
# include <stapl/runtime/utility/malloc_allocator.hpp>
# include <cstdio>
# include <cstdlib>
# include <cstring>
# include <mutex>
# include <set>
# if defined(__GLIBC__)
// we can use backtrace()
#  include <execinfo.h>
# endif

namespace stapl {

namespace runtime {

using std::fprintf;

/// Allocation type.
enum alloc_type
{
  /// Unknown allocation.
  ALLOC_UNKNOWN = 0x0,
  /// Object allocation.
  ALLOC_OBJECT = 0x1,
  /// Array allocation.
  ALLOC_ARRAY = 0x2
};

////////////////////////////////////////////////////////////////////
/// @brief Keeps information about a specific allocation.
///
/// This information includes the type of the allocation
///
/// @ingroup runtimeUtility
////////////////////////////////////////////////////////////////////
struct alloc_t
{
  std::size_t size;
  alloc_type  type;
#if defined(__GLIBC__) || defined(__GNUC__)
  void*       addr;
#endif

  constexpr alloc_t(void) noexcept
  : size(0),
    type(ALLOC_UNKNOWN)
#if defined(__GLIBC__) || defined(__GNUC__)
    , addr(0)
#endif
  { }

  ////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref alloc_t object, attempting to find the address
  ///        where the allocation happened from.
  ////////////////////////////////////////////////////////////////////
  alloc_t(const std::size_t s, const alloc_type t) noexcept
  : size(s),
    type(t)
#if defined(__GLIBC__)
    , addr(0)
#elif defined(__GNUC__)
    , addr(__builtin_return_address(2))
#endif
  {
#if defined(__GLIBC__)
    void* buf[4];
    backtrace(buf, 4);
    addr = buf[3];
#endif
  }

  void print(void* p, FILE* os) const noexcept
  {
    using int_type = long unsigned;

    const char* t    = (type==ALLOC_OBJECT ? "Object" : "Array ");
    const int_type s = int_type(size);
#if defined(__GLIBC__)
    char** fun = backtrace_symbols(&addr, 1);
    if (fun==NULL) {
      fprintf(os, "\t%s @ %p  From: %p Size (bytes): %lu\n", t, p, addr, s);
    }
    else {
      fprintf(os, "\t%s @ %p  From: %s Size (bytes): %lu\n", t, p, fun[0], s);
      std::free(fun);
    }
#elif defined(__GNUC__)
    fprintf(os, "\t%s @ %p  From: %p Size (bytes): %lu\n", t, p, addr, s);
#else
    fprintf(os, "\t%s @ %p  Size (bytes): %lu\n", t, p, s);
#endif
  }
};

static_assert((sizeof(alloc_t)%sizeof(void*))==0, "Incorrect alignment.");


////////////////////////////////////////////////////////////////////
/// @brief Keeps information about heap allocations and deallocations.
///
/// This information includes the maximum allocated memory (high watermark),
/// the number of total allocations and any still active allocations.
///
/// @ingroup runtimeUtility
////////////////////////////////////////////////////////////////////
class heap_tracking
{
private:
  using set_type = std::set<void*, std::less<void*>, malloc_allocator<void*>>;

  FILE*       m_os;
  /// Currently allocated memory.
  std::size_t m_size;
  /// Maximum allocated memory.
  std::size_t m_maxsize;
  /// Number of allocated objects.
  int         m_alloc;
  /// Allocated objects.
  set_type    m_objs;
  std::mutex  m_mtx;

  heap_tracking(void)
  : m_os(NULL),
    m_size(0),
    m_maxsize(0),
    m_alloc(0)
  {
    // find the prefix of the filename
    const char* prefix = std::getenv("STAPL_HEAP_TRACKING_FILE");
    std::size_t prefix_s = 0;
    if (prefix!=NULL) {
      prefix_s = std::strlen(prefix);
    }
    else {
      const char default_prefix[] = "stapl_heap_tracking.";
      prefix   = default_prefix;
      prefix_s = (sizeof(default_prefix)/sizeof(char) - 1);
    }

    // create the filename
    std::size_t fname_s = prefix_s;
    const int pid = getpid();
    for (int i=int(pid); i>0; i/=10, ++fname_s); // find pid size
    char* fname = static_cast<char*>(std::malloc((fname_s + 1) * sizeof(char)));
    if (fname==NULL)
      throw std::bad_alloc();
    std::memcpy(fname, prefix, prefix_s);
    const int n = std::sprintf(fname + prefix_s, "%d", int(pid));
    if (n!=int(fname_s-prefix_s)) {
      std::free(fname);
      throw std::bad_alloc();
    }

    // open the file to dump output to
    m_os = std::fopen(fname, "w");
    std::free(fname);
    if (m_os==NULL)
      throw runtime_error("Failed to open file for heap profiling");
    fprintf(m_os, "# STAPL Heap Debugging and Profiling for pid %d\n", pid);
  }

  heap_tracking(heap_tracking const&) = delete;
  heap_tracking& operator=(heap_tracking const&) = delete;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns an instance of the @ref heap_tracking class.
  ////////////////////////////////////////////////////////////////////
  static heap_tracking& instance(void)
  {
    static heap_tracking h;
    return h;
  }

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Finalizes heap tracking, outputting  all information to the file.
  ///
  /// The files are always named as @c stapl_heap_tracking.#pid where @p pid
  /// is the process id, unless the environment variable
  /// @c STAPL_HEAP_TRACKING_FILE is available with the required file name.
  ////////////////////////////////////////////////////////////////////
  static void finalize(void)
  {
    heap_tracking& h = instance();

    // print usage statistics
    fprintf(h.m_os,
            "Maximum Memory Used:   %.2f Kbytes\n"
            "Number of allocations: %d\n",
            float(h.m_maxsize)/1024.0F, h.m_alloc);

    // print any leaked objects
    std::lock_guard<std::mutex> lock(h.m_mtx);
    const unsigned long nalloc = h.m_objs.size();
    if (nalloc!=0) {
      fprintf(h.m_os,
              "Warning: Memory Leaked, %lu allocation%s not freed, "
              "%.2f Kbytes lost.\n",
              nalloc, (nalloc==1?"":"s"), float(h.m_size)/1024.0F);
      for (auto&& it : h.m_objs) {
        void* p = it;
        void* ptr = (static_cast<char*>(p) - sizeof(alloc_t));
        alloc_t info;
        std::memcpy(&info, ptr, sizeof(alloc_t));
        info.print(p, h.m_os);
      }
    }

    // close the stream
    std::fclose(h.m_os);
    h.m_os = stderr;
  }

  static void* allocate(const std::size_t size, const alloc_type type) noexcept
  {
    heap_tracking& h = instance();

    // allocate memory
    void* ptr = std::malloc(size + sizeof(alloc_t));
    const alloc_t info(size, type);
    if (!ptr) {
      // allocation failed
      using int_type = long unsigned;
      std::lock_guard<std::mutex> lock(h.m_mtx);
      fprintf(h.m_os,
              "Failure: unable to allocate %lu bytes. "
              "Currently allocated memory: %.2f Kbytes\n",
              int_type(size), float(h.m_size)/1024.0F);
      info.print(0, h.m_os);
      return 0;
    }

    // save allocation information
    void* p = (static_cast<char*>(ptr) + sizeof(alloc_t));
    std::memcpy(ptr, &info, sizeof(alloc_t));

    // statistics
    std::lock_guard<std::mutex> lock(h.m_mtx);
    h.m_size   += info.size;
    h.m_maxsize = (h.m_size>h.m_maxsize) ? h.m_size : h.m_maxsize;
    ++h.m_alloc;
    h.m_objs.insert(p);
    return p;
  }

  static void free(void* p, const alloc_type type) noexcept
  {
    if (!p)
      return;

    heap_tracking& h = instance();

    // retrieve allocation information
    void* ptr = (static_cast<char*>(p) - sizeof(alloc_t));
    alloc_t info;
    std::memcpy(&info, ptr, sizeof(alloc_t));

    // clear memory to catch dangling pointers
    std::memset(ptr, 0xDEADBEEF, (info.size + sizeof(alloc_t)));

    // check if deallocation matches allocation type
    if (type!=info.type) {
      switch (info.type) {
        case ALLOC_UNKNOWN:
          fprintf(h.m_os,
                  "Error: unknown allocation.\n");
          throw runtime_error("unknown allocation");
        case ALLOC_OBJECT: {
          fprintf(h.m_os,
                  "Error: delete[] called on memory allocated with new.\n");
          throw runtime_error("delete[] called instead of delete");
        } break;
        case ALLOC_ARRAY: {
          fprintf(h.m_os,
                  "Error: delete called on memory allocated with new[].\n");
          throw runtime_error("delete called instead of delete[]");
        } break;
      }
    }

    {
      // statistics
      std::lock_guard<std::mutex> lock(h.m_mtx);
      h.m_size -= info.size;
      if (h.m_objs.erase(p)!=1) {
        fprintf(h.m_os,
                "Error: deleting unallocated object.\n");
        throw runtime_error("object not allocated through new");
      }
    }

    // release memory
    std::free(ptr);
  }
};


// nifty-counter idiom
static int nifty_counter = 0;

heap_tracker_init::heap_tracker_init(void) noexcept
{
  ++nifty_counter;
}

heap_tracker_init::~heap_tracker_init(void)
{
  if (--nifty_counter==0)
    heap_tracking::finalize();
}

} // namespace runtime

} // namespace stapl


using stapl::runtime::heap_tracking;
using stapl::runtime::ALLOC_OBJECT;
using stapl::runtime::ALLOC_ARRAY;

void* operator new(std::size_t size) throw (std::bad_alloc)
{
  void* p = heap_tracking::allocate(size, ALLOC_OBJECT);
  if (!p)
    throw std::bad_alloc();
  return p;
}

void* operator new(std::size_t size, std::nothrow_t const&) throw()
{
  return heap_tracking::allocate(size, ALLOC_OBJECT);
}

void operator delete(void* ptr) throw()
{
  heap_tracking::free(ptr, ALLOC_OBJECT);
}

void operator delete(void* ptr, std::nothrow_t const&) throw()
{
  heap_tracking::free(ptr, ALLOC_OBJECT);
}


void* operator new[](std::size_t size) throw (std::bad_alloc)
{
  void* p = heap_tracking::allocate(size, ALLOC_ARRAY);
  if (!p)
    throw std::bad_alloc();
  return p;
}

void* operator new[](std::size_t size, std::nothrow_t const&) throw()
{
  return heap_tracking::allocate(size, ALLOC_ARRAY);
}

void operator delete[](void* ptr) throw()
{
  heap_tracking::free(ptr, ALLOC_ARRAY);
}

void operator delete[](void* ptr, std::nothrow_t const&) throw()
{
  heap_tracking::free(ptr, ALLOC_ARRAY);
}

#endif
