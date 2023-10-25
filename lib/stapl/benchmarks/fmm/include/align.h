/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_ALIGN_H
#define STAPL_BENCHMARKS_FMM_ALIGN_H
#include <cstdlib>
#include <memory>

template <typename T, size_t NALIGN>
struct AlignedAllocator
  : public std::allocator<T>
{
  template <typename U>
  struct rebind
  {
    typedef AlignedAllocator<U, NALIGN> other;
  };

  T * allocate(size_t n)
  {
    void *ptr = NULL;
    int rc = posix_memalign(&ptr, NALIGN, n * sizeof(T));
    if (rc != 0) return NULL;
    if (ptr == NULL) throw std::bad_alloc();
    return reinterpret_cast<T*>(ptr);
  }

  void deallocate(T * p, size_t)
  {
    return free(p);
  }
};

#endif // STAPL_BENCHMARKS_FMM_ALIGN_H
