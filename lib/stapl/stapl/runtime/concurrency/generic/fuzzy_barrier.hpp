/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONCURRENCY_GENERIC_FUZZY_BARRIER_HPP
#define STAPL_RUNTIME_CONCURRENCY_GENERIC_FUZZY_BARRIER_HPP

#include "../config.hpp"
#include <atomic>
#include <cstddef>
#include <memory>
#include <utility>

namespace stapl {

namespace runtime {

namespace generic_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Binary-tree based fuzzy barrier.
///
/// @ingroup concurrency
//////////////////////////////////////////////////////////////////////
class fuzzy_barrier
{
public:
  typedef std::size_t size_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Barrier internal binary-tree node.
  //////////////////////////////////////////////////////////////////////
  struct node_t
  {
    /// Number of children not yet arrived.
    STAPL_RUNTIME_CACHELINE_ALIGNED std::atomic<size_type> count;
    /// Pointer to parent @ref node_t object.
    node_t*                                                parent;
    /// Pointer to children @ref node_t objects.
    node_t*                                                child[2];
    /// Number of children nodes.
    size_type                                              num_children;
    /// Sense flag for barrier release.
    STAPL_RUNTIME_CACHELINE_ALIGNED std::atomic<bool>      sense;

    node_t(void) noexcept
    : count(0),
      parent(nullptr),
      child{nullptr, nullptr},
      num_children(0),
      sense(false)
    { }

    node_t(node_t const&) = delete;
    node_t& operator=(node_t const&) = delete;

    template<typename Function>
    void wait(Function&& f)
    {
      bool local_sense = sense.load();

      // wait children
      while (count!=num_children)
        f();

      // notify parent and wait for release
      if (parent) {
        ++(parent->count);
        while (local_sense==sense.load())
          f();
      }

      // release children
      for (size_type i=0; i<num_children; ++i)
        child[i]->sense = !local_sense;

      // reset
      if (num_children>0)
        count -= num_children;
      sense = !local_sense;
    }
  };

  std::unique_ptr<node_t[]> m_nodes;

public:
  fuzzy_barrier(void) = default;

  explicit fuzzy_barrier(const size_type nth)
  {
    if (nth<2)
      return;

    m_nodes.reset(new node_t[nth]);
    for (size_type i=0; i<nth; ++i) {
      node_t& node          = m_nodes[i];
      const size_type left  = (2*(i+1)-1);
      const size_type right = (2*(i+1));
      if (left<nth) {
        if (right<nth) {
          node.num_children = 2;
          node.child[0]     = &m_nodes[left];
          node.child[1]     = &m_nodes[right];
        }
        else {
          node.num_children = 1;
          node.child[0]     = &m_nodes[left];
        }
      }
      node.parent = ((i==0) ? nullptr : &m_nodes[((i-1)/2)]);
    }
  }

  template<typename Function>
  void wait(const size_type tid, Function&& f)
  {
    if (!m_nodes)
      return;
    m_nodes[tid].wait(std::forward<Function>(f));
  }
};

} // namespace generic_impl


using generic_impl::fuzzy_barrier;

} // namespace runtime

} // namespace stapl

#endif
