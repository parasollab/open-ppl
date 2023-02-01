/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TASK_FACTORY_BASE_HPP
#define STAPL_PARAGRAPH_TASK_FACTORY_BASE_HPP

#ifndef PARAGRAPH_MAX_VIEWS
#  define PARAGRAPH_MAX_VIEWS 5
#endif

#include <array>

#include <stapl/skeletons/utility/view_index_partition.hpp>
#include <stapl/runtime/p_object.hpp>

#include "incremental_wf.hpp"
#include "factory_wf.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Intermediate class of the task factory hierarchy that provides the
///   edge_mf_type for all factories that derive from it.
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
class task_factory_base
 : public p_object,
   public incremental_wf,
   public factory_wf
{
private:
  /// @brief Iterators set to local index spaces of the views to be processed.
  std::array<view_index_iterator_base*, PARAGRAPH_MAX_VIEWS> m_iterators;

  /// @brief Stores whether m_iterators has been initialized.
  bool                                                       m_initialized;

protected:
  /// Stores whether the derived factory has finished specifying tasks.
  bool                                                       m_finished;

  bool initialized(void) const
  {
    return m_initialized;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset the iterators over the view indices on this location.
  ///
  /// This is used in factories where multiple tasks will be generated for each
  /// view element to be processed.
  //////////////////////////////////////////////////////////////////////
  void reset_view_indices()
  {
    for (auto&& iter : m_iterators)
      if (iter != nullptr)
        iter->reset();
  }

  void set_view_index_iterator(std::size_t idx, view_index_iterator_base* ptr)
  {
    m_initialized    = true;
    m_iterators[idx] = ptr;
  }

  std::size_t view_indices_size() const
  {
    return m_iterators[0]->size();
  }

  view_index_iterator_base*
  get_view_index_iterator(std::size_t n) const
  {
    stapl_assert(m_iterators[n] != nullptr,
      "get_view_index_iterator(), null pointer at i pos");

    return m_iterators[n];
  }

public:
  task_factory_base(bool b_incremental_generation = false)
    : m_initialized(false),
      m_finished(!b_incremental_generation)
  {
    for (auto&& iter : m_iterators)
      iter = nullptr;
  }

  ~task_factory_base() override
  {
    for (auto&& iter : m_iterators)
      delete iter;
  }

  bool finished() const
  {
    return m_finished;
  }

  virtual void reset()
  { }

  void define_type(typer &t)
  {
    t.transient(m_initialized, false);
    t.transient(m_finished, false);
  }
}; // class task_factory_base

} // namespace stapl

#endif // STAPL_PARAGRAPH_TASK_FACTORY_BASE_HPP
