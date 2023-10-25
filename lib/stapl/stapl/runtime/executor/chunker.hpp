/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_CHUNKER_HPP
#define STAPL_RUNTIME_EXECUTOR_CHUNKER_HPP

#include <deque>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Support class for the sliding window implementation.
///
/// @ingroup executorsImpl
//////////////////////////////////////////////////////////////////////
class chunker
{
public:
  using size_type = std::size_t;

  //////////////////////////////////////////////////////////////////////
  /// @brief Sliding window entry type.
  //////////////////////////////////////////////////////////////////////
  class entry_type
  {
  private:
    size_type m_added;
    size_type m_retired;

  public:
    constexpr entry_type(void) noexcept
    : m_added(0),
      m_retired(0)
    { }

    void add(void) noexcept
    { ++m_added; }

    void retire(void) noexcept
    { ++m_retired; }

    constexpr size_type num_added(void) const noexcept
    { return m_added; }

    constexpr bool all_retired(void) const noexcept
    { return (m_added==m_retired); }
  };

private:
  const size_type        m_max_size;
  const size_type        m_max_chunk_size;
  std::deque<entry_type> m_queue;
  size_type              m_pending;

public:
  explicit chunker(const size_type max_size, const size_type max_chunk_size)
  : m_max_size(max_size),
    m_max_chunk_size(max_chunk_size),
    m_pending(0)
  { }

  size_type max_size(void) const noexcept
  { return m_max_size; }

  size_type max_chunk_size(void) const noexcept
  { return m_max_chunk_size; }

  bool empty(void) const noexcept
  { return (m_pending==0); }

  bool full(void) const noexcept
  { return (m_pending>=m_max_size); }

  size_type pending(void) const noexcept
  { return m_pending; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return available entry.
  //////////////////////////////////////////////////////////////////////
  entry_type& get_entry(void)
  {
    ++m_pending;
    if (m_queue.empty() || (m_queue.back().num_added() == m_max_chunk_size))
      m_queue.emplace_back();
    auto& t = m_queue.back();
    t.add();
    return t;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove empty entries.
  //////////////////////////////////////////////////////////////////////
  void cleanup(void) noexcept
  {
    while (!m_queue.empty()) {
      auto const& chunk = m_queue.front();
      if (!chunk.all_retired())
        return;
      m_pending -= chunk.num_added();
      m_queue.pop_front();
    }
  }
};

} // namespace stapl

#endif
