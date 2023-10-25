/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

////////////////////////////////////////////////////////////////////////////////
/// @file
/// Profilers for sequence traversal operations.
///
/// @todo Add sequence_end_{read,write}_profiler that tests end() and traversing
/// from back.
////////////////////////////////////////////////////////////////////////////////

#ifndef STAPL_PROFILING_SEQUENCE_HPP
#define STAPL_PROFILING_SEQUENCE_HPP

#include <stapl/algorithms/functional.hpp>
#include <utility>
#include "adt.hpp"

namespace stapl {

namespace profiling {

namespace detail {

////////////////////////////////////////////////////////////////////////////////
/// @brief Compute the offset from the beginning of the sequence and the number
/// of positions to be traversed in order to visit @a sz elements of which
/// @a nremote are remote.
///
/// Assumes block distribution of the underlying collection of elements such
/// that a block of size @a sz is on current location.
///
/// @param first Offset to the first element on current location.
/// @param sz Local block size (also the total number of elements visited by
///   traversing the resulting range).
/// @param glob_sz  Total size of the container/view.
/// @param nremote  Number of remote elements to be visited by traversing the
///   resulting range.
///
/// @return std::pair of offsets (<tt>{first,last}</tt>) defining the iteration
///   range for current location (<tt>[begin()+first, begin()+last)</tt>).
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
std::pair<std::ptrdiff_t, std::ptrdiff_t>
determine_range(size_t first, size_t sz, size_t glob_sz, size_t nremote)
{
  assert(sz - nremote > 0);

  // Shift the range by nremote positions to the right
  std::ptrdiff_t ret_first = first + nremote;
  std::ptrdiff_t ret_last = ret_first + sz;

  // If we get past the global size, shift the range to the left instead
  if (ret_last > std::ptrdiff_t(glob_sz-1))
  {
    ret_first = first - nremote;
    ret_last = ret_first + sz;
  }

  // If we get before the first element, adjust the range to cover the whole
  // container
  if (ret_first < 0)
  {
    ret_first = 0;
    ret_last = sz;
  }

  return { ret_first, ret_last };
}

}  // namespace detail

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for traversing through the sequence and reading the values.
///
/// The values read on each location are stored in a local container -- the
/// timing includes the time to copy from the proxy to the container of result
/// values.
///
/// @sa sequence_begin_read_ref_profiler
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class sequence_begin_read_val_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using results_type = std::vector<value_type>;

  /// Vector of values read from the ADT.
  results_type m_results;
  typename results_type::iterator m_results_it;

  /// Offsets to the beginning and end of the range of elements to be traversed.
  std::ptrdiff_t m_first, m_last;

  /// Iterator to the position one past the last position of current range.
  typename ADT::iterator m_end;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Constructor.
  ///
  /// @param name   String identifier for the ADT being profiled.
  /// @param adt    Pointer to the ADT
  /// @param first  Offset from the global beginning of the underlying container
  ///   to the first element on current location
  /// @param sz     Number of elements on current location (also the total
  ///   number of elements visited during the profiler run)
  /// @param nremote Number of remote elements to be visited
  ///
  /// @ingroup profiling
  //////////////////////////////////////////////////////////////////////////////
  sequence_begin_read_val_profiler(std::string name, ADT* adt,
                               size_t first, size_t sz, size_t nremote,
                               int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::iter_begin_read_val", sz, argc, argv),
      m_results(sz)
  {
    std::tie(m_first, m_last) = detail::determine_range(
      first, sz, adt->size(), nremote);

    m_end = this->m_adt->begin()+m_last;
  }

  void initialize_iteration()
  { m_results_it = m_results.begin(); }

  void run()
  {
    for (auto it = this->m_adt->begin() + m_first; it != m_end;
          ++it, ++m_results_it)
      *m_results_it = *it;
  }

  void check_validity()
  {
    m_results_it = m_results.begin();
    for (auto it = this->m_adt->begin() + m_first; it != m_end;
          ++it, ++m_results_it)
      this->m_passed &= (*m_results_it == *it);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for traversing through the sequence and obtaining reference
/// to the stored elements.
///
/// @sa sequence_begin_read_val_profiler
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class sequence_begin_read_ref_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;

  /// Offsets to the beginning and end of the range of elements to be traversed.
  std::ptrdiff_t m_first, m_last;

  /// Iterator to the position one past the last position of current range.
  typename ADT::iterator m_end;

public:
  sequence_begin_read_ref_profiler(std::string name, ADT* adt,
                                   size_t first, size_t sz, size_t nremote,
                                   int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::iter_begin_read_ref", sz, argc, argv)
  {
    std::tie(m_first, m_last) = detail::determine_range(
      first, sz, adt->size(), nremote);

    m_end = this->m_adt->begin()+m_last;
  }

  void run()
  {
    for (auto it = this->m_adt->begin() + m_first; it != m_end; ++it)
      GET_AND_KEEP(*it)
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for traversing through the sequence and writing
///   given values.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class sequence_begin_write_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using values_type = std::vector<value_type>;

  /// Values to be written to each element.
  values_type const& m_values;

  /// @name Offsets to the beginning and end of the range of elements to be
  /// traversed.
  /// @{
  std::ptrdiff_t m_first, m_last;
  /// @}

  /// Iterator to the position one past the last position of current range.
  typename ADT::iterator m_end;

  /// Single value to be written during the profiling phase.
  /// Only one value will be written during profiling to eliminate time to
  /// fetch data from #m_values.
  value_type const& m_value;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @copydoc sequence_begin_write_profiler()
  /// @param vals Values to be written.
  /// @ingroup profiling
  //////////////////////////////////////////////////////////////////////////////
  sequence_begin_write_profiler(std::string name, ADT* adt,
                                size_t first, size_t sz, size_t nremote,
                                values_type const& vals,
                                int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::iter_begin_write", sz, argc, argv),
      mutating<ADT>(adt), m_values(vals), m_value(vals.front())
  {
    std::tie(m_first, m_last) = detail::determine_range(
      first, sz, adt->size(), nremote);

    m_end = this->m_adt->begin()+m_last;
  }

  void run()
  {
    for (auto it = this->m_adt->begin() + m_first; it != m_end; ++it)
      *it = m_value;
  }

  void check_validity()
  {
    for (auto it = this->m_adt->begin() + m_first; it != m_end; ++it)
      this->m_passed &= (*it == m_value);
  }

  void finalize()
  {
    base_type::finalize();
    this->restore_original_container();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the advance() method.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class sequence_advance_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;

  /// @name Starting indices, destination indices and distance in between
  /// @{
  std::vector<index_type> m_start, m_end;
  std::vector<size_t> m_dist;
  /// @}

  /// Vector of indices obtained by advancing #m_start by #m_dist.
  /// Should be elemnt-wise equal to #m_end.
  std::vector<index_type> m_res_idx;

public:
  sequence_advance_profiler(std::string name, ADT* adt,
                            std::vector<index_type> const& idx_start,
                            std::vector<index_type> const& idx_end,
                            int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::advance", idx_start.size(), argc, argv),
      m_start(idx_start), m_end(idx_end),
      m_dist(idx_start.size()), m_res_idx(idx_start.size())
  {
    assert(idx_start.size() == idx_end.size());

    // dist = end - start
    std::transform(idx_end.begin(), idx_end.end(), idx_start.begin(),
      m_dist.begin(), stapl::minus<index_type>());
  }

  void check_validity()
  {
    // res == end ?
    this->m_passed =
      std::equal(m_res_idx.begin(), m_res_idx.end(), m_end.begin());
  }

  void run()
  {
    // res = start + dist
    for (size_t i = 0; i < this->m_test_size; ++i)
      m_res_idx[i] = this->m_adt->advance(m_start[i], m_dist[i]);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the distance() method.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class sequence_distance_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;

  /// @name Starting and ending indices for computing the distance
  /// @{
  std::vector<index_type> m_start, m_end;
  /// @}

  /// Vector of computed distances between #m_start and #m_end
  /// @note STAPL currently implements unsigned distance, i.e.
  /// #m_res_dist = std::abs(#m_end - #m_start)
  std::vector<size_t> m_res_dist;

  /// Signs needed to obtain the signed distance #m_end - #m_start
  std::vector<int> m_signs;

public:
  sequence_distance_profiler(std::string name, ADT* adt,
                             std::vector<index_type> const& idx_start,
                             std::vector<index_type> const& idx_end,
                             int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::distance", idx_start.size(), argc, argv),
      m_start(idx_start), m_end(idx_end), m_res_dist(idx_start.size()),
      m_signs(idx_start.size())
  {
    assert(idx_start.size() == idx_end.size());

    std::transform(idx_end.begin(), idx_end.end(), idx_start.begin(),
      m_signs.begin(),
      [](index_type const& end, index_type const& start) {
        return (end >= start) ? 1 : -1;
      }
    );
  }

  void check_validity()
  {
    for (size_t i = 0; i < this->m_test_size; ++i) {
      this->m_passed &= (
        m_end[i] == this->m_adt->advance(m_start[i], m_signs[i]*m_res_dist[i])
      );
    }
  }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      m_res_dist[i] = this->m_adt->distance(m_start[i], m_end[i]);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the next() method
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class sequence_next_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;

  /// Starting indices
  std::vector<index_type> m_start;

  /// Indices to be reached by calling @p next() on indices from #m_start
  std::vector<index_type> m_end;

public:
  sequence_next_profiler(std::string name, ADT* adt,
                         std::vector<index_type> const& idx_start,
                         int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::next", idx_start.size(), argc, argv),
      m_start(idx_start), m_end(idx_start.size())
  { }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      m_end[i] = this->m_adt->next(m_start[i]);
  }

  void check_validity()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_passed &= (this->m_adt->advance(m_start[i], 1) == m_end[i]);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the prev() method
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class sequence_prev_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;

  /// Starting indices
  std::vector<index_type> m_start;

  /// Indices to be reached by calling @p prev() on indices from #m_start
  std::vector<index_type> m_end;

public:
  sequence_prev_profiler(std::string name, ADT* adt,
                         std::vector<index_type> const& idx_start,
                         int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::prev", idx_start.size(), argc, argv),
      m_start(idx_start), m_end(idx_start.size())
  { }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      m_end[i] = this->m_adt->prev(m_start[i]);
  }

  void check_validity()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_passed &= (this->m_adt->advance(m_end[i], 1) == m_start[i]);
  }
};

template<typename Cont,
  typename std::enable_if<!is_view<Cont>::value>::type* = nullptr>
void add_sequence_profilers(prof_cont_t<Cont>& p,
                            std::string const& name, Cont& cont,
                            size_t first, size_t sz, size_t nremote,
                            std::vector<typename Cont::value_type> const& vals,
                            int argc, char** argv)
{
  p.push_back(new sequence_begin_write_profiler<Cont>(
    name, &cont, first, sz, nremote, vals, argc, argv));
  p.push_back(new sequence_begin_read_val_profiler<Cont>(
    name, &cont, first, sz, nremote, argc, argv));
  p.push_back(new sequence_begin_read_ref_profiler<Cont>(
    name, &cont, first, sz, nremote, argc, argv));
}

template<typename View,
  typename std::enable_if<is_view<View>::value>::type* = nullptr>
void add_sequence_profilers(prof_cont_t<View>& p,
                            std::string const& name, View& vw,
                            size_t first, size_t sz, size_t nremote,
                            std::vector<size_t> const& idx_start,
                            std::vector<size_t> const& idx_end,
                            std::vector<typename View::value_type> const& vals,
                            int argc, char** argv)
{
  p.push_back(new sequence_begin_write_profiler<View>(
    name, &vw, first, sz, nremote, vals, argc, argv));
  p.push_back(new sequence_begin_read_val_profiler<View>(
    name, &vw, first, sz, nremote, argc, argv));
  p.push_back(new sequence_begin_read_ref_profiler<View>(
    name, &vw, first, sz, nremote, argc, argv));

  p.push_back(new sequence_next_profiler<View>(name, &vw, idx_start,argc,argv));
  p.push_back(new sequence_prev_profiler<View>(name, &vw, idx_start,argc,argv));

  p.push_back(new sequence_advance_profiler<View>(
    name, &vw, idx_start, idx_end, argc, argv));
  p.push_back(new sequence_distance_profiler<View>(
    name, &vw, idx_start, idx_end, argc, argv));
}

} // namespace profiling

} // namespace stapl

#endif // STAPL_PROFILING_SEQUENCE_HPP
