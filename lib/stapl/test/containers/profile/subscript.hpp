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
/// Contains profilers for methods of ADTs implementing the subscripting
/// interface.
///
/// @todo Add operator_square_bracket_{read/write}_profiler to test access to
/// the container's elements in monotonically increasing/decreasing order.
////////////////////////////////////////////////////////////////////////////////

#ifndef STAPL_PROFILING_SUBSCRIPT_HPP
#define STAPL_PROFILING_SUBSCRIPT_HPP

#include "adt.hpp"
#include "profiling_util.hpp"

namespace stapl {

namespace profiling {

namespace validators {

template<typename ADT,
  bool = !is_view<ADT>::value || has_identity_mf<ADT>::value>
class subscript
{
  using value_type = typename ADT::value_type;
  using exp_values_type = std::vector<value_type>;
  using index_type = typename view_traits<ADT>::index_type;
  using indices_type = std::vector<index_type>;

  exp_values_type  m_exp_values;

  template<typename C, typename GID,
    typename std::enable_if<is_container<C>::value, int>::type = 0>
  auto get_elem(C const& c, GID const& gid) const
  STAPL_AUTO_RETURN (
    c.get_element(gid)
  )

  template<typename C, typename GID,
    typename std::enable_if<!is_container<C>::value, int>::type = 0>
  auto get_elem(C const& c, GID const& gid) const
  STAPL_AUTO_RETURN (
    c[gid]
  )

public:
  subscript() = default;

  subscript(exp_values_type vals)
    : m_exp_values(std::move(vals))
  { }

  // NOTE: random_access::operator[] const involves a static_cast to a non-const
  // version of the container, preventing computed to be passed by const ref.
  bool check(ADT& computed, indices_type const& indices = indices_type()) const
  {
    bool passed = true;

    if (!m_exp_values.empty()) {
      for (size_t i = 0; i < indices.size(); ++i)
        passed &= computed[indices[i]] == m_exp_values[i];
    }
    else {
      auto const& cont_data = underlying_container<ADT>::get(computed);
      for (size_t i = 0; i < indices.size(); ++i)
        passed &= computed[indices[i]] == get_elem(cont_data, indices[i]);
    }

    return passed;
  }

  bool check(ADT& computed, index_type const& idx, size_t i) const
  { return computed[idx] == m_exp_values[i]; }
};

template<typename ADT>
class subscript<ADT, false>
{
  using value_type = typename ADT::value_type;
  using exp_values_type = std::vector<value_type>;
  using index_type = typename view_traits<ADT>::index_type;
  using indices_type = std::vector<index_type>;

  exp_values_type  m_exp_values;

public:
  subscript(exp_values_type vals)
    : m_exp_values(std::move(vals))
  { }

  // NOTE: random_access::operator[] const involves a static_cast to a non-const
  // version of the container, preventing computed to be passed by const ref.
  bool check(ADT& computed, indices_type const& indices = indices_type()) const
  {
    bool passed = true;

    for (size_t i = 0; i < indices.size(); ++i)
      passed &= computed[indices[i]] == m_exp_values[i];

    return passed;
  }
};

} // namespace validators

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for operator[] used to write into the underlying container.
///
/// Profiles writing to random positions.
///
/// @tparam ADT The container/view type
/// @tparam VF Type of the functor that will be used to validate the results.
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename VF, typename Counter = counter<default_timer>>
class operator_square_bracket_write_random_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;
  using indices_type = std::vector<index_type>;
  using value_type = typename ADT::value_type;
  using values_type = std::vector<value_type>;

  /// Indices of the elements of the ADT to be written.
  indices_type const& indices;

  /// Values to be checked in the validation phase
  values_type const& m_values;

  /// Value to be written during the profiling phase.
  value_type const& m_value;

  /// Custom validation functor
  VF m_validator;

public:
  operator_square_bracket_write_random_profiler(std::string name, ADT* adt,
                                                indices_type const& idx,
                                                values_type const& vals,
                                                VF validator,
                                                int argc = 0,
                                                char** argv = nullptr)
    : base_type(adt, name+"::operator[](rnd)_write", idx.size(), argc, argv),
      mutating<ADT>(adt), indices(idx), m_values(vals), m_value(vals.front()),
      m_validator(std::move(validator))
  {
    assert(indices.size() == m_values.size());
  }

  void validate()
  {
    // Validate by testing a small amount of insertions at each location.
    // Note that the values set by run() can not be used as the indices are
    // not guaranteed to be unique and different elements from m_values might
    // be written to the same position.

    ADT& adt = *this->m_adt;
    for (size_t i = 0; i < std::min(this->m_test_size, 10ul); ++i) {
      adt[indices[i]] = m_values[i];
      this->m_passed &= m_validator.check(adt, indices[i], i);
    }

    rmi_fence();
    this->restore_original_container();
  }

  void run()
  {
    ADT& adt = *this->m_adt;
    for (auto const& ind : indices)
      adt[ind] = m_value;
  }

  void finalize()
  {
    base_type::finalize();
    this->restore_original_container();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for operator[] used to read elements from the underlying
///   container and copying them to a local storage.
///
/// Profiles reading from random positions.
///
/// @tparam ADT The container/view type
/// @tparam VF Type of the functor that will be used to validate the results.
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename VF, typename Counter = counter<default_timer>>
class operator_square_bracket_read_random_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using index_type = typename view_traits<ADT>::index_type;
  using indices_type = std::vector<index_type>;

  /// Vector of values read from the ADT.
  std::vector<value_type> m_results;

  /// Indices of the elements of the ADT to be read.
  indices_type const& indices;

  /// Custom validation functor
  VF m_validator;

public:
  operator_square_bracket_read_random_profiler(std::string name, ADT* adt,
                                               indices_type const& idx,
                                               VF validator,
                                               int argc = 0,
                                               char** argv = nullptr)
    : base_type(adt, name+"::operator[](rnd)_read", idx.size(), argc, argv),
      indices(idx), m_validator(std::move(validator))
  {
    m_results.resize(indices.size());
  }

  void validate()
  {
    this->m_passed = m_validator.check(*this->m_adt, indices);
  }

  void run()
  {
    ADT& adt = *this->m_adt;
    for (size_t i=0; i<this->m_test_size; ++i)
      m_results[i] = adt[indices[i]];
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for operator[] used to retrieve references from the
///   underlying container without storing them.
///
/// Profiles reading from random positions.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class operator_square_bracket_read_ref_random_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;
  using indices_type = std::vector<index_type>;

  /// Indices of the elements of the ADT to be read.
  indices_type const& indices;

public:
  operator_square_bracket_read_ref_random_profiler(std::string name, ADT* adt,
                                                   indices_type const& idx,
                                                   int argc = 0,
                                                   char** argv = nullptr)
    : base_type(adt, name+"::operator[](rnd)_read_ref", idx.size(), argc, argv),
      indices(idx)
  { }

  void run()
  {
    ADT& adt = *this->m_adt;
    for (size_t i=0; i<this->m_test_size; ++i)
      GET_AND_KEEP(adt[indices[i]])
  }
};

template<typename ADT, typename GID, typename VF>
void add_subscript_read_profilers(prof_cont_t<ADT>& p,
                              std::string const& name, ADT& adt,
                              std::vector<GID> const& idx,
                              VF const& vf,
                              int argc, char** argv)
{
  p.push_back(new operator_square_bracket_read_random_profiler<ADT, VF>(
    name, &adt, idx, vf, argc, argv));
  p.push_back(new operator_square_bracket_read_ref_random_profiler<ADT>(
    name, &adt, idx, argc, argv));
}

template<typename ADT, typename GID, typename VF>
void add_subscript_write_profilers(prof_cont_t<ADT>& p,
                              std::string const& name, ADT& adt,
                              std::vector<GID> const& idx,
                              std::vector<typename ADT::value_type> const& vals,
                              VF const& vf,
                              int argc, char** argv)
{
  p.push_back(new operator_square_bracket_write_random_profiler<ADT, VF>(
    name, &adt, idx, vals, vf, argc, argv));
}

} // namespace profiling

} // namespace stapl

#endif // STAPL_PROFILING_SUBSCRIPT_HPP
