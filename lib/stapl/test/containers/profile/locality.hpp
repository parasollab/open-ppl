/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILING_LOCALITY_HPP
#define STAPL_PROFILING_LOCALITY_HPP

#include <algorithm>
#include "adt.hpp"

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the locality() method.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class locality_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using index_type = typename view_traits<ADT>::index_type;

  /// The indices to be testd for locality
  std::vector<index_type> const& indices;

  /// Expected number of remote indices
  size_t m_expected_remote;

  /// Vector of locality information obtained for #indices.
  std::vector<locality_info> m_result;

public:
  locality_profiler(std::string name, ADT* adt,
                    std::vector<index_type> const& idx,
                    size_t expected_remote,
                    int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::locality", idx.size(), argc, argv),
      indices(idx), m_expected_remote(expected_remote), m_result(idx.size())
  { }

  void check_validity()
  {
    size_t nremote = std::count_if(m_result.begin(), m_result.end(),
      [this](locality_info const& loc) {
        return loc.location() != this->get_location_id();
      });

    this->m_passed = nremote == m_expected_remote;
  }

  void run()
  {
    for (size_t i=0; i<this->m_test_size; ++i)
      m_result[i] = this->m_adt->locality(indices[i]);
  }
};

template<typename ADT>
void add_locality_profilers(prof_cont_t<ADT>& p,
                            std::string const& name, ADT& adt,
                            std::vector<size_t> const& indices,
                            size_t expected_remote,
                            int argc, char** argv)
{
  static_assert(is_container<ADT>::value || is_view<ADT>::value,
    "A STAPL container or view must be supplied to add_locality_profilers.");

  p.push_back(new locality_profiler<ADT>(name, &adt, indices, expected_remote,
                                         argc, argv));
}

} // namespace stapl

} // namespace profiling

#endif // STAPL_PROFILING_LOCALITY_HPP
