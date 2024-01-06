/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILING_RESTRUCTURE_HPP
#define STAPL_PROFILING_RESTRUCTURE_HPP

////////////////////////////////////////////////////////////////////////////////
/// @file
/// Contains profilers for methods that change structure of the ADT.
////////////////////////////////////////////////////////////////////////////////

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the resize() method.
///
/// @note Expensive operation - only one invocation is timed.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class resize_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;

  /// The new size.
  size_t m_nsz;

public:
  resize_profiler(std::string name, ADT* adt, size_t sz,
                  int argc = 0, char** argv = nullptr)
    : base_type(adt, name, 1, argc, argv), mutating<ADT>(adt), m_nsz(sz)
  { }

  void run(void)
  {
    this->m_adt->resize(m_nsz);
    rmi_fence();
  }

  void finalize_iteration()
  { this->restore_original_container(); }

  void check_validity()
  { this->m_passed = this->m_adt->size() == m_nsz; }
};

template<typename Cont>
void add_resize_profilers(prof_cont_t<Cont>& p,
                          std::string const& name, Cont& cont,
                          int argc, char** argv)
{
  size_t const sz = cont.size();

  p.push_back(
    new resize_profiler<Cont>(name+"::resize_half", &cont, sz/2, argc, argv));
  p.push_back(
    new resize_profiler<Cont>(name+"::resize_double", &cont, 2*sz, argc, argv));
}

} // namespace profiling

} // namespace stapl

#endif // STAPL_PROFILING_RESTRUCTURE_HPP
