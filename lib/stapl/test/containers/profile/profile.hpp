/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILING_PROFILE_HPP
#define STAPL_PROFILING_PROFILE_HPP

#include "adt.hpp"
#include "value_type_util.hpp"
#include "view.hpp"

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Executes the validation and profiling runs for each of the profilers
///   in @a profs, clearing out the vector at the end.
////////////////////////////////////////////////////////////////////////////////
template<typename ADT>
void profile_and_clear(prof_cont_t<ADT>& profs)
{
  for (auto p : profs)
  {
    p->validate();
    p->collect_profile();
    p->report();

    stapl::rmi_fence();
    delete p;
  }

  profs.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Creates a vector for managing profilers to be executed for given
///   container and initializes it with the profiler instances common to all
///   containers (i.e., basic constructors).
///
/// @param name A string identifier of the container
/// @param cont Reference to the actual container (for testing copy constructor)
/// @param ds Distribution specification (or size) used to construct the
///   container
////////////////////////////////////////////////////////////////////////////////
template<typename Cont, typename DistSpec>
prof_cont_t<Cont>
set_common_container_profilers(std::string const& name,
                               Cont& cont, DistSpec const& ds,
                               int argc = 0, char** argv = nullptr)
{
  return {{
    new constructor_size_profiler<Cont,DistSpec>(name, ds, argc, argv),
    new copy_constructor_profiler<Cont>(name, &cont, argc, argv)
  }};
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Creates a vector for managing profilers to be executed for given
///   view and initializes it with the profiler instances common to all
///   views (i.e., basic constructors, versioning testing).
///
/// Intended for views that are easily constructed using standard constructors.
///
/// @param name A string identifier of the container
/// @param vw Reference to the actual view (for testing copy constructor)
/// @param ds Distribution specification (or size) used to construct the
///   underlying containers
////////////////////////////////////////////////////////////////////////////////
template<typename View, typename DistSpec>
prof_cont_t<View>
set_common_view_profilers(std::string const& name,
                          View& vw, DistSpec const& ds,
                          int argc = 0, char** argv = nullptr)
{
  static_assert(is_view<View>::value, "A view must be supplied to "
    "set_common_view_profilers. Use set_common_container_profilers for "
    "containers.");

  return {{
    new view_constructor_profiler<View, DistSpec>(name, ds, argc, argv),
    new view_owning_constructor_profiler<View, DistSpec>(name, ds, argc, argv),
    new copy_constructor_profiler<View>(name, &vw, argc, argv),
    new is_valid_profiler<View>(name, &vw, true, argc, argv),
    new is_valid_profiler<View>(name, &vw, false, argc, argv)
  }};
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Creates a vector for managing profilers to be executed for given
///   view and initializes it with the profiler instances common to all
///   views (i.e., basic constructors, versioning testing).
///
/// Intended for views that need to be constructed using the functional
/// constructors (i.e., make_xxx_view).
///
/// @param name A string identifier of the container
/// @param vw Reference to the actual view (for testing copy constructor)
/// @param mft Tuple of functional view constructors returning view instances
///   at each level of given view composition (possibly a single-element tuple
///   when no composition exists)
////////////////////////////////////////////////////////////////////////////////
template<typename View, typename... F>
prof_cont_t<View>
set_common_view_profilers(std::string const& name,
                          View& vw, tuple<F...> const& mft,
                          int argc = 0, char** argv = nullptr)
{
  static_assert(is_view<View>::value, "A view must be supplied to "
    "set_common_view_profilers. Use set_common_container_profilers for "
    "containers.");

  return {{
    new view_functional_constructor_profiler<View, tuple<F...>>(
      name, vw, mft, argc, argv),
    new copy_constructor_profiler<View>(name, &vw, argc, argv),
    new is_valid_profiler<View>(name, &vw, true, argc, argv),
    new is_valid_profiler<View>(name, &vw, false, argc, argv)
  }};
}

namespace validators {

struct always_valid
{
  template<typename... Arg>
  bool operator()(Arg&&... arg) const
  { return true; }
};

} // namespace validators

} // namespace profiling

} // namespace stapl

#endif // STAPL_PROFILING_PROFILE_HPP
