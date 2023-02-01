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
/// Profilers for common view methods.
////////////////////////////////////////////////////////////////////////////////

#ifndef STAPL_PROFILING_VIEW_HPP
#define STAPL_PROFILING_VIEW_HPP

#include "adt.hpp"
#include "view_composition_builder.hpp"

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the constructor of a non-owning view over a container.
///
/// In case of composed views, it heap-allocates each view in the composition.
///
/// @tparam View Type of the view
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename View, typename DistSpec,
         typename Counter = counter<default_timer> >
class view_constructor_profiler
  : public adt_profiler<View, Counter>
{
  using base_type = adt_profiler<View, Counter>;
  using cont_type = underlying_container_t<View>;

  /// The size/distribution of the container(s)
  DistSpec m_ds;

  /// A vector of containers over which to construct the views
  std::vector<cont_type*> m_pcs;

  /// A vector of views whose construction will be profiled
  std::vector<View*> m_views;

public:
  view_constructor_profiler(std::string vwname, DistSpec const& ds,
                            int argc = 0, char** argv = nullptr)
    : base_type(nullptr, vwname+"::constructor_size", argc, argv), m_ds(ds)
  {
    this->m_time_per_invocation = true;

    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_pcs.push_back(new cont_type(m_ds));

    m_views.resize(this->m_test_size, nullptr);
  }

  ~view_constructor_profiler()
  {
    rmi_fence();
    for (size_t i=0; i < this->m_test_size; ++i)
      delete this->m_pcs[i];
  }

  void run()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      this->m_views[i] = view_composition<View>::construct(*m_pcs[i]);
  }

  void check_validity()
  {
    for (auto vw_ptr : m_views)
      this->m_passed &= (vw_ptr && vw_ptr->is_valid());
  }

  void finalize_iteration()
  {
    rmi_fence();
    for (size_t i=0; i < this->m_test_size; ++i)
      delete this->m_views[i];
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the constructor of an owning view over a container.
///
/// In case of composed views, it heap-allocates each view in the composition.
///
/// @tparam View Type of the view
/// @tparam DistSpec Type determining the distribution of the ADT
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename View, typename DistSpec,
         typename Counter = counter<default_timer> >
class view_owning_constructor_profiler
  : public adt_profiler<View, Counter>
{
  using base_type = adt_profiler<View, Counter>;
  using cont_type = underlying_container_t<View>;

  /// The size/distribution of the container(s)
  DistSpec m_ds;

  /// A vector of containers over which to construct the views
  std::vector<cont_type*> m_pcs;

  /// A vector of views whose construction will be profiled
  std::vector<View*> m_views;

public:
  view_owning_constructor_profiler(std::string vwname, DistSpec const& ds,
                                   int argc = 0, char** argv = nullptr)
    : base_type(nullptr, vwname+"::constructor_owning_view", argc, argv), m_ds(ds)
  {
    this->m_time_per_invocation = true;

    m_pcs.resize(this->m_test_size, nullptr);
    m_views.resize(this->m_test_size, nullptr);
  }

  void initialize_iteration()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_pcs[i] = new cont_type(m_ds);
  }

  void run()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      this->m_views[i] = view_composition<View>::construct(m_pcs[i]);
  }

  void check_validity()
  {
    for (auto vw_ptr : m_views)
      this->m_passed &= (vw_ptr && vw_ptr->is_valid());
  }

  void finalize_iteration()
  {
    rmi_fence();
    for (size_t i=0; i < this->m_test_size; ++i)
      delete this->m_views[i];
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the functional constructors for (a composition of) views
///   (typically constructed as a sequence of calls to make_xxx_view).
///
/// Uses a tuple of boost::optional instances of the intermediate views
/// to represent the composition throughout the lifetime of the profiler.
///
/// @tparam View Type of the view
/// @tparam MakeFnsTuple  Type of the tuple of callables used to construct the
///   views at each level of the composition.
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename View, typename MakeFnsTuple,
         typename Counter=counter<default_timer>>
class view_functional_constructor_profiler
  : public adt_profiler<View, Counter>
{
  using base_type = adt_profiler<View, Counter>;
  using cont_type = underlying_container_t<View>;

  /// A vector of views whose construction will be profiled
  std::vector<View*> m_views;

  /// Tuple of callables used to construct each level of a view composition
  MakeFnsTuple m_make_funcs;

  /// @brief Instance of the view composition creator for given underlying
  /// container and tuple of make functions for the composed views.
  functional_view_composition<cont_type, MakeFnsTuple> m_view_composition;

public:
  view_functional_constructor_profiler(std::string vwname,
                                       View const& vw, MakeFnsTuple const& mft,
                                       int argc = 0, char** argv = nullptr)
    : base_type(nullptr, vwname+"::make_view", argc, argv),
      m_make_funcs(mft), m_view_composition(underlying_container<View>::get(vw))
  {
    this->m_time_per_invocation = true;
    m_views.resize(this->m_test_size, nullptr);
  }

  void run()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      this->m_views[i] = m_view_composition.create(m_make_funcs);
  }

  void check_validity()
  {
    for (auto vw_ptr : m_views)
      this->m_passed &= (vw_ptr && vw_ptr->is_valid());
  }

  void finalize_iteration()
  {
    base_type::finalize_iteration();
    m_view_composition.clear();
    m_views.clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the is_valid() method of given view.
///
/// @tparam View Type of the view
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename View, typename Counter = counter<default_timer>>
class is_valid_profiler
  : public adt_profiler<View, Counter>,
    public mutating<View>
{
  using base_type = adt_profiler<View, Counter>;

  bool m_result;
  bool m_exp_valid;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Constructor.
  /// @param name Identifier of the view.
  /// @param vw   Pointer to the view.
  /// @param expected_valid Determines whether the view validation failure
  /// or success is to be tested
  //////////////////////////////////////////////////////////////////////////////
  is_valid_profiler(std::string name, View* vw, bool expected_valid,
                    int argc = 0, char** argv = nullptr)
    : base_type(vw, name+"::is_valid", argc, argv), mutating<View>(vw),
      m_result(true), m_exp_valid(expected_valid)
  {
    if (!m_exp_valid)
      this->name += "(false)";
    else
      this->name += "(true)";

    this->m_time_per_invocation = true;
  }

  void initialize()
  {
    if (!m_exp_valid)
      this->m_cont.incr_version();
  }

  void run()
  {
    for (size_t i=0; i<this->m_test_size; ++i)
      m_result &= this->m_adt->is_valid();
  }

  void check_validity()
  { this->m_passed = m_result == m_exp_valid; }

  void finalize()
  {
    base_type::finalize();
    this->restore_original_container();
  }

};

} // namespace profiling

} // namespace stapl

#endif // STAPL_PROFILING_VIEW_HPP
