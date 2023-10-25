/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//////////////////////////////////////////////////////////////////////
/// @file test_multiarray_view_over_array_view.cc
/// @brief Test that executes simple paragraphs over a
///        @p multiarray_view<array_view<array>> (providing 3D access to
///        the underlying 1D array) and its slices.
///
/// In accordance to the primary application (particle transport code PDT),
/// the dimensions are referred to as @em dof (spatial degree of freedom),
/// @em group (energy interval from the multi-group formalism) and
/// @em moment (spherical harmonics integral moment).
///
/// In this test, the 1D arrays are primarily distributed in blocks of size
/// <tt>local_num_dof * global_num_groups * global_num_moments</tt>.
///
/// @todo Extend the test to arbitrarily distributed arrays.
/// @todo Extend the test to use nested paragraphs in addition to looping
///       over the slices sequentially.
//////////////////////////////////////////////////////////////////////

#include <stapl/multiarray.hpp>
#include <stapl/array.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#include <stapl/views/slices_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/utility/pack_ops.hpp>

#include <boost/format.hpp>

#include "../test_report.hpp"
#include "../confint.hpp"
#include <stapl/runtime/counter/default_counters.hpp>

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Returns true if all the arguments forwarded from a work function
///        are local.
//////////////////////////////////////////////////////////////////////
template<typename... T>
bool are_local(T&&... x)
{
  return pack_ops::functional::and_(
    accessor_core_access::is_local(proxy_core_access::accessor(x))...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy @a y to @a x iff both arguments are local.
//////////////////////////////////////////////////////////////////////
struct assign_if_local
{
  template<typename Reference1, typename Reference2>
  void operator()(Reference1 x, Reference2 y) const
  {
    if (are_local(x,y))
      y = x;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function used to test a map_func over the slices along
///        the first dimension of the 3D view over the 1D array_view.
//////////////////////////////////////////////////////////////////////
struct test_dof_slice_wf
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Compares each element of current input slice with original
  ///        full view, returning the result in corresponding element of
  ///        a boolean output view (of the same shape as the input).
  ///
  /// Also asserts locality of the slices by assigning false to the
  /// output view element if any element is non-local.
  ///
  /// @param slice      input slice
  /// @param ret_slice  output slice
  /// @param idx        current index of the slice in the full 3D view
  /// @param full_view  reference to the full 3D view
  //////////////////////////////////////////////////////////////////////
  template<typename Slice, typename RetSlice,
           typename SliceIdx, typename FullView>
  void operator()(Slice&& slice, RetSlice&& ret_slice,
                  SliceIdx&& idx, FullView&& full_view)
  {
    std::size_t ng, nm;
    std::tie(ng, nm) = slice.dimensions();

    for (std::size_t g = 0; g < ng; ++g)
      for (std::size_t m = 0; m < nm; ++m) {
        slice(g,m) *= 2;
        ret_slice(g,m) = slice(g,m) == 2*full_view(idx,g,m)
          && are_local(slice(g,m), ret_slice(g,m));
      }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function used to test a map_func over the slices along
///        the third dimension of the 3D view over the 1D array_view.
//////////////////////////////////////////////////////////////////////
struct test_mom_slice_wf
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Compares each element of current input slice with the original
  ///        full view, returning the result in the element at current slice
  ///        index in a 1D boolean output view.
  ///
  /// Because of the way the underlying 1D array is distributed, locality
  /// can not be generally ensured in this case.
  ///
  /// @param slice      input slice
  /// @param ret_elem   output element
  /// @param idx        current index of the slice in the full 3D view
  /// @param full_view  reference to the full 3D view
  //////////////////////////////////////////////////////////////////////
  template<typename Slice, typename RetElem,
           typename SliceIdx, typename FullView>
  void operator()(Slice&& slice, RetElem&& ret_elem,
                  SliceIdx&& idx, FullView&& full_view)
  {
    std::size_t nd, ng;
    std::tie(nd, ng) = slice.dimensions();

    bool ret = true;

    for (std::size_t d = 0; d < nd; ++d)
      for (std::size_t g = 0; g < ng; ++g) {
        slice(d,g) *= 2;
        ret &= slice(d,g) == 2*full_view(d,g,idx);
      }

    ret_elem = ret;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function used to test a map_func over the slices along
///        two dimensions of the 3D view over the 1D array_view.
//////////////////////////////////////////////////////////////////////
struct test_1d_slice_wf
{
  test_1d_slice_wf(bool expected_local)
    : m_expected_local(expected_local)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Multiplies the elements of the input slice by a constant
  ///        and copies them to the output slice; if the elements being
  ///        referenced are not local, 1 is added to the output to
  ///        force the equality test to fail.
  ///
  /// @param slice      input slice
  /// @param ret_slice  output slice
  /// @param idx        current index of the slice in the full 3D view
  /// @param full_view  reference to the full 3D view
  //////////////////////////////////////////////////////////////////////
  template<typename Slice, typename RetSlice>
  void operator()(Slice&& slice, RetSlice&& ret_slice)
  {
    for (std::size_t i = 0; i < slice.size(); ++i) {
      slice[i] *= 2;
      ret_slice[i] = slice[i] +
        (m_expected_local ? int(!are_local(slice[i], ret_slice[i])) : 0);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_expected_local);
  }

private:
  ///@brief Flag indicating whether local slice elements can be expected
  ///       (one of the two sliced dimensions is the first (dof) dimension)
  bool m_expected_local;
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function used to test a map_func over a fixed slice along
///        the third dimension of the 3D view over the 1D array_view.
//////////////////////////////////////////////////////////////////////
struct test_fixed_mom_slice_wf
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Multiplies current element of the input slice by a constant
  ///        and copies it to the corresponding element of the output view
  ///        (of the same shape as the slice).
  ///
  /// Because of the way the underlying 1D array is distributed, locality
  /// can not be generally ensured in this case.
  ///
  /// @param slice_el   input element
  /// @param ret_slice  output element
  //////////////////////////////////////////////////////////////////////
  template<typename SliceElem, typename RetElem>
  void operator()(SliceElem&& slice_el, RetElem&& ret_el)
  {
    slice_el *= 2;
    ret_el = slice_el;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function used to test a map_func over a fixed slice along
///        the second and third dimension of the 3D view over the 1D array_view.
//////////////////////////////////////////////////////////////////////
struct test_fixed_grp_mom_slice_wf
{
  test_fixed_grp_mom_slice_wf(size_t fixed_g, size_t fixed_m)
    : m_fixed_g(fixed_g), m_fixed_m(fixed_m)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compares current element of the input slice with corresponding
  ///        element of the original full view, returning the result in
  ///        corresponding element of a boolean output view.
  ///
  /// Also asserts locality by assigning false to the output view element
  /// if any element is non-local.
  ///
  /// @param slice_el   element of the input slice
  /// @param ret_elem   element of the output slice
  /// @param idx        current index within the slice
  /// @param full_view  reference to the full 3D view
  //////////////////////////////////////////////////////////////////////
  template<typename SliceElem, typename RetElem,
           typename Idx, typename FullView>
  void operator()(SliceElem&& slice_el, RetElem&& ret_el,
                  Idx&& idx, FullView&& full_view)
  {
    slice_el *= 2;
    ret_el = slice_el == 2*full_view(idx, m_fixed_g, m_fixed_m)
      && are_local(slice_el, ret_el);
  }

  void define_type(typer& t)
  {
    t.member(m_fixed_g);
    t.member(m_fixed_m);
  }

private:
  size_t m_fixed_g;
  size_t m_fixed_m;
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function used to test a map_reduce over linearized
///        @ref slices_segmented_view with slices along the first and
///        second dimension of the 3D view over the 1D array_view.
//////////////////////////////////////////////////////////////////////
struct test_linearized_dof_grp_slice_wf
{
  using result_type = bool;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the result of comparing all elements of two linearized
  ///        input slices (as well as asserting their locality).
  ///
  /// @param slice1   first input slice
  /// @param slice2   second input slice
  //////////////////////////////////////////////////////////////////////
  template<typename Slice1, typename Slice2>
  bool operator()(Slice1&& slice1, Slice2&& slice2)
  {
    bool ret = true;

    for (std::size_t m = 0; m < slice1.size(); ++m)
      ret &= slice1[m] == slice2[m] && are_local(slice1[m], slice2[m]);

    return ret;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Multiplies elements of the linearized input slice by a constant
  ///        and compares them with corresponding elements of the linearized
  ///        full 3D view (i.e., the original underlying 1D array_view).
  ///
  /// @param slice        input slice
  /// @param dg_lin_idx   index of the input slice in the (linearized) full
  ///                     view (linearized dof-group index)
  /// @param full_view    reference to the (linearized) full view
  //////////////////////////////////////////////////////////////////////
  template<typename Slice, typename Idx, typename Full1DView>
  bool operator()(Slice&& slice, Idx&& dg_lin_idx, Full1DView&& full_view)
  {
    bool ret = true;

    size_t nmom = slice.size();
    size_t current_mom_chunk_start = dg_lin_idx * nmom;

    for (std::size_t m = 0; m < slice.size(); ++m) {
      slice[m] *= 2;
      ret &= slice[m] == 2*full_view[current_mom_chunk_start + m]
        && are_local(slice[m]);
    }

    return ret;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Class that manages all the tests and the containers they use.
//////////////////////////////////////////////////////////////////////
class TestDriver
{
  // Types defining the distribution of the basic 1D array.
  using part_info_cont_t = array<arbitrary_partition_info>;
  using part_info_t = array_view<part_info_cont_t>;

  // Type of the basic 1D array and associated array_view.
  template<typename T>
  using general_array_type = array<T,
    view_based_partition<distribution_spec<>, part_info_t>,
    view_based_mapper<distribution_spec<>>>;

  using array_type = general_array_type<int>;
  using array_view_type = array_view<array_type>;

  // Type of the multidimensional view over the 1D array_view.
  using mavw_type = typename make_multiarray_view_traits<
                      array_view_type, size_t, size_t, size_t>::type;

  // Types of the various slices views used in the tests.
  using dof_vw_type     = slices_segmented_view<index_sequence<0>, mavw_type>;
  using dof_grp_vw_type = slices_segmented_view<index_sequence<0,1>, mavw_type>;
  using mom_vw_type     = slices_segmented_view<index_sequence<2>, mavw_type>;
  using grp_mom_vw_type = slices_segmented_view<index_sequence<1,2>, mavw_type>;

  /// @name Global number of dof, groups and moments.
  /// @{
  size_t m_ndof, m_ngrp, m_nmom;
  /// @}

  /// @brief Maximum inputs size beyond which timing will be disabled in tests
  /// that may reference non-local data.
  static size_t constexpr max_size_for_timing = 100000;

  /// Number of controller iterations.
  /// If the value is greater than one, statistical timing data over given
  /// number of runs will be collected and reported, except for large inputs
  /// in tests where locality is not ensured.
  size_t m_num_cont_it;

  /// @brief Flag indicating whether timing is requested but should be
  /// disabled because of the large input size (to be used in tests
  /// referencing non-local data).
  bool m_skip_long;

  /// Number of locations.
  size_t m_nlocs;

  /// @name Timing facilities.
  /// @{
  confidence_interval_controller m_controller;
  counter<default_timer> m_timer;
  /// @}

  /// View defining the distribution of the basic 1D arrays.
  part_info_t* m_part_info;

  /// @name Basic 1D views (one to perform operations on, other for comparison).
  /// @{
  array_view_type* m_vw;
  array_view_type* m_vw_ref;
  /// @}

  /// @name 3D views over the 1D views.
  /// @{
  mavw_type* m_mavw;
  mavw_type* m_mavw_ref;
  /// @}

  /// @name Slices views over the 3D views.
  /// @{
  dof_vw_type* m_dof_vw;
  dof_grp_vw_type* m_dof_grp_vw;
  mom_vw_type* m_mom_vw;
  grp_mom_vw_type* m_grp_mom_vw;
  /// @}

public:
  TestDriver(int num_cont_it, size_t ndof, size_t ngrp, size_t nmom)
    : m_ndof(ndof), m_ngrp(ngrp), m_nmom(nmom), m_num_cont_it(num_cont_it),
      m_skip_long(num_cont_it > 1 && ndof*ngrp*nmom > max_size_for_timing),
      m_nlocs(get_num_locations()), m_controller(num_cont_it, num_cont_it)
  {
    part_info_cont_t* part_info_cont = new part_info_cont_t(m_nlocs);
    set_partition_info_container(*part_info_cont, ngrp * nmom);
    m_part_info = new part_info_t(part_info_cont);

    m_vw         = new array_view_type(new array_type(*m_part_info));
    m_vw_ref     = new array_view_type(new array_type(*m_part_info));

    m_mavw       = new mavw_type(make_multiarray_view(*m_vw, ndof, ngrp, nmom));
    m_mavw_ref   = new mavw_type(
                         make_multiarray_view(*m_vw_ref, ndof, ngrp, nmom));

    m_dof_vw     = new dof_vw_type(make_slices_view<0>(*m_mavw));
    m_dof_grp_vw = new dof_grp_vw_type(make_slices_view<0,1>(*m_mavw));
    m_mom_vw     = new mom_vw_type(make_slices_view<2>(*m_mavw));
    m_grp_mom_vw = new grp_mom_vw_type(make_slices_view<1,2>(*m_mavw));
  }

  ~TestDriver()
  {
    delete m_part_info;
    delete m_vw;
    delete m_vw_ref;
    delete m_mavw;
    delete m_mavw_ref;
    delete m_dof_vw;
    delete m_dof_grp_vw;
    delete m_mom_vw;
    delete m_grp_mom_vw;
  }

  bool test_copy(void);
  bool test_traversal(void);
  bool test_dof_slice(void);
  bool test_mom_slice(void);
  bool test_grp_mom_slice(void);
  bool test_dof_grp_slice(void);
  bool test_fixed_mom_slice(void);
  bool test_fixed_grp_mom_slice(void);
  bool test_linearized_dof_grp_slice(void);

private:
  void set_partition_info_container(part_info_cont_t& part_info_cont,
                                    size_t nel_per_dof);

  //////////////////////////////////////////////////////////////////////
  /// @brief Report whether the test passed or failed; if timing was
  ///        performed, report the timing results as well.
  ///
  /// @param  passed  Boolean indicating the status of the test
  /// @param  label   Test description
  //////////////////////////////////////////////////////////////////////
  void report(bool passed, std::string const& label)
  {
    std::stringstream ss;
    ss << "Testing multiarray view over array view (" << label << ")";

    if (m_num_cont_it == 1) {
      STAPL_TEST_REPORT(passed, ss.str());
    }
    else {
      m_controller.report(ss.str(), passed);
      do_once([&] { std::cout << "\n"; });
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Setup the partition of the basic 1D arrays.
///
/// Each location gets <tt>local_num_dof * nel_per_dof</tt> elements, where
/// @p local_num_dof is obtained by balanced partition of #m_ndof elements
/// across #m_nlocs locations.
///
/// @param  part_info_cont  Array of the size of number of locations
///         which will be filled with local partition extents
/// @param  nel_per_dof  Number of elements per dof (#m_ngrp * #m_nmom for
///         the basic 1D arrays managed by this class).
//////////////////////////////////////////////////////////////////////
void TestDriver::set_partition_info_container(part_info_cont_t& part_info_cont,
                                              size_t nel_per_dof)
{
  do_once([&](void) {
    const unsigned long int chunk_size = (m_ndof / m_nlocs) * nel_per_dof;
    unsigned long int lower_bound = 0;
    unsigned long int upper_bound = chunk_size;

    if (m_nlocs != 1) {
      for (unsigned int i = 0; i != m_nlocs-1; ++i)
      {
        part_info_cont[i] =
          stapl::arbitrary_partition_info(lower_bound, upper_bound-1, i);
        lower_bound = upper_bound;
        upper_bound += chunk_size;
      }
    }
    part_info_cont[m_nlocs-1] = stapl::arbitrary_partition_info(
      lower_bound, std::max(upper_bound, m_ndof*nel_per_dof)-1, m_nlocs-1);
  });
}

//////////////////////////////////////////////////////////////////////
/// @brief Test copying between the multidimensional views.
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_copy(void)
{
  // Remove all samples previously pushed to the controller.
  m_controller.reset();

  // Set m_vw to increasing sequence, copy to m_vw_ref.
  iota(*m_vw, 0);

  while (m_controller.iterate())
  {
    m_timer.reset();
    m_timer.start();

    copy(*m_vw, *m_vw_ref);

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  report(true, "array_view copy");

  m_controller.reset();

  // Set m_vw to all zeros, copy the increasing sequence back from m_vw_ref
  // through mavw_ref and mavw.
  fill(*m_vw, 0);

  while (m_controller.iterate())
  {
    m_timer.reset();
    m_timer.start();

    // Perform the same operation as copy(mavw_original, mavw), but also assert
    // locality.
    stapl::map_func(assign_if_local(), *m_mavw_ref, *m_mavw);

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  // Make sure m_vw is again the increasing sequence we started from
  bool passed = map_reduce(equal_to<int>(), logical_and<bool>(),
    *m_vw, counting_view(m_vw->size(), 0));

  report(passed, "multiarray_view<array_view> copy");

  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test sequential loop through the multidimensional views and slices.
///
/// If timing is requested (#m_num_cont_it > 1), this test will be skipped for
/// large array sizes.
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_traversal(void)
{
  if (m_skip_long)
    return true;

  iota(*m_vw, 0);

  bool passed_dof = true, passed_mom = true;

  do_once([&] {
    for (size_t d = 0; d < m_ndof; ++d) {
      auto&& dof_slice = (*m_dof_vw)[d];

      for (size_t g = 0; g < m_ngrp; ++g) {
        auto&& dof_grp_slice = (*m_dof_grp_vw)(d,g);

        for (size_t m = 0; m < m_nmom; ++m) {
          passed_dof &= (
            (dof_slice(g,m) == dof_grp_slice[m]) &&
            (dof_slice(g,m) == (*m_mavw)(d,g,m))
          );
        }
      }
    }

    for (size_t m = 0; m < m_nmom; ++m) {
      auto&& mom_slice = (*m_mom_vw)[m];

      for (size_t g = 0; g < m_ngrp; ++g) {
        auto&& grp_mom_slice = (*m_grp_mom_vw)(g,m);

        for (size_t d = 0; d < m_ndof; ++d) {
          passed_mom = passed_mom && (
            (mom_slice(d,g) == grp_mom_slice[d]) &&
            (mom_slice(d,g) == (*m_mavw)(d,g,m))
          );
        }
      }
    }
  });

  STAPL_TEST_REPORT(passed_dof,
    "Testing multiarray view over array view (dof slices traversal)");

  // Add a blank line like in the other tests that actually report timing
  if (m_num_cont_it > 1)
    do_once([&] { std::cout << "\n"; });

  STAPL_TEST_REPORT(passed_mom,
    "Testing multiarray view over array view (mom slices traversal)");

  if (m_num_cont_it > 1)
    do_once([&] { std::cout << "\n"; });

  return passed_dof && passed_mom;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test a map_func over the slices along first dimension of the
///        3D view over the 1D array_view.
/// @see   test_dof_slice_wf
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_dof_slice(void)
{
  general_array_type<bool> results(*m_part_info);

  auto results_vw = make_array_view(results);
  auto results_mavw = make_multiarray_view(results_vw, m_ndof, m_ngrp, m_nmom);
  auto results_slices = make_slices_view<0>(results_mavw);

  m_controller.reset();
  iota(*m_vw_ref, 0);

  while (m_controller.iterate())
  {
    iota(*m_vw, 0);

    m_timer.reset();
    m_timer.start();

    map_func(test_dof_slice_wf(),
      *m_dof_vw, results_slices, counting_view(m_ndof, 0),
      make_repeat_view(*m_mavw_ref));

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  bool passed = all_of(results_vw, bind1st(equal_to<bool>(), true));

  report(passed, "dof slices map_func");

  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test a map_func over the slices along the third dimension
///        of the 3D view over the 1D array_view.
///
/// If timing is requested (#m_num_cont_it > 1), this test will be skipped
/// for large array sizes.
///
/// @see   test_mom_slice_wf
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_mom_slice(void)
{
  if (m_skip_long)
    return true;

  array<bool> results(m_nmom);
  auto results_vw = make_array_view(results);

  m_controller.reset();
  iota(*m_vw_ref, 0);

  while (m_controller.iterate())
  {
    iota(*m_vw, 0);

    m_timer.reset();
    m_timer.start();

    map_func(test_mom_slice_wf(),
      *m_mom_vw, results_vw, counting_view(m_nmom, 0),
      make_repeat_view(*m_mavw_ref));

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  bool passed = all_of(results_vw, bind1st(equal_to<bool>(), true));

  report(passed, "mom slices map_func");

  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test a map_func over the slices along the first two dimensions
///        of the 3D view over the 1D array_view.
/// @see   test_1d_slice_wf
/// @todo  fix issues with GID metadata alignment, so that counting_view_nd
///        and repeat_view_nd can be used in the following test like in the
///        1D slices_view tests.
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_dof_grp_slice(void)
{
  auto dof_grp_vw_ref = make_slices_view<0,1>(*m_mavw_ref);

  m_controller.reset();

  while (m_controller.iterate())
  {
    iota(*m_vw, 0);
    fill(*m_vw_ref, 0);

    m_timer.reset();
    m_timer.start();

    map_func(test_1d_slice_wf(true),
      *m_dof_grp_vw, dof_grp_vw_ref);

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  bool passed = map_reduce(equal_to<int>(), logical_and<bool>(),
    *m_vw, *m_vw_ref);

  report(passed, "dof-grp slices map_func");

  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test a map_func over the slices along the second and third
///        dimension of the 3D view over the 1D array_view.
///
/// If timing is requested (#m_num_cont_it > 1), this test will be skipped
/// for large array sizes.
///
/// @see   test_1d_slice_wf
/// @todo  fix issues with GID metadata alignment, so that counting_view_nd
///        and repeat_view_nd can be used in the following test like in the
///        1D slices_view tests.
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_grp_mom_slice(void)
{
  if (m_skip_long)
    return true;

  auto grp_mom_vw_ref = make_slices_view<1,2>(*m_mavw_ref);

  m_controller.reset();

  while (m_controller.iterate())
  {
    iota(*m_vw, 0);
    fill(*m_vw_ref, 0);

    m_timer.reset();
    m_timer.start();

    map_func(test_1d_slice_wf(false),
      *m_grp_mom_vw, grp_mom_vw_ref);

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  bool passed = map_reduce(equal_to<int>(), logical_and<bool>(),
    *m_vw, *m_vw_ref);

  report(passed, "grp-mom slices map_func");

  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test a map_func over a fixed slice along the third dimension
///        of the 3D view over the 1D array_view.
///
/// If timing is requested (#m_num_cont_it > 1), this test will be skipped
/// for large array sizes.
///
/// @see   test_fixed_mom_slice_wf
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_fixed_mom_slice(void)
{
  if (m_num_cont_it > 1 && m_ndof*m_ngrp < max_size_for_timing)
    return true;

  // Create a partition for the output array (to have the same shape as the
  // fixed moment slice).
  part_info_cont_t part_info_cont(m_nlocs);
  set_partition_info_container(part_info_cont, m_ngrp);
  part_info_t part_info(part_info_cont);

  // Create a 1D array and a 2D multiarray_view over it (dof-grp dimensions)
  // to store the results.
  array_type ary_dg(part_info);

  auto vw_dg = make_array_view(ary_dg);
  auto mavw_dg = make_multiarray_view(vw_dg, m_ndof, m_ngrp);

  // Fixed moment index.
  const size_t fm = 0;

  // Reset the reference view and the timer.
  iota(*m_vw_ref, 0);
  m_controller.reset();

  // Run the main execution and timing loop.
  while (m_controller.iterate())
  {
    iota(*m_vw, 0);
    fill(vw_dg, 0);

    m_timer.reset();
    m_timer.start();

    map_func(test_fixed_mom_slice_wf(),
      (*m_mom_vw)[fm], mavw_dg);

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  // Compare against the reference view with fixed moment index.
  bool passed = true;

  do_once([&] {
  for (size_t d = 0; d < m_ndof; ++d)
    for (size_t g = 0; g < m_ngrp; ++g)
      passed &= mavw_dg(d,g) == 2*(*m_mavw_ref)(d,g,fm);
  });

  report(passed, "fixed mom slice map_func");

  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test a map_func over a fixed slice along the second and third
///        dimension of the 3D view over the 1D array_view.
/// @see   test_fixed_grp_mom_slice_wf
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_fixed_grp_mom_slice(void)
{
  array<bool> results(m_ndof);
  auto results_vw = make_array_view(results);

  const size_t fm = 0;
  const size_t fg = m_ngrp/2;

  m_controller.reset();

  while (m_controller.iterate())
  {
    iota(*m_vw, 0);
    iota(*m_vw_ref, 0);
    fill(results_vw, false);

    m_timer.reset();
    m_timer.start();

    map_func(test_fixed_grp_mom_slice_wf(fg,fm),
      (*m_grp_mom_vw)(fg,fm), results_vw,
      counting_view(m_ndof, 0), make_repeat_view(*m_mavw_ref));

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  bool passed = all_of(results_vw, bind1st(equal_to<bool>(), true));

  report(passed, "fixed grp-mom slice map_func");

  return passed;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test a map_reduce over linearized @ref slices_segmented_view
///        with slices along the first and second dimension of the 3D
///        view over the 1D array_view.
///
/// @see   test_linearized_dof_grp_slice_wf
//////////////////////////////////////////////////////////////////////
bool TestDriver::test_linearized_dof_grp_slice(void)
{
  auto dof_grp_vw_ref = make_slices_view<0,1>(*m_mavw_ref);

  bool passed = true;
  m_controller.reset();

  while (m_controller.iterate())
  {
    iota(*m_vw, 0);
    iota(*m_vw_ref, 0);

    m_timer.reset();
    m_timer.start();

    // Test the map_reduce over two linearized views of the same original shape.
    passed &= map_reduce(
      test_linearized_dof_grp_slice_wf(), stapl::logical_and<bool>(),
      linear_view(*m_dof_grp_vw), linear_view(dof_grp_vw_ref));

    // Test the map_reduce over a linearized view, when different types of
    // views are passed to the map operation.
    passed &= map_reduce(
      test_linearized_dof_grp_slice_wf(), stapl::logical_and<bool>(),
      linear_view(*m_dof_grp_vw), counting_view(m_ndof*m_ngrp, 0),
      make_repeat_view(*m_vw_ref));

    m_timer.stop();
    m_controller.push_back(m_timer.value());
  }

  report(passed, "linearized dof-grp slices map_reduce");

  return passed;
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4) {
    do_once([argv]{
      std::cout << "usage: <exec_cmd>" << argv[0]
                << "ndof ngrp nmom [num_cont_it=1]\n";
    });

    return EXIT_FAILURE;
  }

  const size_t ndof = atol(argv[1]);
  const size_t ngrp = atol(argv[2]);
  const size_t nmom = atol(argv[3]);
  const int num_cont_it = argc >= 5 ? atoi(argv[4]) : 1;

  bool passed = true;

  TestDriver t(num_cont_it, ndof, ngrp, nmom);

  passed &= t.test_traversal();


  passed &= t.test_copy();
  passed &= t.test_dof_slice();
  passed &= t.test_mom_slice();
  passed &= t.test_dof_grp_slice();
  passed &= t.test_grp_mom_slice();
  passed &= t.test_fixed_mom_slice();
  passed &= t.test_fixed_grp_mom_slice();
  passed &= t.test_linearized_dof_grp_slice();

  if (!passed)
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
