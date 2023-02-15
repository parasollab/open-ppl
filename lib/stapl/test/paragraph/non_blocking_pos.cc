/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>

#include <iostream>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm_prototype.hpp>
#include <stapl/skeletons/explicit/map_prototype.hpp>

using stapl::typer;
using stapl::location_type;
using stapl::loc_qual;
using stapl::get_location_id;
using stapl::get_num_locations;
using stapl::get_affinity;
using stapl::affinity_tag;
using stapl::locality_info;


// Old View Require a object on which get_id()
// is called to get component id
struct component_holder
{
  size_t m_val;

  component_holder(size_t val)
    : m_val(val)
  { }

  size_t get_id() const
  {
    return m_val;
  }
};


// SubView/Element Type
struct tester_element
{
  size_t m_idx;

  typedef tester_element fast_view_type;

  tester_element(size_t idx)
    : m_idx(idx)
  { }

  bool is_local() const
  {
    return true;
  }

  void pre_execute() const
  { }

  void post_execute() const
  { }

  size_t elem() const
  {
    return m_idx;
  }

  void define_type(typer& t)
  {
    t.member(m_idx);
  }
};


struct tester_container
  : public stapl::p_object
{
  size_t m_n_components;

  tester_container(size_t val)
    : m_n_components(val)
  { }

  size_t version(void) const
  {
    return 0;
  }
};


// Top Level View Type
struct tester_view
  : public stapl::p_object
{
  stapl::pointer_wrapper<tester_container> m_ct;

  tester_view(tester_container* ct)
    : m_ct(ct)
  { }

  void define_type(typer& t)
  {
    t.member(m_ct);
  }

  typedef size_t          cid_type;
  typedef tester_element  subview_type;

  size_t get_num_local_subviews() const
  {
    return m_ct->m_n_components;
  }

  component_holder
  get_local_component(size_t idx) const
  {
    return component_holder(idx);
  }

  tester_element get_subview(cid_type idx) const
  {
    return tester_element(idx);
  }

  size_t get_num_subviews() const
  {
    return m_ct->m_n_components * m_ct->get_num_locations();
  }

  void pre_execute() const
  { }

  void post_execute() const
  { }

  locality_info locality(size_t idx)
  {
    return locality_info(
      stapl::LQ_CERTAIN, stapl::invalid_affinity_tag,
      m_ct->get_rmi_handle(), idx
    );
  }

  size_t version(void) const
  {
    return 0;
  }

  bool validate(void) const
  {
    return true;
  }

  size_t size()
  {
    return m_ct->m_n_components * m_ct->get_num_locations();
  }
};


typedef stapl::array<size_t> ct_t;


struct increment
{
  void operator()(size_t& val) const
  {
    ++val;
  }
};


struct inner_nest_wf
{
  stapl::pointer_wrapper<ct_t> m_ct;

  void define_type(typer& t)
  {
    t.member(m_ct);
  }

  inner_nest_wf(ct_t& ct)
    : m_ct(&ct)
  { }

  typedef void result_type;

  template<typename View>
  void operator()(View vw) const
  {
    stapl_assert(m_ct->distribution().is_local(vw.elem()),
      "inner_nest_wf executed in wrong loc");

    m_ct->apply_set(vw.elem(), increment());
  }
};


struct outer_nest_wf
{
  tester_view const*   m_view_ptr;
  inner_nest_wf        m_wf;

  outer_nest_wf(tester_view const& view, inner_nest_wf const& wf)
    : m_view_ptr(&view), m_wf(wf)
  { }

  void define_type(typer& t)
  {
    t.member(m_view_ptr);
    t.member(m_wf);
  }

  typedef void result_type;

  template<typename Reference>
  void operator()(Reference elem) const
  {
    stapl::prototype::map_func<false>(m_wf, *m_view_ptr);
  }
};


stapl::exit_code stapl_main(int, char*[])
{
  typedef stapl::array_view<ct_t>                               vw_t;
  typedef stapl::prototype::result_of::count<vw_t, int>::type   res_t;

  if (get_location_id() == 0)
    std::cout << "paragraph non blocking positive with "
              << get_num_locations() << " locations... ";

  const size_t nelems = get_num_locations();

  // Container and View Construction
  ct_t ct1(nelems); vw_t vw1(ct1);
  ct_t ct2(nelems); vw_t vw2(ct2);
  ct_t ct3(nelems); vw_t vw3(ct3);
  ct_t ct4(nelems); vw_t vw4(ct4);
  ct_t ct5(nelems); vw_t vw5(ct5);

  // Fill the containers;
  stapl::prototype::fill(vw1, 0);
  stapl::prototype::fill(vw2, 0);
  stapl::prototype::fill(vw3, 0);
  stapl::prototype::fill(vw4, 0);
  stapl::prototype::fill(vw5, 0);

  stapl::get_executor()(stapl::execute_all);

  tester_container tc(nelems);
  tester_view t_view(&tc);

  // Apply Fancy increment
  stapl::prototype::for_each(vw1, outer_nest_wf(t_view, inner_nest_wf(ct1)));
  stapl::prototype::for_each(vw2, outer_nest_wf(t_view, inner_nest_wf(ct2)));
  stapl::prototype::for_each(vw3, outer_nest_wf(t_view, inner_nest_wf(ct3)));
  stapl::prototype::for_each(vw4, outer_nest_wf(t_view, inner_nest_wf(ct4)));
  stapl::prototype::for_each(vw5, outer_nest_wf(t_view, inner_nest_wf(ct5)));

  stapl::get_executor()(stapl::execute_all);

  // Use count to verify results
  res_t ref5 = stapl::prototype::count(vw5, nelems);
  res_t ref4 = stapl::prototype::count(vw4, nelems);
  res_t ref3 = stapl::prototype::count(vw3, nelems);
  res_t ref2 = stapl::prototype::count(vw2, nelems);
  res_t ref1 = stapl::prototype::count(vw1, nelems);

  stapl::get_executor()(stapl::execute_all);

  size_t result = ref1 + ref2 + ref3 + ref4 + ref5;

  if (get_location_id() == 0)
  {
    if (result == nelems * 5)
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  }

  return EXIT_SUCCESS;
}
