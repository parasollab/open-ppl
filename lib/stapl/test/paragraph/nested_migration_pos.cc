/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/skeletons/explicit/map_prototype.hpp>
using namespace stapl;


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
};


// Top Level View Type
struct tester_view
  : public stapl::p_object
{
  pointer_wrapper<tester_container> m_ct;

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
    return m_ct->m_n_components * get_num_locations();
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
};


typedef array<int>        ct_t;
typedef array_view<ct_t> vw_t;


struct increment
{
  void operator()(int& val) const
  {
    ++val;
  }
};


struct inner_nest_wf
{
  pointer_wrapper<ct_t> m_ct;

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
  tester_view const*  m_view_ptr;
  inner_nest_wf       m_wf;

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
  void operator()(Reference) const
  {
    stapl::prototype::map_func<false>(m_wf, *m_view_ptr);
  }
};


stapl::exit_code stapl_main(int, char*[])
{
  using stapl::bind1st;
  using stapl::not_equal_to;
  using stapl::map_func;
  using stapl::fill;
  using stapl::count_if;

  const size_t n_locs = stapl::get_num_locations();

  stapl::do_once([&] {
    std::cout << "paragraph nested migration positive with "
              << n_locs << " locations... ";
  });

  const int nelems = stapl::get_num_locations();

  ct_t ct(nelems);
  vw_t vw(ct);
  fill(vw, 0);

  tester_container tc(nelems);
  tester_view t_view(&tc);

  for (std::size_t i = 0; i != 10; ++i)
    prototype::map_func(outer_nest_wf(t_view, inner_nest_wf(ct)), vw);

  stapl::get_executor()(stapl::execute_all);

  const size_t cnt
    = count_if(vw, bind1st(not_equal_to<int>(), nelems*10));

  stapl::do_once([&] {
   if (cnt == 0)
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  });

  return EXIT_SUCCESS;
}
