/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <string>

#include "paragraph_test_patterns.hpp"

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

template<typename Pattern, typename ViewIn>
struct compute_result_wf
{
  Pattern m_pattern;
  ViewIn  m_vw_in;

  typedef void result_type;

  compute_result_wf(Pattern const& pattern, ViewIn const& vw_in)
    : m_pattern(pattern),
      m_vw_in(vw_in)
  { }

 template<typename Reference1, typename Reference2>
 //void operator()(Reference1 const& tid, Reference2 const& result_val) const
 //void operator()(Reference1 const& tid, Reference2 & result_val) const
 void operator()(Reference1 tid, Reference2 result_val) const
 {
   result_val = m_vw_in[tid];
   // get the pred list for tid
   std::vector<size_t> pred_list = m_pattern.get_pred_list(tid);
   for (unsigned int j=0; j<pred_list.size(); ++j)
     result_val = result_val + m_vw_in[pred_list[j]];
 }

  void define_type(stapl::typer& t)
  {
    t.member(m_pattern);
    t.member(m_vw_in);
  }
};


template<typename ViewIn, typename ViewOut>
struct add_wf
{
  typedef int result_type;

  ViewIn   m_vw_in;
  ViewOut  m_vw_out;
  size_t   m_tid;

  add_wf(ViewIn& vw_in, ViewOut& vw_out, size_t tid)
    : m_vw_in(vw_in),
      m_vw_out(vw_out),
      m_tid(tid)
  { }

  // no preds
  result_type operator()() const
  {
    m_vw_out[m_tid] = m_vw_in[m_tid];
    return m_vw_in[m_tid];
  }

  // one pred
  template<typename Reference>
  result_type operator()(Reference const& ref) const
  {
    m_vw_out[m_tid] = m_vw_in[m_tid] + ref;
    return m_vw_in[m_tid];
  }

  // two preds
  template<typename Reference1, typename Reference2>
  result_type operator()(Reference1 const& ref1, Reference2 const& ref2) const
  {
    m_vw_out[m_tid] = m_vw_in[m_tid] + ref1 + ref2;
    return m_vw_in[m_tid];
  }

  // three preds
  template<typename Reference1, typename Reference2, typename Reference3>
  result_type operator()(Reference1 const& ref1, Reference2 const& ref2,
                         Reference3 const& ref3) const
  {
    m_vw_out[m_tid] = m_vw_in[m_tid] + ref1 + ref2 + ref3;
    return m_vw_in[m_tid];
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_vw_in);
    t.member(m_vw_out);
    t.member(m_tid);
  }
};


struct pattern_tester
{
  template<typename Pattern>
  void test_pattern(Pattern const& pattern, std::string name) const
  {
    // test for data flow of values
    //
    if (stapl::get_location_id() == 0)
      std::cout << "Testing Pattern " << name << " for delayed num succs ...";

    typedef stapl::array<int>        cnt_t;
    typedef stapl::array_view<cnt_t> view_t;

    int num_elems = pattern.num_elems();
    cnt_t  ct_in(num_elems);
    cnt_t  ct_out(num_elems);
    view_t vw_in(ct_in);
    view_t vw_out(ct_out);

    stapl::iota(vw_in, 0);
    stapl::fill(vw_out, 0);

    // prepare the result container and view
    // each task with tid returns value vw_in[tid]
    // each task outputs vw_out[tid] = vw_in[tid] + vw_in[pred0] +
    //   vw_in[pred1] + vw_in[pred2]
    // a maximum of three predecessors are supported for this test
    //
    cnt_t  ct_result(num_elems);
    view_t vw_result(ct_result);

    stapl::map_func(
      compute_result_wf<Pattern, view_t>(pattern, vw_in),
      stapl::counting_view<int>(num_elems, 0),
      vw_result
    );

    // test the pattern
    typedef delayed_succs_factory_t<
      Pattern, add_wf<view_t, view_t>, view_t, view_t> val_factory_t;

    typedef stapl::paragraph<
      stapl::persistent_scheduler, val_factory_t>         val_tg_t;

    val_tg_t ppg(val_factory_t(pattern, vw_in, vw_out));

    ppg();

    // compare the result and the output view
    bool result = stapl::equal(vw_out, vw_result);

    ppg();

    // compare the result and the output view
    result = result && stapl::equal(vw_out, vw_result);

    if (result)
    {
      if (stapl::get_location_id() == 0)
        std::cout << "Passed" << std::endl;
    }
    else
    {
      if (stapl::get_location_id() == 0)
        std::cout << "Failed" << std::endl;
    }
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using std::cout;
  using std::endl;

  if (stapl::get_location_id() == 0)
  {
    cout << "Testing paragraph for delayed set num succs on "
         << stapl::get_num_locations() << " locations...";
    cout << endl;
  }

  int rows           = 4;
  int cols_per_proc  = 1;

  if (argc>=2)
    rows = atoi(argv[1]);

  if (argc>=3)
  {
    stapl_assert(false, "varying cols_per_proc parameter not supported yet");
    cols_per_proc         = atoi(argv[2]);
  }

  int cols           = cols_per_proc * stapl::get_num_locations();

  // create a pattern tester to test different patterns
  pattern_tester  tester;

  // test basic patterns
  typedef basic_pattern_one<>   p_one_t;
  typedef basic_pattern_two<>   p_two_t;
  typedef basic_pattern_three<> p_three_t;
  typedef basic_pattern_four<>  p_four_t;
  typedef basic_pattern_five<>  p_five_t;

  tester.test_pattern(p_one_t(rows, cols), "basic_pattern_one");
  tester.test_pattern(p_two_t(rows, cols), "basic_pattern_two");
  tester.test_pattern(p_three_t(rows, cols), "basic_pattern_three");
  tester.test_pattern(p_four_t(rows, cols), "basic_pattern_four");
  tester.test_pattern(p_five_t(rows, cols), "basic_pattern_five");

  // test composed patterns
  //  COMPOSED_PATTERN_ONE
  //  <-- cols -->
  //  O-->O-->O-->O   |
  //  |   |   |   |   |
  //  v   v   v   v   |
  //  O-->O-->O-->O   | rows
  //  |   |   |   |   |
  //  v   v   v   v   |
  //  O-->O-->O-->O   v
  tester.test_pattern(composed_pattern<p_two_t, p_three_t>(
                        p_two_t(rows, cols), p_three_t(rows, cols)
                      ),
                      "composed_pattern_one");

  //  COMPOSED_PATTERN_TWO
  //  <-- cols -->
  //  O   O   O   O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   | rows
  //  O   O   O   O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   |
  //  O   O   O   O   v
  tester.test_pattern(composed_pattern<p_three_t, p_four_t>(
                        p_three_t(rows, cols), p_four_t(rows, cols)
                      ),
                      "composed_pattern_two");

  //  COMPOSED_PATTERN_THREE
  //  O-->O-->O-->O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   | rows
  //  O-->O-->O-->O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   |
  //  O-->O-->O-->O   v
  typedef composed_pattern<p_three_t, p_four_t> cp_three_four_t;
  cp_three_four_t cp_three_four(p_three_t(rows, cols), p_four_t(rows, cols));
  tester.test_pattern(composed_pattern<p_two_t, cp_three_four_t>(
                        p_two_t(rows, cols), cp_three_four
                      ),
                      "composed_pattern_three");

  //  COMPOSED_PATTERN_FOUR
  //  O  O  O  O  O   |
  //  |\/|\/|\/|\/|   |
  //  |/\|/\|/\|/\|   |
  //  v  v  v  v  v   | rows
  //  O  O  O  O  O   |
  //  |\/|\/|\/|\/|   |
  //  |/\|/\|/\|/\|   |
  //  v  v  v  v  v   |
  //  O  O  O  O  O   v
  tester.test_pattern(composed_pattern<cp_three_four_t, p_five_t>(
                        cp_three_four, p_five_t(rows, cols)
                      ),
                      "composed_pattern_four");

  return EXIT_SUCCESS;
}
