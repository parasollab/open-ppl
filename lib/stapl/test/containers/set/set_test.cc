/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/containers/set/set.hpp>
#include <stapl/views/set_view.hpp>
#include <stapl/utility/do_once.hpp>

using namespace std;
typedef stapl::set<size_t> st_type;
typedef stapl::set_view<st_type> st_view_type;
int test_func(st_view_type const& view);


struct insert_wf
{
  typedef void result_type;
  template <typename Index, typename View>
  result_type operator()(Index i, View v)
  {
    v.insert(i);
  }
};

struct msg
{
  private:
  const char* m_txt;
  public:
  msg(const char* text)
    : m_txt(text)
    { }

  typedef void result_type;
  result_type operator() () {
    cout << m_txt << endl;
  }
  void define_type(stapl::typer& t) {
    t.member(m_txt);
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  st_type st1;
  st_view_type st_view1(st1);
  stapl::map_func(insert_wf(),stapl::counting_view<size_t>(5),
    stapl::make_repeat_view(st_view1));
  st_view_type st_view2(st1);
  int result1 = test_func(st_view2);
  int result2 = (st1.size()-1)*st1.size()/2;
  if (result1==result2)  {
    stapl::do_once( msg( "Test for set const correctness PASSED" ) );
  }  else {
    stapl::do_once( msg( "Test for set const correctness FAILED" ) );
  }
  return EXIT_SUCCESS;
}


int test_func(st_view_type const& view)
{
  return stapl::accumulate(view,0);
}

