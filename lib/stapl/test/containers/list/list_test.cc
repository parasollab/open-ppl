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
#include <stapl/containers/list/list.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/utility/do_once.hpp>

using namespace std;
typedef stapl::list<size_t> lst_type;
typedef stapl::list_view<lst_type> lst_view_type;
int test_func(lst_view_type const& view);

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
  lst_type lst1(5);
  lst_view_type lst_view1(lst1);
  stapl::generate(lst_view1, stapl::sequence<int>(0,1));
  int result1 = test_func(lst_view1);
  int result2 = (lst1.size()-1)*lst1.size()/2;
 if (result1==result2)  {
    stapl::do_once( msg( "Test for list const correctness PASSED" ) );
  }  else {
    stapl::do_once( msg( "Test for list const correctness FAILED" ) );
  }
  return EXIT_SUCCESS;
}


int test_func(lst_view_type const& view)
{
  return stapl::accumulate(view,0);
}
