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
#include <stapl/containers/map/map.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/utility/do_once.hpp>

using namespace std;
typedef stapl::map<size_t, size_t> mp_type;
typedef stapl::map_view<mp_type> mp_view_type;
int test_func(mp_view_type const& view);


struct insert_wf
{
  typedef void result_type;
  template <typename Index, typename View>
  result_type operator()(Index i, View v)
  {
    v.insert(i,i+1);
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

struct get_second_wf
{
  typedef int result_type;
  template<typename View>
  result_type operator() (View v)
  {
    return v.second;
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  mp_type mp1;
  mp_view_type mp_view1(mp1);
  stapl::map_func(insert_wf(),stapl::counting_view<size_t>(5),
  stapl::make_repeat_view(mp_view1));
  mp_view_type mp_view2(mp1);
  int result1 = test_func(mp_view2);
  int result2 = 15;
  if (result1==result2)  {
    stapl::do_once( msg( "Test for map const correctness PASSED" ) );
  }  else {
    stapl::do_once( msg( "Test for map const correctness FAILED" ) );
  }
  return EXIT_SUCCESS;
}


int test_func(mp_view_type const& view)
{
  return stapl::map_reduce(get_second_wf(), stapl::plus<size_t>(), view);
}

