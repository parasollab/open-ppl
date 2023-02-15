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
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/utility/do_once.hpp>

using namespace std;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, size_t, size_t>
  grph_type;
typedef stapl::graph_view<grph_type> grph_view_type;
int test_func(grph_view_type const& view);


struct get_descriptor_wf
{
  typedef int result_type;
  template<typename View>
  result_type operator() (View v)
    {
      return v.descriptor();
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
  grph_type grph1(5);
  grph_view_type grph_view1(grph1);
  int result1 = test_func(grph_view1);
  int result2 = 10;
  if (result1==result2)  {
    stapl::do_once( msg( "Test for graph const correctness PASSED" ) );
  }  else {
     stapl::do_once( msg( "Test for graph const correctness FAILED" ) );
  }
  return EXIT_SUCCESS;
}


int test_func(grph_view_type const& view)
{
  return stapl::map_reduce(get_descriptor_wf(), stapl::plus<size_t>(), view);
}

