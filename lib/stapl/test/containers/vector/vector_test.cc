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
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/utility/do_once.hpp>

using namespace std;
typedef stapl::vector<int> vec_type;
typedef stapl::vector_view<vec_type> vec_view_type;
int test_func(vec_view_type const& view);

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
  vec_type vec1(5);
  vec_view_type vec_view1(vec1);
  stapl::generate(vec_view1, stapl::sequence<int>(0,1));
  int result1 = test_func(vec_view1);
  int result2 = (vec1.size()-1)*vec1.size()/2;
  if (result1==result2)  {
    stapl::do_once( msg( "Test for vector const correctness PASSED" ) );
  }  else {
    stapl::do_once( msg( "Test for vector const correctness FAILED" ) );
  }
  return EXIT_SUCCESS;
}


int test_func(vec_view_type const& view)
{
  return stapl::accumulate(view,0);
}
