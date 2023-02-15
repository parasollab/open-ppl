/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/utility/do_once.hpp>

using namespace std;
typedef stapl::multiarray<2, size_t> m_array_type;
typedef m_array_type::dimensions_type dimensions_type;
typedef stapl::multiarray_view<m_array_type> m_array_view_type;
template <typename View >
int test_func(View const& view);

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
  m_array_type m_array1 (dimensions_type(1,5));
  m_array_view_type m_array_view1(m_array1);
  stapl::generate(stapl::linear_view(m_array_view1),stapl::sequence<int>(0,1));
  int result1 = test_func(stapl::linear_view(m_array_view1));
  int result2 = 10;
  if (result1==result2)  {
    stapl::do_once( msg( "Test for multiarray const correctness PASSED" ) );
  }  else {
    stapl::do_once( msg( "Test for multiarray const correctness FAILED" ) );
  }
  return EXIT_SUCCESS;
}


template <typename View >
int test_func(View const& view)
{
  return stapl::accumulate(view,0);
}
