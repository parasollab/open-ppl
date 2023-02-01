/*
// Copyright (c) 2000-2013, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/containers/generators/functor.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <iostream>
#include <sstream>
#include <utility>

#include "../test.h"

//#define DEBUG


#ifdef DEBUG
struct print_functor
{
  template<class T>
  void operator()(T value)
  {
    std::cerr << value.first << " " << value.second << std::endl;
  }
};
#endif


//Compare two pairs with < using their 'first' member
struct less_than_comparison_functor
{
  typedef bool result_type;

  //======================================

  bool operator()(
    std::pair<unsigned int, unsigned int> const& left,
    std::pair<unsigned int, unsigned int> const& right) const
  {
    return left.first < right.first;
  }
};


class test_view_initializer
{
  unsigned int m_index_1;
  unsigned int m_index_2;

  //allow access only through the static setup function
  test_view_initializer(unsigned int index_1, unsigned int index_2)
    : m_index_1(index_1), m_index_2(index_2)
  { }

public:
  typedef void result_type;

  //create an array of 'size' pairs of the form (2, index)
  // the elements at the given indicies, 'index_1' and 'index_2',
  // will instead be of the form (1, index)
  static void initialize
  (
    stapl::array_view< stapl::array< std::pair<unsigned int, unsigned int> > >
      &test_view,
    unsigned int index_1,
    unsigned int index_2
  )
  {
    if (test_view.size() < index_1 || test_view.size() < index_2)
    {
      std::cerr
        << "test_view_initializer: supplied view is not large enough"
        << std:: endl;
      return;
    }

    stapl::map_func
    (
      test_view_initializer(index_1, index_2),
      test_view,
      stapl::counting_view<unsigned int>(test_view.size())
    );
  }

  template <typename PairType, typename IndexType>
  void operator()(PairType pair, IndexType index)
  {
    pair = std::pair<unsigned int,unsigned int>
      ((index == m_index_1 || index == m_index_2) ? 1 : 2, index);
  }

  //======================================

  void define_type(stapl::typer& t)
  {
    t.member(m_index_1);
    t.member(m_index_2);
  }
};


//verify that the value returned by min_element is at index_1
bool test_min_elements_at(unsigned int index_1, unsigned int index_2)
{
  typedef std::pair<unsigned int, unsigned int> pair_type;
  typedef stapl::array<pair_type> array_type;
  typedef stapl::array_view<array_type> view_type;

  std::stringstream ss;
  ss << "First Min Element With Min At Indicies "
     << index_1 << " And " << index_2;
  test_result test(ss.str());

  //Create a container of pairs
  // The 'first' member of the pair is the value for comparison
  // The 'second' member of the pair is the index of the pair
  array_type test_array(100);
  view_type test_view(test_array);
  test_view_initializer::initialize(test_view, index_1, index_2);

  //get the minimum element
  pair_type minimum_element =
    stapl::min_element(view_type(test_array), less_than_comparison_functor());

  //test passes if the minimum element returned is the element
  // at the first index.
  test.set_result(minimum_element.second == index_1, true);

  //print the results
  test_report(test);
  //add the version to the output
  //test_report uses std::cerr for reporting
  std::cerr << "Version: stapl" << std::endl << std::endl;

#ifdef DEBUG
  stapl::for_each(view_type(test_array),print_functor());
#endif

  if (!test.get_result())
  {
    return false;
  }
  return true;
}


//Verify that the element returned by stapl::min_element is the first
// (left-most) element in the sequence.
stapl::exit_code stapl_main(int argc, char* argv[])
{
  bool result = true;

  //Boundary Cases:
  result &= test_min_elements_at(0,1);
  result &= test_min_elements_at(1,2);
  result &= test_min_elements_at(98,99);
  result &= test_min_elements_at(97,98);
  result &= test_min_elements_at(49,50);
  result &= test_min_elements_at(0,99);

  //typical Use Case:
  result &= test_min_elements_at(30,60);

  if (!result)
  {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
