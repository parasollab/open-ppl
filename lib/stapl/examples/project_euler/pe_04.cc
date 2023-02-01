/*
// Copyright (c) 2015, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <string>
#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/runtime.hpp>

typedef long long t_type;
typedef stapl::array<t_type> ary_t_tp;
typedef stapl::array_view<ary_t_tp> ary_t_vw_tp;

// Checks if number is a palindrome
struct pal_check
{
  template<typename T>
  t_type operator()(T i)
  {
    std::string s = std::to_string(i);
    std::reverse(s.begin(),s.end());

    t_type j = std::stoll(s);

    if (i == j)
      return i;
    else
      return 0;
  }
};

// Work function that transforms an integer in the original
// sequence to a product of a pair of n-digit numbers.
// It replaces the first 10^n - 10^(n-1) items in the sequence
// with the products (offset+0)*(offset+0),(offset+1)*(offset+0), etc.
// and the next set of items with the products
// (offset+0)*(offset+1), (offset+1)*(offset+1) to n-digit numbers
struct create_product
{
  create_product(t_type divisor, t_type offset)
    :m_divisor(divisor), m_offset(offset)
  {}

  template<typename T>
  t_type operator()(T i)
  {
    return (i%m_divisor + m_offset)*(i/m_divisor + m_offset);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_divisor);
    t.member(m_offset);
  }

private:
  t_type m_divisor;
  t_type m_offset;
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Input n, where n is a positive integer
  // Program finds largest palindrome made from the
  // product of two n-digit numbers
  t_type arg = std::stoll(argv[1]);

  stapl::counter<stapl::default_timer> exec_timer;
  exec_timer.start();

  t_type elems;
  t_type divisor;
  t_type offset;
  t_type lgst_pal;

  // Determines divisor, offset, and number of elements
  divisor = 9 * std::pow(10, arg-1);
  offset = std::pow(10, arg-1);
  elems = std::pow(divisor, 2);

  // Sets divisor and offset according to input
  create_product cp(divisor,offset);

  // Array declarations
  ary_t_tp in(elems);
  ary_t_tp out(elems);

  // Array view declarations
  ary_t_vw_tp in_vw(in);
  ary_t_vw_tp out_vw(out);

  // Creates sequence from 1 to the size of the view
  stapl::iota(in_vw, 1);

  // Applies work function to input view
  // Stores the changed values in the output view
  stapl::transform(in_vw, out_vw, cp);

  // Applies work function that checks for palindromes on the output view
  // Then performs a reduction to find the largest palindrome
  // Returns the largest palindrome
  lgst_pal = stapl::map_reduce(pal_check(), stapl::max<t_type>(), out_vw);

  double t = exec_timer.stop();

  stapl::do_once([&]
  {
  std::cout << "GREATEST PALINDROME: " << lgst_pal << std::endl
            << "Computation finished (time taken: " << t << " s)"
            << std::endl;
  });

  return EXIT_SUCCESS;
}



