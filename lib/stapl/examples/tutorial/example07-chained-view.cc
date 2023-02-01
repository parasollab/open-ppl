/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/numeric.hpp>
#include <stapl/vector.hpp>
#include <stapl/array.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/utility/do_once.hpp>

// Mapping function to transform view indices to container gids.
struct even_index
{
  // Define the types required to reference a specific container gid.
  typedef size_t index_type;
  typedef size_t gid_type;

  // Map the index "idx" from the view domain to a gid in the domain of the
  // underlying container.
  gid_type operator()(index_type idx) const
  {
    return idx*2;
  }
};

// Mapping function to transform view indices to container gids.
struct odd_index
{
  // Define the types required to reference a specific container gid.
  typedef size_t index_type;
  typedef size_t gid_type;

  // Map the index "idx" from the view domain to a gid in the domain of the
  // underlying container.
  gid_type operator()(index_type idx) const
  {
    return idx*2 + 1;
  }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Declares the type of domain used in the code.
  using domain_type = stapl::indexed_domain<size_t>;

  // Declares the types of views used in the code. Note the chain of views.
  using vector_type = stapl::vector<double>;
  using vector_view_type = stapl::vector_view<vector_type>;
  using arr_view_even_type = stapl::array_view<vector_view_type,
                                               domain_type,
                                               even_index >;
  using arr_view_odd_type = stapl::array_view<vector_view_type,
                                              domain_type,
                                              odd_index >;

  // Declare the domain of elements that will be used for an array_view.
  domain_type dom(5);

  // Declares a stapl::vector of doubles of size 10.
  vector_type double_vector(10);

  // Declares a stapl::vector_view over a stapl::vector.
  vector_view_type vector_view(double_vector);

  // Declares a stapl::array_view with domain [0, 4] over a stapl::vector_view
  // that is referencing a stapl::vector container with domain [0, 9]. Note
  // the use of the function "even_index" to map the domain [0, 4] of the
  // array_view to the elements with even indexes in the vector view, i.e.,
  // the elements with indexes 0,2,4,6 and 8.
  arr_view_even_type a_view(vector_view, dom, even_index());

  // Declares a stapl::array_view with domain [0, 4] over a stapl::vector_view
  // that is referencing a stapl::vector container with domain [0, 9]. Note
  // the use of the function "odd_index" to map the domain [0, 4] of the
  // array_view to the elements with odd indexes in the vector view, i.e.,
  // the elements with indexes 1,3,5,7 and 9.
  arr_view_odd_type b_view(vector_view, dom, odd_index());

  // Initialize the elements referenced by the views. The function
  // "stapl::iota" requires a view, an initial value and that the view's
  // element type is numeric. This function initializes the elements of the
  // view such that the first element is assigned "value", the next element
  // "value"+1, etc.
  stapl::iota(a_view, 0.5);
  stapl::iota(b_view, 10);

  // Prints the current state of double_vector, a_view and b_view after
  // initialize the elements of a_view and b_view.
  stapl::do_once(
    [&]( ){
      std::cout << "--Printing the values stored in double_vector after\n"
                << "  initialize the values of a_view and b_view:\n";

      std::cout << "Current state of the Container:" << std::endl;
      for(unsigned int i = 0; i < double_vector.size(); ++i)
      {
        std::cout << "Vector[" << i << "] = " << double_vector.get_element(i)
                  << std::endl;
      }
      std::cout << std::endl;

      std::cout << "Current state of 'a_view':" << std::endl;
      for(unsigned int i = 0; i < a_view.size(); ++i)
      {
        std::cout << "A_View[" << i << "] = " << a_view[i]
                  << std::endl;
      }
      std::cout << std::endl;

      std::cout << "Current state of 'b_view':" << std::endl;
      for(unsigned int i = 0; i < b_view.size(); ++i)
      {
        std::cout << "B_View[" << i << "] = " << b_view[i]
                  << std::endl;
      }
      std::cout << std::endl;
    }
  );

  return EXIT_SUCCESS;
}
