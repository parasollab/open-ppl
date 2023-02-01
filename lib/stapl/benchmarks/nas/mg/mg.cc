/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <iomanip>

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/utility/tuple.hpp>

#include "mg.hpp"

using namespace stapl;

struct problem_traits
{
  int    dim;          /// Number of cells in the x, y, and z dimensions.
  int    nit;          /// Number of V-cycle iterations to perform.
  double verify_value; /// Verification value to compare result against.

  problem_traits(int size, int it, double verify)
    : dim(size), nit(it), verify_value(verify)
  { }
};


const static problem_traits s_traits(  32,  4, 0.5307707005734E-04);
const static problem_traits w_traits( 128,  4, 0.6467329375339E-05);
const static problem_traits a_traits( 256,  4, 0.2433365309069E-05);
const static problem_traits b_traits( 256, 20, 0.1800564401355E-05);
const static problem_traits c_traits( 512, 20, 0.5706732285740E-06);
const static problem_traits d_traits(1024, 50, 0.1583275060440E-09);
const static problem_traits e_traits(2048, 50, 0.8157592357404E-10);


problem_traits get_problem_traits(std::string const& problem_class)
{
  if (problem_class == "s") return s_traits;
  if (problem_class == "w") return w_traits;
  if (problem_class == "a") return a_traits;
  if (problem_class == "b") return b_traits;
  if (problem_class == "c") return c_traits;
  if (problem_class == "d") return d_traits;
  if (problem_class == "e") return e_traits;

  std::cerr << "Unknown problem class " << problem_class << "\n";
  std::exit(1);
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << argv[0] << " <problem class> (s, w, a, b, c, d, e)\n";
    return EXIT_FAILURE;
  }

  // get problem traits
  const problem_traits traits = get_problem_traits(argv[1]);
  const std::size_t dim = traits.dim;

  typedef stapl::multiarray<3, double>        grid_type;
  typedef stapl::multiarray_view<grid_type>   view_type;
  typedef std::vector<view_type>              multi_grid_views_type;

  // v is the fine grained grid.
  view_type v(new grid_type(make_tuple(dim, dim, dim)));

  // compute random values for the right hand side
  zran3(v);


  // call multigrid method
  auto grids = mg(v, traits.nit);

  view_type solution = get<0>(grids);
  view_type residual = get<1>(grids);

  // compute how close the norm of the residual is to the verification value
  const double epsilon      = 1e-08;
  const double err          =
    std::abs(norm2u3(residual) - traits.verify_value) / traits.verify_value;

  stapl::do_once([&](){
    std::cout << "Error: " << err << std::endl;
    if (err <= epsilon)
      std::cout << "VERIFICATION SUCCESSFUL\n";
    else
      std::cout << "VERIFICATION FAILED\n";
  });

  return EXIT_SUCCESS;
}
