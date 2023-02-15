/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/views/repeated_view.hpp>

#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include "barnes_hut.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Generate a random particle with a position between 0 and 10
///        in each dimension and a mass between 0 and 50.
//////////////////////////////////////////////////////////////////////
struct random_particle
{
  typedef particle result_type;
  typedef std::size_t index_type;

  std::size_t m_seed;

  random_particle(std::size_t seed)
    : m_seed(seed)
  { }

  result_type operator()(index_type const& idx) const
  {
    typedef boost::mt19937 engine_type;
    typedef boost::random::uniform_real_distribution<double> dist_type;
    typedef boost::variate_generator<engine_type, dist_type> generator_type;

    engine_type eng(idx*m_seed + idx);

    generator_type pos_gen(eng, dist_type(0, 10));
    generator_type mass_gen(eng, dist_type(0, 50));

    double x = pos_gen();
    double y = pos_gen();
    double z = pos_gen();
    double m = mass_gen();

    return particle(point(x, y, z), m);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_seed);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 5)
  {
    std::cout << "usage: " << argv[0] << " n seed steps dt" << std::endl;
    exit(1);
  }

  const std::size_t n = atoi(argv[1]);
  const std::size_t seed = atoi(argv[2]);
  const std::size_t steps = atoi(argv[3]);
  const double dt = atof(argv[4]);

  using container_type   = array<particle>;
  using view_type        = array_view<container_type>;
  using random_view_type =
    functor_view_type<functor_container<random_particle, 1>>::type;

  // create container of particles and storage for a copy
  container_type particles(n);
  container_type original(n);

  view_type particle_view(particles);
  view_type original_particles(original);

  // create a view of randomly generated particles
  random_view_type random_particles = functor_view(n, random_particle(seed));

  // populate the particle view with random particles
  copy(random_particles, particle_view);

  // save a copy of the particles to compare with later
  copy(particle_view, original_particles);

  // compute forces between particles
  barnes_hut(particle_view, steps, dt);

  // make sure that at least one of the particles moved from
  // their original position
  bool any_moved = !stapl::equal(particle_view, original_particles);

  do_once([&]() {
    std::cout << "Testing Barnes-Hut: ";
    if (any_moved)
      std::cout << "Passed" << std::endl;
    else
      std::cout << "Failed" << std::endl;
  });

  return EXIT_SUCCESS;
}
