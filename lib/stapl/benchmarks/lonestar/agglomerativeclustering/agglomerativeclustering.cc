/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <iostream>
#include <sstream>
#include <map>

#include <stapl/utility/do_once.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include "agglomerativeclustering.hpp"
#include "cluster_testing.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Generate a random point with a position between 0 and 1
///        in each dimension.
//////////////////////////////////////////////////////////////////////
struct random_point
{
  typedef my_point result_type;
  typedef std::size_t index_type;

  std::size_t m_seed;

  random_point(std::size_t seed = get_location_id())
    : m_seed(seed)
  { }

  result_type operator()(index_type const& idx) const
  {
    typedef boost::mt19937 engine_type;
    typedef boost::random::uniform_real_distribution<double> dist_type;
    typedef boost::variate_generator<engine_type, dist_type> generator_type;

    engine_type eng(idx*m_seed + idx);

    generator_type pos_gen(eng, dist_type(0, 1));

    my_point point;
    point.m_x = pos_gen();
    point.m_y = pos_gen();
    point.m_z = pos_gen();
    return point;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_seed);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Fills an array with randomly generated points
//////////////////////////////////////////////////////////////////////
template <typename ArrayView>
void fill_with_random_points(ArrayView& av, size_t seed = get_location_id())
{
  using random_view_type =
    typename functor_view_type<functor_container<random_point, 1>>::type;

  // create a view of randomly generated points
  random_view_type random_points = functor_view(av.size(), random_point(seed));

  // populate the point view with random points
  copy(random_points, av);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef array<my_point> p_array_points;
  typedef array_view<p_array_points> parrayClustersView;
  typedef agglomerative_clustering::return_type clusters_type;

  int num_elem = (argc > 1) ? atoi(argv[1]) : 100;
  bool testdata = (argc > 2) ? atoi(argv[2]) : false;

  do_once([num_elem](void) {
    std::cerr << "Number of elements: " << num_elem << std::endl;
  });

  p_array_points p_array_test(num_elem);
  parrayClustersView p_view_test(p_array_test);

  if (!testdata) {
    fill_with_random_points(p_view_test);
  } else {
    fill_with_test_points(p_view_test);
  }

  stapl::counter<stapl::default_timer> t;
  t.start();
  agglomerative_clustering clustering;
  clusters_type clusters = clustering.cluster(p_view_test);
  t.stop();

  do_once([&t](void) {
    std::cerr << "TOTAL TIME ALGO: " << double(t.value()) << std::endl;
  });

  if (testdata) {
    cluster_testing(clusters);
  }

  #ifdef SHOW_RESULTS
  clustering.printgraph2dotfile(clusters);
  #endif
  return EXIT_SUCCESS;
}
