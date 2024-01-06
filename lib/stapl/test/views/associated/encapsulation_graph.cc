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
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>

#include <stapl/views/associated/associated_view.hpp>
#include <stapl/views/associated/simple_tracker.hpp>

#include "../../test_report.hpp"


using namespace stapl;

struct verify_mapping
{
  typedef bool result_type;

  template<typename T, typename V>
  bool operator()(T x, V view)
  {
    typedef typename V::follower_type::iterator iterator;

    typename V::follower_type associated = view.associated(index_of(x));

    std::vector<size_t> ground = x.property();
    std::vector<size_t> inferred;

    for (iterator it = associated.begin(); it != associated.end(); ++it)
      inferred.push_back(index_of(*it));

    std::sort(ground.begin(), ground.end());
    std::sort(inferred.begin(), inferred.end());

    return std::equal(ground.begin(), ground.end(), inferred.begin());
  }
};


struct generator
{
  typedef void result_type;

  template<typename V, typename View, typename T>
  void operator()(V region, View roadmap, T samples)
  {
    srand(region.descriptor());
    size_t s = samples;

    for (size_t i = 0; i < s; ++i)
    {
      if (valid(i))
      {
        size_t sample = roadmap.add_vertex(static_cast<double>(rand()));
        region.property().push_back(sample);
      }
    }
  }

  bool valid(size_t i)
  {
    return rand() % 10 > 6;
  }
};


struct connect
{
  typedef void result_type;

  template<typename V, typename R, typename View>
  void operator()(V region, R regions, View roadmap)
  {
    typedef typename View::edge_descriptor  edge_descriptor;
    typedef std::vector<size_t>             samples_type;
    typedef typename samples_type::iterator sample_iterator;

    samples_type samples = region.property();

    for (sample_iterator i = samples.begin(); i != samples.end(); ++i)
    {
      for (sample_iterator j = samples.begin(); j != samples.end(); ++j)
      {
        typename View::vertex_reference u = roadmap[*i];
        typename View::vertex_reference v = roadmap[*j];

        regions.track(index_of(region), index_of(u));
        regions.track(index_of(region), index_of(v));

        if (connectable(u, v))
          roadmap.add_edge_async(
            edge_descriptor(u.descriptor(), v.descriptor()));
      }
    }
  }

  template<typename V>
  bool connectable(V x, V y)
  {
    double z = std::pow(x.property(), y.property());
    return static_cast<int>(floor(z)) % 2;
  }
};


template<typename V0, typename V1>
void generate_samples(V0 const& regions, V1 const& roadmap, size_t m)
{
  map_func(generator(), regions, make_repeat_view(roadmap),
           make_repeat_view(m));
}


template<typename V0, typename V1>
void connect_samples(V0 const& regions, V1 const& roadmap)
{
  map_func(connect(), regions, make_repeat_view(regions),
           make_repeat_view(roadmap));
}


template<typename V>
bool correct_association(V const& regions)
{
  return map_reduce(verify_mapping(), stapl::logical_and<bool>(),
    regions, make_repeat_view(regions));
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " regions sample_count" << std::endl;
    exit(1);
  }

  size_t n  = atoi(argv[1]);
  size_t m  = atoi(argv[2]);

  typedef std::vector<size_t>                                 r_type;
  typedef dynamic_graph<DIRECTED, MULTIEDGES, r_type>         region_type;
  typedef dynamic_graph<DIRECTED, MULTIEDGES, double>         roadmap_type;
  typedef graph_view<region_type>                             region_view_type;
  typedef graph_view<roadmap_type>                            roadmap_view_type;
  typedef simple_tracker<region_view_type, roadmap_view_type> mapping_type;
  typedef associated_view<region_view_type,
    roadmap_view_type, mapping_type>                          association_type;

  region_type g(n);
  roadmap_type h;

  region_view_type rg(g);
  roadmap_view_type roadmap(h);

  mapping_type samples_in_region(rg);
  association_type regions(rg, roadmap, samples_in_region);

  generate_samples(regions, roadmap, m);

  connect_samples(regions, roadmap);

  bool passed = correct_association(regions);

  STAPL_TEST_REPORT(passed,
                    "Encapsulation association for graphs using tracking");

  return EXIT_SUCCESS;
}
