/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_DISPLAY
#define STAPL_BENCHMARK_LONESTAR_BH_DISPLAY

#include <stapl/runtime/runtime.hpp>

struct print_graph
{
  typedef void result_type;

  template <typename V>
  void operator()(V v)
  {
    point p = v.property().coord();
    double mass = v.property().mass();
    int pid = v.property().particle_id();

    if (v.property().is_leaf())
    {
      printf("leaf: coord=(%f, %f, %f), mass=%f, particle_id=%u, desc=%u\n",
        p.x, p.y, p.z, mass, pid, v.descriptor()
      );
    }

    else
    {
      printf("int: coord=(%f, %f, %f), mass=%f, particle_id=%u, desc=%u\n",
        p.x, p.y, p.z, mass, pid, v.descriptor()
      );

      for (auto&& e : v)
        if (e.property().first)
          printf(" -> child %lu\n", e.target());
    }
  }
};


struct print_dot
{
  typedef void result_type;

  template <typename V>
  void operator()(V v)
  {
    point p = v.property().coord();
    double mass = v.property().mass();
    std::size_t pid = v.property().particle_id();

    printf("v%lu [label=\"%f, %f, %f\\n %f \\n %lu\"]\n",
      v.descriptor(), p.x, p.y, p.z, mass, pid
    );

    if (!v.property().is_leaf())
    {
      for (auto&& e : v)
        if (e.property().first)
          printf("v%lu-> v%lu [label=\"%s\"]\n", v.descriptor(), e.target(),
            pretty_print(e.property().second).c_str()
          );
    }
  }

  std::string pretty_print(int i)
  {
    switch (i)
    {
      case 0: return "front bottom left";
      case 1: return "front bottom right";
      case 2: return "front top left";
      case 3: return "front top right";
      case 4: return "back bottom left";
      case 5: return "back bottom right";
      case 6: return "back top left";
      case 7: return "back top right";
      default: return "whaaaa";
    }
  }
};


struct print_simple
{
  typedef void result_type;

  template<typename T>
  void operator()(T x)
  {
    point p = x.coord();
    double mass = x.mass();

      printf("body %lu: (%f, %f, %f), mass = %f\n", stapl::index_of(x),
        p.x, p.y, p.z, mass
      );
  }
};


template<typename ParticleView, typename OctreeView>
void display_octree(ParticleView& particle_view, OctreeView& oct)
{
#ifndef LONESTAR_BH_QUIET
  map_func(print_dot(), oct);
#endif
}


template<typename ParticleView>
void display_particles(ParticleView& particle_view)
{
#ifndef LONESTAR_BH_QUIET
  map_func(print_simple(), particle_view);
#endif
}


template<typename V>
void display_bodies(V const& v)
{
  if (stapl::get_location_id() == 0)
  {
    system("clear");
    const std::size_t screen_size = 40;

    std::vector<std::vector<std::size_t> > d(screen_size,
      std::vector<std::size_t>(screen_size)
    );

    const std::size_t n = v.size();

    for (std::size_t i = 0; i < screen_size; ++i)
      for (std::size_t j = 0; j < screen_size; ++j)
        d[i][j] = 0;

    for (std::size_t i = 0; i < n; ++i)
    {
      point pos = v[i].coord();

      std::size_t x = pos.x + screen_size/2;
      std::size_t y = pos.y + screen_size/2;

      if (x < screen_size && y << screen_size)
        d[x][y]++;
    }

    for (std::size_t i = 0; i < screen_size; ++i)
    {
      for (std::size_t j = 0; j < screen_size; ++j)
        if (d[i][j] == 0)
          std::cout << " ";
        else
          std::cout << d[i][j];
      std::cout << std::endl;
    }

    system("sleep 2");
    system("clear");

  }
  stapl::rmi_fence();
}

#endif
