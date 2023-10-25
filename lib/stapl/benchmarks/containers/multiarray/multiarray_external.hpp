/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BENCHMARKS_CONTAINERS_MULTIARRAY_EXTERNAL_HPP
#define BENCHMARKS_CONTAINERS_MULTIARRAY_EXTERNAL_HPP

#include <iostream>
#include <functional>

#include <Kripke/SubTVec.h>
#include <boost/multi_array.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

#ifdef KOKKOS_DEFINED
#include <Kokkos_Core.hpp>

typedef Kokkos::View<double *****, Kokkos::LayoutStride, Kokkos::Serial>
          kokkos_arr_t;
#endif

typedef stapl::counter<stapl::default_timer>  counter_t;
typedef boost::multi_array<double, 5>         boost_arr_t;
typedef boost::general_storage_order<5>       storage_t;


// populate function used by Boost and SubTVec
void populate(double* ptr, size_t size)
{
  for (size_t i = 0; i != size; ++ptr, ++i)
    *ptr = 0;
}

#ifdef KOKKOS_DEFINED
// populate function used by Kokkos multiarray
void populate(kokkos_arr_t* ptr)
{
  srand48(31415926530);

  for (size_t i = 0; i != ptr->dimension(0); ++i)
    for (size_t j = 0; j != ptr->dimension(1); ++j)
      for (size_t k = 0; k != ptr->dimension(2); ++k)
        for (size_t d = 0; d != ptr->dimension(3); ++d)
          for (size_t g = 0; g != ptr->dimension(4); ++g)
            (*ptr)(i, j, k, d, g) = 0;
}

template<Nesting_Order>
void traverse(kokkos_arr_t*, int, int, int, int, int)
{
  std::cerr << "Nesting order unrecognized for Kokkos array traversal.\n";
  exit(1);
}

template<>
void traverse<NEST_ZDG>(kokkos_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int k = 0; k != b5; ++k)
    for (int j = 0; j != b4; ++j)
      for (int i = 0; i != b3; ++i)
        for (int d = 0; d != b2; ++d)
          for (int g = 0; g != b1; ++g)
            (*bcontainer)(i, j, k, d, g) += 1.4142135624;
}

template<>
void traverse<NEST_ZGD>(kokkos_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int k = 0; k != b5; ++k)
    for (int j = 0; j != b4; ++j)
      for (int i = 0; i != b3; ++i)
        for (int g = 0; g != b1; ++g)
          for (int d = 0; d != b2; ++d)
            (*bcontainer)(i, j, k, d, g) += 1.4142135624;
}

template<>
void traverse<NEST_GZD>(kokkos_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int g = 0; g != b1; ++g)
    for (int k = 0; k != b5; ++k)
      for (int j = 0; j != b4; ++j)
        for (int i = 0; i != b3; ++i)
          for (int d = 0; d != b2; ++d)
            (*bcontainer)(i, j, k, d, g) += 1.4142135624;
}

template<>
void traverse<NEST_GDZ>(kokkos_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int g = 0; g != b1; ++g)
    for (int d = 0; d != b2; ++d)
      for (int k = 0; k != b5; ++k)
        for (int j = 0; j != b4; ++j)
          for (int i = 0; i != b3; ++i)
            (*bcontainer)(i, j, k, d, g) += 1.4142135624;
}

template<>
void traverse<NEST_DGZ>(kokkos_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int d = 0; d != b2; ++d)
    for (int g = 0; g != b1; ++g)
      for (int k = 0; k != b5; ++k)
        for (int j = 0; j != b4; ++j)
          for (int i = 0; i != b3; ++i)
            (*bcontainer)(i, j, k, d, g) += 1.4142135624;
}

template<>
void traverse<NEST_DZG>(kokkos_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int d = 0; d != b2; ++d)
    for (int k = 0; k != b5; ++k)
      for (int j = 0; j != b4; ++j)
        for (int i = 0; i != b3; ++i)
          for (int g = 0; g != b1; ++g)
            (*bcontainer)(i, j, k, d, g) += 1.4142135624;
}

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_kokkos(Nesting_Order actual_nest, int size1, int size2,
           int size3, int size4, int size5)
{
  int layout[5];
  size_t dims[] = {unsigned(size3), unsigned(size4),
    unsigned(size5), unsigned(size2), unsigned(size1)};

  switch(actual_nest)
  {
    case NEST_DGZ:
      layout[0] = 0;
      layout[1] = 1;
      layout[2] = 2;
      layout[3] = 4;
      layout[4] = 3;
      break;
    case NEST_DZG:
      layout[0] = 4;
      layout[1] = 0;
      layout[2] = 1;
      layout[3] = 2;
      layout[4] = 3;
      break;
    case NEST_GDZ:
      layout[0] = 0;
      layout[1] = 1;
      layout[2] = 2;
      layout[3] = 3;
      layout[4] = 4;
      break;
    case NEST_GZD:
      layout[0] = 3;
      layout[1] = 0;
      layout[2] = 1;
      layout[3] = 2;
      layout[4] = 4;
      break;
    case NEST_ZDG:
      layout[0] = 4;
      layout[1] = 3;
      layout[2] = 0;
      layout[3] = 1;
      layout[4] = 2;
      break;
    case NEST_ZGD:
      layout[0] = 3;
      layout[1] = 4;
      layout[2] = 0;
      layout[3] = 1;
      layout[4] = 2;
      break;
  }

  Kokkos::LayoutStride stride
    = Kokkos::LayoutStride::order_dimensions(5, layout, dims);

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(actual_nest);

  // Construction
  timer.start();
  kokkos_arr_t* bc = new kokkos_arr_t("kokko_arr", stride);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(bc);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  traverse<Nest>(bc, size1, size2, size3, size4, size5);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete bc;
  std::get<4>(times) = timer.stop();

  return times;
}
#endif

template<Nesting_Order>
void traverse(boost_arr_t*, int, int, int, int, int)
{
  std::cerr << "Nesting order unrecognized for Boost array traversal.\n";
  exit(1);
}

template<>
void traverse<NEST_ZDG>(boost_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int k = 0; k != b5; ++k)
    for (int j = 0; j != b4; ++j)
      for (int i = 0; i != b3; ++i)
        for (int d = 0; d != b2; ++d)
          for (int g = 0; g != b1; ++g)
            (*bcontainer)[i][j][k][d][g] += 1.4142135624;
}

template<>
void traverse<NEST_ZGD>(boost_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int k = 0; k != b5; ++k)
    for (int j = 0; j != b4; ++j)
      for (int i = 0; i != b3; ++i)
        for (int g = 0; g != b1; ++g)
          for (int d = 0; d != b2; ++d)
            (*bcontainer)[i][j][k][d][g] += 1.4142135624;
}

template<>
void traverse<NEST_GZD>(boost_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int g = 0; g != b1; ++g)
    for (int k = 0; k != b5; ++k)
      for (int j = 0; j != b4; ++j)
        for (int i = 0; i != b3; ++i)
          for (int d = 0; d != b2; ++d)
            (*bcontainer)[i][j][k][d][g] += 1.4142135624;
}

template<>
void traverse<NEST_GDZ>(boost_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int g = 0; g != b1; ++g)
    for (int d = 0; d != b2; ++d)
      for (int k = 0; k != b5; ++k)
        for (int j = 0; j != b4; ++j)
          for (int i = 0; i != b3; ++i)
            (*bcontainer)[i][j][k][d][g] += 1.4142135624;
}

template<>
void traverse<NEST_DGZ>(boost_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int d = 0; d != b2; ++d)
    for (int g = 0; g != b1; ++g)
      for (int k = 0; k != b5; ++k)
        for (int j = 0; j != b4; ++j)
          for (int i = 0; i != b3; ++i)
            (*bcontainer)[i][j][k][d][g] += 1.4142135624;
}

template<>
void traverse<NEST_DZG>(boost_arr_t* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int d = 0; d != b2; ++d)
    for (int k = 0; k != b5; ++k)
      for (int j = 0; j != b4; ++j)
        for (int i = 0; i != b3; ++i)
          for (int g = 0; g != b1; ++g)
            (*bcontainer)[i][j][k][d][g] += 1.4142135624;
}

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_boost(int size1, int size2, int size3, int size4, int size5)
{
  boost_arr_t::size_type layout[5];
  bool ordering[] = {true, true, true, true, true};

  switch(Nest)
  {
    case NEST_DGZ:
      layout[0] = 0;
      layout[1] = 1;
      layout[2] = 2;
      layout[3] = 4;
      layout[4] = 3;
      break;
    case NEST_DZG:
      layout[0] = 4;
      layout[1] = 0;
      layout[2] = 1;
      layout[3] = 2;
      layout[4] = 3;
      break;
    case NEST_GDZ:
      layout[0] = 0;
      layout[1] = 1;
      layout[2] = 2;
      layout[3] = 3;
      layout[4] = 4;
      break;
    case NEST_GZD:
      layout[0] = 3;
      layout[1] = 0;
      layout[2] = 1;
      layout[3] = 2;
      layout[4] = 4;
      break;
    case NEST_ZDG:
      layout[0] = 4;
      layout[1] = 3;
      layout[2] = 0;
      layout[3] = 1;
      layout[4] = 2;
      break;
    case NEST_ZGD:
      layout[0] = 3;
      layout[1] = 4;
      layout[2] = 0;
      layout[3] = 1;
      layout[4] = 2;
      break;
  }

  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(Nest);

  // Construction
  timer.start();
  boost_arr_t* bc
    = new boost_arr_t(boost::extents[size3][size4][size5][size2][size1],
        storage_t(layout, ordering));
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(bc->origin(), size1*size2*size3*size4*size5);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  traverse<Nest>(bc, size1, size2, size3, size4, size5);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete bc;
  std::get<4>(times) = timer.stop();

  return times;
}

// Traverse a SubTVec instance given the nesting order.
// Note: restrict keyword not used at this point.
template<Nesting_Order>
void traverse(SubTVec*, int, int, int, int, int)
{
  std::cerr << "Nesting order unrecognized for SubTVec traversal.\n";
  exit(1);
}

template<>
void traverse<NEST_ZDG>(SubTVec* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int k = 0; k != b5; ++k)
  {
    for (int j = 0; j != b4; ++j)
    {
      for (int i = 0; i != b3; ++i)
      {
        int z = i + j * b3 + k * b3 * b4;
        for (int d = 0; d != b2; ++d)
        {
          double* bc_z_d = bcontainer->ptr(0, d, z);
          for (int g = 0; g != b1; ++g)
          {
            bc_z_d[g] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<>
void traverse<NEST_ZGD>(SubTVec* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int k = 0; k != b5; ++k)
  {
    for (int j = 0; j != b4; ++j)
    {
      for (int i = 0; i != b3; ++i)
      {
        int z = i + j * b3 + k * b3 * b4;
        for (int g = 0; g != b1; ++g)
        {
          double* bc_z_g = bcontainer->ptr(g, 0, z);
          for (int d = 0; d != b2; ++d)
          {
            bc_z_g[d] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<>
void traverse<NEST_GZD>(SubTVec* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int g = 0; g != b1; ++g)
  {
    for (int k = 0; k != b5; ++k)
    {
      for (int j = 0; j != b4; ++j)
      {
        for (int i = 0; i != b3; ++i)
        {
          int z = i + j * b3 + k * b3 * b4;
          double* bc_g_z = bcontainer->ptr(g, 0, z);
          for (int d = 0; d != b2; ++d)
          {
            bc_g_z[d] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<>
void traverse<NEST_GDZ>(SubTVec* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int g = 0; g != b1; ++g)
  {
    for (int d = 0; d != b2; ++d)
    {
      double* bc_g_d = bcontainer->ptr(g, d, 0);
      for (int k = 0; k != b5; ++k)
      {
        for (int j = 0; j != b4; ++j)
        {
          int z = j * b3 + k * b3 * b4;
          for (int i = 0; i != b3; ++i, ++z)
          {
            bc_g_d[z] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<>
void traverse<NEST_DGZ>(SubTVec* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int d = 0; d != b2; ++d)
  {
    for (int g = 0; g != b1; ++g)
    {
      double* bc_d_g = bcontainer->ptr(g, d, 0);
      for (int k = 0; k != b5; ++k)
      {
        for (int j = 0; j != b4; ++j)
        {
          int z = j * b3 + k * b3 * b4;
          for (int i = 0; i != b3; ++i, ++z)
          {
            bc_d_g[z] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<>
void traverse<NEST_DZG>(SubTVec* bcontainer,
                        int b1, int b2, int b3, int b4, int b5)
{
  for (int d = 0; d != b2; ++d)
  {
    for (int k = 0; k != b5; ++k)
    {
      for (int j = 0; j != b4; ++j)
      {
        for (int i = 0; i != b3; ++i)
        {
          int z = i + j * b3 + k * b3 * b4;
          double* bc_d_z = bcontainer->ptr(0, d, z);
          for (int g = 0; g != b1; ++g)
          {
            bc_d_z[g] += 1.4142135624;
          }
        }
      }
    }
  }
}

template<Nesting_Order Nest>
std::tuple<std::string, double, double, double, double>
run_subtvec(Nesting_Order actual_nest, int size1, int size2,
            int size3, int size4, int size5)
{
  counter_t timer;
  timer.reset();
  std::tuple<std::string, double, double, double, double> times;
  std::get<0>(times) = nestingString(actual_nest);

  // Construction
  timer.start();
  SubTVec* bc = new SubTVec(Nest, size1, size2, size3*size4*size5);
  std::get<1>(times) = timer.stop();
  timer.reset();

  // Initialization over raw data
  timer.start();
  populate(bc->ptr(), size1*size2*size3*size4*size5);
  std::get<2>(times) = timer.stop();
  timer.reset();

  // Traversal using index operators
  timer.start();
  traverse<Nest>(bc, size1, size2, size3, size4, size5);
  std::get<3>(times) = timer.stop();
  timer.reset();

  // Destruct
  timer.start();
  delete bc;
  std::get<4>(times) = timer.stop();

  return times;
}

#endif
