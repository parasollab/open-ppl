/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_UTILITY
#define STAPL_BENCHMARK_LONESTAR_BH_UTILITY

#include "point.hpp"
#include <cmath>

double theta = 2.5;

//////////////////////////////////////////////////////////////////////
/// @brief Compute the center of mass of two points.
///
/// @param a The position of the first particle
/// @param b The position of the second particle
/// @param m1 The mass of the first particle
/// @param m2 The mass of the second particle
//////////////////////////////////////////////////////////////////////
point center_of_mass(point const& a, point const& b, double m1, double m2)
{
  double total_mass = m1 + m2;

  return point(
   (a.x*m1 + b.x*m2) / total_mass,
   (a.y*m1 + b.y*m2) / total_mass,
   (a.z*m1 + b.z*m2) / total_mass
  );
}


//////////////////////////////////////////////////////////////////////
/// @brief Determine which octant a child cell should be in for a
///        a given parent cell
///
/// @param root_coord The position of the parent cell
/// @param root_coord The position of the child cell
/// @return The octant (0-7) that the child cell should go into
//////////////////////////////////////////////////////////////////////
std::size_t which_child(point root_coord, point ptr_coord)
{
  std::size_t n = 0;
  if (ptr_coord.x > root_coord.x)
    n += 1;
  if (ptr_coord.y > root_coord.y)
    n += 2;
  if (ptr_coord.z > root_coord.z)
    n += 4;
  return n;
}


//////////////////////////////////////////////////////////////////////
/// @brief Determines if a cell can approximate the contribution of the
///        particles it represents
///
/// @param crd1 Position of the particle that is being computed
/// @param crd2 Position of the cell that is being queried
/// @param lvl The level of the cell in the octree
//////////////////////////////////////////////////////////////////////
bool can_approx(point crd1, point crd2, std::size_t lvl)
{
  double dx = crd1.x - crd2.x;
  double dy = crd1.y - crd2.y;
  double dz = crd1.z - crd2.z;
  double delta = sqrt(pow(dx, 2)+pow(dy, 2)+pow(dz, 2));

  if ((10/pow(2, lvl))/delta < theta)
    return true;

  else
    return false;
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the coordinate (center) of a child cell in the octree
///
/// @param root_coord The coordinate of the parent cell in the octree
/// @param idx Index of the child to compute the coordinate for
/// @param lvl The level of the parent coordinate in the tree
//////////////////////////////////////////////////////////////////////
point octant_coordinate(point root_coord, std::size_t idx, std::size_t lvl)
{
  double sl = 10/(pow(2,lvl+1));

  point new_point(root_coord.x, root_coord.y, root_coord.z);

  if (idx % 2 == 1)
    new_point.x += sl;
  else
    new_point.x -= sl;

  if (idx % 4 > 1)
    new_point.y += sl;
  else
    new_point.y -= sl;

  if (idx > 3)
    new_point.z += sl;
  else
    new_point.z -= sl;

  return new_point;
}


//////////////////////////////////////////////////////////////////////
/// @brief Generate a descriptor for a child vertex based on the parent
///        descriptor and the octant of the child. Note that the generated
///        descriptor is not guaranteed to be unique.
///
/// @param parent Descriptor of the parent vertex
/// @param octant The octant of the child vertex
/// @return A randomly generated descriptor seed with the parent and octant
//////////////////////////////////////////////////////////////////////
std::size_t child_descriptor(std::size_t parent, int octant)
{
  typedef boost::mt19937 engine_type;
  typedef boost::random::uniform_int_distribution<std::size_t> dist_type;
  typedef boost::variate_generator<engine_type, dist_type> generator_type;

  engine_type eng(8*parent+octant);

  generator_type gen(eng, dist_type(0,
    std::numeric_limits<std::size_t>::max()-1)
  );

  return gen();
}


//////////////////////////////////////////////////////////////////////
/// @brief Return the child node in a given octant from a vertex. If
///        There is no child in that octant, return an empty boost::optional
///
/// @param v The parent node
/// @param octant The octant of the child vertex
/// @return A boost::optional that either contains the ID of the child
///         in that octant, or an empty optional if no such child exists
//////////////////////////////////////////////////////////////////////
template<typename Vertex>
boost::optional<std::size_t> retrieve_child_vertex(Vertex v, int octant)
{
  for (auto&& e : v)
    if (e.property().first && e.property().second == octant)
      return boost::optional<std::size_t>(e.target());

   return boost::optional<std::size_t>();
}

#endif
