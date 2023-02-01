/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOUNDARY_SPEC_HPP
#define BOUNDARY_SPEC_HPP

#include <stapl/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/algorithms/algorithm.hpp>


template<typename Spec1, typename Spec2,
         typename Traversal1, typename Traversal2>
class get_boundary_spec_wf
{
private:
  using system_ct_t = stapl::dist_view_impl::system_container;
  using location_groups_t = std::vector<system_ct_t>;

  using gid1_type = typename Spec1::gid_type;
  using gid2_type = typename Spec2::gid_type;

  gid1_type                           m_size1;
  gid2_type                           m_size2;
  gid1_type                           m_loc_dims;

  std::shared_ptr<location_groups_t>  m_loc_groups_ptr;

  void initialize_loc_groups()
  {
    const unsigned int num_loc_groups =
      stapl::get<0>(m_size1) * stapl::get<1>(m_size1) * stapl::get<2>(m_size1);

    const unsigned int num_locs = stapl::get_num_locations();

    const unsigned int loc_group_size = num_locs / num_loc_groups;

#ifndef STAPL_NDEBUG
    auto t = stapl::dist_spec_impl::modulo(m_loc_dims, m_size1);
    bool b = stapl::vs_map_reduce([](unsigned int val) { return val == 0; },
                                  stapl::logical_and<bool>(), true, t);

    stapl_assert(b, "Non Conformable location / cellset dims");
#endif

    for (unsigned int idx = 0; idx < num_loc_groups; ++idx)
    {
      std::vector<stapl::location_type> locs;

      unsigned int loc = idx * loc_group_size;

      for ( ; loc < loc_group_size * (idx + 1) ; ++loc)
        locs.push_back(loc);

      m_loc_groups_ptr->emplace_back(locs);
    }
  }

public:
  get_boundary_spec_wf(gid1_type size1, gid2_type size2, gid1_type loc_dims)
    : m_size1(size1),
      m_size2(size2),
      m_loc_dims(loc_dims),
      m_loc_groups_ptr(std::make_shared<location_groups_t>())
  {
    initialize_loc_groups();
  }

  // Query for Top Level Container
  Spec1 operator()(void) const
  {
    return stapl::uniform_nd<Traversal1>(m_size1, m_loc_dims);
  }

   // Query for Second Level Container
  Spec2 operator()(typename Spec1::gid_type gid) const
  {
    const unsigned int loc_group =
      stapl::nd_linearize<typename Spec1::gid_type,
                          stapl::default_traversal<3>::type>(m_size1)(gid);

    // Use Gid to define the set of locs on which nested container
    // is initialized.
    return stapl::volumetric<Traversal2>(
      m_size2, m_loc_groups_ptr->operator[](loc_group).get_locs());
  }
};


template<typename Spec1, typename Spec2,
         typename Traversal1, typename Traversal2>
class get_boundary_spec_wf_1D
{
private:
  using system_ct_t = stapl::dist_view_impl::system_container;
  using location_groups_t = std::vector<system_ct_t>;

  using gid1_type = typename Spec1::gid_type;
  using gid2_type = typename Spec2::gid_type;

  gid1_type                           m_size1;
  gid2_type                           m_size2;
  gid1_type                           m_loc_dims;

  std::shared_ptr<location_groups_t>  m_loc_groups_ptr;

  void initialize_loc_groups()
  {
    const unsigned int num_loc_groups =
      stapl::get<0>(m_size1) * stapl::get<1>(m_size1) * stapl::get<2>(m_size1);

    const unsigned int num_locs = stapl::get_num_locations();

    const unsigned int loc_group_size = num_locs / num_loc_groups;

#ifndef STAPL_NDEBUG
    auto t = stapl::dist_spec_impl::modulo(m_loc_dims, m_size1);
    bool b = stapl::vs_map_reduce([](unsigned int val) { return val == 0; },
                                  stapl::logical_and<bool>(), true, t);

    stapl_assert(b, "Non Conformable location / cellset dims");
#endif

    for (unsigned int idx = 0; idx < num_loc_groups; ++idx)
    {
      std::vector<stapl::location_type> locs;

      unsigned int loc = idx * loc_group_size;

      for ( ; loc < loc_group_size * (idx + 1) ; ++loc)
        locs.push_back(loc);

      m_loc_groups_ptr->emplace_back(locs);
    }
  }

public:
  get_boundary_spec_wf_1D(gid1_type size1, gid2_type size2, gid1_type loc_dims)
    : m_size1(size1),
      m_size2(size2),
      m_loc_dims(loc_dims),
      m_loc_groups_ptr(std::make_shared<location_groups_t>())
  {
    initialize_loc_groups();
  }

  // Query for Top Level Container
  Spec1 operator()(void) const
  {
    return stapl::uniform_nd<Traversal1>(m_size1, m_loc_dims);
  }

   // Query for Second Level Container
  Spec2 operator()(typename Spec1::gid_type gid) const
  {
    const unsigned int loc_group =
      stapl::nd_linearize<typename Spec1::gid_type,
                          stapl::default_traversal<3>::type>(m_size1)(gid);

    // Use Gid to define the set of locs on which nested container
    // is initialized.
    return stapl::balance(
      m_size2, m_loc_groups_ptr->operator[](loc_group).get_locs());
  }
};

// work function to generate boundary input
// The problem boundary is initialized to 1.0
struct gen_bdry_ones
{
  using index_type = std::tuple<std::size_t, std::size_t, std::size_t>;
  // using result_type = std::vector<std::array<double, 3>>;
  using result_type = std::vector<double>;

  static result_type * get_plane(index_type const& dims, int num_groups,
                                 int num_directions,
                                 size_t d)
  {
    using stapl::skeletons::direction;

    direction dir = (direction)d;

    int dimx = std::get<0>(dims);
    int dimy = std::get<1>(dims);
    int dimz = std::get<2>(dims);

    int groups_dirs = num_groups * num_directions;

    if (dir == direction::direction0)
      return new result_type(dimy * dimz * groups_dirs, 1.0);

    if (dir == direction::direction1)
      return new result_type(dimx * dimz * groups_dirs, 1.0);

    return new result_type(dimx * dimy * groups_dirs, 1.0);
  }
}; // gen_bdry_ones


#endif // BOUNDARY_SPEC_HPP
