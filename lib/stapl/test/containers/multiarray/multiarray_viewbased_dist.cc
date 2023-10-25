/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/multiarray.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/utility/tuple/print.hpp>
#include <stapl/utility/integer_sequence.hpp>

template<typename Spec1, typename Spec2,
         typename Traversal1, typename Traversal2>
class get_spec_wf
{
private:
  typedef stapl::dist_view_impl::system_container   system_ct_t;
  typedef std::vector<system_ct_t>                  location_groups_t;

  typedef typename Spec1::gid_type gid1_type;
  typedef typename Spec2::gid_type gid2_type;

  gid1_type                           m_size1;
  gid2_type                           m_size2;
  gid1_type                           m_loc_dims;

  std::shared_ptr<location_groups_t>  m_loc_groups_ptr;

public:
   get_spec_wf(gid1_type size1, gid2_type size2, gid1_type loc_dims)
     : m_size1(size1), m_size2(size2), m_loc_dims(loc_dims),
       m_loc_groups_ptr(std::make_shared<location_groups_t>())
   {
     const unsigned int num_loc_groups =
       stapl::get<0>(m_size1) * stapl::get<1>(m_size1) * stapl::get<2>(m_size1);

     const unsigned int num_locs = stapl::get_num_locations();

     const unsigned int loc_group_size = num_locs / num_loc_groups;

#ifndef STAPL_NDEBUG
    auto t = stapl::dist_spec_impl::modulo(m_loc_dims, m_size1);
    bool b = stapl::vs_map_reduce([](unsigned int val) { return val == 0; },
                                  stapl::logical_or<bool>(), true, t);

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

   // Query for Top Level Container
   Spec1 operator()(void) const
   {
     return stapl::balanced_nd<Traversal1>(m_size1, m_loc_dims);
   }

   // Query for Second Level Container
   Spec2 operator()(typename Spec1::gid_type gid) const
   {
     const unsigned int loc_group =
       stapl::nd_linearize<
         typename Spec1::gid_type, stapl::default_traversal<3>::type
       >(m_size1)(gid);

     // Use Gid to define the set of locs on which nested container
     // is initialized.
     typedef stapl::index_sequence<4> part_dims;

     return stapl::sliced_volumetric<Traversal2, part_dims>(
       m_size2, m_loc_groups_ptr->operator[](loc_group));
   }
};


struct current_loc
  : public stapl::p_object
{
  size_t reflect_value(size_t value) const
  { return value; }
};


struct set_val
{
private:
  current_loc* m_loc;

public:
  typedef void result_type;

  set_val(current_loc &loc)
    : m_loc(&loc)
  { }

  template <typename Elem>
  void operator()(Elem e)
  { e = m_loc->get_location_id(); }

  void define_type(stapl::typer& t)
  { t.member(m_loc); }
};

struct fill_func
{
  typedef void result_type;

  template <typename Elem>
  void operator()(Elem e)
  {
    stapl::fill(e, 0.0);
  }
};

template<typename Container, typename LocMappings>
void test_multiarray(Container&& vma,
                     std::string name,
                     LocMappings&& loc_mappings)
{
  current_loc cl;

  auto vvw = make_multiarray_view(vma);

  stapl::map_func(set_val(cl), vvw);

  //
  // All configurations yield same number of elements per location.
  //
  const int sum = stapl::accumulate(linear_view(vvw), 0);

  const int expected_sum =
    (int)((cl.get_num_locations()-1)*cl.get_num_locations()*128/2);

  //
  // All configurations yield one base container per location.
  //
  const size_t num_base_containers =
    stapl::allreduce_rmi(stapl::plus<size_t>(), cl.get_rmi_handle(),
      &current_loc::reflect_value,
      vma.distribution().container_manager().size()).get();

  const size_t expected_num_base_containers = 4;

  stapl::do_once([&vma, name, &loc_mappings,
                  sum, expected_sum,
                  num_base_containers, expected_num_base_containers]()
  {
    // each location has 4x4x8 == 128 elements
    const bool b_correct_elems_per_location = sum == expected_sum;

    const bool b_correct_num_base_containers =
      num_base_containers == expected_num_base_containers;

    bool b_correct_loc_mapping = true;

    //
    // Verify all gid mappings
    //
    for (auto&& loc_mapping : loc_mappings)
      if (vma[loc_mapping.first] != loc_mapping.second)
        b_correct_loc_mapping = false;

    std::cout << "Test : multiarray_viewbased_dist (" << name << ")\n"
              << "Version : STAPL\n";

    if (b_correct_elems_per_location
        && b_correct_num_base_containers
        && b_correct_loc_mapping)
    {
      std::cout << "Status : PASS\n";
    }
    else
    {
      std::cout << "Status : FAIL\n";

      if (!b_correct_elems_per_location)
        std::cout << "sum = " << sum << ", expected " << expected_sum << "\n";

      if (!b_correct_num_base_containers)
        std::cout << "num base containers = " << num_base_containers
                  << ", expected " << expected_num_base_containers << "\n";

      if (!b_correct_loc_mapping)
        for (auto&& loc_mapping : loc_mappings)
          if (vma[loc_mapping.first] != loc_mapping.second) {
            std::cout << "mapping ";
            stapl::print_tuple(std::cout, loc_mapping.first);
            std::cout << " to " << vma[loc_mapping.first] << ", expected "
                      << loc_mapping.second << "\n";
          }
    }

    std::cout << "\n";
  });
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  using stapl::md_distribution_spec;
  using stapl::sliced_md_distribution_spec;
  using stapl::view_based_partition;
  using stapl::view_based_mapper;
  using stapl::multiarray;

  using stapl::balanced_nd;
  using stapl::volumetric;
  using stapl::sliced_volumetric;

  typedef stapl::tuple<
    unsigned long int, unsigned long int, unsigned long int> gid_type;

  gid_type size(8, 8, 8);

  typedef std::vector<std::pair<gid_type,stapl::location_type>>  loc_mappings_t;

  // Specify traversal that will cover z, y, and then x dimension.
  typedef stapl::index_sequence<2, 1, 0>                            zyx_traversal;



  //
  // Test Basic Volumetric
  //
  typedef md_distribution_spec<zyx_traversal>::type nd_partitioning_view_type;

  typedef multiarray<3, int, nd_partitioning_view_type> vb_ma_t;

  test_multiarray(vb_ma_t(volumetric<zyx_traversal>(size)),
                  "volumetric",
                  loc_mappings_t { { gid_type(0, 0, 0), 0 },
                                   { gid_type(0, 0, 4), 1 },
                                   { gid_type(4, 0, 0), 2 },
                                   { gid_type(4, 0, 4), 3 } });

  //
  // Test balanced_nd
  //
  stapl::tuple<size_t, size_t, size_t> loc_dims(1, 1, 4);

  test_multiarray(vb_ma_t(balanced_nd<zyx_traversal>(size, loc_dims)),
                  "balanced_nd",
                  loc_mappings_t { { gid_type(0, 0, 0), 0 },
                                   { gid_type(0, 0, 2), 1 },
                                   { gid_type(4, 0, 4), 2 },
                                   { gid_type(4, 0, 6), 3 } });


  //
  // Test Sliced Volumetric (First Two Dimensions)
  //
  typedef stapl::index_sequence<0, 1>                                part_dims;

  typedef sliced_md_distribution_spec<part_dims, zyx_traversal>::type
    sliced_nd_partitioning_view_type;

  typedef multiarray<3, int, sliced_nd_partitioning_view_type>    svb_ma_t;


  test_multiarray(svb_ma_t(sliced_volumetric<zyx_traversal, part_dims>(size)),
                  "sliced basic",
                  loc_mappings_t { { gid_type(0, 0, 0), 0 },
                                   { gid_type(3, 3, 1), 0 },
                                   { gid_type(0, 4, 2), 1 },
                                   { gid_type(3, 7, 3), 1 },
                                   { gid_type(4, 0, 4), 2 },
                                   { gid_type(7, 3, 5), 2 },
                                   { gid_type(4, 4, 6), 3 },
                                   { gid_type(7, 7, 7), 3 } });


  //
  // Test Sliced Volumetric (Last Two Dimensions)
  //
  typedef stapl::index_sequence<1, 2>                              part_dims2;

  typedef sliced_md_distribution_spec<part_dims2, zyx_traversal>::type
    sliced_nd_partitioning2_view_type;

  typedef multiarray<3, int, sliced_nd_partitioning2_view_type>    svb2_ma_t;

  test_multiarray(svb2_ma_t(sliced_volumetric<zyx_traversal, part_dims2>(size)),
    "sliced basic2",
    loc_mappings_t { { gid_type(0, 0, 0), 0 },
                     { gid_type(1, 3, 3), 0 },
                     { gid_type(2, 0, 4), 1 },
                     { gid_type(3, 3, 7), 1 },
                     { gid_type(4, 4, 0), 2 },
                     { gid_type(5, 7, 3), 2 },
                     { gid_type(6, 4, 4), 3 },
                     { gid_type(7, 7, 7), 3 } });

  //
  // Test Sliced Volumetric (First Dimension)
  //
  typedef stapl::index_sequence<0>                                      part_dims3;

  typedef sliced_md_distribution_spec<part_dims3, zyx_traversal>::type
    sliced_single_nd_partitioning_view_type;

  typedef multiarray<3, int, sliced_single_nd_partitioning_view_type> ssvb_ma_t;


  test_multiarray(ssvb_ma_t(sliced_volumetric<zyx_traversal, part_dims3>(size)),
    "sliced basic3",
    loc_mappings_t { { gid_type(0, 0, 0), 0 }, { gid_type(0, 7, 7), 0 },
                     { gid_type(1, 0, 0), 0 }, { gid_type(1, 7, 7), 0 },
                     { gid_type(2, 0, 0), 1 }, { gid_type(2, 7, 7), 1 },
                     { gid_type(3, 0, 0), 1 }, { gid_type(3, 7, 7), 1 },
                     { gid_type(4, 0, 0), 2 }, { gid_type(4, 7, 7), 2 },
                     { gid_type(5, 0, 0), 2 }, { gid_type(5, 7, 7), 2 },
                     { gid_type(6, 0, 0), 3 }, { gid_type(6, 7, 7), 3 },
                     { gid_type(7, 0, 0), 3 }, { gid_type(7, 7, 7), 3 } });

  //
  // Test Sliced Volumetric (Permuted)
  //
  typedef stapl::index_sequence<1, 0>                              perm_part_dims;

  typedef sliced_md_distribution_spec<perm_part_dims, zyx_traversal>::type
    sliced_perm_single_nd_partitioning_view_type;

  typedef multiarray<3, int, sliced_perm_single_nd_partitioning_view_type>
    perm_ssvb_ma_t;

  test_multiarray(
    perm_ssvb_ma_t(sliced_volumetric<zyx_traversal, perm_part_dims>(size)),
    "sliced permuted",
    loc_mappings_t { { gid_type(0, 0, 0), 0 },
                     { gid_type(3, 3, 1), 0 },
                     { gid_type(4, 0, 2), 1 },
                     { gid_type(7, 3, 3), 1 },
                     { gid_type(0, 4, 4), 2 },
                     { gid_type(3, 7, 5), 2 },
                     { gid_type(4, 4, 6), 3 },
                     { gid_type(7, 7, 7), 3 } });

  //
  // Composed multiarray distribution specification (Kripke Derived)
  //
  stapl::do_once([]() {
    std::cout << "Test : multiarray_viewbased_dist (composed)\n"
              << "Version : STAPL\n";
  });

  using stapl::heterogeneous_composed_dist_spec;
  using stapl::make_heterogeneous_composed_dist_spec;
  using std::make_tuple;
  using std::get;

  auto loc_dims2    = std::make_tuple(1, 1, 4);

  auto num_zones    = make_tuple(12 * get<0>(loc_dims2),
                                 12 * get<1>(loc_dims2),
                                 12 * get<2>(loc_dims2));

  auto num_zonesets = make_tuple(1, 1, 2);

  const unsigned int num_directions = 5;
  const unsigned int num_groups     = 5;

  auto zone_set_size = make_tuple(get<0>(num_zones) / get<0>(num_zonesets),
                                  get<1>(num_zones) / get<1>(num_zonesets),
                                  get<2>(num_zones) / get<2>(num_zonesets));

  auto outer_ct_size = num_zonesets;
  auto inner_ct_size = std::tuple_cat(zone_set_size,
                                      make_tuple(num_directions, num_groups));

  // Inner Multiarray Related Typedefs
  typedef stapl::index_sequence<4, 3, 2, 1, 0>                  D5_traversal;
  typedef stapl::index_sequence<4>                              D5_part_dims;
  typedef sliced_md_distribution_spec<D5_part_dims, D5_traversal>::type D5_spec;
  typedef multiarray<5, double, D5_spec>                        inner_ct_t;

  // Outer Multiarray Related Typedefs
  typedef stapl::index_sequence<2, 1, 0>                         D3_traversal;
  typedef md_distribution_spec<D3_traversal>::type               D3_spec;
  typedef multiarray<3, inner_ct_t, D3_spec>                     outer_ct_t;

  typedef get_spec_wf<
    D3_spec, D5_spec, D3_traversal, D5_traversal>             spec_generator_t;

  auto composed_spec =
    make_heterogeneous_composed_dist_spec<spec_generator_t, D3_spec, D5_spec>(
      spec_generator_t(outer_ct_size, inner_ct_size, loc_dims2));

  outer_ct_t ct(composed_spec);

  auto ct_vw = make_multiarray_view(ct);
  map_func(fill_func(), ct_vw);

  stapl::do_once([]() {std::cout << "Status : PASS\n"; });

  return EXIT_SUCCESS;
}
