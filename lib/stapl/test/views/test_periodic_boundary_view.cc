/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/views/periodic_boundary_view.hpp>

#include <test/algorithms/test_utils.h>

#include "../test_report.hpp"

using namespace stapl;


size_t compute_ct_index(size_t idx, size_t dim)
{
  if (idx == 0)
    return dim-1;

  if (idx == dim+1)
    return 0;

  return idx-1;
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cout<< "usage: exe n m r" <<std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);
  size_t m = atoi(argv[2]);
  size_t r = atoi(argv[3]);

  typedef int                                        value_type;
  typedef indexed_domain<size_t>                     vector_domain_type;
  typedef balanced_partition<vector_domain_type>     balanced_type;
  typedef stapl::default_traversal<3>::type          traversal_type;
  typedef nd_partition<
            stapl::tuple<balanced_type, balanced_type, balanced_type>,
            traversal_type>                          partition_type;
  typedef stapl::tuple<size_t, size_t, size_t>       gid_type;
  typedef multiarray<3, value_type, traversal_type,
          partition_type>                            multiarray_type;

  balanced_type p0(vector_domain_type(0, n-1), get_num_locations());
  balanced_type p1(vector_domain_type(0, m-1), get_num_locations());
  balanced_type p2(vector_domain_type(0, r-1), get_num_locations());
  partition_type part(p0,p1,p2);

  multiarray_type c(part);

  STAPL_TEST_REPORT(c.size() == n*m*r, "Testing size");

  periodic_boundary_view<multiarray_type> pb_view(c);

  multiarray_type::dimensions_type c_dims = c.dimensions();
  multiarray_type::dimensions_type pb_dims = pb_view.dimensions();

  bool passed = get<0>(c_dims) + 2 == get<0>(pb_dims) &&
                get<1>(c_dims) + 2 == get<1>(pb_dims) &&
                get<2>(c_dims) + 2 == get<2>(pb_dims);

   STAPL_TEST_REPORT(passed, "Testing dimensions");

   if (get_location_id() == 0) {
     for (size_t i = 0; i < n; ++i) {
       for (size_t j = 0; j < m; ++j) {
         for (size_t k = 0; k < r; ++k) {
           gid_type g(i,j,k);
           c.set_element(g, i*n+j);
         }
       }
     }
   }

   rmi_fence();

   passed = true;
   for (size_t i = 0; i < n+2; ++i) {
     for (size_t j = 0; j < m+2; ++j) {
       for (size_t k = 0; k < r+2; ++k)
       {
         gid_type g(i,j,k);

         size_t i_ct = compute_ct_index(i, n);
         size_t j_ct = compute_ct_index(j, m);
         size_t k_ct = compute_ct_index(k, r);

         gid_type ct_g(i_ct, j_ct, k_ct);

         if (c.get_element(ct_g) != pb_view.get_element(g))
           passed = false;
       }
     }
   }

   // check if set_element test passed on all locations
   stapl_bool res(passed);
   bool global_result = res.reduce();
   STAPL_TEST_REPORT(global_result, "Testing get_element");

   passed = true;
   for (size_t i = 0; i < n+2; ++i) {
     for (size_t j = 0; j < m+2; ++j) {
       for (size_t k = 0; k < r+2; ++k)
       {
         size_t i_ct = compute_ct_index(i, n);
         size_t j_ct = compute_ct_index(j, m);
         size_t k_ct = compute_ct_index(k, r);

         gid_type ct_g(i_ct, j_ct, k_ct);

         if (c.get_element(ct_g) != pb_view(i,j,k))
           passed = false;
       }
     }
   }

   // check if operator() test passed on all locations
   res = stapl_bool(passed);
   global_result = res.reduce();
   STAPL_TEST_REPORT(global_result, "Testing operator()");

   passed = true;
   for (size_t i = 0; i < n; ++i) {
     for (size_t j = 0; j < m; ++j) {
       for (size_t k = 0; k < r; ++k) {
         gid_type g(i,j,k);
           size_t i_ct = compute_ct_index(i, n);
           size_t j_ct = compute_ct_index(j, m);
           size_t k_ct = compute_ct_index(k, r);

           gid_type ct_g(i_ct, j_ct, k_ct);

         if (c[ct_g] != pb_view[g])
           passed = false;
       }
     }
   }

   // check if operator[] test passed on all locations
   res = stapl_bool(passed);
   global_result = res.reduce();
   STAPL_TEST_REPORT(global_result, "Testing operator[]");

  return EXIT_SUCCESS;
}
