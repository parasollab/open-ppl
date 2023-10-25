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

#include <test/algorithms/test_utils.h>

#include "../../test_report.hpp"

using namespace stapl;

template<typename MultiArray>
void test_multiarray(MultiArray& c, size_t n, size_t m, size_t r, std::string s)
{
  STAPL_TEST_MESSAGE(s)

  typedef typename MultiArray::gid_type gid_type;

  typename MultiArray::dimensions_type dims2 = c.dimensions();
  bool passed = get<0>(dims2) == n &&
                get<1>(dims2) == m &&
                get<2>(dims2) == r;
  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(c.size() == n*m*r, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
        for (size_t k = 0; k < r; ++k) {
          gid_type g(i,j,k);
          c.set_element(g, i*n+j);
        }
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(c.get_element(g)) != i*n+j)
          passed = false;
      }
    }
  }

  // check if set_element test passed on all locations
  stapl_bool red(passed);
  bool global_result = red.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(c[g]) != i*n+j)
          passed = false;
      }
    }
  }

  red = stapl_bool(passed);
  global_result = red.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cout<< "usage: exe n m r" <<std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);
  const size_t m = atoi(argv[2]);
  const size_t r = atoi(argv[3]);

  typedef int                                        value_type;
  typedef indexed_domain<size_t>                     vector_domain_type;
  typedef balanced_partition<vector_domain_type>     balanced_type;
  typedef stapl::default_traversal<3>::type          traversal_type;
  typedef stapl::default_traversal<2>::type          traversal_type2;
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

  // ========================================================

  multiarray<3, value_type> c;
  c.resize(gid_type(n, m, r));
  test_multiarray(c, n, m, r, "Testing Default Constructor / Resize");

  // ========================================================
  multiarray_type d(part);

  test_multiarray(d, n, m, r, "Testing Partition Constructor");

  // =====================================================================
  typedef stapl::multidimensional_mapper<container_traits<multiarray_type>
       ::base_container_type::cid_type> mapper_tp;

  mapper_tp mymap(part);

  multiarray_type e(part, mymap);
  bool passed = true;
  passed = e.size() == n*m*r;
  STAPL_TEST_REPORT(passed, "Testing Partition Mapper Constructor")

  multiarray_type::dimensions_type dims3 = e.dimensions();
  passed = get<0>(dims3) == n &&
           get<1>(dims3) == m &&
           get<2>(dims3) == r;

  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(e.size() == n*m*r, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
        for (size_t k = 0; k < r; ++k) {
          gid_type g(i,j,k);
          e.set_element(g, i*n+j);
        }
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(e.get_element(g)) != i*n+j)
          passed = false;
      }
    }
  }

  // check if set_element test passed on all locations
  stapl_bool ree(passed);
  bool global_result = ree.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(e[g]) != i*n+j)
          passed = false;
      }
    }
  }

  ree = stapl_bool(passed);
  global_result = ree.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")
  // =====================================================================

  typedef multiarray<3, value_type, traversal_type>  multiarray_type2;
  multiarray_type2 f(gid_type(n,m,r), 5);
  passed = true;
  passed = f.size() == n*m*r;
  STAPL_TEST_REPORT(passed,
    "Testing dimensions_type, value_type Constructor 3D")

  multiarray_type2::dimensions_type dims4 = f.dimensions();
  passed = get<0>(dims4) == n &&
           get<1>(dims4) == m &&
           get<2>(dims4) == r;

  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(f.size() == n*m*r, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
        for (size_t k = 0; k < r; ++k) {
          gid_type g(i,j,k);
          f.set_element(g, i*n+j);
        }
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(f.get_element(g)) != i*n+j)
          passed = false;
      }
    }
  }

  // check if set_element test passed on all locations
  stapl_bool ref(passed);
  global_result = ref.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(f[g]) != i*n+j)
          passed = false;
      }
    }
  }

  ref = stapl_bool(passed);
  global_result = ref.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")


  // ========================================================
  // Initialize elements to 2.
  multiarray_type2 dv(gid_type(n,m,r), 2);
  typedef multiarray_view<multiarray_type2> view_type;
  multiarray_view<multiarray_type2> dv_view(dv);
  auto linear_dv_view = linear_view(dv_view);
  size_t sum = stapl::accumulate(linear_dv_view, size_t(0));
  STAPL_TEST_REPORT(sum == 2*n*m*r, "Testing construction with default value")


  // =====================================================================
  typedef multiarray<2, value_type, traversal_type2> multiarray_type3;
  typedef stapl::tuple<size_t, size_t>       gid_type2;

  multiarray_type3 s(gid_type2(n,m), 5);
  passed = true;
  passed = s.size() == n*m;
  STAPL_TEST_REPORT(passed,
    "Testing dimensions_type, value_type Constructor 2D")

  multiarray_type3::dimensions_type dims5 = s.dimensions();
  passed = get<0>(dims5) == n &&
           get<1>(dims5) == m;

  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(s.size() == n*m, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
          gid_type2 g(i,j);
          s.set_element(g, i*n+j);
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
        gid_type2 g(i,j);
        if (static_cast<size_t>(s.get_element(g)) != i*n+j)
          passed = false;
    }
  }

  // check if set_element test passed on all locations
  stapl_bool ress(passed);
  global_result = ress.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
        gid_type2 g(i,j);
        if (static_cast<size_t>(s[g]) != i*n+j)
          passed = false;
    }
  }

  ress = stapl_bool(passed);
  global_result = ress.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")
  // =====================================================================

  multiarray_type2::domain_type dom2(gid_type(n,m,r));
  multiarray_type2::partition_type part2(dom2,
  multiarray_impl::make_multiarray_size<3>()(get_num_locations()));

  typedef stapl::multidimensional_mapper<container_traits<multiarray_type2>
       ::base_container_type::cid_type> mapper_tp2;

  mapper_tp2 mymap2(part2);

  multiarray_type2 t(gid_type(n,m,r), mymap2);
  passed = true;
  passed = t.size() == n*m*r;
  STAPL_TEST_REPORT(passed,
    "Testing dimensions_type, mapper_type Constructor 3D")

  multiarray_type2::dimensions_type dimst = t.dimensions();
  passed = get<0>(dimst) == n &&
           get<1>(dimst) == m &&
           get<2>(dimst) == r;

  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(t.size() == n*m*r, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
        for (size_t k = 0; k < r; ++k) {
          gid_type g(i,j,k);
          t.set_element(g, i*n+j);
        }
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(t.get_element(g)) != i*n+j)
          passed = false;
      }
    }
  }

  // check if set_element test passed on all locations
  stapl_bool ret(passed);
  global_result = ret.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(t[g]) != i*n+j)
          passed = false;
      }
    }
  }

  ret = stapl_bool(passed);
  global_result = ret.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")
  // =====================================================================

  multiarray_type3::domain_type dom3(gid_type2(n,m));
  multiarray_type3::partition_type part3(dom3,
    multiarray_impl::make_multiarray_size<2>()(get_num_locations()));

  typedef stapl::multidimensional_mapper<container_traits<multiarray_type3>
       ::base_container_type::cid_type> mapper_tp3;

  mapper_tp3 mymap3(part3);

  multiarray_type3 t3(gid_type2(n,m), mymap3);
  passed = true;
  passed = t3.size() == n*m;
  STAPL_TEST_REPORT(passed,
    "Testing dimensions_type, mapper_type Constructor 2D")

  multiarray_type3::dimensions_type dimst3 = t3.dimensions();
  passed = get<0>(dimst3) == n &&
           get<1>(dimst3) == m;

  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(t3.size() == n*m, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
          gid_type2 g(i,j);
          t3.set_element(g, i*n+j);
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
        gid_type2 g(i,j);
        if (static_cast<size_t>(t3.get_element(g)) != i*n+j)
          passed = false;
    }
  }

  // check if set_element test passed on all locations
  stapl_bool ret3(passed);
  global_result = ret3.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
        gid_type2 g(i,j);
        if (static_cast<size_t>(t3[g]) != i*n+j)
          passed = false;
    }
  }

  ret3 = stapl_bool(passed);
  global_result = ret3.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")
  // =====================================================================


  multiarray_type2 q(gid_type(n,m,r));
  passed = true;
  passed = q.size() == n*m*r;
  STAPL_TEST_REPORT(passed,"Testing dimensions_type Constructor 3D")

  multiarray_type2::dimensions_type dimsq = q.dimensions();
  passed = get<0>(dimsq) == n &&
           get<1>(dimsq) == m &&
           get<2>(dimsq) == r;

  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(q.size() == n*m*r, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
        for (size_t k = 0; k < r; ++k) {
          gid_type g(i,j,k);
          q.set_element(g, i*n+j);
        }
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(q.get_element(g)) != i*n+j)
          passed = false;
      }
    }
  }

  // check if set_element test passed on all locations
  stapl_bool req(passed);
  global_result = req.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
      for (size_t k = 0; k < r; ++k) {
        gid_type g(i,j,k);
        if (static_cast<size_t>(q[g]) != i*n+j)
          passed = false;
      }
    }
  }

  req = stapl_bool(passed);
  global_result = req.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")
  // =====================================================================

  multiarray_type3 th(gid_type2(n,m));
  passed = true;
  passed = th.size() == n*m;
  STAPL_TEST_REPORT(passed,"Testing dimensions_type Constructor 2D")

  multiarray_type3::dimensions_type dimsth = th.dimensions();
  passed = get<0>(dimsth) == n &&
           get<1>(dimsth) == m;

  STAPL_TEST_REPORT(passed, "Testing dimensions")
  STAPL_TEST_REPORT(th.size() == n*m, "Testing size")

  do_once([&](void) {
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < m; ++j) {
          gid_type2 g(i,j);
          th.set_element(g, i*n+j);
      }
    }
  });

  STAPL_TEST_REPORT(true, "Testing set_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
        gid_type2 g(i,j);
        if (static_cast<size_t>(th.get_element(g)) != i*n+j)
          passed = false;
    }
  }

  // check if set_element test passed on all locations
  stapl_bool reth(passed);
  global_result = reth.reduce();
  STAPL_TEST_REPORT(global_result, "Testing get_element")

  passed = true;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) {
        gid_type2 g(i,j);
        if (static_cast<size_t>(th[g]) != i*n+j)
          passed = false;
    }
  }

  reth = stapl_bool(passed);
  global_result = reth.reduce();
  STAPL_TEST_REPORT(global_result, "Testing operator[]")
  // =====================================================================

  return EXIT_SUCCESS;
}
