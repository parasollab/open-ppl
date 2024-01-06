/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/heap/base_container.hpp>
#include <stapl/containers/distribution/container_manager/cm_heap.hpp>
#include <vector>

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>

#include "../../test_report.hpp"

using namespace stapl;

typedef  heap_base_container<int,std::less<int>,
         heap_base_container_traits<int,std::less<int> > >   bc_type;
typedef container_manager_heap<bc_type>  cm_type;
typedef cm_type::iterator cm_iter;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);
  srand(time(NULL));

  //Create and fill a base container
  bc_type* ptr_bc = new bc_type();
  for (size_t i = 0 ; i < n ; i++) {
    ptr_bc->push(i);
  }

  //Create random number of partition
  int my_f = 0;
  int nb_part = 0;
  if (n > 10) {
    my_f = rand() % ((int) (n/10));
  }
  if (my_f == 0) {
    nb_part = 1;
  } else {
    nb_part = my_f;
  }

  //Create domain, partitionner, mapper
  indexed_domain <size_t> dom(0,n-1); //Domain from 0 to n-1
  //Creating trunc(n/2) partition
  balanced_partition< indexed_domain<size_t> >  part(dom,nb_part);
  //Map the partitions to the previous domain
  mapper<size_t> Mapper(indexed_domain<size_t> (0,nb_part - 1));

  //Create container managers
  cm_type cm1(part,Mapper);
  cm_type cm2(cm1);
  cm_iter ite1 = cm1.begin();
  bool passed = true;

  //If cm1 hold data, return false
  if (cm1.begin() != cm1.end()) {
    passed = false;
  }

  STAPL_TEST_REPORT(passed,"Testing constructors");
  passed = true;

  //If size !=0 after clear, return false
  cm2.clear();
  if (cm2.size() != 0) {
    passed = false;
  }

  STAPL_TEST_REPORT(passed,"testing clear() and size()");
  passed = true;

  //Push_bc test aborted, pushing several same base containers
  //works but crash at the end of program.
  //The program try to free the same container several time ...
  cm2.push_bc(ptr_bc);

  if (cm2.size() != n)
   passed = false;

  STAPL_TEST_REPORT(passed,"testing push_bc()");
  passed = true;

  //Create a fake gid and check if it is in cm2
  for (ite1 = cm2.begin() ; ite1 != cm2.end() ; ite1++) {
    bc_type::gid_type my_gid((*(cm2.begin()))->end(),
                              NULL, get_location_id() + 1);
    if (cm2.contains(my_gid)) {
      passed = false;
    }
  }

  //Create a real gid and check if it is in cm2
  (*(*(cm2.begin()))).push(n);
  bc_type::gid_type my_gid((*(cm2.begin()))->end(),
                    *(cm2.begin()),get_location_id());
  if (!cm2.contains(my_gid)) {
    passed = false;
    std::cout<<"Element n not contained"<<std::endl;
  }

  STAPL_TEST_REPORT(passed,"Testing contains(gid)");

  return EXIT_SUCCESS;
}
