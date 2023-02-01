/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/heap/heap.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/containers/distribution/directory/vector_directory.hpp>
#include <stapl/containers/distribution/directory/container_directory.hpp>
#include <vector>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>

#include "../../test_report.hpp"

using namespace stapl;
using namespace std;

typedef heap_base_container_traits
        <int,std::less<int> >                            bc_traits;
typedef heap_base_container
        <int,std::less<int>, bc_traits >                 bc_type;
typedef bc_type::gid_type                                gid_type;
typedef bc_type::value_type                              value_type;
typedef container_manager_heap<bc_type>                  cm_type;
typedef cm_type::iterator                                cm_iter;

typedef balanced_partition< indexed_domain<size_t> >     partitioner_type;
typedef mapper<size_t>                                   mapper_type;
typedef list_manager<partitioner_type, mapper_type>      heap_manager_type;
typedef heap_traits<int, std::less<int>,
        partitioner_type, mapper_type>                   heap_traits_type;
typedef heap<int, std::less<int>,
        partitioner_type, mapper_type,
        heap_traits_type>                                heap_type;

typedef heap_distribution<heap_type>                     dis_type;
typedef heap_traits_type::directory_type                 directory_type;
typedef stapl::vector<size_t>                            vec_type;
typedef stapl::vector_view< vec_type >                   vec_view_type;


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);
  srand(time(NULL));

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
  bool passed = true;

  indexed_domain <size_t> dom(0,n-1); //Domain size_t from 0 to n-1
  partitioner_type  Partitionner(dom,nb_part) ; //Creating trunc(n/2) partition
  //map the partitions to the previous domain
  mapper_type Mapper(indexed_domain<size_t> (0,nb_part - 1));

  //Fake pointer to the heap needed by the distribution
  //At heap level, this pointer will be real
  heap_type* ptr_pheap = new heap_type();

  cm_type cm(Partitionner,Mapper);
  cm_type cm_full(Partitionner, Mapper);
  cm_type cm_full2(Partitionner, Mapper);
  cm_type cm_full3(Partitionner, Mapper);

  directory_type Directory;
  cm_full.push_bc();
  cm_full2.push_bc();
  cm_full3.push_bc();
  for (size_t i = 0 ; i < n ; ++i) {
    (*(*(cm_full.begin()))).push(i);
    (*(*(cm_full2.begin()))).push(i);
    (*(*(cm_full3.begin()))).push(i);
  }

  dis_type distrib1(Directory,cm_full);
  distrib1.end();
  dis_type distrib5(Directory,cm_full3);
  dis_type distrib2(distrib1);
  dis_type distrib3(Directory,cm);

  bc_type* bc = new bc_type();
  for (size_t i = 0 ; i < n ; ++i) {
    bc->push(2*i);
  }
  distrib5.container_manager().push_bc(bc);
  if ( (distrib1.size() != n*get_num_locations()) ||
       distrib5.size() != 2*n*get_num_locations()
       || *(*(distrib1.container_manager().begin()))->begin() != int (n-1)
       || (distrib2.size() != n*get_num_locations())
       || (distrib3.size() != 0) )
    passed = false;

  STAPL_TEST_REPORT(passed,"Testing constructors");
  passed = true;

  for (size_t i = 0 ; i < n ; ++i) {
    int my_r = rand() % 50;
    distrib1.push(my_r);
    distrib3.push(my_r);
  }

  STAPL_TEST_REPORT(passed,"Testing push()");
  passed = true;

  gid_type top_result = distrib5.get_top().first;

  value_type val_read = *(*(distrib5.container_manager().begin()))->begin();
  for (cm_iter it = distrib5.container_manager().begin() ;
       it != distrib5.container_manager().end() ; ++it) {
    for (bc_type::iterator iter = (*it)->begin() ;
         iter != (*it)->end() ; ++iter) {
      if (*iter > val_read)
        val_read = *iter;
    }
  }
  if (val_read != distrib5.get_element(top_result))
    passed = false;

  STAPL_TEST_REPORT(passed,"Testing get_top() and top()");
  passed = true;

  if (stapl::get_location_id() == 0)
    distrib1.push(50 + 2*n);

  value_type result_pop = distrib1.pop();
  if (result_pop != int(50 + 2*n))
    std::cout << "ERROR in pop() !!!" <<std::endl;

  //Look for it in the heap
  for (cm_iter it = distrib1.container_manager().begin() ;
       it != distrib1.container_manager().end() ; ++it) {
    for (bc_type::iterator iter = (*it)->begin() ;
         iter != (*it)->end() ; ++iter) {
      if (*iter == result_pop) {
        passed = false;
       std::cout << "Hum .. this is embarassing ... top element should ";
       std::cout << "have been removed from heap o_O" <<std::endl;
      }
    }
  }

  STAPL_TEST_REPORT(passed,"Testing pop()");
  passed = true;

  if (!distrib1.is_heap()) {
    passed = false;
    std::cout<<"Not heap but should"<<endl;
  }
  cm_iter it = distrib1.container_manager().begin();
  bool is_root = false;
  size_t rand_bc = rand()%(distrib1.container_manager().num_bc());

  //Get a random element in a random bc
  if (rand_bc > 0)
    --rand_bc;
  for (size_t i = 0 ; i < rand_bc ; ++i)
    ++it;
  bc_type::iterator iter = (*it)->begin();
  size_t rand_elem = rand() % ((*it)->size());
  if (rand_elem > 0)
    --rand_elem;
  for (size_t i = 0 ; i < rand_elem ; ++i)
    ++iter;

  //Corrupt value if not top
  if (iter == (*it)->begin())
    is_root = true;
  else (*iter) = 50 + 3*n;

  if (distrib1.is_heap())
    if (!is_root) {
      std::cout<<"Heap but should'nt"<<endl;
      passed = false;
    }

  STAPL_TEST_REPORT(passed,"Testing is_heap()");

  passed = true;
  dis_type distrib_it(Directory,cm_full2);

  size_t k = 0;
  stapl::rmi_fence();
  dis_type::iterator itera = distrib_it.end();
  for (dis_type::iterator iterator = distrib_it.begin() ;
       iterator != itera ; ++iterator) {
    ++k;
  }
  if (k/stapl::get_num_locations() != cm_full2.size())
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing Global iteration");

  delete ptr_pheap;

  return EXIT_SUCCESS;
}
