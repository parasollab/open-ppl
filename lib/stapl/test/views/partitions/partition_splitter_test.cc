/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <cstdlib>
#include <vector>
#include <iostream>
#include <algorithm>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <stapl/containers/partitions/splitter.hpp>
#include <stapl/domains/indexed.hpp>
#include "../../test_report.hpp"

// #define VERBOSE 1

template <typename Part, typename ResVec, typename Dom>
bool test_find_result(Part const & p, ResVec const& resvec, Dom const& dom)
{
  size_t num_doms = resvec.size();
  typedef typename ResVec::value_type::first_type dom_t;
  dom_t fdom = resvec[0].first;

  if (num_doms==1) {
    if (dom.contains(p[fdom.first()].first()) &&
        dom.contains(p[fdom.last()].last()) )
      return true;
  }
  if (num_doms==2) {
    if (dom.contains(p[fdom.first()].first()) &&
        dom.contains(p[fdom.last()].last()) &&
        resvec[0].second & !resvec[1].second)
      return true;
  }
  return false;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::indexed_domain<size_t> dom_t;
  dom_t domainn(10);
#ifdef VERBOSE
  std::cout << "Test program"<< std::endl;
  std::cout << "Domain size is " << domainn.size()<< std::endl;;
  std::cout << "Domain first is " << domainn.first()<< std::endl;
  std::cout << "Domain last is " << domainn.last()<< std::endl;
#endif

  std::vector<size_t> vec;
  boost::random::mt19937 gen(5);
  boost::random::uniform_int_distribution<size_t> dist(0, 9);
  vec.push_back(dist(gen));
  vec.push_back(dist(gen));
  vec.push_back(dist(gen));
  vec.push_back(dist(gen));
  vec.push_back(dist(gen));
  vec.push_back(dist(gen));
  std::sort(vec.begin(),vec.end());
  std::vector<size_t>::iterator new_end = std::unique(vec.begin(),vec.end());
  vec.resize(std::distance(vec.begin(), new_end));

#ifdef VERBOSE
  std::cout <<"elements of partitioner are "<< std::endl;
  for (size_t i=0;i<vec.size();i++)
    std::cout << vec[i] <<" ";
  std::cout << std::endl;
#endif

  stapl::splitter_partition<dom_t, std::vector<size_t> > part(domainn,vec);

#ifdef VERBOSE
  size_t size_of_partitioner = part.size();
  std::cout << "No of partitions are: "
            << size_of_partitioner << std::endl;
  for (size_t i=0;i<part.size();++i) {
    std::cout << "Size of partition  "<< i <<" is "
              << part[i].size() << std::endl;
    std::cout << "Range of the sub domain " << i << " is "
              << part[i] << std::endl;
  }
#endif

 std::vector<std::pair<dom_t,bool> > new_vec;
  new_vec = part.contained_in<dom_t,size_t > (dom_t(1,5),3);

#ifdef VERBOSE
  std::cout << "Trying contained_in for the domain (1,5)" << std::endl;
  for (size_t i=0;i<new_vec.size();i++) {
    std::cout << "Domain range "<< new_vec[i].first << std::endl;
    std::cout << (new_vec[i].second==true? "true":"false") << std::endl;
  }
#endif
  bool res = test_find_result(part,new_vec,dom_t(1,5));

  new_vec = part.contained_in<dom_t,size_t >(dom_t(1,4),3);

#ifdef VERBOSE
  std::cout << std::endl << "trying contained_in for the domain (1,4)"
            << std::endl;
  for (size_t i=0;i<new_vec.size();i++) {
    std::cout << "Domain range "<< new_vec[i].first << std::endl;
    std::cout << (new_vec[i].second==true? "true":"false") << std::endl;
  }
#endif
  res &= test_find_result(part,new_vec,dom_t(1,4));

  STAPL_TEST_REPORT(res,"Testing splitter_partition_test");

  return EXIT_SUCCESS;
}
