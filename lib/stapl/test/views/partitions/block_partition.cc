/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/domains/domains.hpp>
#include <p_vector.h>
#include "views/partitions/block_partition.hpp"
#include <iostream>
#include <p_array.h>
#include <cstdlib>
#include <vector>

//////////////////////////////////////////////////////////////////////
/// @file block_partition.cc
/// @todo This test is outdated and is not included in the nightly
///       validation. The tests need to return pass/fail and the used
///       block partition updated to use
///       stapl/containers/partitions/blocked_partition.hpp.
//////////////////////////////////////////////////////////////////////

using namespace std;
using namespace stapl;

void print_vec(std::vector<std::pair<indexed_domain<size_t>,bool > > vec)
{
  for (size_t i=0;i<vec.size();i++)
  {
    std::cout<<vec[i].first<<" value "<<vec[i].second<<endl;
  }
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  indexed_domain<size_t> domainn(19);
  cout << endl << "*********Testing block_partitioning program*****" << endl;
  cout << "The domain to be partitioned is " << domainn << endl;
  block_partitioner<indexed_domain<size_t> > b_partition(domainn,4,true);
  cout << "***Ignoring the last partition(Remainder) " << endl;
  cout << "The number of partitions in the block partitioner: "
       << b_partition.size() << endl;
  cout << endl << "****PRINTING THE INDIVIDUAL BLOCK PARTITIONED DOMAINS****"
       << endl;
  for (size_t i=0;i<b_partition.size();i++)
  {
    cout << "domain is " << b_partition[i] << endl;;
  }
  cout << endl << "*********** TESTING FIND**********" << endl;
  indexed_domain<size_t> dom_find(4,13);
  cout << "The new domain is " <<  dom_find << endl;
  b_partition.find<indexed_domain<size_t>,int > (dom_find,5);
  std::vector<std::pair<indexed_domain<size_t>,bool > > find_vecs;
  find_vecs= b_partition.contained_in<indexed_domain<size_t>,int>(dom_find,5);
  print_vec(find_vecs);

  //****************//
  //testing without ignoring the last domain
  cout << endl << endl << "****Not ignoring the last domain" << endl;
  block_partitioner<indexed_domain<size_t> > B_partition(domainn,4,false);
  cout << "The original domain is " << domainn << endl;
  cout << "The number of partitions in the block partitioner: "
       << B_partition.size() << endl;
  cout << endl << "****PRINTING THE INDIVIDUAL BLOCK PARTITIONED DOMAINS****"
       << endl;
  for (size_t i=0;i<B_partition.size();i++)
  {
    cout << "domain is " << B_partition[i] << endl;;
  }
  cout << endl << "*********** TESTING FIND**********" << endl;
  indexed_domain<size_t> Dom_find(3,16);
  cout << "The new domain is " << Dom_find << endl;
  B_partition.contained_in<indexed_domain<size_t>,int > (Dom_find,5);
  std::vector<std::pair<indexed_domain<size_t>,bool > > Find_vecs;
  Find_vecs= B_partition.contained_in<indexed_domain<size_t>,int>(Dom_find,5);
  print_vec(Find_vecs);

  return EXIT_SUCCESS;
}
