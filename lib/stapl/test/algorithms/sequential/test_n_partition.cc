/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>

#include <stapl/algorithms/sorting.hpp>
#include <stapl/algorithms/generator.hpp>
#include <stapl/algorithms/sequential/n_partition.hpp>
#include <stapl/views/array_view.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Comparator for std::pair<K, V> elements.
/// @tparam T std::pair type of the elements to be compared.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct AwesomeComparator
  : public stapl::ro_binary_function<T, T, bool>
{
  // generation comparison
  bool operator()(std::size_t a, std::size_t b)
  { return (a < b); }

  // objects' data comparison
  bool operator()(T l, T r)
  { return (l.second < r.second); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor to validate output for n_partition.
///   The second elements of std::pair<K, V> are checked.
//////////////////////////////////////////////////////////////////////
struct validator
{
  template<typename T>
  bool operator()(T l, T r)
  { return (l.second == r.second); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef std::size_t                   k_t;
  typedef std::size_t                   v_t;

  typedef std::pair<k_t,v_t>            value_t;
  typedef std::vector<value_t>          s_cont_t;
  typedef stapl::array_view<s_cont_t>   s_cont_vt;
  typedef s_cont_vt::iterator           s_cont_it;

  typedef AwesomeComparator<value_t>    comparator_t;

  typedef stapl::splitter_partition<stapl::indexed_domain<std::size_t> >
    split_part_t;
  typedef stapl::segmented_view<s_cont_vt, split_part_t>
    partitions_vt;

  size_t nb_data = 10;
  size_t nb_splitters = nb_data;

  // set variables if arguments are given to the executable
  for (int i = 1; i < argc; ++i) {
    if      (strcmp("-d", argv[i]) == 0)
      nb_data = boost::lexical_cast<std::size_t>(argv[++i]);
    else if (strcmp("-s", argv[i]) == 0)
      nb_splitters = boost::lexical_cast<std::size_t>(argv[++i]);
  }

  comparator_t comparator;

  typedef stapl::sequence<k_t>          seq_t;
  typedef stapl::random_sequence        rdm_seq_t;
  typedef stapl::unique_sorted_sequence us_seq_t;

  seq_t     seq_data(0, 1);
  rdm_seq_t rdm_seq_data;
  seq_t     seq_splitters(0, 1);
  us_seq_t  us_seq_splitters(nb_splitters, comparator);

  stapl::associative_sequence<seq_t, rdm_seq_t, k_t, v_t>
    data_seq(seq_data, rdm_seq_data);

  stapl::associative_sequence<seq_t, us_seq_t, k_t, v_t>
    spl_seq(seq_splitters, us_seq_splitters);

  // data
  s_cont_t data(nb_data);
  s_cont_vt data_v(data);

  std::generate(data_v.begin(), data_v.end(), data_seq);

  // for test correctness
  s_cont_t data_cp(nb_data);
  s_cont_vt data_cp_v(data_cp);
  std::copy(data_v.begin(), data_v.end(), data_cp_v.begin());

  // splitters | unique
  s_cont_t unique_splitters(nb_splitters);
  s_cont_vt unique_splitters_v(unique_splitters);

  std::generate(unique_splitters_v.begin(), unique_splitters_v.end(), spl_seq);
  std::sort(unique_splitters_v.begin(), unique_splitters_v.end(), comparator);
  s_cont_it last_spl = std::unique(unique_splitters_v.begin(),
                                   unique_splitters_v.end());
  unique_splitters_v.container()
    .resize(std::distance(unique_splitters_v.begin(), last_spl));

  // here we go
  partitions_vt result = stapl::sequential::n_partition(data_v,
                                                        unique_splitters_v,
                                                        comparator);

  // output
  // partitions receptacle
  s_cont_t data_concat(data.size());

  // sort() each partition & concatenate them into a data container
  std::size_t i = 0;
  for (partitions_vt::iterator partIt = result.begin(); partIt != result.end();
    ++partIt) {
    std::sort((*partIt).begin(), (*partIt).end(), comparator);
    std::copy((*partIt).begin(), (*partIt).end(), data_concat.begin() + i);
    i += (*partIt).size();
  }

  // sort copied data
  std::sort(data_cp.begin(), data_cp.end(), comparator);

  // check output
  if (data_concat.size() == data_cp.size()
      && std::equal(data_concat.begin(), data_concat.end(), data_cp.begin(),
                    validator()))
    std::cout << "sequential n_partition PASSED\n";
  else
    std::cout << "sequential n_partition FAILED\n";

  return EXIT_SUCCESS;
}

