/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/sorting.hpp>
#include <stapl/algorithms/generator.hpp>
#include <stapl/containers/partitions/splitter.hpp>
#include <stapl/views/segmented_view.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Comparator for std::pair<K, V> elements.
/// @tparam A Value type of the elements to compare.
//////////////////////////////////////////////////////////////////////
template<typename A>
struct AwesomeComparator
  : public stapl::ro_binary_function<A, A, bool>
{
  // generation comparison
  bool operator()(std::size_t a, std::size_t b)
  { return (a > b); }

  // objects' data comparison
  bool operator()(std::pair<std::size_t, std::size_t> l,
                  std::pair<std::size_t, std::size_t> r)
  { return (l.second > r.second); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor to check the correctness for n_partition.
///   The second elements of std::pair<K, V> are checked.
//////////////////////////////////////////////////////////////////////
struct validator
{
  template<typename L, typename R>
  bool operator()(L l, R r)
  { return (l.second == r.second); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function that will process the elements of each partition.
//////////////////////////////////////////////////////////////////////
template<typename Compare>
struct partition_functor
{
  typedef void result_type;

  Compare m_comp;

  partition_functor(Compare c)
    : m_comp(c)
  {}

  template<typename Bucket, typename Offset>
  result_type operator()(Bucket&& b, Offset&&)
  {
    std::sort(b.begin(), b.end(),
              stapl::algo_details::comparator_wrapper<Compare>(m_comp));
  }

  void define_type(stapl::typer& t)
  { t.member(m_comp); }
};


typedef std::size_t                                         key_type;
typedef std::size_t                                         value_t;
typedef std::pair<key_type, value_t>                        data_value_t;
typedef stapl::static_array<data_value_t>                   data_ct;
typedef stapl::array_view<data_ct>                          data_vt;
typedef data_vt::domain_type                                domain_t;
typedef stapl::splitter_partition<domain_t>                 split_part_t;
typedef stapl::segmented_view<data_vt, split_part_t>      partitions_vt;
typedef AwesomeComparator<data_value_t>                     comparator_t;


struct check_functor1
{
  size_t           m_size;
  partitions_vt&   m_result;
  data_vt&         m_data_cp_v;
  comparator_t     m_comparator;
  std::string      m_str;
  double           m_exec_time;

  check_functor1(size_t sz, partitions_vt& result, data_vt& data_cp_v,
                 comparator_t comparator, std::string str, double exec_time)
    : m_size(sz), m_result(result), m_data_cp_v(data_cp_v),
      m_comparator(comparator), m_str(str), m_exec_time(exec_time)
  { }

  typedef void result_type;

  void operator()(void) const
  {
    // partitions receptacle
    std::vector<data_value_t> data_concat_v(m_size);

    // sort() each partition & concatenate them into a data container
    std::size_t i = 0;
    for (partitions_vt::iterator partIt = m_result.begin();
         partIt != m_result.end(); ++partIt) {
      #pragma warning ( disable : 473 )
      std::sort((*partIt).begin(), (*partIt).end(), m_comparator);
      std::copy((*partIt).begin(), (*partIt).end(), data_concat_v.begin() + i);
      i += (*partIt).size();
    }

    // sort copied data
    std::sort(m_data_cp_v.begin(), m_data_cp_v.end(), m_comparator);

    bool result=false;
    // check output
    if (std::equal(data_concat_v.begin(), data_concat_v.end(),
                   m_data_cp_v.begin(), validator()))
      result=true;
    printf("Test: n_partition_");
    std::cout << m_str << "\nStatus: ";
    if (result)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\nTime: %f\n\n", m_exec_time);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::counter<stapl::default_timer> timer;

  std::size_t nb_data = 10;
  std::size_t nb_splitters = nb_data;

  // set variables if arguments are given to the executable
  for (int i = 1; i < argc; ++i) {
    if      (strcmp("-d", argv[i]) == 0)
      nb_data = boost::lexical_cast<std::size_t>(argv[++i]);
    else if (strcmp("-s", argv[i]) == 0)
      nb_splitters = boost::lexical_cast<std::size_t>(argv[++i]);
  }

  comparator_t comparator;

  typedef stapl::sequence<key_type>        seq_t;
  typedef stapl::random_sequence        rdm_seq_t;
  typedef stapl::unique_sorted_sequence us_seq_t;

  seq_t     seq_data(0, 1);
  rdm_seq_t rdm_seq_data;
  seq_t     seq_splitters(0, 1);
  us_seq_t  us_seq_splitters(nb_splitters, comparator);

  stapl::associative_sequence<seq_t, rdm_seq_t, key_type, value_t>
    data_seq(seq_data, rdm_seq_data);

  stapl::associative_sequence<seq_t, us_seq_t, key_type, value_t>
    spl_seq(seq_splitters, us_seq_splitters);

  /*
   *  n_partition implicitly shares splitters
   */
  // data
  data_ct data(nb_data);
  data_vt data_v(data);

  stapl::generate(data_v, data_seq);

  // for test correctness
  data_ct data_cp(nb_data);
  data_vt data_cp_v(data_cp);

  stapl::copy(data_v, data_cp_v);

  // splitters
  data_ct unique_splitters(nb_splitters);
  data_vt unique_splitters_v(unique_splitters);

  stapl::generate(unique_splitters_v, spl_seq);

  // here we go
  timer.reset();
  timer.start();

  partitions_vt result = stapl::n_partition(data_v,
                                            unique_splitters_v,
                                            comparator);
  double exec_time = timer.stop();

  stapl::do_once(check_functor1(data_v.size(), result, data_cp_v,
                                comparator, "implicit", exec_time));

  /*
  *  n_partition takes explicit shared splitters
  */
  typedef std::vector<data_value_t>      splitters_t;
  typedef stapl::array_view<splitters_t> splitters_vt;

  // data
  data_ct data_second(nb_data);
  data_vt data_second_v(data_second);

  stapl::generate(data_second_v, data_seq);

  // for test correctness
  data_ct data_cp_second(nb_data);
  data_vt data_cp_second_v(data_cp_second);

  stapl::copy(data_second_v, data_cp_second_v);

  // splitters
  splitters_t unique_splitters_second(nb_splitters);
  splitters_vt unique_splitters_second_v(unique_splitters_second);

  std::generate(unique_splitters_second_v.begin(),
                unique_splitters_second_v.end(),
                spl_seq);

  // here we go
  partition_functor<comparator_t> f(comparator);

  timer.reset();
  timer.start();

  partitions_vt result_second = stapl::n_partition(data_second_v,
                                          unique_splitters_second_v.container(),
                                          comparator,
                                          f);
  exec_time = timer.stop();

  stapl::do_once(check_functor1(data_second_v.size(), result_second,
                  data_cp_second_v, comparator, "explicit", exec_time));

  return EXIT_SUCCESS;
}
