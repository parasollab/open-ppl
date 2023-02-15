/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <map>
#include <set>
#include <cmath>
#include <ctime>
#include <cstdio>
#include <string>
#include <cstdlib>

#include <iostream>
#include <fstream>
#include <list>
#include <stapl/algorithms/functional.hpp>

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>

#include <boost/random/uniform_int_distribution.hpp>
#include <stapl/utility/random.hpp>


using namespace std;

typedef stapl::identity<int>    id_int_wf;
typedef stapl::identity<size_t> id_un_wf;
typedef stapl::negate<int>      neg_int_wf;

typedef stapl::plus<int>        add_int_wf;
typedef stapl::minus<int>       sub_int_wf;
typedef stapl::multiplies<int>  mul_int_wf;
typedef stapl::min<int>         min_int_wf;
typedef stapl::max<int>         max_int_wf;

typedef stapl::bit_xor<size_t>  xor_un_wf;
typedef stapl::bit_or<size_t>   ior_un_wf;
typedef stapl::bit_and<size_t>  and_un_wf;

// random generator for populating containers.
struct rand_gen
{
  boost::random::mt19937 m_rng;
  typedef boost::random::uniform_int_distribution<size_t> rng_dist_t;

  rand_gen(size_t seed=std::time(0))
    : m_rng(seed)
  { }

  size_t rand(size_t min, size_t max)
  {
    if (max == std::numeric_limits<size_t>::max())
      return rng_dist_t(min, max-1)(m_rng);
    else
      return rng_dist_t(min, max)(m_rng);
  }
};

//size_t xor_second(const size_t lhs, const std::pair<size_t, size_t> & rhs)
size_t xor_second(const size_t lhs,
                  std::map<size_t,size_t>::value_type const& rhs)
{
  return lhs ^ rhs.second;
}


// populate innermost level map
std::map<size_t,size_t> initialize_map(size_t n)
{
  std::map<size_t, size_t> inner_map;

  rand_gen gen;
  auto random_number = gen.rand(1, n);

  for (size_t i = 0; i < random_number; i++)
  {
    inner_map.insert(std::pair<size_t, size_t>(i, gen.rand(1,n)));
  }

  return inner_map;
}


struct pop_linear_assoc
{
private:
  stapl::stream<ifstream> m_zin;
  size_t m_n;

public:
  pop_linear_assoc(stapl::stream<ifstream> const& zin, size_t n)
    : m_zin(zin), m_n(n)
  { }

  typedef size_t result_type;
  template <typename InCont>
  result_type operator()(InCont view)
  {
    std::size_t num;
    rand_gen gen;
    for (auto i = 0; i != m_n; ++i)
    {
      m_zin >> num;
      view.insert(std::pair<size_t,size_t>(i, gen.rand(1,m_n)));
    }

    auto val = std::accumulate(view.begin(), view.end(), 0, xor_second);
    return val;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_n);
  }
};

struct pop_assoc_assoc
{
private:
  stapl::stream<ifstream> m_zin;
  size_t m_n;

public:
  pop_assoc_assoc(stapl::stream<ifstream> const& zin, size_t n)
    : m_zin(zin), m_n(n)
  { }

  typedef size_t result_type;
  template <typename InCont>
  result_type operator()(InCont view)
  {
    std::size_t num;
    rand_gen gen;
    for (auto i = 0; i != m_n; ++i)
    {
      m_zin >> num;
      view.second.insert(std::pair<size_t,size_t>(i, gen.rand(1,m_n)));
    }

    auto val = std::accumulate(view.second.begin(),
      view.second.end(), 0, xor_second);
    return val;

  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_n);
  }
};

// struct to populate linear container
struct pop_linear
{
private:
  stapl::stream<ifstream> m_zin;

public:
  pop_linear(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef size_t result_type;
  template <typename InCont>
  result_type operator()(InCont view)
  {
    std::size_t num;
    for (auto iter = view.begin(); iter != view.end(); ++iter) {
      m_zin >> num;
      (*iter) = num;
    }
    return std::accumulate(view.begin(), view.end(), 0, xor_un_wf());
  }

  void define_type(stapl::typer& t) {
    t.member(m_zin);
  }
};


// struct to populate associative containers
struct pop_assoc
{
private:
  stapl::stream<ifstream> m_zin;

public:
  pop_assoc(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef size_t result_type;
  template <typename InCont>
  result_type operator()(InCont elem)
  {
    std::size_t num;
    //m_zin >> num;
    for (int i=0; i < elem.second.size(); i++)
    {
      m_zin >> num;
      elem.second[i] = num;
    }

    return std::accumulate(elem.second.begin(), elem.second.end(), 0,
      xor_un_wf());
  }

  void define_type(stapl::typer& t) {
    t.member(m_zin);
  }
};

// struct to populate graph vertex property
struct pop_graph
{
private:
  std::size_t m_n;
  stapl::stream<ifstream> m_zin;

public:
  pop_graph(std::size_t n, stapl::stream<ifstream> const& zin)
    : m_n(n), m_zin(zin)
  { }

  typedef size_t result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    std::size_t num;
    for (int i=0; i < m_n; i++)
    {
      m_zin >> num;
      v.property().push_back(num);
    }

    return std::accumulate(v.property().begin(), v.property().end(),
      0, xor_un_wf());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_n);
    t.member(m_zin);
  }
};

struct pop_graph_assoc
{
private:
  std::size_t m_n;
  stapl::stream<ifstream> m_zin;

public:
  pop_graph_assoc(std::size_t n, stapl::stream<ifstream> const& zin)
    : m_n(n), m_zin(zin)
  { }

  typedef size_t result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    std::size_t num;
    rand_gen gen;

    for (int i=0; i < m_n; i++)
    {
      m_zin >> num;
      v.property().insert(std::pair<size_t,size_t>(i, gen.rand(1,m_n)));
    }

    return std::accumulate(v.property().begin(), v.property().end(),
      0, xor_second);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_n);
    t.member(m_zin);
  }
};

void set_filendata(size_t& data_size, char * arg, std::string &input_filename)
{
  switch ( arg[0]  ) {
  case 't':
    data_size = 1;
    input_filename = "data/tiny_bits.zin";
    break;
  case 's':
    data_size = 100;
    input_filename = "data/small_bits.zin";
    break;
  case 'm':
    data_size = 10000;
    input_filename = "data/medium_bits.zin";
    break;
#ifdef BIG_HUGE_DATA
  case 'b':
    data_size = 10000000;
    input_filename = "data/big_bits.zin";
    break;
  case 'h':
    data_size = 100000000;
    input_filename = "data/huge_bits.zin";
    break;
#endif
  default:
    std::cerr << "usage: exe --data tiny/small/medium/big/huge\n";
    exit(1); // exit
  }
}

void print_tests(std::set<std::string> &test_set)
{
  std::cerr << "Tests:\n";
  int i = 1;
  for (auto it = test_set.begin(); it != test_set.end(); ++it) {
    std::cerr << i << ". " << (*it) << "\n";
    ++i;
  }
  exit(1); // must exit program
}

void select_tests(std::vector<std::string> &tests_to_run,
  std::set<std::string> &test_set, char const * arg)
{
  if (test_set.find(arg) != test_set.end()) {
    tests_to_run.push_back(arg);
  }
  else {
    std::cerr << "\nInvalid test name\n";
    print_tests(test_set);
    exit(1);
  }
}

struct inner_seq_elem_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return elem;
  }
};



struct inner_map_elem_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    typename Element::second_reference map_arg = elem.second;
    return map_arg;
  }
};

struct l1_map_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return std::accumulate(elem.second.begin(), elem.second.end(), 0,
      xor_second);
  }
};

struct l1_map_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return std::accumulate(elem.second.begin(), elem.second.end(), 0,
      xor_un_wf());
  }
};

struct l1_seq_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    auto val = std::accumulate(elem.begin(), elem.end(), 0, xor_second);
    return val;
  }
};

struct l1_seq_l0_seq_cksum_wf
{
  typedef size_t result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return std::accumulate(elem.begin(), elem.end(), 0, xor_un_wf());
  }
};

struct l2_seq_l1_seq_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_seq_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_map_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_map_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_map_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_map_l1_seq_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
                             elem.second);
  }
};

struct l2_map_l1_seq_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_map_cksum_wf(), xor_un_wf(),
                             elem.second);
  }
};

struct l2_map_l1_map_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(),
                             elem.second);
  }
};

struct l2_map_l1_map_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(),
                             elem.second);
  }
};

//-----

struct inner_graph_elem_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return elem.property();
  }
};

struct outer_graph_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_graph_elem_wf(), xor_un_wf(),
                             elem.property());
  }
};

struct outer_graph_inner_map_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return std::accumulate(elem.property().begin(), elem.property().end(), 0,
      xor_second);
  }
};

struct outer_graph_cont_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return std::accumulate(elem.property().begin(), elem.property().end(),
      0, xor_un_wf());
  }
};



struct roll_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 length, View2 val) const
  {
    length = 1 + (rand() % val);
  }
};

struct inner_roll_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 length, View2 val) const
  {
    length = 1 + (rand() % val);
  }
};

struct outer_roll_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 v1, View2 v2) const
  {
    stapl::map_func(inner_roll_wf(), v1, stapl::repeat_view(v2));
  }
};

struct tuple2_rand_init_wf
{
  typedef void result_type;
  template<typename TupleRef, typename Elem>
  result_type operator()(TupleRef ref, Elem val) const
  {
    const size_t x = 1 + (rand() % val);
    const size_t y = 1 + (rand() % val);
    ref = stapl::make_tuple(x,y);
  }
};

struct tuple3_rand_init_wf
{
  typedef void result_type;
  template<typename TupleRef, typename Elem>
  result_type operator()(TupleRef ref, Elem val) const
  {
    const size_t x = 1 + (rand() % val);
    const size_t y = 1 + (rand() % val);
    const size_t z = 1 + (rand() % val);
    ref = stapl::make_tuple(x,y,z);
  }
};

///////////////////////////////////////////////////////////////////////////////
// Prints help options for the user. Function is invoked if user provides
// -h or --help as command line arguments.
///////////////////////////////////////////////////////////////////////////////
void show_help()
{
  stapl::do_once([&]() {
    std::cerr << "Allowed options: \n";
    std::cerr << "\t-h [ --help ] \t\t Print this help message \n";
    std::cerr << "\t-d [ --data ] arg \t Data set \n";
    std::cerr << "\t-l [ --list ] \t\t Print tests provided \n";
    std::cerr << "\t-t [ --test ] arg \t Test to run \n";
    std::cerr << "\t-i [ --iterations ]\t arg Number of times test is repeated";
    std::cerr << " in timed section\n";
    std::cerr << "\n";
    std::cerr << "\nNOTE: With no options the program will run all tests in";
    std::cerr << "a medium data set 32 times.\n";
  });
}

void print_results(const char *name, stapl::tuple<bool, double> fin_res,
  int num_runs)
{
  bool passed = stapl::get<0>(fin_res);
  double exec_time = stapl::get<1>(fin_res) / num_runs;
  stapl::do_once([&](){
    std::cerr << "Test: " << name << std::endl;
    if (passed == true)
      std::cerr << "Status: PASSED" << std::endl;
    else
      std::cerr << "Status: FAILED" << std::endl;

    std::cerr << "Time: " << std::to_string(exec_time).c_str()
      << "\n" << std::endl;
  });
}
