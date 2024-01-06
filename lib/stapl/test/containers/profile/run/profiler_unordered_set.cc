/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/unordered_set/unordered_set.hpp>
#include <stapl/utility/do_once.hpp>
#include "../profiling_util.hpp"
#include "../value_type_util.hpp"
#include "../adt.hpp"
#include <vector>

//for each
#include <stapl/algorithms/algorithm.hpp>

using namespace stapl;
using namespace profiling;

using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::string;

size_t N;
size_t CPL;

template<class Key> struct my_hash;

template<class Key>
using unordered_set_type =
  unordered_set<Key, my_hash<Key>, stapl::equal_to<Key>,
                unordered_map_partition<my_hash<Key>,continuous_domain<Key>>
               >;

////////////////////////////////////////////////////////////////////////////////
/// @brief Populates a vector with strings of length between 1 and 3.
///
/// @param roots The container to hold the strings
/// @param k The size of the container
////////////////////////////////////////////////////////////////////////////////
void generate_strings(vector<string>& roots, size_t k)
{
  for (size_t i=0; i!=k; ++i)
  {
    string s;
    size_t nt = rand() % 3 + 1;
    for (size_t j = 0; j != nt; ++j) {
      char c = char('a' + rand()%26);
      s+=c;
    }
    roots.push_back(s);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Produces a container populated with random values bounded by @p maxn.
///
/// @param vals The container to hold the resulting values
/// @param num_vals The number of values to produce
/// @param maxn The upper bound for value generation
////////////////////////////////////////////////////////////////////////////////
void generate_ints(vector<size_t>& vals, size_t num_vals, size_t max_val)
{
  for (size_t i=0; i!=num_vals; ++i)
    vals[i] = lrand48() % max_val;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns the value passed in.
///
/// @tparam T The value type
////////////////////////////////////////////////////////////////////////////////
template <class T>
struct my_hash
{
  size_t operator()(T const& s) const
  {
    return s;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns the hash value of a string of characters.
////////////////////////////////////////////////////////////////////////////////
template <>
struct my_hash<std::string>
{
  size_t operator()(std::string const& s) const
  {
    unsigned long h = 0;
    for (size_t i = 0; i<s.size(); ++i)
      h = 5*h + s[i];
    return size_t(h);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the insert() function of the unordered set container.
///
/// @tparam ADT The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class ADT, class Input, class Counter=counter<default_timer> >
class us_insert_profiler
  : public adt_profiler<ADT, Counter>
{
  typedef adt_profiler<ADT, Counter>        base_type;
  typedef typename ADT::value_type          value_type;

  /// The input container
  Input*   m_ic;
  /// Iterators for the input container
  typename Input::iterator m_it, m_it_end;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for every index
  ///
  /// @param pcname A string that contains the name of the container
  /// @param pc The container
  /// @param in The input
  //////////////////////////////////////////////////////////////////////////////
  us_insert_profiler(std::string pcname, ADT* pc, Input* in,
                     int argc = 0, char **argv = 0)
  : base_type(pc,pcname+"::insert", in->size(), argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
  }

  void run()
  {
    for (m_it = m_ic->begin();m_it!=m_it_end;++m_it)
      this->m_adt->insert(value_type(*m_it));
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_adt->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the find function of the unordered set container.
///
/// @tparam ADT The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class ADT, class Input, class Counter=counter<default_timer> >
class us_find_profiler
  : public adt_profiler<ADT, Counter>
{
  typedef adt_profiler<ADT, Counter>        base_type;
  typedef typename ADT::value_type          value_type;

  /// The input container
  Input*   m_ic;
  /// Iterators to the input container
  typename Input::iterator m_it, m_it_end;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for every index
  ///
  /// @param pcname A string that contains the name of the container
  /// @param pc The container
  /// @param in The input
  //////////////////////////////////////////////////////////////////////////////
  us_find_profiler(std::string pcname, ADT* pc, Input* in,
                   int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"::find_val", in->size(), argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->m_adt->clear();
    for (;m_it!=m_it_end;++m_it)
      this->m_adt->insert( value_type(*m_it));
    rmi_fence();
  }

  void run()
  {
    for (m_it=m_ic->begin();m_it!=m_it_end;++m_it)
      typename ADT::iterator val = (*(this->m_adt)).find(*m_it);
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_adt->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the erase function of the unordered set container.
///
/// @tparam ADT The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
/// @tparam Timer the work function that returns the time value
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class ADT, class Input, class Counter=counter<default_timer> >
class us_erase_profiler
  : public adt_profiler<ADT, Counter>
{
  typedef adt_profiler<ADT, Counter>            base_type;
  typedef typename ADT::value_type              value_type;

  /// The input container
  Input*   m_ic;
  /// Iterators to the input container
  typename Input::iterator m_it, m_it_end;

public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for every index
  ///
  /// @param pcname A string that contains the name of the container
  /// @param pc The container
  /// @param in The input
  //////////////////////////////////////////////////////////////////////////////
  us_erase_profiler(std::string pcname, ADT* pc, Input* in,
                    int argc = 0, char **argv = 0)
  : base_type(pc, pcname+"::erase", in->size(), argc, argv),
    m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
  }

  void initialize_iteration()
  {
    for (m_it=m_ic->begin();m_it!=m_it_end;++m_it)
      this->m_adt->insert( value_type(*m_it) );
    rmi_fence();
  }

  void run()
  {
    for (m_it=m_ic->begin();m_it!=m_it_end;++m_it)
      this->m_adt->erase(*m_it);
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_adt->clear();
  }

};


////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the functions of the unordered set container.
///
/// @param pcname A string containing the name of the container
/// @param p The container
/// @param words The input data
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class ADT, class SC>
void profile(std::string pcname, ADT& p, SC& words, int argc, char** argv)
{
  us_insert_profiler<ADT, SC>
    aip(pcname, &p, &words, argc, argv);
  aip.collect_profile();
  aip.report();

  us_find_profiler<ADT, SC>
    afp(pcname, &p, &words, argc, argv);
  afp.collect_profile();
  afp.report();

  us_erase_profiler<ADT, SC>
    aep(pcname, &p, &words, argc, argv);
  aep.collect_profile();
  aep.report();

  rmi_fence();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Generate the string test data, initialize the container, and call the
/// profilers.
///
/// @param N The test size
/// @param nroots The number of roots to generate
/// @param rep The length of the words to generate
////////////////////////////////////////////////////////////////////////////////
void evaluate_string(size_t N, int argc, char** argv)
{
  typedef string MYKEY;

  size_t block = N / get_num_locations();
  typedef vector<string> SC;
  ts_print("generate data(strings)\n");
  SC roots;
  generate_strings(roots, block);
  ts_print("done generating data(strings)\n");

  using unordered_set_type = unordered_set_type<MYKEY>;

  ts_print("build unordered_set_stl\n");
  unordered_set_type unordered_stl;
  ts_print("done build unordered_set_stl\n");
  profile("us_stl", unordered_stl, roots, argc, argv);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Generate the int test data, initialize the container, and call the
/// profilers.
///
/// @param N The test size
////////////////////////////////////////////////////////////////////////////////
void evaluate_int(size_t N, int argc, char** argv)
{
  typedef size_t MYKEY;

  size_t block = N / get_num_locations();
  typedef vector<MYKEY> SC;
  SC words(block);
  generate_ints(words, block, N);
  ts_print("done generating data\n");

  using unordered_set_type = unordered_set_type<MYKEY>;

  ts_print("build unordered_set_stl\n");
  unordered_set_type unordered_stl;
  ts_print("done build unordered_map_stl\n");
  profile("us_stl_int", unordered_stl, words, argc, argv);
}

stapl::exit_code stapl_main(int argc,char** argv)
{
  int myid   = stapl::get_location_id();
  int nprocs = stapl::get_num_locations();
  size_t N;
  size_t case_id;
  if (argc < 3) {
    if (myid == 0)
      cout << "***Incorrect number of parameters: using CaseID=0 and N=100\n";
    case_id = 0;
    N       = 100;
  }
  else {
    case_id = atol(argv[1]);
    N       = atol(argv[2]);
  }

  if (N % nprocs != 0) {
    N = N - (N % nprocs);
    if (myid == 0)
      cout<<"***N not evenly divisible by nproc:  using N="<<N<<std::endl;
  }

  if (myid == 0) {
    cout << "Case ID=" << case_id << "\n";
    cout << " Number of procs:" << nprocs << " N=" << N << endl;
  }

  srand(myid);

  if (case_id == 0) {
    ts_print("us string\n");
    evaluate_string( N, argc, argv );
  }
  else if (case_id == 1) {
    ts_print("us int\n");
    evaluate_int( N, argc, argv);
  }
  else {
    ts_print("Invalid case id\n");
  }
  rmi_fence();

  return EXIT_SUCCESS;
}
