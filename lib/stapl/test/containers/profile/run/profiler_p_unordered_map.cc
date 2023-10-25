/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "../p_container_profiler.hpp"
#include "../p_container_algo_profiler.hpp"
#include "../profiler_util.h"
#include "../value_type_util.h"
#include "stapl/containers/unordered_map/unordered_map.hpp"
#include <string>
#include <vector>

//for each
#include <stapl/algorithms/algorithm.hpp>

#ifdef TBB_AVAIL
//tbb component
#include "pContainers/associative/phash_map/p_hash_map_tbb.h"

//tbb stuff
#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"
#include "tbb/tick_count.h"
#include "tbb/task_scheduler_init.h"

#endif

using namespace stapl;
using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::string;

size_t N;
size_t CPL;

////////////////////////////////////////////////////////////////////////////////
/// @brief Populates a vector with strings of length between 1 and 3.
///
/// @param roots The container to hold the strings
/// @param k The size of the container
////////////////////////////////////////////////////////////////////////////////
void generate_roots(vector<string>& roots, size_t k)
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
/// @brief Uses a vector of roots to generate a vector of words.
///
/// @param words The container to hold the resulting words
/// @param roots The container of roots to build the words from
/// @param nwords The number of words to generate
/// @param k The length of the words
////////////////////////////////////////////////////////////////////////////////
void generate_words(vector<string>& words, vector<string>& roots,
                    size_t nwords, size_t k)
{
  size_t nroots = roots.size();
  for (size_t i=0;i!=nwords;++i)
  {
    for (size_t j = 0; j != k; ++j) {
      size_t pos = rand() % nroots;
      words[i]+=roots[pos];
      words[i]+=" ";
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Produces a container populated with random values bounded by @p maxn.
///
/// @param vals The container to hold the resulting numbers
/// @param num_vals The number of numbers to produce
/// @param max_val The upper bound for number generation
////////////////////////////////////////////////////////////////////////////////
void generate_ints(vector<size_t>& vals, size_t num_vals, size_t max_val)
{
  for (size_t i=0; i!=num_vals; ++i)
    vals[i] = lrand48() % max_val;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Increments the second value of a pair.
////////////////////////////////////////////////////////////////////////////////
struct update_elem
{
  template <class T>
  void operator()(T& data) const
  {
    data.second += 1;
  }

  template <class T>
  void operator()(T& data, T const&) const
  {
    data.second += 1;
  }
};

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
    unsigned long __h = 0;
    for (size_t i = 0; i<s.size(); ++i)
      __h = 5*__h + s[i];
    return size_t(__h);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Provides the facilities to compare two string map entries.
////////////////////////////////////////////////////////////////////////////////
struct map_compare
{
  size_t operator()(std::string const& s) const
  {
    return map_compare::map(s);
  }

  static size_t map( std::string const& x )
  {
    size_t h = 0;
    for ( const char* s = x.c_str(); *s; s++ )
      h = (h*17)^*s;
    return h;
  }
  //! True if strings are equal
  static bool equal( std::string const& x, std::string const& y )
  {
    return x==y;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Provides the facilities to compare tow int map entries.
////////////////////////////////////////////////////////////////////////////////
struct map_compare_int
{
  size_t operator()(size_t const& s) const
  {
    return map_compare_int::map(s);
  }

  static size_t map( size_t const& x )
  {
    return x;
  }
  //! True if strings are equal
  static bool equal( size_t const& x, size_t const& y )
  {
    return x==y;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the insert() function of the unordered map container.
///
/// @tparam pC The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Input, class Counter=counter<default_timer> >
class um_insert_profiler
  : public p_container_profiler<pC,Counter>
{
  typedef p_container_profiler<pC,Counter> base_type;
  typedef typename pC::value_type          value_type;

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
  um_insert_profiler(std::string pcname, pC* pc, Input* in,
                    int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"_insert", argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->n_times = m_ic->size();
  }

  void run()
  {
    for (m_it = m_ic->begin();m_it!=m_it_end;++m_it)
      this->m_pc->insert(value_type(*m_it,0));
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_pc->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the insert() function with a predicate of the unordered map
/// container.
///
/// @tparam pC The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Input, class Counter=counter<default_timer> >
class um_insert_pred_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC,Counter> base_type;
  typedef typename pC::value_type          value_type;

  /// The input container
  Input*   m_ic;
  /// iterators for the input container
  typename Input::iterator m_it, m_it_end;
  /// The work function to apply to the container
  update_elem pred;

public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for every index
  ///
  /// @param pcname A string that contains the name of the container
  /// @param pc The container
  /// @param in The input
  //////////////////////////////////////////////////////////////////////////////
  um_insert_pred_profiler(std::string pcname, pC* pc, Input* in,
                          int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"_insert_pred", argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->n_times = m_ic->size();
  }

  void run()
  {
    for (m_it = m_ic->begin();m_it!=m_it_end;++m_it)
      this->m_pc->template insert<update_elem>( value_type(*m_it,0), pred );
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_pc->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the bracket operator of the unordered map container.
///
/// @tparam pC The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Input, class Counter=counter<default_timer> >
class um_find_val_profiler
  : public p_container_profiler<pC,Counter>
{
  typedef p_container_profiler<pC,Counter> base_type;
  typedef typename pC::value_type          value_type;

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
  um_find_val_profiler(std::string pcname, pC* pc, Input* in,
                       int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"_find_val", argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->n_times = m_ic->size();
    this->m_pc->clear();
    for (;m_it!=m_it_end;++m_it)
      this->m_pc->insert( value_type(*m_it,0) );
    rmi_fence();
  }

  void run()
  {
    for (m_it=m_ic->begin();m_it!=m_it_end;++m_it)
      int val = (*(this->m_pc))[*it];
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_pc->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the find_split() function of the unordered map container.
///
/// @tparam pC The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Input, class Counter=counter<default_timer> >
class um_find_split_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;

  /// The input container
  Input*   m_ic;
  /// Iterators to the input container
  typename Input::iterator m_it, m_it_end;
  /// The number of handles
  size_t m_k;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for every index
  ///
  /// @param pcname A string that contains the name of the container
  /// @param pc The container
  /// @param in The input
  //////////////////////////////////////////////////////////////////////////////
  um_find_split_profiler(std::string pcname, pC* pc, Input* in,
                         int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"_find_split", argc, argv), m_ic(in)
  {
    m_k = 10; //set default; if provided as arg reset below
    for ( int i = 1; i < argc; i++)
    {
      if ( !strcmp("--nhandles", argv[i]))
        m_k = atoi(argv[++i]);
    }
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->n_times = m_ic->size();
    this->m_pc->clear();
    for (;m_it!=m_it_end;++m_it)
      this->m_pc->insert( value_type(*m_it,0));
    rmi_fence();
  }

  void run()
  {
    vector<future<std::pair<value_type, bool> > > handles(m_k);
    size_t fc = 0;
    for (m_it=m_ic->begin();m_it!=m_it_end;++m_it)
    {
      handles[fc] = this->m_pc->find_split( *m_it ); //<-- the magic line
      if (fc == m_k - 1)
      {
        for (size_t j=0;j != m_k; ++j)
          handles[j].get();
        fc = 0;
      }
      else
        ++fc;
    }
    for (size_t j=0;j != fc; ++j)
      handles[j].get();
    rmi_fence();//ensure all opes committed
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the erase() function of the unordered map container.
///
/// @tparam pC The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Input, class Counter=counter<default_timer> >
class um_erase_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter>     base_type;
  typedef typename pC::value_type               value_type;
  typedef extractor<typename Input::value_type> ext;

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
  um_erase_profiler(std::string pcname, pC* pc, Input* in,
                    int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"_erase", argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->n_times = m_ic->size();
  }

  void initialize_iteration()
  {
    for (m_it=m_ic->begin();m_it!=m_it_end;++m_it)
      this->m_pc->insert( value_type(*m_it, 0) );
    rmi_fence();
  }

  void run()
  {
    for (m_it=m_ic->begin();m_it!=m_it_end;++m_it)
      this->m_pc->erase( *m_it );
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_pc->clear();
  }
};

#ifdef TBB_AVAIL
////////////////////////////////////////////////////////////////////////////////
/// @brief Populates a table with strings.
///
/// @tparam StringTable Stores the strings
/// @tparam MYKEY The key type
/// @tparam DATA The value type
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class StringTable, class MYKEY, class DATA>
struct tbb_tally
{
  typedef std::pair<const MYKEY,DATA>  value_type;
  StringTable& table;
  tbb_tally( StringTable& table_ ) : table(table_)
  { }
  void operator()(const tbb::blocked_range<typename vector<MYKEY>::iterator>
                    range ) const
  {
    for (typename vector<MYKEY>::iterator p=range.begin(); p!=range.end(); ++p)
    {
      typename StringTable::accessor a;
      table.insert( a, value_type(*p, 0) );
      a->second += 1;
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Populates a table with data
///
/// @tparam View The view type of the table
/// @tparam MYKEY The key type
/// @tparam DATA The value type
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class View, class MYKEY, class DATA>
struct stapl_tally
{
  typedef std::pair<const MYKEY,DATA>  value_type;

  update_elem pred;
  View& table;

  stapl_tally(View& table_)
    : table(table_)
  { }

  void operator()(const tbb::blocked_range<typename vector<MYKEY>::iterator>
                    range ) const
  {
    for (typename vector<MYKEY>::iterator p=range.begin(); p!=range.end(); ++p)
      table.insert<update_elem>(value_type(*p,0), pred );
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the insert() function with a predicate of the unordered map
/// container using multithreading.
///
/// @tparam pC The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Input, class Counter=counter<default_timer> >
class um_mt_insert_pred_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;
  typedef typename pC::key_type             MYKEY;
  typedef typename pC::data_type            DATA;

  /// The input container
  Input*   m_ic;
  /// Iterators for the input container
  typename Input::iterator m_it, m_it_end;
  /// The work function to apply to the container
  update_elem pred;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for every index
  ///
  /// @param pcname A string that contains the name of the container
  /// @param pc The container
  /// @param in The input
  //////////////////////////////////////////////////////////////////////////////
  um_mt_insert_pred_profiler(std::string pcname, pC* pc, Input* in,
                             int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"_insert_pred", argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->n_times = m_ic->size();
  }

  void run()
  {
    tbb::parallel_for(
     tbb::blocked_range<typename vector<MYKEY>::iterator>(m_it, m_it_end, 1000),
     stapl_tally<pC,MYKEY,DATA>(*(this->m_pc))
    );
  }

  void finalize_iteration()
  {
    std::cout << "UM size=" << this->m_pc->size() << "\n";
    this->m_pc->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the insert() function with a predicate of the tbb map
/// container using multithreading.
///
/// @tparam pC The container type
/// @tparam Input The input container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Input, class Counter=counter<default_timer> >
class tbb_mt_insert_pred_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;
  typedef typename pC::key_type             MYKEY;
  typedef typename pC::mapped_type          DATA;

  /// The input container
  Input*   m_ic;
  /// Iterators for the input container
  typename Input::iterator m_it, m_it_end;
  /// The work function to apply to the container
  update_elem pred;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for every index
  ///
  /// @param pcname A string that contains the name of the container
  /// @param pc The container
  /// @param in The input
  //////////////////////////////////////////////////////////////////////////////
  tbb_mt_insert_pred_profiler(std::string pcname, pC* pc, Input* in,
                              int argc=0, char **argv=NULL)
  : base_type(pc,pcname+"_insert_pred", argc, argv), m_ic(in)
  {
    m_it = m_ic->begin(); m_it_end = m_ic->end();
    this->n_times = m_ic->size();
  }

  void run()
  {
    tbb::parallel_for(
     tbb::blocked_range<typename vector<MYKEY>::iterator>(m_it, m_it_end, 1000),
     tbb_tally<pC,MYKEY,DATA>(*(this->m_pc))
    );
  }

  void finalize_iteration()
  {
    std::cout << "UM size=" << this->m_pc->size() << "\n";
    this->m_pc->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the multithreading profilers for insert() with a predicate with
/// string type keys.
///
/// @param N The number of words to generate and test
/// @param nroots The number of roots to generate
/// @param rep The length of the words
////////////////////////////////////////////////////////////////////////////////
void mt_evaluate_string_int(size_t N, size_t nroots, size_t rep,
                            int argc,char** argv)
{
  typedef string MYKEY;
  typedef size_t    DATA;
  //------------------  stapl with tbb concurent unordered map
  typedef tbbum_traits<MYKEY,DATA,map_compare> tbb_part_traits;
  typedef partition_associative_hashed<tbb_part_traits, map_val_tag> part_type;
  typedef unordered_map_traits<MYKEY,DATA,map_compare,std::equal_to<MYKEY>,
                                 part_type> tbb_traits;
  typedef stapl::unordered_map<MYKEY,DATA,
    map_compare,
    std::equal_to<MYKEY>,
    part_type,
    tbb_traits> tbb_p_unordered_map_type;

  ts_print("build unordered_map_tbb\n");
  tbb_p_unordered_map_type p(CPL);  //pcontainer
  ts_print("done build unordered_map_tbb\n");

  typedef vector<string> SC;
  SC roots;
  generate_roots(roots, nroots);
  SC words(N);
  generate_words(words, roots, N, rep);
  ts_print("done generating data(strings)\n");

  um_mt_insert_pred_profiler<tbb_p_unordered_map_type, SC, counter_type>
    aip("um_tbb_string_int", &p, &words, argc, argv);
  aip.collect_profile();
  aip.report();

  //------------------  tbb concurent unordered map
  typedef tbb::concurrent_hash_map<MYKEY,DATA,map_compare> tbb_table;
  ts_print("build tbb unordered map\n");
  tbb_table table;
  ts_print("build tbb unordered map\n");

  tbb_mt_insert_pred_profiler<tbb_table, SC, counter_type>
    taip("tbb_string_int", &table, &words, argc, argv);
  taip.collect_profile();
  taip.report();


  //------------------  stapl with STL unordered map
  typedef partition_associative_hashed<p_hash_map_ps_traits<MYKEY,DATA,
                                       my_unordered_map<MYKEY>,
                                       std::equal_to<MYKEY>
                                       >, map_val_tag>
            default_ps_type;
  typedef stapl::unordered_map<MYKEY,DATA, my_unordered_map<MYKEY>,
                               std::equal_to<MYKEY>, default_ps_type>
            unordered_map_type;
  unordered_map_type unordered_stl(CPL);
  um_mt_insert_pred_profiler<unordered_map_type, SC, counter_type>
    saip("um_stl_string_int", &unordered_stl, &words, argc, argv);
  saip.collect_profile();
  saip.report();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the multithreading profilers for insert() with a predicate with
/// int type keys.
///
/// @param N The number of values to generate and test
////////////////////////////////////////////////////////////////////////////////
void mt_evaluate_int_int(size_t N, int argc,char** argv)
{
  typedef size_t MYKEY;
  typedef size_t    DATA;
  typedef tbbhm_traits<MYKEY,DATA,map_compare_int> tbb_part_traits;
  typedef partition_associative_hashed<tbb_part_traits, map_val_tag> part_type;
  typedef unordered_map_traits<MYKEY,DATA,map_compare_int,std::equal_to<MYKEY>,
                               part_type> tbb_traits;

  typedef stapl::unordered_map<MYKEY,DATA,
    map_compare_int,
    std::equal_to<MYKEY>,
    part_type,
    tbb_traits> tbb_unordered_map_type;

  ts_print("build unordered_map_tbb\n");
  tbb_unordered_map_type p(CPL);  //pcontainer
  ts_print("done build unordered_map_tbb\n");

  typedef vector<MYKEY> SC;
  SC words(N);
  generate_ints(words, N, N*get_num_locations());
  ts_print("done generating data(ints)\n");

  um_mt_insert_pred_profiler<tbb_unordered_map_type, SC, counter_type>
    aip("um_tbb_int_int", &p, &words, argc, argv);
  aip.collect_profile();
  aip.report();

  //tbb concurent unordered map
  typedef tbb::concurrent_hash_map<MYKEY,DATA,map_compare_int> tbb_table;
  ts_print("build tbb unordered map\n");
  tbb_table table;
  ts_print("build tbb unordered map\n");

  tbb_mt_insert_pred_profiler<tbb_table, SC, counter_type>
    taip("tbb_int_int", &table, &words, argc, argv);
  taip.collect_profile();
  taip.report();

  //------------------  stapl with STL unordered map
  typedef partition_associative_hashed<p_hash_map_ps_traits<MYKEY,DATA,
                                       my_unordered_map<MYKEY>,
                                       std::equal_to<MYKEY> >,
                                       map_val_tag>
            default_ps_type;
  typedef stapl::unordered_map<MYKEY,DATA, my_unordered_map<MYKEY>,
                               std::equal_to<MYKEY>, default_ps_type>
            unordered_map_type;
  unordered_map_type unordered_stl(CPL);
  um_mt_insert_pred_profiler<unordered_map_type, SC, counter_type>
    saip("um_stl_string_int", &unordered_stl, &words, argc, argv);
  saip.collect_profile();
  saip.report();

}
#endif //if tbb avail  --- end multithreading

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the functions of the unordered map container.
///
/// @param pcname Astring containing the name of the container
/// @param p The container
/// @param words The input data
////////////////////////////////////////////////////////////////////////////////
template <class pC, class SC>
void profile(std::string pcname, pC& p, SC& words, int argc,char** argv)
{

  um_insert_profiler<pC, SC, counter_type>
    aip(pcname, &p, &words, argc, argv);
  aip.collect_profile();
  aip.report();

  um_insert_pred_profiler<pC, SC, counter_type>
    paip(pcname, &p, &words, argc, argv);
  paip.collect_profile();
  paip.report();

  um_find_val_profiler<pC, SC, counter_type>
    afp(pcname, &p, &words, argc, argv);
  afp.collect_profile();
  afp.report();

  um_find_split_profiler<pC, SC, counter_type>
    afsp(pcname, &p, &words, argc, argv);
  afsp.collect_profile();
  afsp.report();

  um_erase_profiler<pC, SC, counter_type>
    aep(pcname, &p, &words, argc, argv);
  aep.collect_profile();
  aep.report();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Generate the string test data, initialize the container, and call the
/// profilers.
///
/// @param N The test size
/// @param nroots The number of roots to generate
/// @param rep The length of the words to generate
////////////////////////////////////////////////////////////////////////////////
void evaluate_string_int(size_t N, size_t nroots, size_t rep,
                         int argc,char** argv)
{
  typedef string MYKEY;
  typedef size_t    DATA;

  typedef vector<string> SC;
  ts_print("generate data(strings)\n");
  SC roots;
  generate_roots(roots, nroots);
  SC words(N);
  generate_words(words, roots, N, rep);
  ts_print("done generating data(strings)\n");

#ifdef TBB_AVAIL
  typedef tbbhm_traits<MYKEY,DATA,map_compare> tbb_part_traits;
  typedef partition_associative_hashed<tbb_part_traits, map_val_tag> part_type;
  typedef unordered_map_traits<MYKEY,DATA,map_compare,std::equal_to<MYKEY>,
                               part_type> tbb_traits;

  typedef stapl::unordered_map<MYKEY,DATA,
    map_compare,
    std::equal_to<MYKEY>,
    part_type,
    tbb_traits> tbb_unordered_map_type;

  typedef tbb_unordered_map_type::iterator iterator;

  ts_print("build unordered_map_tbb\n");
  tbb_unordered_map_type unordered_tbb(CPL);  //pcontainer
  ts_print("done build unordered_map_tbb\n");

  profile("um_tbb", unordered_tbb, words, argc, argv);
#endif

  typedef stapl::unordered_map<MYKEY,DATA, my_unordered_map<MYKEY>,
                               std::equal_to<MYKEY> > unordered_map_type;
  ts_print("build unordered_map_stl\n");
  unordered_map_type unordered_stl;
  ts_print("done build unordered_map_stl\n");
  profile("um_stl", unordered_stl, words, argc, argv);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Generate the int test data, initialize the container, and call the
/// profilers.
///
/// @param N The test size
////////////////////////////////////////////////////////////////////////////////
void evaluate_int_int(size_t N, int argc,char** argv)
{
  typedef size_t MYKEY;
  typedef size_t DATA;

  size_t block = N / get_num_locations();
  typedef vector<MYKEY> SC;
  SC words(block);
  generate_ints(words,block, N*get_num_locations() );
  ts_print("done generating data\n");

#ifdef TBB_AVAIL
  typedef tbbhm_traits<MYKEY,DATA,map_compare_int> tbb_part_traits;
  typedef partition_associative_hashed<tbb_part_traits, map_val_tag> part_type;
  typedef unordered_map_traits<MYKEY,DATA,map_compare_int,std::equal_to<MYKEY>,
                               part_type> tbb_traits;

  typedef stapl::unordered_map<MYKEY,DATA,
    map_compare_int,
    std::equal_to<MYKEY>,
    part_type,
    tbb_traits> tbb_unordered_map_type;

  typedef tbb_unordered_map_type::iterator iterator;

  ts_print("build unordered_map_tbb\n");
  tbb_unordered_map_type unordered_tbb(CPL);  //pcontainer
  ts_print("done build unordered_map_tbb\n");

  profile("um_tbb_int", unordered_tbb, words, argc, argv);
#endif

  typedef stapl::unordered_map<MYKEY,DATA, my_unordered_map<MYKEY>,
                               std::equal_to<MYKEY> > unordered_map_type;
  ts_print("build unordered_map_stl\n");
  unordered_map_type unordered_stl;//(CPL);
  ts_print("done build unordered_map_stl\n");
  profile("um_stl_int", unordered_stl, words, argc, argv);
}

void stapl_main(int argc,char** argv)
{
  int myid  =stapl::get_location_id();
  int nprocs=stapl::get_num_locations();
  size_t nroots;
  size_t rep=3;
  size_t case_id;
  size_t n_threads;
  if (argc < 6) {
    case_id = 0;
    N       = 100;
    nroots  = 100;
    CPL     = 1;
    n_threads = 1;
  }
  else {
    case_id = atol(argv[1]);
    N       = atol(argv[2]);
    nroots  = atol(argv[3]);
    CPL     = atol(argv[4]);
    n_threads = atol(argv[5]);
  }

  if (myid == 0) {
    cout << "Case ID=" << case_id << "\n";
    cout << " Number of procs:" << nprocs << " N=" << N << " nroots=" <<
      nroots << " Comp per loc=" << CPL << endl;
    cout << " Number of threads used:" << n_threads << "\n";
  }

  srand(myid);

  if (case_id == 0) {
    ts_print("um from string to int\n");
    evaluate_string_int( N, nroots, rep, argc, argv );
  }
  else if (case_id == 1) {
    ts_print("um from int to int\n");
    evaluate_int_int( N,argc, argv );
  }
#ifdef TBB_AVAIL
  else if (case_id == 2) {
    ts_print("multithreading : maps from string to int\n");
    tbb::task_scheduler_init init(n_threads);
    mt_evaluate_string_int( N, nroots, rep, argc, argv );
  }
  else if (case_id == 3) {
    ts_print("multithreading : maps from int to int\n");
    tbb::task_scheduler_init init(n_threads);
    mt_evaluate_int_int( N,argc, argv );
  }
#endif
  else {
    ts_print("Invalid case id\n");
  }
  rmi_fence();
}
