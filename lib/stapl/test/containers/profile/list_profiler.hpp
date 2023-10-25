/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_LIST_PROFILER_HPP
#define STAPL_TEST_LIST_PROFILER_HPP

#include "p_container_profiler.hpp"
#include <stapl/profiler/base_profiler.hpp>

using namespace stapl;

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the insert() function of the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <typename pC, typename Counter = counter<default_timer> >
class insert_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;
  typedef typename pC::iterator             Iterator;

  /// Iterator for the container of local elements
  Iterator m_it_local;
  /// Vector containing all of the elements
  std::vector<Iterator> m_it_mix;
  /// Vector containing the remote elements
  std::vector<Iterator> m_it_remote;
  /// Percent of elements that are local
  const size_t m_percent;
  /// Vector of index values
  const std::vector<value_type>& indices;
  /// The number of index values
  const size_t m_sz;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for each index.
  ///
  /// @param pcname A string containing the name of the container
  /// @param casename A string containing the percent of local elements
  /// @param idx The index values to be tested
  //////////////////////////////////////////////////////////////////////////////
  insert_profiler(std::string pcname, std::string casename,
                  std::vector<value_type> const& idx, int argc=0,
                  char **argv = NULL)
    : base_type(NULL, pcname+"_insert_"+casename, argc, argv),
      m_percent(atoi(casename.c_str())), indices(idx), m_sz(idx.size())
  {
    this->n_times = indices.size();
  }

  void initialize_iteration()
  {
    size_t nElements = this->get_num_locations();
    this->m_pc = new pC(nElements);
    Iterator end = this->m_pc->end();
    Iterator it = this->m_pc->begin();
    rmi_fence();

    m_it_remote.clear();
    for (; it != end; ++it)
      m_it_remote.push_back(it);
    rmi_fence();

    size_t loc_id = this->get_location_id();
    m_it_local = m_it_remote[loc_id];
    if (this->get_num_locations() != 1)
      m_it_remote.erase(
        std::find(m_it_remote.begin(), m_it_remote.end(), m_it_local));
    size_t num_local = (m_sz * m_percent) / 100;
    size_t num_remote = m_sz - num_local;

    m_it_mix.resize(m_sz);
    m_it_mix.clear();
    size_t j = 0;
    size_t pos = 0;
    for (j = 0; j != num_local; ++j) {
      m_it_mix[pos] = m_it_local;
      ++pos;
    }
    size_t k = 0;

    if (nElements-1 == 0)
    {
      for (j = 0; j != num_remote; ++j) {
        m_it_mix[pos] = m_it_remote[0];
        ++pos;
      }
    }
    else
    {
      for (j = 0; j != num_remote; ++j) {
        k = lrand48() % (nElements-1);
        m_it_mix[pos] = m_it_remote[k];
        ++pos;
      }
    }
    rmi_fence();
  }

  void run()
  {
    for (size_t i=0; i!=m_sz; ++i)
      this->m_pc->insert(m_it_mix[i], indices[i]);
    rmi_fence();
  }

  void finalize_iteration()
  {
    rmi_fence();
    delete this->m_pc;
    m_it_mix.clear();
    rmi_fence();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential insert() function for the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <typename pC, typename Counter = counter<default_timer> >
class list_insert_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;

  /// A vector of index values
  const std::vector<value_type>& indices;
  /// The number of index values
  const size_t sz;
  /// The sequential list to be tested
  std::list<value_type> l;
  /// Iterator for the sequential list
  typename std::list<value_type>::iterator m_it;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for each index.
  ///
  /// @param pcname A string containing the name of the container
  /// @param idx The index values to be tested
  //////////////////////////////////////////////////////////////////////////////
  list_insert_profiler(std::string pcname, std::vector<value_type> const& idx,
                      int argc=0, char **argv=NULL)
    : base_type(NULL, pcname+"_insert", argc, argv), indices(idx),
      sz(idx.size())
  {
    this->n_times = indices.size();
  }

  void initialize_iteration()
  {
    m_it = l.begin();
  }

  void run()
  {
    for (size_t i=0; i!=sz; ++i)
      l.insert(m_it, indices[i]);
  }

  void finalize_iteration()
  {
    l.clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential push_back() function for the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <typename pC, typename Counter = counter<default_timer> >
class list_push_back_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef typename pC::value_type           value_type;
  typedef p_container_profiler<pC, Counter> base_type;

  /// A vector containing the index values
  const std::vector<value_type>& indices;
  /// The number of index values
  const size_t sz;
  /// The sequential list to be tested
  std::list<value_type> l;

public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for each index.
  ///
  /// @param pcname A string containing the name of the container
  /// @param idx The index values to be tested
  //////////////////////////////////////////////////////////////////////////////
  list_push_back_profiler(std::string pcname,
                          std::vector<value_type> const& idx,
                          int argc=0, char **argv=NULL)
    : base_type(NULL, pcname+"_push_back", argc, argv), indices(idx),
      sz(idx.size())
  {
    this->n_times = indices.size();
  }

  void run()
  {
    for (size_t i=0; i!=sz; ++i)
      l.push_back(indices[i]);
  }

  void finalize_iteration()
  {
    l.clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the internal_list_ranking() function for the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <typename pC, typename Counter = counter<default_timer> >
class list_ranking_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;

public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function
  ///
  /// @param pc The container to be tested
  //////////////////////////////////////////////////////////////////////////////
  list_ranking_profiler(pC* pc, int argc=0, char **argv=NULL)
    : base_type(pc,"list_ranking", argc, argv)
  {
    this->m_pc = pc;
  }

  void run()
  {
    internal_list_ranking(*this->m_pc);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the push_front() function for the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
/// @tparam Predicate The work function used to extract the time value
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template<typename pC, typename Counter = counter<default_timer> >
class push_front_profiler_list
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;

  /// A vector containing the index values
  const std::vector<value_type>& indices;
  /// The number of index values
  const size_t sz;

public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for each index value
  ///
  /// @param pc The container to be tested
  /// @param pcname A string containing the name of the container
  /// @param idx The index values to be tested
  //////////////////////////////////////////////////////////////////////////////
  push_front_profiler_list(pC* pc, std::string pcname,
                           std::vector<value_type> const& idx,
                           int argc=0, char **argv=NULL)
    : base_type(pc, pcname+"_push_front", argc, argv), indices(idx),
      sz(idx.size())
  {
    this->n_times = indices.size();
  }

  void run()
  {
    for (size_t i=0; i!=sz; ++i)
      this->m_pc->push_front(indices[i]);
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_pc->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the push_back() function for the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template<typename pC, typename Counter = counter<default_timer> >
class push_back_profiler_list
  : public p_container_profiler<pC,Counter>
{
  typedef p_container_profiler<pC,Counter> base_type;
  typedef typename pC::value_type          value_type;

  /// A vector containing the index values
  std::vector<value_type> const& indices;
  /// The number of index values
  const size_t sz;

public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for each index value
  ///
  /// @param pc The container to be tested
  /// @param pcname A string containing the name of the container
  /// @param idx The index values to be tested
  //////////////////////////////////////////////////////////////////////////////
  push_back_profiler_list(pC* pc, std::string pcname,
                           std::vector<value_type> const& idx,
                     int argc=0, char **argv=NULL)
    : base_type(pc, pcname+"_push_back", argc, argv), indices(idx),
      sz(idx.size())
  {
    this->n_times = indices.size();
  }

  void run()
  {
    for (size_t i=0; i!=sz; ++i)
      this->m_pc->push_back(indices[i]);
    rmi_fence();
  }

  void finalize_iteration()
  {
    this->m_pc->clear();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the add() function for the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @note This function replaces the push_anywhere() function.
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template<typename pC, typename Counter = counter<default_timer> >
class list_add_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;

  /// A vector containing the index values
  const std::vector<value_type>& indices;
  /// The number of index values
  const size_t sz;

public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that calls the function once for each index value
  ///
  /// @param pcname A string containing the name of the container
  /// @param idx The index values to be tested
  //////////////////////////////////////////////////////////////////////////////
  list_add_profiler(std::string pcname, std::vector<value_type> const& idx,
                    int argc=0, char **argv=NULL)
    : base_type(NULL, pcname+"_add", argc, argv), indices(idx), sz(idx.size())
  {
    this->n_times = indices.size();
  }

  void initialize_iteration()
  {
    this->m_pc = new pC(this->get_num_locations());
  }

  void run()
  {
    for (size_t i=0; i!=sz; ++i)
      this->m_pc->add(indices[i]);
    rmi_fence();
  }

  void finalize_iteration()
  {
    rmi_fence();
    delete this->m_pc;
    rmi_fence();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the erase() function for the list container.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC,class Counter=counter<default_timer> >
class erase_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;
  typedef typename pC::iterator             Iterator;

  /// Iterator for the target container
  Iterator it;
  /// The number of elements on the location
  size_t sz;
  /// The case to be tested
  std::string m_casename;
  /// The total number of elements in the container
  const size_t nElements;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that populates a container with an even spread of
  /// elements and then delete all of the elements.
  ///
  /// @param pcname A string containing the name of the container
  /// @param casename A string containing the name of the test case to run
  /// @param N The total number of elements in the container
  //////////////////////////////////////////////////////////////////////////////
  erase_profiler(std::string pcname, std::string casename, size_t N,
                 int argc=0, char **argv=NULL) :
  base_type(NULL, pcname+"_erase_"+casename, argc, argv), m_casename(casename),
  nElements(N)
  { }

  void initialize_iteration()
  {
    if (m_casename == "best")
    {
      sz = nElements / this->get_num_locations();
      it = this->m_pc->begin();
      this->m_pc = new pC();
      this->m_pc->add(value_type());
      for (size_t i = 0; i < sz-1 ;++i)
        this->m_pc->add(value_type());
      rmi_fence();
    }
  }

  void run()
  {
    for (size_t i=0; i!=sz-1; ++i)
      this->m_pc->erase(it);
    rmi_fence();
  }

  void finalize_iteration()
  {
    rmi_fence();
    delete this->m_pc;
    rmi_fence();
  }
};

#endif
