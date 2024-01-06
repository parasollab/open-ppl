/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILER_P_CONTAINER_ALGO_PROFILER_HPP
#define STAPL_PROFILER_P_CONTAINER_ALGO_PROFILER_HPP

#include <cstdlib>
#include <string>

#include "p_container_profiler.hpp"
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>

//#include <stapl/algorithms/euler_tour.hpp>

namespace stapl
{

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns a random value less than the value passed in.
////////////////////////////////////////////////////////////////////////////////
struct rand_seq
{
  /// The upper bound of the value returned
  size_t m_sz;

  rand_seq(size_t mx)
    : m_sz(mx)
  { }

  void define_type(typer& t)
  {
    t.member(m_sz);
  }

  size_t operator()(void) const
  {
    return lrand48() % m_sz;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns the value 1.
////////////////////////////////////////////////////////////////////////////////
struct one_seq
{
  size_t operator()(void) const
  {
    return 1;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the generate() algorithm.
///
/// @tparam pC The container type
/// @tparam View The view type of the container
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class View, class Counter=counter<default_timer> >
class p_generate_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef View                              view_type;

  /// A view of the container
  view_type& m_vw;
  /// The upper bound of the random number generation
  rand_seq gen;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to store the generated values
  /// @param vw The view of the container
  /// @param N The upper bound for the random number generation
  //////////////////////////////////////////////////////////////////////////////
  p_generate_profiler(std::string pcname, pC* pc, view_type& vw, size_t N,
                     int argc=0, char **argv=NULL)
   : base_type(pc,pcname+"_p_generate", argc, argv), m_vw(vw), gen( N )
  { }

  void run()
  {
    generate(m_vw, gen);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the for_each() algorithm.
///
/// @tparam pC The container type
/// @tparam View The view type of the container
/// @tparam Funct The work function to pass to the algorithm
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class View, class Funct,
          class Counter=counter<default_timer> >
class p_for_each_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef View                              view_type;

  /// The view of the container
  view_type& m_vw;
  /// The work function to apply to the container
  Funct const& m_f;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to store the generated values
  /// @param vw The view of the container
  /// @param f The work function to apply to the container
  //////////////////////////////////////////////////////////////////////////////
  p_for_each_profiler(std::string pcname, pC* pc, view_type& vw, Funct const& f,
                      int argc=0, char **argv=NULL)
    : base_type(pc, pcname+"_p_for_each", argc, argv), m_vw(vw), m_f(f)
  { }

  void run()
  {
    for_each(m_vw, m_f);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the copy() algorithm.
///
/// @tparam pC The container type
/// @tparam View The view type of the container
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class View, class Counter=counter<default_timer> >
class p_copy_profiler
   : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef View                              view_type;

  /// The view of the source container
  view_type& m_vw1;
  /// The view of the target container
  view_type& m_vw2;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc1 The container to copy from
  /// @param pc2 The container to copy to
  /// @param vw1 The view of the container to copy from
  /// @param vw2 The view of the container to copy to
  //////////////////////////////////////////////////////////////////////////////
  p_copy_profiler(std::string pcname, pC* pc1, pC* pc2,
                 view_type& vw1, view_type& vw2,
                 int argc=0, char **argv=NULL)
  : base_type(pc1, pcname+"_p_copy", argc, argv), m_vw1(vw1), m_vw2(vw2)
  { }

  void run()
  {
    copy(m_vw1, m_vw2);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the accumulate() algorithm.
///
/// @tparam pC The container type
/// @tparam View The view type of the container
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC,class View, class Counter=counter<default_timer> >
class p_accumulate_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef View                              view_type;

  /// The view of the container
  view_type& m_vw;
  /// The result of the algorithm
  typename pC::value_type t;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to pass to the algorithm
  /// @param vw The view of the container
  /// @param init_val The initial value to begin accumulating from
  //////////////////////////////////////////////////////////////////////////////
  p_accumulate_profiler(std::string pcname, pC* pc, view_type& vw,
                        typename pC::value_type init_val,
                        int argc=0, char **argv=NULL)
    : base_type(pc, pcname+"_p_accumulate", argc, argv), m_vw(vw), t(init_val)
  { }

  void run()
  {
     t = accumulate(m_vw, t);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the partial_sum() algorithm.
///
/// @tparam pC The container type
/// @tparam View The view type of the container
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class View, class Counter=counter<default_timer> >
class p_partial_sum_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef View                              view_type;

  /// The view of the container
  view_type& m_vw;
  /// The initial value passed in
  typename pC::value_type t;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that populates the container and tests the
  /// algorithm.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to pass to the algorithm
  /// @param vw The view of the container
  /// @param init_val The initial value to begin testing from
  //////////////////////////////////////////////////////////////////////////////
  p_partial_sum_profiler(std::string pcname, pC* pc, view_type& vw,
                         typename pC::value_type init_val,
                         int argc=0, char **argv=NULL)
    : base_type(pc, pcname+"_p_partial_sum", argc, argv), m_vw(vw), t(init_val)
  {
    generate(m_vw, one_seq());
    if (get_location_id()==0)
      m_vw[0] = 0;
  }

  void run()
  {
     partial_sum(m_vw, m_vw);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the find() algorithm.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Counter=counter<default_timer> >
class p_find_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::view_type            view_type;

  /// The view of the container
  view_type& m_vw;
  /// The index to find
  typename pC::value_type t;
  /// Iterator for the container
  typename pC::view_type::reference_t it;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to pass to the algorithm
  /// @param vw The view of the container
  /// @param init_val The index value to find
  //////////////////////////////////////////////////////////////////////////////
  p_find_profiler(std::string pcname, pC* pc, view_type& vw,
                  typename pC::value_type init_val,
                  int argc=0, char **argv=NULL)
    : base_type(pc, pcname+"_p_find", argc, argv), m_vw(vw), t(init_val)
  { }

  void run()
  {
    it = find(m_vw,t);
  }

  void finalize_iteration()
  {
    stapl_assert(it == stapl::null_reference(), "-----Find Error\n");
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential generate() algorithm.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC, class Counter=counter<default_timer> >
class generate_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;

  /// The upper bound of the random number generation
  rand_seq gen;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm sequentially.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to pass to the algorithm
  /// @param N The upper bound of the random number generation
  //////////////////////////////////////////////////////////////////////////////
  generate_profiler(std::string pcname, pC* pc, size_t N,
                   int argc=0, char **argv=NULL)
   : base_type(pc, pcname+"_generate", argc, argv), gen( N )
  { }

  void run()
  {
    std::generate(this->m_pc->begin(), this->m_pc->end(), gen);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential for_each() algorithm.
///
/// @tparam pC The container type
/// @tparam Funct The work function to apply to the container
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC,class Funct, class Counter=counter<default_timer> >
class for_each_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;

  /// The work function to apply to the container
  const Funct& m_f;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm sequentially.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to pass to the algorithm
  /// @param f The work function apply to the container
  //////////////////////////////////////////////////////////////////////////////
  for_each_profiler(std::string pcname, pC* pc, const Funct& f,
                    int argc=0, char **argv=NULL)
    : base_type(pc, pcname+"_for_each", argc, argv), m_f(f)
  { }

  void run()
  {
    std::for_each(this->m_pc->begin(), this->m_pc->end(), m_f);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential copy() algorithm.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC,class Counter=counter<default_timer> >
class copy_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;

  /// The target container
  pC* m_pc2;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm sequentially.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc1 The container to copy from
  /// @param pc2 The container to copy to
  //////////////////////////////////////////////////////////////////////////////
  copy_profiler(std::string pcname, pC* pc1, pC* pc2,
               int argc=0, char **argv=NULL)
   : base_type(pc1, pcname+"_copy", argc, argv), m_pc2(pc2)
  { }

  void run()
  {
    std::copy(this->m_pc->begin(), this->m_pc->end(), m_pc2->begin() );
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential accumulate() algorithm.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC,class Counter=counter<default_timer> >
class accumulate_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;

  /// The initial value to begin accumulating from
  typename pC::value_type t;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm sequentially.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to test the algorithm on
  /// @param init_val The initial value to begin testing from
  //////////////////////////////////////////////////////////////////////////////
  accumulate_profiler(std::string pcname, pC* pc,
                      typename pC::value_type init_val,
                      int argc=0, char **argv=NULL)
   : base_type(pc, pcname+"_accumulate", argc, argv), t(init_val)
  { }

  void run()
  {
    t = std::accumulate(this->m_pc->begin(), this->m_pc->end(), t);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential partial_sum() algorithm.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////

template <class pC,class Counter=counter<default_timer> >
class partial_sum_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;
  typedef typename pC::value_type           value_type;

  /// The initial value to begin from
  value_type t;
  /// Stores the resulting sums
  std::vector<value_type> sums;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm sequentially.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to test the algorithm on
  /// @param init_val The initial value to begin testing from
  //////////////////////////////////////////////////////////////////////////////
  partial_sum_profiler(std::string pcname, pC* pc,
                      typename pC::value_type init_val,
                      int argc=0, char **argv=NULL)
   : base_type(pc, pcname+"_accumulate", argc, argv), t(init_val)
  {
    int sz = this->m_pc->size();
    for (int i=0; i<sz; i++)
      sums.push_back(value_type());
  }

  void run()
  {
    std::partial_sum(this->m_pc->begin(), this->m_pc->end(), sums.begin());
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Stores the iterator for a specific type.
///
/// @tparam T The type
////////////////////////////////////////////////////////////////////////////////
template <class T>
struct infer_iterator
{
  typedef typename T::iterator type;
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Stores the iterator for an array of a specific type.
///
/// @tparam T The type
////////////////////////////////////////////////////////////////////////////////
template <class T>
struct infer_iterator<std::valarray<T> >
{
  typedef T* type;
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the sequential find() algorithm.
///
/// @tparam pC The container type
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC,class Counter=counter<default_timer> >
class find_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;

  /// The index to find
  typename pC::value_type t;
  /// The iterator for the container
  typename infer_iterator<pC>::type it;

public:
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler that tests the algorithm sequentially.
  ///
  /// @param pcname A string containing the name of the container
  /// @param pc The container to test the algorithm on
  /// @param init_val The index to find
  //////////////////////////////////////////////////////////////////////////////
  find_profiler(std::string pcname, pC* pc, typename pC::value_type init_val,
               int argc=0, char **argv=NULL)
   : base_type(pc, pcname+"_find", argc, argv), t(init_val)
  { }

  void run()
  {
    it = std::find(this->m_pc->begin(), this->m_pc->end(), t);
  }

  void finalize_iteration()
  {
    stapl_assert(it == this->m_pc->end(), "-----Find Error\n");
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles a collection of operations: creating a container, populating
/// the container, and applying a work function to the elements.
///
/// @tparam pC The container type
/// @tparam Funct The work function to apply to the container
/// @tparam Counter The timing utility
///
/// @ingroup performanceMonitor
////////////////////////////////////////////////////////////////////////////////
template <class pC,class Funct,class Counter=counter<default_timer> >
class application_profiler
  : public p_container_profiler<pC, Counter>
{
  typedef p_container_profiler<pC, Counter> base_type;

  /// Iterator for the container
  typename infer_iterator<pC>::type it;
  /// The initial size of the container
  size_t m_sz;
  // The number of elements to add to the container
  size_t m_k;

public:
  application_profiler(std::string pcname, pC* pc, size_t sz, size_t k,
                       int argc=0, char **argv=NULL)
    : base_type(pc,pcname+"_application", argc, argv), m_sz(sz), m_k(k)
  { }

  void run()
  {
    pC p(m_sz);

    for (size_t i=0; i!=m_k; ++i)
      p.push_anywhere(lrand48()%m_sz);

    p_for_each(p.view(),Funct());
  }
};

} // namespace stapl
#endif
