/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

////////////////////////////////////////////////////////////////////////////////
/// @file
/// Contains functions used for profiling sequential containers for comparisons
/// with STAPL containers / views.
////////////////////////////////////////////////////////////////////////////////

#ifndef STAPL_PROFILING_SEQ_CONTAINER_HPP
#define STAPL_PROFILING_SEQ_CONTAINER_HPP

#include <valarray>
#include <vector>

#include "profile.hpp"
#include "subscript.hpp"
#include "sequence.hpp"
#include "restructure.hpp"
#include "vector.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Define traits for std::vector
//////////////////////////////////////////////////////////////////////
template<typename T>
struct view_traits<std::vector<T>>
{
  using container = std::vector<T>;
  using value_type = typename container::value_type;
  using index_type = typename container::size_type;
  using reference = typename container::reference;
  using const_reference = typename container::const_reference;
};

//////////////////////////////////////////////////////////////////////
/// @brief Define traits for std::valarray
//////////////////////////////////////////////////////////////////////
template<typename T>
struct view_traits<std::valarray<T>>
{
  using container = std::valarray<T>;
  using value_type = T;
  using index_type = size_t;
  using reference = T&;
  using const_reference = T const&;
};

namespace profiling {

template<typename Cont>
void add_seq_cont_profilers(prof_cont_t<Cont>& p,
                            std::string const& name, Cont& cont,
                            std::vector<size_t> const& indices,
                            std::vector<typename Cont::value_type> const& vals,
                            int argc, char** argv)
{
  add_resize_profilers(p, name, cont, argc, argv);
}

template<typename T>
void
add_seq_cont_profilers(prof_cont_t<std::vector<T>>& p,
                       std::string const& name, std::vector<T>& cont,
                       std::vector<size_t> const& indices,
                       std::vector<T> const& vals,
                       int argc, char** argv)
{
  add_sequence_profilers(p, name, cont, 0, cont.size(), 0, vals, argc, argv);
  add_resize_profilers(p, name, cont, argc, argv);
  add_vector_profilers(p, name, cont, indices, vals, argc, argv);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the constructor of an std::valarray with a given size
/// and value.
///
/// Note -- the only difference from the generic constructor_size_value_profiler
/// is in the order of the size and value arguments passed to the constructor,
/// which is different for std::valarray and all the other array-like containers
///
/// @tparam T Type of the values stored in the valarray.
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename T>
class constructor_size_value_profiler<std::valarray<T>, size_t>
  : public adt_profiler<std::valarray<T>>
{
  using cont_type = std::valarray<T>;
  using base_type = adt_profiler<cont_type>;

  /// The size of the container(s)
  size_t m_ne;

  /// A vector of containers of type @p std::valarray<T>
  std::vector<cont_type*> m_pcs;

  /// The value to initialize the container elements with
  typename cont_type::value_type m_value;

public:
  constructor_size_value_profiler(std::string name, size_t N,
                                  int argc = 0, char** argv = nullptr)
    : base_type(nullptr, name+"::constructor_size_value", argc, argv), m_ne(N)
  {
    this->m_time_per_invocation = true;
    m_pcs.resize(this->m_test_size);
  }

  void run()
  {
    for (size_t i=0;i < this->m_test_size; ++i)
      this->m_pcs[i] = new cont_type(m_value, m_ne);
  }

  void finalize_iteration()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      delete this->m_pcs[i];
  }
};

} // namespace profiling


////////////////////////////////////////////////////////////////////////////////
/// @brief Create a vector of profiler instances, run them and clean them up.
///
/// @tparam Cont   Type of the container to be profiled.
///
/// @param name    String identifier of the container being profiled
/// @param sz      Size of the container
/// @param indices Set of indices used for profiling the element access methods.
/// @param argc, argv Additional command line arguments
////////////////////////////////////////////////////////////////////////////////
template<typename Cont>
void profile_seq_container(std::string const& name, size_t sz,
                           std::vector<size_t> const& indices,
                           int argc, char** argv)
{
  using namespace profiling;

  Cont cont(sz);

  std::vector<typename Cont::value_type> vals(sz);
  std::iota(vals.begin(), vals.end(), 0);

  auto profs = set_common_container_profilers<Cont>(name, cont, sz, argc, argv);
  profs.push_back(
    new constructor_size_value_profiler<Cont, size_t>(name, sz, argc, argv));

  add_subscript_read_profilers(
    profs, name, cont, indices, validators::subscript<Cont>(), argc, argv);
  add_subscript_write_profilers(
    profs, name, cont, indices, vals, validators::subscript<Cont>(vals),
    argc, argv);
  add_seq_cont_profilers(profs, name, cont, indices, vals, argc, argv);

  profile_and_clear(profs);
}

} // namespace stapl

#endif // STAPL_PROFILING_SEQ_CONTAINER_HPP
