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
/// Profilers for the methods that are common to the vector and list ADT.
///
/// @todo Add profilers for insert/erase at random position / end.
////////////////////////////////////////////////////////////////////////////////

#ifndef STAPL_PROFILING_LIST_VECTOR_HPP
#define STAPL_PROFILING_LIST_VECTOR_HPP

#include <stapl/containers/list/list.hpp>
#include <algorithm>
#include "adt.hpp"
#include "profiling_util.hpp"

namespace stapl {

namespace profiling {

namespace list_vector_detail {

////////////////////////////////////////////////////////////////////////////////
/// @brief Synchronize the view domain with the container's domain after
/// finishing all the push_back / pop_back operations.
///
/// @todo: This is a temporary fix until a proper way of updating vector_view's
/// domain is implemented.
////////////////////////////////////////////////////////////////////////////////
template<typename ADT,
  typename std::enable_if<is_view<ADT>::value>::type* = nullptr>
void synchronize_domain(ADT* vw)
{
  rmi_fence();
  using domain_type = typename ADT::domain_type;
  vw->domain() = domain_type(underlying_container<ADT>::get(*vw).size());
}

template<typename ADT,
  typename std::enable_if<is_container<ADT>::value>::type* = nullptr>
void synchronize_domain(ADT* c)
{
  rmi_fence();
}

template<typename ADT,
  typename std::enable_if<!is_p_object<ADT>::value>::type* = nullptr>
void synchronize_domain(ADT* c)
{ }

}  // namespace list_vector_detail

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the push_back() method.
///
/// @note Because of low performance for distributed containers, only first @p N
///   of the provided set of values will be pushed back by each location.
///   By default, <tt>N = original_size / 500</tt>, but can be changed using the
///   command line argument --reduced_local_size.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class push_back_profiler
  : public adt_profiler<ADT,Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT,Counter>;
  using value_type = typename ADT::value_type;

  /// A vector containing the values to be pushed back.
  std::vector<value_type> const& m_values;

  /// Expected size after pushing back all the elements (for validation).
  size_t m_exp_size;

public:
  push_back_profiler(std::string name, ADT* adt,
                     std::vector<value_type> const& vals,
                     int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::push_back",
                determine_reduced_local_size(vals.size(), argc, argv),
                argc, argv),
      mutating<ADT>(adt), m_values(vals)
  {
    m_exp_size = allreduce_rmi(
      stapl::plus<size_t>(), this->get_rmi_handle(),
      &push_back_profiler::template reflect_value<size_t>,
      this->m_test_size).get();
  }

  void initialize_iteration()
  {
    base_type::initialize_iteration();
    this->m_adt->clear();
  }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_adt->push_back(m_values[i]);
    list_vector_detail::synchronize_domain(this->m_adt);
  }

  /// @todo: Use a better test of validity (e.g., that the last few values
  /// on each location are in the vector)
  void check_validity()
  { this->m_passed = this->m_adt->size() == m_exp_size; }

  void finalize()
  {
    this->restore_original_container();
    base_type::finalize();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the pop_back() method.
///
/// @note Because of low performance for distributed containers, only first @p N
///   of the provided set of values will be pushed back by each location in the
///   initialization phase and later popped back in the actual profiling phase.
///   By default, <tt>N = original_size / 500</tt>, but can be changed using the
///   command line argument --reduced_local_size.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @todo pop_back does not work currently on vectors because of the bug
///   reported on GFORGE with tracker# 1299.
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class pop_back_profiler
  : public adt_profiler<ADT,Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT,Counter>;
  using value_type = typename ADT::value_type;

  /// A vector containing the index values
  std::vector<value_type> const& m_values;

public:
  pop_back_profiler(std::string name, ADT* adt,
                    std::vector<value_type> const& vals,
                    int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::pop_back",
                determine_reduced_local_size(vals.size(), argc, argv),
                argc, argv),
      mutating<ADT>(adt), m_values(vals)
  { }

  void initialize_iteration()
  {
    base_type::initialize_iteration();
    this->m_adt->clear();
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_adt->push_back(m_values[i]);
    list_vector_detail::synchronize_domain(this->m_adt);
  }

  void run()
  {
    for (size_t i=0; i!=this->m_test_size; ++i)
      this->m_adt->pop_back();
    list_vector_detail::synchronize_domain(this->m_adt);
  }

  void check_validity()
  { this->m_passed = this->m_adt->size() == 0; }

  void finalize()
  {
    base_type::finalize();
    this->restore_original_container();
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the add() method
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class add_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;

  /// A vector containing the values to be added.
  std::vector<value_type> const& m_values;

  /// Expected size after adding all the elements (for validation purposes).
  size_t m_exp_size;

public:

  add_profiler(std::string name, ADT* adt, std::vector<value_type> const& vals,
               int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::add",
                determine_reduced_local_size(vals.size(), argc, argv),
                argc, argv),
      mutating<ADT>(adt), m_values(vals)
  {
    m_exp_size = allreduce_rmi(
      stapl::plus<size_t>(), this->get_rmi_handle(),
      &add_profiler::template reflect_value<size_t>, this->m_test_size).get();
  }

  void initialize_iteration()
  {
    base_type::initialize_iteration();
    this->m_adt->clear();
  }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_adt->add(m_values[i]);
    this->m_adt->flush();
  }

  /// @todo Add a better validity test (e.g., going through the local
  /// base containers and checking that they contain the values from #m_values.
  void check_validity()
  { this->m_passed = this->m_adt->size() == m_exp_size; }

  void finalize()
  {
    this->restore_original_container();
    base_type::finalize();
  }
};

namespace detail {

template<typename C, typename GID>
GID get_pos(C* c, GID gid)
{
  return gid;
}

template<typename T, typename GID>
typename std::vector<T>::iterator
get_pos(std::vector<T>* c, GID gid)
{
  return c->begin()+gid;
}

template<typename GID, typename... ListParam>
typename list<ListParam...>::iterator
get_pos(list<ListParam...>* c, GID gid)
{
  return c->begin()+gid;
}

} // namespace detail

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the insert() method.
///
/// Inserts elements to the global beginning of the ADT and hence becomes
/// slow in distributed case. THe default number of elements inserted at each
/// location is <tt>N = size_of_original_input_set / 500</tt>, but can be
/// changed using the command line argument --reduced_local_size.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template <typename ADT, typename Counter = counter<default_timer> >
class insert_beg_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using index_type = typename view_traits<ADT>::index_type;
  using indices_type = std::vector<index_type>;

  /// Indices at which to insert the elements from #m_values
  indices_type const& indices;

  /// Value to be inserted into the container
  value_type m_value;

  /// Expected size after inserting the elements.
  size_t m_exp_size;

public:
  insert_beg_profiler(std::string name, ADT* adt, indices_type const& idx,
                  std::vector<value_type> const& vals,
                  int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::insert_begin",
                determine_reduced_local_size(idx.size(), argc, argv),
                argc, argv),
      mutating<ADT>(adt), indices(idx), m_value(vals.front())
  {
    size_t added_size = allreduce_rmi(
      stapl::plus<size_t>(), this->get_rmi_handle(),
      &insert_beg_profiler::template reflect_value<size_t>,
      this->m_test_size).get();

    m_exp_size = this->m_adt->size() + added_size;
  }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_adt->insert(detail::get_pos(this->m_adt, 0), m_value);
    rmi_fence();
  }

  void check_validity()
  {
    // Check that the first value is one of the added values and that the total
    // size increased correctly.

    bool i_added_last = this->m_adt->front() == m_value;

    this->m_passed = (this->m_adt->size() == m_exp_size) &
      allreduce_rmi(stapl::logical_or<bool>(),
        this->get_rmi_handle(),
        &insert_beg_profiler::template reflect_value<bool>,
        i_added_last).get();
  }

  void finalize()
  {
    this->restore_original_container();
    base_type::finalize();
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the erase() method.
///
/// Erases elements from the global beginning of the ADT and hence becomes
/// slow in distributed case. Therefore, not all elements are erased -- the
/// default number of elements erased at each location is
/// <tt>N = original_size / 500</tt>, but can be changed using the command line
/// argument --reduced_local_size.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template <typename ADT, typename Counter = counter<default_timer> >
class erase_beg_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;

  /// Expected size after erasing all the elements
  int m_exp_size;

public:
  erase_beg_profiler(std::string name, ADT* adt, size_t N,
                 int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::erase_begin",
                determine_reduced_local_size(N, argc, argv),
                argc, argv),
      mutating<ADT>(adt)
  {
    size_t rem_cnt = allreduce_rmi(
      stapl::plus<size_t>(), this->get_rmi_handle(),
      &erase_beg_profiler::template reflect_value<size_t>,
      this->m_test_size).get();

    // Expected size = original size - number of erased elements
    m_exp_size = adt->size() - rem_cnt;
  }

  void initialize()
  {
    base_type::initialize();

    if (m_exp_size < 0) {
      this->m_adt->resize(this->m_adt->size() - m_exp_size);
      m_exp_size = 0;
    }
  }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_adt->erase(detail::get_pos(this->m_adt, 0));
    rmi_fence();
  }

  void check_validity()
  { this->m_passed = this->m_adt->size() == size_t(m_exp_size); }

  void finalize_iteration()
  { this->restore_original_container(); }
};


}  // namespace profiling

} // namespace stapl

#endif // STAPL_PROFILING_LIST_VECTOR_HPP
