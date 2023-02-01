/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILING_ADT_HPP
#define STAPL_PROFILING_ADT_HPP

#include <stapl/profiler/base_profiler.hpp>
#include <boost/lexical_cast.hpp>

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Determine the reduced size of the local block for computationally
/// intensive profilers by querying the command line arguments or setting it
/// to @a full_size/500.
////////////////////////////////////////////////////////////////////////////////
size_t determine_reduced_local_size(size_t full_size, int argc, char** argv)
{
  size_t red_sz = std::max(1ul, full_size/500);

  for ( int i = 1; i < argc; i++) {
    if (!strcmp("--reduced_local_size", argv[i])) {
      red_sz = std::min(full_size, boost::lexical_cast<size_t>(argv[++i]));
      break;
    }
  }

  return red_sz;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Query the command line arguments for the number of invocations for
/// methods that are not operating on the elements referenced by the ADT
/// (e.g., constructors). Use @a def as a default.
/// ////////////////////////////////////////////////////////////////////////////////
size_t determine_number_of_invocations(int argc, char** argv, size_t def = 20)
{
  size_t n_invokes = def;
  for (int i = 1; i < argc; i++)
    if (!strcmp("--ninvokes", argv[i])) {
      n_invokes = boost::lexical_cast<size_t>(argv[++i]);
      break;
    }
  return n_invokes;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief A mixin class for profilers which mutate the ADT (a view in this
///   case).
///
/// Stores a reference to the view being profiled and its underlying container,
/// as well as a copy of the original objects, allowing to reset the profiled
/// structures at any point (typically at the end of each profiling iteration).
////////////////////////////////////////////////////////////////////////////////
template <typename ADT, bool = is_view<ADT>::value>
class mutating
{
protected:
  using cont_type = underlying_container_t<ADT>;

  cont_type& m_cont;
  cont_type m_orig_cont;

  ADT& m_view;
  ADT m_orig_view;

  mutating(ADT* adt)
    : m_cont(underlying_container<ADT>::get_mutable(*adt)), m_orig_cont(m_cont),
      m_view(*adt), m_orig_view(*adt)
  { }

  void restore_original_container(void)
  {
    m_cont = m_orig_cont;
    m_view = m_orig_view;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Specializaiton of the mutating mixin for non-views.
////////////////////////////////////////////////////////////////////////////////
template <typename ADT>
class mutating<ADT, false>
{
protected:
  using cont_type = ADT;

  cont_type& m_cont;
  cont_type m_orig_cont;

  ADT& m_view;
  ADT& m_orig_view;

  mutating(ADT* adt)
    : m_cont(*adt), m_orig_cont(m_cont),
      m_view(m_cont), m_orig_view(m_orig_cont)
  { }

  void restore_original_container(void)
  { m_cont = m_orig_cont; }
};

namespace detail {

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction to get the "version" of the profiler used when
/// reporting the profiling results.
///
/// Currently, if ADT is a @ref p_object, the version string is set to "STAPL",
/// otherwise to STL.
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, bool = is_p_object<ADT>::value>
struct version
{
  static constexpr char const* str = "STAPL";
};

template<typename ADT>
struct version<ADT, false>
{
  static constexpr char const* str = "STL";
};

} // namespace detail


////////////////////////////////////////////////////////////////////////////////
/// @brief Base profiler class for methods of containers/views.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template <class ADT, class Counter = counter<default_timer>>
class adt_profiler
  : public base_profiler<Counter>
{
  using base_type = base_profiler<Counter>;

protected:
  using base_type::reflect_value;

  /// The container/view to test
  ADT* m_adt;

  /// @brief The individual test size.
  ///
  /// Represents the number of invocations of the profiled method per every
  /// profiling iteration. Typically the number of elements processed by given
  /// method.
  size_t m_test_size;

  /// Dictates whether or not to coarsen results to a single invocation
  bool m_time_per_invocation;

  /// If true, an extended report including the test size will be given.
  bool m_ext_report;

public:
  using base_type::report;

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler with given number of invocations.
  ///
  /// @param adt The container/view to test
  /// @param name A string identificator of the @p adt
  /// @param sz Number of invocations of profiled function
  //////////////////////////////////////////////////////////////////////////////
  adt_profiler(ADT* adt, std::string name, size_t sz,
               int argc, char** argv)
    : base_type(name, detail::version<ADT>::str, argc, argv),
      m_adt(adt), m_test_size(sz),
      m_time_per_invocation(false), m_ext_report(false)
  {
    for (int i = 1; i < argc; i++)
      if (!strcmp("--extended_report", argv[i])) {
        m_ext_report = true;
        break;
      }
  }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Create a profiler with a default (20) or command-line-given number
  ///   of invocations.
  ///
  /// @param adt The container/view to test
  /// @param name A string identificator of the @p adt
  //////////////////////////////////////////////////////////////////////////////
  adt_profiler(ADT* adt, std::string name, int argc, char** argv)
    : base_type(name, detail::version<ADT>::str, argc, argv), m_adt(adt),
      m_test_size(determine_number_of_invocations(argc, argv, 20)),
      m_time_per_invocation(false), m_ext_report(false)
  {
    for (int i = 1; i < argc; i++)
      if (!strcmp("--extended_report", argv[i])) {
        m_ext_report = true;
        break;
      }
  }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Print relevant data collected during testing.
  //////////////////////////////////////////////////////////////////////////////
  void report(std::stringstream& ss)
  {
    if (m_time_per_invocation)
      for (auto& sample : this->m_samples)
        sample /= m_test_size;

    base_type::report(ss);
  }

  void print_extra_information(std::stringstream& ss)
  {
    if (m_ext_report)
    {
      //       Note :
      ss << "       num_invocations_per_iteration = " << m_test_size << "\n";
#ifdef _STAPL
      ss << "       num_locations = " << this->get_num_locations();
#endif
      ss << "\n";
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the constructor of a container/view with a given
/// distribution/size.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename DistSpec,
         typename Counter = counter<default_timer> >
class constructor_size_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;

  /// @brief Distribution specification (size in the simplest case, in which
  /// the default distribution will be used, but can also be a partitioner
  /// instance or a view-based distribution specification)
  DistSpec m_ds;

  /// A vector of containers/views of type @p ADT
  std::vector<ADT*> m_adts;

public:
  constructor_size_profiler(std::string name, DistSpec const& ds,
                            int argc = 0, char** argv = nullptr)
    : base_type(nullptr, name+"::constructor_size", argc, argv), m_ds(ds)
  {
    this->m_time_per_invocation = true;
    m_adts.resize(this->m_test_size, nullptr);
  }

  void run()
  {
    for (size_t i=0;i < this->m_test_size; ++i)
      this->m_adts[i] = new ADT(m_ds);
  }

  void finalize_iteration()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      delete this->m_adts[i];
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiles the constructor of a container/view with a given
/// distribution/size and initial value.
///
/// @tparam ADT      The container/view type
/// @tparam DistSpec Type determining the distribution of the ADT
/// @tparam Counter  Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename DistSpec,
         typename Counter = counter<default_timer> >
class constructor_size_value_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;

  /// @brief Distribution specification (size in the simplest case, in which
  /// the default distribution will be used, but can also be a partitioner
  /// instance or a view-based distribution specification)
  DistSpec m_ds;

  /// A vector of containers/views of type @p ADT
  std::vector<ADT*> m_adts;

  /// The value to initialize the container elements with
  value_type m_value;

public:
  constructor_size_value_profiler(std::string name, DistSpec const& ds,
                                  int argc = 0, char** argv = nullptr)
    : base_type(nullptr, name+"::constructor_size_value", argc, argv),
      m_ds(ds), m_value(value_type())
  {
    this->m_time_per_invocation = true;
    m_adts.resize(this->m_test_size);
  }

  void run()
  {
    for (size_t i=0;i < this->m_test_size; ++i)
      this->m_adts[i] = new ADT(m_ds, m_value);
  }

  void finalize_iteration()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      delete this->m_adts[i];
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief  Profiles the copy constructor of an arbitrary container.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer> >
class copy_constructor_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;

  /// A vector of containers/views of type @p ADT
  std::vector<ADT*> m_adts;

public:
  copy_constructor_profiler(std::string name, ADT* adt,
                            int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::copy_constructor", argc, argv)
  {
    this->m_time_per_invocation = true;
    m_adts.resize(this->m_test_size);
  }

  void run()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      m_adts[i] = new ADT(*this->m_adt);
  }

  void check_validity()
  {
    for (auto adt_ptr : m_adts)
      this->m_passed &= adt_ptr->size() == this->m_adt->size();
  }

  void finalize_iteration()
  {
    for (size_t i=0; i < this->m_test_size; ++i)
      delete this->m_adts[i];
  }
};

/// @brief Vector of pointers to the base ADT profiler class used to manage
///   the various profilers for given ADT's methods in a polymorphic way.
template<typename ADT>
using prof_cont_t = std::vector<adt_profiler<ADT>*>;

} // namespace profiling

void print_adt_profilers_cmdline_help(std::ostream& os)
{
#ifdef _STAPL
  if (get_location_id()!=0)
    return;
#endif
  os << "Displaying the ADT profilers options.\n"
     << "--ninvokes #num  Number of invocations of profiled methods per every "
        "profiling iteration.\n"
     << "--reduced_local_size #num  Reduced size of the local block for "
     << "computationally intensive profilers.\n"
     << "--extended_report  Print out additional info while reporting the "
        "results\n\n";
}

} // namespace stapl

#endif // STAPL_PROFILING_ADT_HPP
