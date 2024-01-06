/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILING_READ_WRITE_HPP
#define STAPL_PROFILING_READ_WRITE_HPP

namespace stapl {

namespace profiling {

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the set_element() function.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class set_element_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using index_type = typename view_traits<ADT>::index_type;

  /// A vector of indices of elements to be set
  std::vector<index_type> const& indices;

  /// Values to be set
  std::vector<value_type> const& m_values;

public:
  set_element_profiler(std::string name, ADT* adt,
                       std::vector<index_type> const& idx,
                       std::vector<value_type> const& vals,
                       int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::set_element", idx.size(), argc, argv),
      mutating<ADT>(adt), indices(idx), m_values(vals)
  { }

  void finalize()
  {
    base_type::finalize();
    this->restore_original_container();
  }

  void run()
  {
    for(size_t i = 0; i < this->m_test_size; ++i)
      this->m_adt->set_element(indices[i], m_values[i]);
    rmi_fence();
  }

  void validate()
  {
    // Validate by testing a small amount of insertions at each location.
    // Note that the values set by run() can not be used as the indices are
    // not guaranteed to be unique and different elements from m_values might
    // be written to the same position.
    //
    // TODO: Check directly the underlying container instead of using
    // m_adt->get_element as this may hide potential bugs in the application
    // of view mapping functions.

    for(size_t i = 0; i < std::min(this->m_test_size, 10ul); ++i) {
      this->m_adt->set_element(indices[i], m_values[i]);
      this->m_passed &= (this->m_adt->get_element(indices[i]) == m_values[i]);
    }

    rmi_fence();
    this->restore_original_container();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the get_element() method  -- the case when the returned
///   reference is copied to a local storage.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class get_element_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using index_type = typename view_traits<ADT>::index_type;

  /// Obtained elements
  std::vector<value_type> m_results;

  /// The indices to be used for testing
  std::vector<index_type> const& indices;

public:
  get_element_profiler(std::string name, ADT* adt,
                       std::vector<index_type> const& idx,
                       int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::get_element", idx.size(), argc, argv), indices(idx)
  {
    m_results.resize(this->m_test_size);
  }

  void run()
  {
    for(size_t i = 0; i < this->m_test_size; ++i)
      m_results[i] = this->m_adt->get_element(indices[i]);
  }

  void check_validity()
  {
    // TODO: Check directly the underlying container instead of using
    // m_adt->get_element as this may hide potential bugs in the application
    // of view mapping functions.

    for(size_t i = 0; i < this->m_test_size; ++i)
      this->m_passed &= (m_results[i] == this->m_adt->get_element(indices[i]));
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the get_element() method -- the case when the returned
///   reference is not stored.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class get_element_ref_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;

  /// The indices to be used for testing
  std::vector<index_type> const& indices;

public:
  get_element_ref_profiler(std::string name, ADT* adt,
                           std::vector<index_type> const& idx,
                           int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::get_element_ref", idx.size(), argc, argv),
      indices(idx)
  { }

  void run()
  {
    for(size_t i = 0; i < this->m_test_size; ++i)
      GET_AND_KEEP(this->m_adt->get_element(indices[i]))
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the get_element_split() method.
///
/// @tparam ADT The container/view type
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Counter = counter<default_timer>>
class get_element_split_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using index_type = typename view_traits<ADT>::index_type;

  /// The sum of handle values
  std::vector<value_type> m_results;

  /// The indices to be used for testing
  std::vector<index_type> const& indices;

  /// The number of future handles to be buffered before flushing.
  size_t m_k;

  /// Buffer for future handles.
  future<value_type>* m_handles;

public:
  get_element_split_profiler(std::string name, ADT* adt,
                             std::vector<index_type> const& idx,
                             int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::get_element_split", idx.size(), argc, argv),
      indices(idx), m_k(1000)
  {
    for (int i = 1; i < argc; i++) {
      if (!strcmp("--nhandles", argv[i])) {
        m_k = atoi(argv[++i]);
        break;
      }
    }

    m_results.resize(this->m_test_size);
    m_handles = new future<value_type>[m_k];
  }

  ~get_element_split_profiler()
  { delete [] m_handles; }

  void run()
  {
    size_t fc = 0;

    auto res_it = m_results.begin();
    for (size_t i = 0; i < this->m_test_size; ++i)
    {
      // Assign a future handle
      m_handles[fc] = this->m_adt->get_element_split( indices[i] );

      if (fc == m_k-1) {
        // Flush the handles
        for (size_t j = 0; j != m_k; ++j)
          *res_it++ = m_handles[j].get();
        fc = 0;
      }
      else {
        ++fc;
      }
    }

    // Flush the remaining handles
    for (size_t j = 0; j != fc; ++j)
      *res_it++ = m_handles[j].get();
  }

  void check_validity()
  {
    for(size_t i = 0; i < this->m_test_size; ++i)
      this->m_passed &= (m_results[i] == this->m_adt->get_element(indices[i]));
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the apply_set() method.
///
/// @tparam ADT The container/view type
/// @tparam Func  The functor to apply to the elements
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Func,
         typename Counter = counter<default_timer> >
class apply_set_profiler
  : public adt_profiler<ADT, Counter>,
    public mutating<ADT>
{
  using base_type = adt_profiler<ADT, Counter>;
  using index_type = typename view_traits<ADT>::index_type;

  /// The function to be applied to the elements at positions in #indices
  Func const& m_f;

  // The indices to be used for testing
  std::vector<index_type> const& indices;

public:
  apply_set_profiler(std::string name, ADT* adt, Func const& f,
                     std::vector<index_type> const& idx,
                     int argc=0, char **argv=nullptr)
    : base_type(adt, name+"::apply_set", idx.size(), argc, argv),
      mutating<ADT>(adt), m_f(f), indices(idx)
  { }

  void run()
  {
    for (auto const& ind : indices)
      this->m_adt->apply_set(ind, m_f);
    rmi_fence();
  }

  void check_validity()
  {
    for (auto const& ind : indices)
    {
      auto e = this->m_orig_view.get_element(ind);
      m_f(e);
      this->m_passed &= ( this->m_adt->get_element(ind) == e );
    }
  }

  void finalize()
  {
    base_type::finalize();
    this->restore_original_container();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Profiler for the apply_get() method.
///
/// @tparam ADT The container/view type
/// @tparam Func  The functor to apply to the elements
/// @tparam Counter Counter for the profile metric
///
/// @ingroup profiling
////////////////////////////////////////////////////////////////////////////////
template<typename ADT, typename Func,
         typename Counter = counter<default_timer> >
class apply_get_profiler
  : public adt_profiler<ADT, Counter>
{
  using base_type = adt_profiler<ADT, Counter>;
  using value_type = typename ADT::value_type;
  using index_type = typename view_traits<ADT>::index_type;

  /// Stores the return value of the function
  std::vector<typename Func::result_type> m_results;

  /// The function to be applied to the elements at positions in #indices
  Func const& m_f;

  /// The indices to be used for testing
  std::vector<index_type> const& indices;

public:
  apply_get_profiler(std::string name, ADT* adt, Func const& f,
                     std::vector<index_type> const& idx,
                     int argc = 0, char** argv = nullptr)
    : base_type(adt, name+"::apply_get", idx.size(), argc, argv),
      m_f(f), indices(idx)
  {
    m_results.resize(this->m_test_size);
  }

  void run()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      m_results[i] = this->m_adt->apply_get(indices[i], m_f);
  }

  void check_validity()
  {
    for (size_t i = 0; i < this->m_test_size; ++i)
      this->m_passed &=
        (m_results[i] == m_f(this->m_adt->get_element(indices[i])));
  }
};

template<typename ADT, typename Func, typename GID>
void add_read_profilers(prof_cont_t<ADT>& p,
                        std::string const& name, ADT& adt, Func const& f,
                        std::vector<GID> const& idx,
                        int argc, char** argv)
{
  static_assert(is_container<ADT>::value || is_view<ADT>::value,
    "A STAPL container or view must be supplied to add_read_profilers.");

  p.push_back(new get_element_profiler<ADT>(name, &adt, idx, argc, argv));
  p.push_back(new get_element_ref_profiler<ADT>(name, &adt, idx, argc, argv));
  p.push_back(new apply_get_profiler<ADT, Func>(name, &adt, f, idx, argc,argv));
  p.push_back(new get_element_split_profiler<ADT>(name, &adt, idx, argc, argv));
}

template<typename ADT, typename Func, typename GID>
void add_write_profilers(prof_cont_t<ADT>& p,
                        std::string const& name, ADT& adt, Func const& f,
                        std::vector<GID> const& idx,
                        std::vector<typename ADT::value_type> const& vals,
                        int argc, char** argv)
{
  static_assert(is_container<ADT>::value || is_view<ADT>::value,
    "A STAPL container or view must be supplied to add_write_profilers.");

  p.push_back(new set_element_profiler<ADT>(name, &adt, idx, vals, argc, argv));
  p.push_back(new apply_set_profiler<ADT, Func>(name, &adt, f, idx, argc,argv));
}

} // namespace profiling

void print_rw_profilers_cmdline_help(std::ostream& os)
{
#ifdef _STAPL
  if (get_location_id()!=0)
    return;

  os << "Displaying the read/write profilers options.\n"
     << "--nhandles #num  Number of future handles to be kept before flushing."
     << "\n\n";
#endif
}

} // namespace stapl

#endif // STAPL_PROFILING_READ_WRITE_HPP
