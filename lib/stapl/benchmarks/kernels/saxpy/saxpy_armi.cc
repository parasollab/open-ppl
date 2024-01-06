/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <type_traits>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/generator.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/irange.hpp>
#include "../runtime_base_profiler.hpp"

// #define BENCHMARK_RANDOM_SEQUENCE 1

template<typename Integer>
class integer_iterator
  : public boost::iterator_facade<
             integer_iterator<Integer>,
             Integer,
             boost::random_access_traversal_tag,
             Integer,
             std::ptrdiff_t
           >
{
private:
  using base_t = boost::iterator_facade<
                   integer_iterator<Integer>,
                   Integer,
                   boost::random_access_traversal_tag,
                   Integer,
                   std::ptrdiff_t>;

  friend class ::boost::iterator_core_access;

public:
  using typename base_t::value_type;
  using typename base_t::difference_type;
  using typename base_t::reference;

private:
  value_type m_value;

public:
  integer_iterator(void) noexcept
    : m_value()
  { }

  explicit integer_iterator(value_type x) noexcept
    : m_value(x)
  { }

private:
  void increment(void) noexcept
  { ++m_value; }

  void decrement(void) noexcept
  { --m_value; }

  void advance(difference_type offset) noexcept
  { m_value += offset; }

  difference_type distance_to(integer_iterator const& other) const noexcept
  {
      return std::is_signed<value_type>::value
               ? (other.m_value - m_value)
               : (other.m_value >= m_value)
                 ? static_cast<difference_type>(other.m_value - m_value)
                 : -static_cast<difference_type>(m_value - other.m_value);
  }

  bool equal(integer_iterator const& other) const noexcept
  { return m_value == other.m_value; }

  reference dereference(void) const noexcept
  { return m_value; }
};


template<typename Integer>
class integer_range
{
public:
  using size_type      = std::size_t;
  using const_iterator = integer_iterator<Integer>;
  using iterator       = const_iterator;

private:
  Integer m_first;
  Integer m_last;

public:
  integer_range(Integer first, Integer last)
    : m_first(first),
      m_last(last)
  { }

  bool empty(void) const noexcept
  { return (m_first==m_last); }

  size_type size(void) const noexcept
  { return (m_last - m_first); }

  const_iterator begin(void) const noexcept
  { return integer_iterator<Integer>{m_first}; }

  const_iterator end(void) const noexcept
  { return integer_iterator<Integer>{m_last}; }

  void define_type(stapl::typer& t)
  {
    t.member(m_first);
    t.member(m_last);
  }
};


template<typename Integer>
integer_range<Integer> irange(Integer first, Integer last)
{
  BOOST_ASSERT( first <= last );
  return integer_range<Integer>(first, last);
}


using namespace stapl;


template<typename C>
struct saxpy_wf
{
  using result_type = void;

  const float m_alpha;
  C&          m_x;
  C&          m_y;

  saxpy_wf(float const& alpha, C& x, C& y)
    : m_alpha(alpha),
      m_x(x),
      m_y(y)
  { }

  void operator()(void) const
  {
    auto x_begin = m_x.distribution().container_manager().begin()->begin();
    auto x_end   = m_x.distribution().container_manager().begin()->end();
    auto y_begin = m_y.distribution().container_manager().begin()->begin();

    for (; x_begin != x_end; ++x_begin, ++y_begin)
    {
      *y_begin += m_alpha * (*x_begin);
    }
  }
};


template<typename C>
class runtime_factory
  : public p_object
{
private:
  const std::size_t                         m_levels;
  const float                               m_alpha;
  lazy_reference_wrapper<C>                 m_x;
  lazy_reference_wrapper<C>                 m_y;
  p_object_pointer_wrapper<runtime_factory> m_parent;
  bool                                      m_done;

public:
  template<typename T1, typename T2>
  runtime_factory(const std::size_t levels,
                  float alpha, T1&& t1, T2&& t2)
    : m_levels(levels),
      m_alpha(alpha),
      m_x(std::forward<T1>(t1)),
      m_y(std::forward<T1>(t2)),
      m_done(false)
  { }

  template<typename T1, typename T2, typename T3>
  runtime_factory(const std::size_t levels,
                  float alpha, T1&& t1, T2&& t2, T3&& t3)
    : m_levels(levels),
      m_alpha(alpha),
      m_x(std::forward<T1>(t1)),
      m_y(std::forward<T1>(t2)),
      m_parent(std::forward<T3>(t3)),
      m_done(false)
  { }

  void set_done(void)
  {
    stapl_assert(!m_done, "Already called.");
    m_done = true;
  }

  void operator()()
  {
    const auto levels = (m_levels - 1);
    if (levels==0) {
      // end of recursion
      saxpy_wf<C>{m_alpha, m_x, m_y}();
    }
    else {
      const std::size_t factor = 2;
      const std::size_t nlocs  = (this->get_num_locations() / factor);
      if (nlocs==0)
        abort("Not enough locations.");
      if ((nlocs*factor) != this->get_num_locations())
        abort("Locations not evenly divided.");

      if ((this->get_location_id() % nlocs) == 0) {
        // we have a leader
        const unsigned int first = this->get_location_id();
        const unsigned int last  = first + nlocs;

        async_construct<runtime_factory>(
          [](runtime_factory* p)
          {
            (*p)();
            delete p;
          },
          this->get_rmi_handle(), location_range(irange(first, last)),
          levels,
          m_alpha, m_x, m_y, pointer(this));
      }

      block_until([&] { return m_done; });
    }

    if (m_parent)
      m_parent->set_done();
  }
};


template<typename C>
class saxpy_prof
  : public runtime_base_profiler<>
{
private:
  std::size_t m_levels;
  float       m_alpha;
  C&          m_x;
  C&          m_y;

  static std::string make_name(std::size_t levels)
  {
    return std::string{"saxpy(levels="}               +
             boost::lexical_cast<std::string>(levels) +
           std::string{") "}                          +
           boost::lexical_cast<std::string>(stapl::get_num_locations());
  }

public:
  saxpy_prof(std::size_t levels, float alpha, C& x, C& y, int argc, char** argv)
    : runtime_base_profiler<>(make_name(levels), argc, argv),
      m_levels(levels),
      m_alpha(alpha),
      m_x(x),
      m_y(y)
  { }

  void run(void)
  {
    runtime_factory<C> factory{m_levels, m_alpha, m_x, m_y};
    factory();
  }
};


exit_code stapl_main(int argc, char* argv[])
{
  const std::size_t num_elems = atoi(argv[1]);
  const std::size_t levels    = atoi(argv[2]);
  const bool print            = (argc==3);

  const float alpha = 3.64;

  // set-up containers
  const auto my_id = stapl::get_location_id();

  using cnt_t = static_array<float>;

  cnt_t ct_x(num_elems, 2.0);
  cnt_t ct_y(num_elems, 1.0);

#ifdef BENCHMARK_RANDOM_SEQUENCE
  using view_t = array_view<cnt_t>;

  view_t vw_x(ct_x);
  view_t vw_y(ct_y);
  generate(vw_x, random_sequence());
  generate(vw_y, random_sequence());
#endif

  rmi_fence();

  if (print && my_id == 0) {
    std::cout << "Size " << num_elems << " Levels " << levels << std::endl;

    std::cout << "X =\n\t";
    for (unsigned int idx = 0; idx<ct_x.size(); ++idx)
    {
      std::cout << ct_x[idx] << ' ';
    }
    std::cout << "\n\n\n";

    std::cout << "Y =\n\t";
    for (unsigned int idx = 0; idx<ct_x.size(); ++idx)
    {
      std::cout << ct_y[idx] << ' ';
    }
    std::cout << "\n\n\n";
  }

  rmi_fence();

  double exec_elapsed = 0.0;

  if (print) {
    counter<default_timer> timer;
    timer.start();

    runtime_factory<cnt_t> factory{levels, alpha, ct_x, ct_y};
    factory();

    exec_elapsed = timer.stop();
  }
  else {
    saxpy_prof<cnt_t> pr{levels, alpha, ct_x, ct_y, argc, argv};
    pr.collect_profile();
    pr.report();
  }

  rmi_fence();

  if (print && my_id == 0) {
    std::cout << "Y =\n\t";
    for (unsigned int idx = 0; idx<ct_x.size(); ++idx)
    {
      std::cout << ct_y[idx] << ' ';
    }
    std::cout << "\n\n\n";

    if (my_id == 0)
      printf ("Elapsed Time is %f\n", exec_elapsed);
  }

  return EXIT_SUCCESS;
}
