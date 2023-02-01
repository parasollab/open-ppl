/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/generator.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/paragraph/factory_wf.hpp>

template<typename C>
struct saxpy_wf
{
  typedef void result_type;

  const float m_alpha;
  C&          m_x;
  C&          m_y;
  bool        m_b_print;

  saxpy_wf(float const& alpha, C& x, C& y, bool b_print)
    : m_alpha(alpha), m_x(x), m_y(y), m_b_print(b_print)
  { }

  void operator()(void)
  {
    stapl::counter<stapl::default_timer> ttime;
    ttime.start();

    typedef typename C::distribution_type::container_manager_type::
      base_container_type::iterator iter_t;

    iter_t x_begin = m_x.distribution().container_manager().begin()->begin();
    iter_t y_begin = m_y.distribution().container_manager().begin()->begin();
    iter_t x_end   = m_x.distribution().container_manager().begin()->end();

    for (; x_begin != x_end; ++x_begin, ++y_begin)
    {
      *y_begin += m_alpha * (*x_begin);
    }

    double elapsed = ttime.stop();
    // printf("print = %d\n" m_b_print);

    if (m_b_print)
      printf("SAXPY Time = %f\n", elapsed);
  }

  void define_type(stapl::typer&)
  {
    stapl::abort("define_type called");
  }
};


class block_mapper
{
private:
  const unsigned int m_block_size;

public:
  typedef std::tuple<unsigned int> member_types;

  block_mapper(const unsigned int block_size)
    : m_block_size(block_size)
  { }

  unsigned int operator()(unsigned int n) const
  {
    // std::cout << "Mapping " << n << " to " << n % m_block_size << "\n";
    return n % m_block_size;
  }

  friend bool operator==(block_mapper const& b1, block_mapper const& b2)
  {
    return (b1.m_block_size==b2.m_block_size);
  }
};


class block_reverse_mapper
{
private:
  const unsigned int m_offset;

public:
  typedef std::tuple<unsigned int> member_types;

  block_reverse_mapper(const unsigned int offset)
    : m_offset(offset)
  { }

  unsigned int operator()(unsigned int n) const
  {
    // std::cout << "Reverse mapping " << n << " to " << m_offset + n << "\n";
    return m_offset + n;
  }

  friend bool operator==(block_reverse_mapper const& b1,
                         block_reverse_mapper const& b2)
  {
    return (b1.m_offset==b2.m_offset);
  }
};


namespace stapl {

template<typename PGV, typename Factory>
void create_nested_paragraph(PGV pg_view, Factory const& factory)
{
  const unsigned int parent_num_locs = get_num_locations();
  const unsigned int num_subgroups = 2;
  const unsigned int block_size    =
    std::ceil(parent_num_locs / (double) num_subgroups);
  const unsigned int outer_id      = get_location_id();
  const unsigned int offset        = (outer_id / block_size) * block_size;

  // const unsigned int inner_id      = get_location_id() % block_size;
  // printf("%d ==> %d // offset = %d // blocksize = %d // level = %d\n",
  //        outer_id, inner_id, offset, block_size, factory.m_nesting_level-1);

  unsigned int loc_id = stapl::get_location_id();

  stapl::counter<stapl::default_timer> ttime;
  ttime.start();
  gang g(block_size, block_mapper(block_size), block_reverse_mapper(offset));
  double elapsed = ttime.stop();

  stapl::counter<stapl::default_timer> pg_ttime;
  pg_ttime.start();
  stapl::make_paragraph(factory)();
  double pg_elapsed = pg_ttime.stop();

  if (loc_id == 0 || loc_id == parent_num_locs-1)
    printf("%d: gang creation at level %d is %f, pg time is %f\n",
       loc_id, factory.m_nesting_level-1, elapsed, pg_elapsed);
}

} // namespace stapl


template<typename WF>
struct nesting_factory
  : public stapl::factory_wf
{
  typedef void               result_type;

  const unsigned int m_nesting_level;
  const unsigned int m_num_levels;
  const WF           m_wf;
  const unsigned int m_location;

  nesting_factory(unsigned int nesting_level, unsigned int num_levels,
                  WF const& wf, unsigned int location)
    : m_nesting_level(nesting_level), m_num_levels(num_levels),
      m_wf(wf), m_location(location)
  { }

  template<typename PGV>
  void operator()(PGV const& pg_view) const
  {
    unsigned int loc_id = stapl::get_location_id();

    stapl::counter<stapl::default_timer> factory_time;

    if (m_nesting_level == 1)
      factory_time.start();

    if (m_nesting_level < m_num_levels)
    {
      stapl::counter<stapl::default_timer> ttime;
      ttime.start();
      // printf("%d creating pg\n", m_nesting_level);
      create_nested_paragraph(
        pg_view, nesting_factory(m_nesting_level + 1,
        m_num_levels, m_wf, m_location)
      );
      double elapsed = ttime.stop();

      if (loc_id == 0 || loc_id == stapl::get_num_locations()-1)
        printf("%d: create_nested_pg at level %d is %f\n",
          loc_id, m_nesting_level, elapsed);
    }
    else
    {
      // printf("%d Adding Task\n", m_location);
      pg_view.add_task(m_wf);
    }

    if (m_nesting_level == 1)
    {
      double factory_elapsed = factory_time.stop();

      if (loc_id == 0 || loc_id == stapl::get_num_locations()-1)
        printf("%d: factory_pg at level %d is %f\n",
          loc_id, m_nesting_level, factory_elapsed);
    }
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl_assert(argc == 3, "error");

  const int levels    = atoi(argv[1]);
  const int num_elems = atoi(argv[2]);

  typedef stapl::static_array<float>   cnt_t;
  typedef stapl::array_view<cnt_t>     view_t;

  const float alpha = 3.64;

  cnt_t ct_x(num_elems);
  cnt_t ct_y(num_elems);

  view_t vw_x(ct_x);
  view_t vw_y(ct_y);

  stapl::generate(vw_x, stapl::random_sequence());
  stapl::generate(vw_y, stapl::random_sequence());

  // stapl::rmi_fence();

  // if (stapl::get_location_id() == 0)
  // {
  //   std::cout << "X\n";
  //   for (unsigned int idx = 0; idx<ct_x.size(); ++idx)
  //   {
  //     std::cout << ct_x[idx] << ", ";
  //   }
  //   std::cout << "\n\n\n\n";
  // }

  // stapl::rmi_fence();

  // if (stapl::get_location_id() == stapl::get_num_locations() - 1)
  // {
  //   std::cout << "X\n";
  //   for (unsigned int idx = 0; idx<ct_x.size(); ++idx)
  //   {
  //     std::cout << ct_x[idx] << ", ";
  //   }
  //   std::cout << "\n";
  // }

  stapl::rmi_fence();

  typedef saxpy_wf<cnt_t>                                wf_t;
  typedef typename stapl::result_of::make_paragraph<
    stapl::default_scheduler, nesting_factory<wf_t>
  >::type                                                pg_t;

  stapl::counter<stapl::default_timer> ttime;
  ttime.start();

  pg_t pg = stapl::make_paragraph(nesting_factory<wf_t>(
    1, levels, wf_t(alpha, ct_x, ct_y, stapl::get_location_id() == 0),
    stapl::get_location_id()
  ));

  double elapsed = ttime.stop();

  stapl::counter<stapl::default_timer> exec_ttime;
  exec_ttime.start();
  pg();
  double exec_elapsed = exec_ttime.stop();

  if (stapl::get_location_id() == 0)
    printf ("Elapsed Time is %f, %f\n", elapsed, exec_elapsed);

  return EXIT_SUCCESS;
}
