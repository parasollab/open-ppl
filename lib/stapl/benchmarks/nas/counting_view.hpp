/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef COUNTING_VIEW_HPP
#define COUNTING_VIEW_HPP

#include <boost/iterator/counting_iterator.hpp>

namespace stapl {

namespace detail {

struct coarse_view
{
  typedef coarse_view                       fast_view_type;
  typedef boost::counting_iterator<size_t>  iterator;
  typedef size_t                            reference;

  size_t m_start;
  size_t m_stop;

  coarse_view(size_t start, size_t stop)
    : m_start(start), m_stop(stop)
  { }

  bool is_local() const
  {
    return true;
  }

  void pre_execute() const
  { }

  void post_execute() const
  { }

  iterator begin() const
  {
    return iterator(m_start);
  }

  iterator end() const
  {
    return iterator(m_stop);
  }

  size_t size() const
  {
    return m_stop - m_start;
  }
};


struct component_holder
{
  size_t m_val;

  component_holder(size_t val)
    : m_val(val)
  { }

  size_t get_id() const
  {
    return m_val;
  }
};

} // namespace detail

namespace nas {

struct counting_view
{
public:
  // Number of elements in view
  //
  size_t m_n_elements;

  // Elements of view are logically block distributed across
  // first m_n_locs locations
  //
  size_t m_n_locs;

  counting_view(size_t n, std::size_t n_locs = get_num_locations())
    : m_n_elements(n), m_n_locs(n_locs)
  {
    stapl_assert(m_n_locs <= get_num_locations(),
                 "counting_view: n_locs is too big");

    stapl_assert(m_n_elements % m_n_locs == 0,
                 "counting_view: need same size per proc");
  }

  void define_type(typer&)
  {
    stapl_assert(0, "Why are you packing counting_view?");
  }

  // Factory Required Typedefs...
  typedef detail::coarse_view              subview_type;
  typedef subview_type                     reference;
  typedef size_t                           cid_type;
  typedef size_t                           value_type;

  size_t size(void) const
  {
    return m_n_elements;
  }

  //
  // Factory Required Methods
  //
  size_t get_num_local_subviews() const
  {
    if (get_location_id() < m_n_locs)
      return 1;

    // else
    return 0;
  }

  size_t get_num_subviews() const
  {
    return m_n_locs;
  }

  detail::component_holder
  get_local_component(size_t idx) const
  {
    stapl_assert(idx == 0,
      "counting_view: invalid local_component id");

    const location_type loc = get_location_id();

    stapl_assert(loc < m_n_locs,
      "counting_view::get_local_component: called on empty location");

    return detail::component_holder(loc);
  }

  subview_type get_subview(cid_type idx) const
  {
    const size_t location_id = get_location_id();

    stapl_assert(idx == location_id, "Invalid cid");

    const size_t block_size  = m_n_elements / m_n_locs;
    const size_t remainder   = m_n_elements % m_n_locs;

    size_t my_block_size     = block_size;
    size_t start;

    if (location_id < remainder)
    {
      ++my_block_size;
      start = location_id * my_block_size;
    }
    else
    {
      start  = remainder * (my_block_size + 1);
      start += (location_id - remainder) * my_block_size;
    }

    const size_t stop = start + my_block_size;

    return subview_type(start, stop);
  }

  //
  // Task Graph (i.e., add_task) Required Methods
  //
  std::pair<location_type, loc_qual>
  get_preferred_location(cid_type idx)
  {
    // FIXME broken on nested
    return std::pair<location_type, loc_qual>(get_location_id(), LQ_CERTAIN);
  }
}; // struct counting_view

} // namespace nas

} // namespace stapl

#endif // ifndef COUNTING_VIEW_HPP
