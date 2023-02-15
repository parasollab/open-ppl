/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_SET_POINTS_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_SET_POINTS_HPP

#include <benchmarks/lonestar/delaunay/triangulation/delaunay_triangulation_merge_sequential.hpp>

#include <boost/random.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief Used to generate random points in the range [min, max) with a
/// specified significance
///////////////////////////////////////////////////////////////////////////////
struct random_point
{
  typedef point2d result_type;
  typedef size_t index_type;

  size_t m_seed;
  double m_min;
  double m_max;
  size_t m_sig;
  double m_scaled_sig;
  double m_scaled_range;

  /////////////////////////////////////////////////////////////////////////////
  /// @param seed Used to seed random number generator
  /// @param min Indicates the smallest value in range of points
  /// @param max Indicates the largest value in range of points
  /// @param sig Indicates the level of significance (decimal point) to cutoff
  /////////////////////////////////////////////////////////////////////////////
  random_point(size_t seed, double min=0.0, double max=10000.0, size_t sig=6)
    : m_seed(seed), m_min(min), m_max(max), m_sig(sig),
      m_scaled_sig(pow(10.0, sig)), m_scaled_range((max-min)*m_scaled_sig)
  {
  }

  result_type operator()(index_type const& idx) const
  {
    typedef boost::mt11213b engine_type;
    typedef boost::random::uniform_01<double> dist_type;
    typedef boost::variate_generator<engine_type, dist_type> generator_type;

    engine_type eng(idx*m_seed + idx);
    generator_type pos_gen(eng, dist_type());

    double x = round(pos_gen() * m_scaled_range) / m_scaled_sig - m_min;
    double y = round(pos_gen() * m_scaled_range) / m_scaled_sig - m_min;

    return point2d(x, y);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_seed);
    t.member(m_min);
    t.member(m_max);
    t.member(m_sig);
    t.member(m_scaled_sig);
    t.member(m_scaled_range);
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Used to check if an array contains any duplicates before calling
/// stapl::unique.
///////////////////////////////////////////////////////////////////////////////
struct check_duplicate_wf
{
  typedef std::tuple<bool, point2d, point2d> result_type;
  template <typename View>
  result_type operator()(View const& view) const
  {
    auto it_p = view.begin();
    auto it_e = view.end();
    for (auto it = view.begin() + 1; it != it_e; ++it, ++it_p) {
      if (*it == *it_p) {
        return std::make_tuple(true, point2d(0.0), point2d(0.0));
      }
    }
    return std::make_tuple(false,
                          (*view.begin()).get_copy(), (*(it_e-1)).get_copy());
  }

  template<typename Reference1, typename Reference2>
  result_type operator()(Reference1 const& px, Reference2 const& py) const
  {
    result_type x = px;
    result_type y = py;
    if (!std::get<0>(x) && !std::get<0>(y)) {
      if (std::get<2>(x) != std::get<1>(y)) {
        return std::make_tuple(false, std::get<1>(x), std::get<2>(y));
      }
    }
    return std::make_tuple(true, std::get<1>(x), std::get<2>(y));
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Copies from from array to graph and allocates the edgelist in the
/// process.
///////////////////////////////////////////////////////////////////////////////
struct copy_points_wf
{
  typedef void result_type;

  template <typename Point, typename Vertex>
  result_type operator()(Point p, Vertex v)
  {
    v.property().set_point(p);
    v.reserve(6);
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief A wrapper class for the initial array of points that handles sorting,
/// removing of duplicates, and filling array with random points.
///////////////////////////////////////////////////////////////////////////////
struct set_points
{
  typedef stapl::array<point2d>               container_type;
  typedef stapl::array_view<container_type>   view_type;

  view_type m_point_view;
  view_type* m_removed_duplicates;

  bool m_has_duplicates;
  double m_generation_time;
  double m_sort_time;

  set_points(size_t num_points)
    : m_point_view(new container_type(num_points)),
      m_removed_duplicates(&m_point_view), m_has_duplicates(true),
      m_generation_time(0.0),  m_sort_time(0.0)
  { }

  set_points(view_type& view)
    : m_point_view(view), m_removed_duplicates(&m_point_view),
      m_has_duplicates(true), m_generation_time(0.0), m_sort_time(0.0)
  { }

  set_points(view_type const& view)
    : m_point_view(view), m_removed_duplicates(&m_point_view),
      m_has_duplicates(true), m_generation_time(0.0), m_sort_time(0.0)
  { }

  ~set_points()
  {
    if (m_removed_duplicates != &m_point_view) {
      delete m_removed_duplicates;
    }
  }

  operator view_type()
  { return get_view(); }

  view_type& operator()()
  { return get_view(); }

  view_type& get_view()
  {
    if (m_has_duplicates) {
      return m_point_view;
    } else {
      return *m_removed_duplicates;
    }
  }

  size_t size() const
  {
    if (m_has_duplicates) {
      return m_point_view.size();
    } else {
      return m_removed_duplicates->size();
    }
  }

  void fill_with_random_points(size_t seed=0)
  {
    counter<default_timer> timer;
    timer.start();

    size_t n = m_point_view.size();

    using random_view_t =
      typename functor_view_type<functor_container<random_point>>::type;

    // create a view of randomly generated points
    random_view_t random_points = functor_view(n, random_point(seed));

    // populate the point view with random points
    copy(random_points, m_point_view);

    m_generation_time = timer.stop();
    m_has_duplicates = true;
  }

  struct sort_wf
  {
    typedef void result_type;
    template <typename View>
    void operator()(View view)
    {
      std::sort(view.begin(), view.end());
    }
  };

  void sort_points()
  {
    counter<default_timer> timer;
    timer.start();
    if (get_num_locations() <= 1 && get_num_processes() <= 1) {
      map_func(sort_wf(), native_view(m_point_view));
    } else {
      stapl::sort(m_point_view);
    }
    m_sort_time = timer.stop();
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Removes duplicates and stores the view over the unique array.
  /// Changes the flag so that @ref set_points::operator()() returns the unique
  /// array.
  /// @todo Improve performance of stapl::unique or implement faster method.
  /// Checking if the array has duplicates is over 250 times faster than calling
  /// stapl::unique. (Should only contain at most a few duplicates)
  /////////////////////////////////////////////////////////////////////////////
  void remove_duplicates()
  {
    if (!m_has_duplicates) {
      return;
    }

    auto check = map_reduce(check_duplicate_wf(), check_duplicate_wf(),
                            native_view(m_point_view));
    if (std::get<0>(check)) {
      m_removed_duplicates = new view_type(stapl::unique(m_point_view).first);
    } else if (m_removed_duplicates != &m_point_view) {
      delete m_removed_duplicates;
      m_removed_duplicates = &m_point_view;
    }
    m_has_duplicates = false;
  }

  template <typename View>
  void copy_points(View& view)
  {
    view = View(size());
    map_func(copy_points_wf(), get_view(), view);
  }
};


} // namespace delaunay


} // namespace stapl




#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_SET_POINTS_HPP */
