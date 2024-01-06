#include <cmath>
#include <cstdio>
#include <sstream>
#include <vector>
#include <thread>
#include <boost/lexical_cast.hpp>
#include <stapl/array.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/containers/distribution/composed_specification.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/numeric.hpp>
#include <stapl/utility/do_once.hpp>

#include "../shared/nested_factories.hpp"

//////////////////////////////////////////////////////////////////////
/// @brief Provides the location id inside a work function and allows
/// accumulation of values on a per-location basis and the reduction to
/// a final global value.
//////////////////////////////////////////////////////////////////////
struct location_beacon
  : public stapl::p_object
{
private:
  int m_val;

public:
  location_beacon()
    : m_val(0)
  { }

  void add(int val)
  { m_val += val; }

  int local_val(void)
  { return m_val; }

  int global_val(void)
  {
    return stapl::allreduce_rmi(stapl::plus<int>(), this->get_rmi_handle(),
             &location_beacon::local_val).get();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Maps partitions of an array to the first column of locations.
///
/// Assumes array elements have been mapped into the correct number of
/// partitions so that there is only one partition per location.
//////////////////////////////////////////////////////////////////////
struct map_to_col0
{
private:
  // number of locations in a row
  long unsigned int m_row_width;

public:
  typedef long unsigned int    gid_type;
  typedef stapl::location_type index_type;

  map_to_col0(long unsigned int row_width)
    : m_row_width(row_width)
  { }

  index_type operator()(gid_type id) const
  { return id * m_row_width; }

  void update(std::vector<std::tuple<
    std::pair<long unsigned int, long unsigned int>, long unsigned int,
    stapl::location_type>> const&, size_t)
  { }

  void define_type(stapl::typer& t)
  { t.member(m_row_width); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Maps partitions of an array to a row of locations.
///
/// Assumes array elements have been mapped into the correct number of
/// partitions so that there is only one partition per location.
//////////////////////////////////////////////////////////////////////
struct map_to_row
{
private:
  // First location to which the container is mapped.
  // Locations are indexed row-major in this test, so widths aren't needed.
  long unsigned int m_offset;

public:
  typedef long unsigned int    gid_type;
  typedef stapl::location_type index_type;

  map_to_row(unsigned int row_index, unsigned int row_width)
    : m_offset(row_index * row_width)
  { }

  index_type operator()(gid_type id) const
  { return m_offset + id; }

  void define_type(stapl::typer& t)
  { t.member(m_offset); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specifies the distribution of each container in the composed
/// array instance being tested.
///
/// The outer container is balance distributed over the locations in
/// location column 0 of the 2-D location layout.  Each of the inner
/// containers is balanced distributed across a location row.
//////////////////////////////////////////////////////////////////////
struct get_spec_wf
{
private:
  unsigned int         m_outer_size;
  unsigned int         m_inner_size;
  stapl::location_type m_row_width;
  stapl::location_type m_col_length;

  // gid->pid mapper used to determine to which location row each inner
  // container is mapped.  A member is used to avoid repeated construction.
  // GID to partition is balanced across the number of locations in column.
  stapl::balance_map<1, long unsigned int, long unsigned int> m_outer_gid_map;

public:
  typedef stapl::distribution_spec<> result_type;

  get_spec_wf(unsigned int outer_size, unsigned int inner_size,
              stapl::location_type row_width, stapl::location_type col_length)
    : m_outer_size(outer_size), m_inner_size(inner_size),
      m_row_width(row_width), m_col_length(col_length),
      m_outer_gid_map(m_outer_size, m_col_length)
  { }

  result_type operator()(std::vector<long unsigned int> const& index) const
  {
    if (index.empty())
    {
      // Construct the partition to location mapping function for the
      // distribution.
      map_to_col0 part_map(m_row_width);

      // Construct and return the specification for the outer container.
      return stapl::arbitrary(m_outer_size, m_col_length,
                              m_outer_gid_map, part_map);
    }
    else
    {
      stapl_assert(index.size()==1,"gen_spec_wf supports 2D containers only.");

      // Each inner container is distributed across a row of locations.
      // The row is determined by the index of the container, and matches
      // the balanced distribution of the outer container. I.e., elements 0
      // through n/col_length will be distributed across the first row of
      // locations.
      unsigned int location_row = m_outer_gid_map(index[0]);
      map_to_row part_map(location_row, m_row_width);

      std::vector<stapl::location_type> locs(m_row_width);
      std::iota(locs.begin(), locs.end(),
        m_outer_gid_map(index[0])*m_row_width);
      return stapl::balance(m_inner_size, locs);
    }
  }
};


struct outer_init_wf
{
private:
  unsigned int m_inner_size;

public:
  typedef void result_type;

  outer_init_wf(unsigned int inner_size)
    : m_inner_size(inner_size)
  { }

  template <typename View, typename Index>
  result_type operator()(View vw, Index i)
  { stapl::iota(vw, i*m_inner_size); }

  void define_type(stapl::typer& t)
  { t.member(m_inner_size); }
};


struct add_loc_val_product
{
private:
  stapl::pointer_wrapper<location_beacon> m_lb;

public:
  typedef void result_type;

  add_loc_val_product(location_beacon* lb)
    : m_lb(lb)
  { }

  template <typename Ref>
  result_type operator()(Ref r) const
  {
    if (!m_lb)
      stapl::abort("failed to resolve handle.\n");
    m_lb->add(m_lb->get_location_id()*r);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_lb);
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  stapl_assert(argc==3,"./row_dist_arrays m n");
  unsigned int m = boost::lexical_cast<unsigned int>(argv[1]);
  unsigned int n = boost::lexical_cast<unsigned int>(argv[2]);
  stapl::location_type nlocs = stapl::get_num_locations();

  location_beacon lb;

  // Factor the locations into a 2D layout.
  stapl::location_type col_length  = sqrt(nlocs);
  stapl::location_type row_width =
    col_length * col_length == nlocs ? col_length : col_length * 2;
  stapl_assert(row_width * col_length == nlocs,
    "# locations must be a square or 2*square");

  typedef stapl::array<unsigned int,
    stapl::view_based_partition<stapl::distribution_spec<>>,
    stapl::view_based_mapper<stapl::distribution_spec<>>>    inner_cont_t;

  typedef stapl::array<inner_cont_t,
    stapl::view_based_partition<stapl::distribution_spec<>>,
    stapl::view_based_mapper<stapl::distribution_spec<>>>    outer_cont_t;

  // Construct the container and view
  get_spec_wf gen_wf(m, n, row_width, col_length);
  stapl::composed_dist_spec comp_spec(gen_wf);
  outer_cont_t cont(comp_spec);
  stapl::array_view<outer_cont_t> view(cont);

  // Initialize the array with a sequence
  stapl::map_func(outer_init_wf(n),
                  view, stapl::counting_view<unsigned int>(m));

  // Compute the sum of the elements after multiplying each of them by the
  // id of the quadrant in which they're found.  Quadrants are row-major and
  // indexed beginning at the top left corner.
  nested_map_func(add_loc_val_product(&lb), view);
  int sum = lb.global_val();

  int ref = 0;
  for (unsigned int i = 0; i != m; ++i)
  {
    for (unsigned int j = 0; j != n; ++j)
    {
      // compute the linear index of the element and multiply by quadrant id
      ref += (i*m + j) * ((i < m/2 ? 0 : 2) + (j < n/2 ? 0 : 1));
    }
  }

  stapl::do_once([&] {
    if (ref == sum)
      printf("Test: row_dist_arrays\nVersion: stapl\nStatus: PASS\n");
    else
    {
      printf("Test: row_dist_arrays\nVersion: stapl\nStatus: FAIL\n");
      printf("Note: sum == %d, expected == %d\n", sum, ref);
    }
  });

  return EXIT_SUCCESS;
}
