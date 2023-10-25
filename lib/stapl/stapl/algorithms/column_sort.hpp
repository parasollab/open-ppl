/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_COLUMN_SORT_HPP
#define STAPL_ALGORITHMS_COLUMN_SORT_HPP

#include <limits>
#include <type_traits>
#include <stapl/views/slices_view.hpp>
#include <stapl/containers/partitions/ndim_partition.hpp>
#include <stapl/containers/distribution/specifications.hpp>


namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Traits class used to define the plus and minus infinity values
/// used to pad the matrix in column sort.
///
/// The extreme values of the signed numeric type are used. The value of
/// the respective infinities is assigned based on their ordering by the
/// comparator used in the sort.
///
/// @tparam T Signed numeric type of the input data being sorted
/// @tparam Compare Comparator functor that defines a strict weak ordering
//////////////////////////////////////////////////////////////////////
template <typename T, typename Compare>
struct column_sort_constants
{
private:
  Compare m_compare;
  T m_min;
  T m_max;

public:
  column_sort_constants(Compare comp)
    : m_compare(std::move(comp)), m_min(std::numeric_limits<T>::min()),
      m_max(std::numeric_limits<T>::max())
  { }

  T plus_infinity(void)
  { return m_compare(m_min, m_max) ? m_max : m_min; }

  T minus_infinity(void)
  { return m_compare(m_min, m_max) ? m_min : m_max; }
};


//////////////////////////////////////////////////////////////////////
/// @brief enum defining which transform operation is to be performed.
//////////////////////////////////////////////////////////////////////
enum {TRANSPOSE_AND_RESHAPE, RESHAPE_AND_TRANSPOSE};


using dimensions = std::tuple<std::size_t, std::size_t>;


//////////////////////////////////////////////////////////////////////
/// @brief Iterator class used to sort each of the slices
///
/// The iterator is defined because the view over the slice representing a
/// column does not define the sequence methods (i.e., begin and end) needed
/// to call std::sort on the view.
///
/// This class is based on the @ref index_iterator class in view_iterator.hpp.
/// It differs from that class by defining the basic operations needed for
/// sequence operations on the view domain instead of the view itself.
///
/// @tparam View The view to iterate over
/// @tparam Category Iterator category
//////////////////////////////////////////////////////////////////////
template <typename View, typename Category = std::forward_iterator_tag>
class simple_index_iterator
  : public iterator_facade<simple_index_iterator<View, Category>,
                           detail::index_accessor<View>,
                           Category>
{
  friend class stapl::iterator_core_access;

public:
  typedef typename View::domain_type                              domain_type;
  typedef typename domain_type::gid_type                          gid_type;
  typedef typename View::index_type                               index_type;
  typedef detail::index_accessor<View>                            accessor_t;

private:
  typedef iterator_facade<simple_index_iterator, accessor_t, Category>
                                                                  base_type;
  typedef typename base_type::difference_type                     diff_t;

  View*      m_view;
  index_type m_index;

public:
  simple_index_iterator(void)
    : m_view(0), m_index()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator over the given @c view at position
  ///        @c index.
  //////////////////////////////////////////////////////////////////////
  simple_index_iterator(View const& view, index_type index)
    : m_view(&const_cast<View&>(view)), m_index(index)
  { }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the associated accessor with the iterator.
  //////////////////////////////////////////////////////////////////////
  accessor_t access(void) const
  {
    return accessor_t(const_cast<View const*>(m_view), m_index);
  }

  void increment(void)
  {
    m_index = m_view->domain().advance(m_index, 1);
  }

  void decrement(void)
  {
    m_index = m_view->domain().advance(m_index, -1);
  }

  void advance(diff_t n)
  {
    m_index = m_view->domain().advance(m_index, n);
  }

  bool equal(simple_index_iterator const& rhs) const
  {
    return m_index == rhs.m_index;
  }

  template <typename OtherView>
  bool equal(simple_index_iterator<OtherView, Category> const& rhs) const
  {
    return m_index == rhs.m_index;
  }

  diff_t distance_to(simple_index_iterator const& rhs) const
  {
    return m_view->domain().distance(m_index, rhs.m_index);
  }

  bool less_than(simple_index_iterator const& rhs) const
  {
    return m_view->domain().less_than(m_index, rhs.m_index);
  }

public:
  void define_type(typer& t)
  {
    t.member(m_view);
    t.member(m_index);
  }

  index_type index(void)
  {
    return m_index;
  }
}; //class simple_index_iterator


//////////////////////////////////////////////////////////////////////
/// @brief Create a multiarray container for the matrices required
///        by the column_sort algorithm.
/// @tparam T The type of the elements in the multiarray container created
/// @param rows Number of rows of the resulting matrix
/// @param columns Number of columns of the resulting matrix
//////////////////////////////////////////////////////////////////////
template <typename T>
stapl::multiarray<2, T,
  stapl::sliced_md_distribution_spec<stapl::index_sequence<1>,
    stapl::index_sequence<0, 1>>::type>
create_multiarray(size_t rows, size_t columns)
{
  using reverse_traversal = stapl::index_sequence<0, 1>;
  using sliced_dims       = stapl::index_sequence<1>;
  using sliced_part_t     = stapl::sliced_md_distribution_spec<
                              sliced_dims, reverse_traversal>::type;

  return stapl::multiarray<2, T, sliced_part_t>(
    stapl::sliced_volumetric<reverse_traversal, sliced_dims>(
      std::make_tuple(rows, columns))
    );
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function that applies the RESHAPE_and_TRANSPOSE or
///        TRANSPOSE_and_RESHAPE operations to the input matrix.
//////////////////////////////////////////////////////////////////////
struct transform_matrix
{
private:
  dimensions m_input_size;

  int m_operation;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param input_size The size of the input matrix.
  /// @param operation The transformation to apply to the input matrix.
  //////////////////////////////////////////////////////////////////////
  transform_matrix(dimensions input_size, int operation)
    : m_input_size(std::move(input_size)), m_operation(operation)
  { }

  ////////////////////////////////////////////////////////////////////
  /// @param element Current element to process
  /// @param view The output view.
  ////////////////////////////////////////////////////////////////////
  template <typename Element, typename View>
  void operator()(Element&& element, View&& view)
  {
    auto element_idx = stapl::index_of(element);
    size_t linear_idx, out_x, out_y = 0;

    size_t element_idx_x  = std::get<0>(element_idx);
    size_t element_idx_y  = std::get<1>(element_idx);
    size_t input_size_x   = std::get<0>(m_input_size);
    size_t input_size_y   = std::get<1>(m_input_size);

    if (m_operation == TRANSPOSE_AND_RESHAPE)
    {
      linear_idx  = (input_size_y * element_idx_x) + element_idx_y;
      out_x       = linear_idx % input_size_x;
      out_y       = linear_idx / input_size_x;
    } else {
      linear_idx  = (input_size_x * element_idx_y) + element_idx_x;
      out_x       = linear_idx / input_size_y;
      out_y       = linear_idx % input_size_y;
    }

    view.set_element(make_tuple(out_x, out_y), element);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_input_size);
    t.member(m_operation);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that sorts a column, used by column_sort
///        to sort each column of the matrix.
//////////////////////////////////////////////////////////////////////
template <typename Compare>
struct sort_column
{
private:
  Compare m_compare;

public:
  sort_column(Compare compare)
    : m_compare(std::move(compare))
  { }

  ////////////////////////////////////////////////////////////////////
  /// @param column The column to sort.
  ////////////////////////////////////////////////////////////////////
  template <typename Column>
  void operator()(Column&& column)
  {
    typename Column::index_type end =
      column.domain().advance(column.domain().last(), 1);

    simple_index_iterator<Column> first(column, column.domain().first());
    simple_index_iterator<Column> last(column, end);
    std::sort(first, last, m_compare);
  }

  void define_type(stapl::typer& t)
  { t.member(m_compare); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function for shifting and un-shifting the matrix.
/// This is the first phase of the shift step of the algorithm.
//////////////////////////////////////////////////////////////////////
struct shift_matrix
{
private:
  dimensions m_input_dimensions;

public:
  shift_matrix(dimensions input_dimensions)
    : m_input_dimensions(std::move(input_dimensions))
  { }

  ///////////////////////////////////////////////////////////////////
  /// @param element Current element of the view.
  /// @param view input view.
  ///////////////////////////////////////////////////////////////////
  template <typename Element, typename View>
  void operator()(Element&& element, View&& view)
  {
    dimensions view_dimensions = view.dimensions();
    size_t element_x    = std::get<0>(stapl::index_of(element));
    size_t element_y    = std::get<1>(stapl::index_of(element));
    size_t input_dim_x  = std::get<0>(m_input_dimensions);
    size_t input_dim_y  = std::get<1>(m_input_dimensions);
    size_t vew_dim_x    = std::get<0>(view_dimensions);
    size_t vew_dim_y    = std::get<1>(view_dimensions);
    size_t view_size    = vew_dim_x * vew_dim_y;
    size_t input_size   = input_dim_x * input_dim_y;

    size_t element_linear_idx, shift;
    dimensions output_idx;

    if (view_size > input_size)
    {
      // shift matrix
      shift = ceil(input_dim_x / 2);
      element_linear_idx            = (element_y * input_dim_x) + element_x;
      size_t output_element_linear  = element_linear_idx + shift;
      size_t output_element_x       = output_element_linear % input_dim_x;
      size_t output_element_y       = output_element_linear / input_dim_x;

      // check if the output position of the element is inside the
      // boundaries of the output view.
      if (element_linear_idx >= shift ||
          element_linear_idx < (input_size + shift))
      {
        output_idx = std::make_pair(output_element_x, output_element_y);
        view.set_element(output_idx, element);
      }
    } else {
      // unshift matrix
      shift = ceil(vew_dim_x / 2);
      element_linear_idx            = (element_y * vew_dim_x) + element_x;
      size_t output_element_linear  = element_linear_idx - shift;
      size_t output_element_x       = output_element_linear % vew_dim_x;
      size_t output_element_y       = output_element_linear / vew_dim_x;

      // check if the output position of the element is inside the
      // boundaries of the output view.
      if (element_linear_idx >= shift &&
          element_linear_idx < (view_size + shift))
      {
        output_idx = std::make_pair(output_element_x, output_element_y);
        view.set_element(output_idx, element);
      }
    }
  }

  void define_type(stapl::typer& t)
  { t.member(m_input_dimensions); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function for padding the edges of the matrix. This
/// is the second phase of the shift step of the algorithm.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct set_edges
{
private:
  dimensions m_view_dimensions;
  T m_minus_infinity, m_plus_infinity;

public:
  set_edges(dimensions view_dimensions, T minus_inf, T plus_inf):
      m_view_dimensions(std::move(view_dimensions)),
      m_minus_infinity(minus_inf),
      m_plus_infinity(plus_inf)
  { }

  ///////////////////////////////////////////////////////////////////
  /// @param element Current element of the view. If the given
  /// element is at the edge of the matrix, then a value of
  /// -inf or +inf is set to the value. Otherwise the value is not
  /// modified
  ///////////////////////////////////////////////////////////////////
  template <typename Element>
  void operator()(Element&& element)
  {
    size_t view_dim_x           = std::get<0>(m_view_dimensions);
    size_t view_dim_y           = std::get<1>(m_view_dimensions);
    size_t view_length          = view_dim_x * view_dim_y;
    size_t shift                = ceil(view_dim_x / 2);
    dimensions element_idx      = stapl::index_of(element);
    size_t element_idx_x        = std::get<0>(element_idx);
    size_t element_idx_y        = std::get<1>(element_idx);
    size_t element_linear_idx = (element_idx_y * view_dim_x) + element_idx_x;

    // At the upper-left edge of the matrix
    if (element_linear_idx < shift)
      element = m_minus_infinity;

    // At the lower-right edge of the matix
    else if (element_linear_idx >= (view_length + shift))
      element = m_plus_infinity;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_view_dimensions);
    t.member(m_minus_infinity);
    t.member(m_plus_infinity);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that copies the input view to a 2-D matrix.
//////////////////////////////////////////////////////////////////////
struct fill_matrix
{
private:
  size_t m_padding;

public:
  fill_matrix(size_t padding)
    : m_padding(padding)
  { }

  ///////////////////////////////////////////////////////////////////
  /// @param element Current element of the view.
  /// @param view The output view.
  ///////////////////////////////////////////////////////////////////
  template <typename Element, typename View>
  void operator()(Element&& element, View&& view)
  {
    dimensions view_dims  = view.dimensions();
    size_t out_value_idx  = stapl::index_of(element) + m_padding;
    size_t view_dims_x    = std::get<0>(view_dims);
    size_t out_value_x    = out_value_idx % view_dims_x;
    size_t out_value_y    = out_value_idx / view_dims_x;
    dimensions out_idx    = std::make_pair(out_value_x, out_value_y);

    view.set_element(out_idx, element);
  }

  void define_type(stapl::typer& t)
  { t.member(m_padding); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Calculate the padding needed to meet the required
///   dimensions of the matrices, based on the Leigthon's constraint of:
///   rows >= 2(columns - 1)^2
/// @param view Input view to be sorted
/// @return The required padding to meet the restriction
//////////////////////////////////////////////////////////////////////
template <typename View>
size_t calculate_padding(View&& view)
{
  size_t input_length   = view.size();
  size_t num_locations  = stapl::get_num_locations();
  size_t min_rows       = 2 * ((num_locations - 1) * (num_locations - 1));

  // Case 1: The input is smaller or equal than (min_rows * num_locations)
  if (input_length <= (min_rows * num_locations))
    return (min_rows * num_locations) - input_length;

  size_t remainder  = input_length % num_locations;
  size_t padding    = num_locations - remainder;
  size_t rows       = (input_length + padding) / num_locations;

  // Case 2: Odd number of rows, this wont happen becase of min_rows is even
  if (rows % 2 != 0)
    padding = padding + num_locations;

  return padding;
}


//////////////////////////////////////////////////////////////////////
/// @brief Calculate the dimensions of the matrix.
//////////////////////////////////////////////////////////////////////
template <typename Size>
dimensions calculate_dimensions(Size input_length)
{
  size_t num_locations = stapl::get_num_locations();

  if (input_length <= num_locations)
    return dimensions(input_length, 1); // rows, columns

  if (input_length % num_locations != 0)
  {
    // At this point, num_locations must be a factor of
    // input_length since calculate_padding *must* have been
    // called prior this method.
    abort("stapl::get_num_locations() must be a factor of input length");
  }

  size_t rows = input_length / num_locations;

  return dimensions(rows, num_locations); // rows, columns
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function for copying the the 2-D matrix into the
///        input view.
//////////////////////////////////////////////////////////////////////
struct copy_to_view
{
private:
  size_t m_padding;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param padding The padding used to fill the matrix.
  //////////////////////////////////////////////////////////////////////
  copy_to_view(size_t padding)
    : m_padding(padding)
  { }

  ///////////////////////////////////////////////////////////////////
  /// @param element current element of the view.
  /// @param input matrix view where the value is stored
  /// @param output output view where the value is copied
  ///////////////////////////////////////////////////////////////////
  template <typename Element, typename InputView, typename OutputView>
  void operator()(Element&& element, InputView&& input, OutputView&& output)
  {
    dimensions element_idx    = stapl::index_of(element);
    dimensions input_dims     = input.domain().dimensions();

    size_t input_dims_x       = std::get<0>(input_dims);
    size_t element_idx_x      = std::get<0>(element_idx);
    size_t element_idx_y      = std::get<1>(element_idx);
    size_t element_linear_idx = (input_dims_x * element_idx_y) + element_idx_x;

    if (element_linear_idx >= m_padding)
      output.set_element(element_linear_idx - m_padding, element);
  }

  void define_type(typer& t)
  { t.member(m_padding); }
};
} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided using Leighton's column sort algorithm.
///
///  F. T. Leighton. Tight bounds on the complexity of parallel sorting.
///  IEEE Trans. Comput., 34(4):344 354, 1985.
///
///  The algorithm requires definition of a -infinity and +infinity value,
///  and is restricted to views over signed numeric types.
///
/// @param view Input to sort
/// @param compare The strict weak ordering comparison functor
//////////////////////////////////////////////////////////////////////
template <typename View, typename Compare>
typename std::enable_if<std::is_signed<typename View::value_type>::value>::type
column_sort(View& view, Compare compare)
{
  if (view.size() == 1)
    return;

  typedef typename View::value_type T;
  size_t padding = algo_details::calculate_padding(view);

  algo_details::dimensions matrix_dimensions =
    algo_details::calculate_dimensions(view.size() + padding);

  size_t dims_x = std::get<0>(matrix_dimensions);
  size_t dims_y = std::get<1>(matrix_dimensions);

  auto matrix_container = algo_details::create_multiarray<T>(dims_x, dims_y);
  auto matrix           = make_multiarray_view(matrix_container);

  algo_details::column_sort_constants<T, Compare> constants(compare);

  // Step 0: copy the input view into the 2-D matrix container
  stapl::fill(matrix, constants.minus_infinity());

  map_func(algo_details::fill_matrix(padding),
           view, stapl::make_repeat_view(matrix));

  auto temp_container   = algo_details::create_multiarray<T>(dims_x, dims_y);
  auto temp             = make_multiarray_view(temp_container);

  auto shift_container  = algo_details::create_multiarray<T>(dims_x, dims_y+1);
  auto shifted          = make_multiarray_view(shift_container);

  // step 1: sort
  map_func(algo_details::sort_column<Compare>(compare),
           make_slices_view<1>(matrix));

  // step 2: reshape-and-transpose
  map_func(algo_details::transform_matrix(matrix_dimensions,
                                          algo_details::RESHAPE_AND_TRANSPOSE),
    matrix, stapl::make_repeat_view_nd<2>(temp));

  // step 3: sort
  map_func(algo_details::sort_column<Compare>(compare),
    make_slices_view<1>(temp));

  // step 4: transpose-and-reshape
  map_func(algo_details::transform_matrix(matrix_dimensions,
                                          algo_details::TRANSPOSE_AND_RESHAPE),
    temp, stapl::make_repeat_view_nd<2>(matrix));

  // step 5: sort
  map_func(algo_details::sort_column<Compare>(compare),
    make_slices_view<1>(matrix));

  // step 6: shift
  // shift step is divided in two steps: first copy the input matrix
  // into the shifted matrix, then set the edges of the matrix to
  // -inf and +inf, according to the column sort algorithm.
  map_func(algo_details::shift_matrix(matrix.dimensions()),
    matrix, stapl::make_repeat_view_nd<2>(shifted));

  // set the edges of the shifted matrix
  map_func(algo_details::set_edges<T>(matrix.dimensions(),
                                      constants.minus_infinity(),
                                      constants.plus_infinity()),
    shifted);

  // step 7: sort
  map_func(algo_details::sort_column<Compare>(compare),
    make_slices_view<1>(shifted));

  // step 8: unshift
  map_func(algo_details::shift_matrix(shifted.dimensions()),
    shifted, stapl::make_repeat_view_nd<2>(matrix));

  // Step 9: copy the sorted matrix into the output view
  map_func(algo_details::copy_to_view(padding),
    matrix, stapl::make_repeat_view_nd<2>(matrix),
    stapl::make_repeat_view_nd<2>(view));
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided using Leighton's column sort algorithm.
///
/// The comparator @ref less is used to compare the elements.
///
/// @param view Input sort
//////////////////////////////////////////////////////////////////////
template <typename View>
void column_sort(View& view)
{
  column_sort(view, less<typename View::value_type>());
}

} //namespace stapl

#endif
