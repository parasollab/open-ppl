/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_OUTPUT_VIEW_HPP
#define STAPL_VIEWS_OUTPUT_VIEW_HPP

#include <stapl/domains/infinite.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/utility/use_default.hpp>
#include <ostream>

namespace stapl {

namespace view_impl {


//////////////////////////////////////////////////////////////////////
/// @brief Container that reroutes the set_element method to insert
///        into an STL ostream.
/// @tparam T Type of elements that are written to the output stream.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct ostream_container
{
  typedef T                                             value_type;
  typedef value_type                                    reference;
  typedef void                                          iterator;
  typedef infinite                                      domain_type;
  typedef size_t                                        gid_type;
  typedef typename domain_type::size_type               size_type;
  typedef infinite_locality_metadata<ostream_container> loc_dist_metadata;

  typedef std::char_traits<char>                        traits_type;
  typedef std::basic_ostream<char, traits_type>         ostream_type;

  ostream_type*  m_stream;
  const char*    m_string;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that receives an ostream and a string delimiter.
  ///
  /// @param stream Stream object used to redirect the output.
  /// @param string Delimiter used after every write to the output stream..
  //////////////////////////////////////////////////////////////////////
  ostream_container(ostream_type& stream, char const* string)
    : m_stream(&stream), m_string(string)
  { }

  void set_element(gid_type, value_type v) const
  {
    *m_stream << v << m_string;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc repeat_container::size
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return domain_type().size();
  }

  domain_type domain(void) const
  {
    return domain_type();
  }
};


template <typename C,
          typename Dom     = typename container_traits<C>::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>,
          typename Derived = use_default>
class output_view;

//////////////////////////////////////////////////////////////////////
/// @brief A write-only view. Reading from this view is not supported.
///
/// @tparam C Container type.
/// @tparam Dom Domain type.
/// @tparam MapFunc Mapping function type.
/// @tparam Derived Type of the most derived class (default: itself).
///
/// @todo Replace write_operation and begin_end_operation with the
///       respective classes in the view_operation namespace.
//////////////////////////////////////////////////////////////////////
template <typename C,
          typename Dom,
          typename MapFunc,
          typename Derived>
class output_view
  : public core_view<C,Dom,MapFunc>,
    public write_operation<output_view<C,Dom,MapFunc,Derived> >,
    public begin_end_operation<output_view<C,Dom,MapFunc,Derived> >
{
  typedef core_view<C,Dom,MapFunc>             base_type;

public:

  STAPL_VIEW_REFLECT_TRAITS(output_view)

 typedef typename iterator_selector<
   output_view,value_type>::type                             iterator;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  output_view(view_container_type* vcont, domain_type const& dom,
              map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  /// @todo Verify if this constructor is called or is dead code.
  //////////////////////////////////////////////////////////////////////
  output_view(view_container_type& vcont, domain_type const& dom,
              map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*)
  /// @todo Verify if this constructor is called or is dead code.
  //////////////////////////////////////////////////////////////////////
  output_view(view_container_type* vcont)
    : base_type(vcont, Dom(), MapFunc())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type&)
  //////////////////////////////////////////////////////////////////////
  output_view(view_container_type& vcont)
    : base_type(vcont, Dom(), MapFunc())
  { }
};

} // namespace view_impl


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Defines ostream_view type to write elements of type T.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct ostream_view
{
  typedef view_impl::output_view<detail::ostream_container<T> > type;
};

} // result_of namespace


//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates an output stream view on top
///        of the given stream object.
///
/// @tparam T type of elements that are written.
/// @param stream object to redirect the outputs.
/// @param delimiter used to separate each output element
/// @todo this view needs a unit test.
//////////////////////////////////////////////////////////////////////
template<typename T>
typename result_of::ostream_view<T>::type
ostream_view(typename detail::ostream_container<T>::stream_type& stream,
             char const* delimiter)
{
  return view_impl::output_view<detail::ostream_container<T> >(
             new detail::ostream_container<T>(stream, delimiter));
}

} // namespace stapl

#endif /* STAPL_VIEWS_OUTPUT_VIEW_HPP */
