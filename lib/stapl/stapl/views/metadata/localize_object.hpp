/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_LOCALIZE_OBJECT_HPP
#define STAPL_VIEWS_METADATA_LOCALIZE_OBJECT_HPP

#include <utility>
#include <stapl/views/proxy/trivial_accessor.hpp>

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(preferred_location_type)


template <typename View, typename Info, typename CID>
class mix_view;


template <typename T, typename Accessor>
class proxy;


namespace loc_ref_detail {

namespace localize {

//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction that returns static true for all types
///  except @ref mix_view and @ref proxy.
//////////////////////////////////////////////////////////////////////
template<typename Ref>
struct is_static_dontcare
  : std::true_type
{ };


template<typename View, typename Info, typename CID>
struct is_static_dontcare<mix_view<View,Info,CID>>
  : std::false_type
{ };


template<typename T, typename Accessor>
struct is_static_dontcare<proxy<T,Accessor>>
  : std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to query the preferred location information
///        from the given reference.
///
/// Base specialization returning the current location with don't care
/// qualification.
//////////////////////////////////////////////////////////////////////
template <typename Ref>
locality_info locality(Ref const&)
{
  return LQ_DONTCARE;
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to query the preferred location information
///        from the given @ref mix_view.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Info, typename CID>
locality_info locality(mix_view<View,Info,CID>& ref)
{
  return ref.locality();
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to query the preferred location information
///        from the given @ref proxy.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Accessor>
locality_info locality(proxy<T,Accessor> const& ref)
{
  return ref.locality();
}

} // namespace localize


//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that provides the methods for data
///        localization required for the task.
///
/// This class is used to report locality information over literal
/// values.
//////////////////////////////////////////////////////////////////////
template<typename Value>
struct stub_literal_localization
{
  typedef Value                         view_type;
  typedef trivial_accessor<Value>       accessor_type;
  typedef proxy<Value, accessor_type>   subview_type;
  typedef size_t                        cid_type;

  Value m_value;

  stub_literal_localization(Value const& ref)
    : m_value(ref)
  { }

  subview_type get_subview(std::size_t) const
  {
    return subview_type(accessor_type(m_value));
  }

  typedef std::true_type task_placement_dontcare;

  locality_info locality(size_t) const
  {
    return LQ_DONTCARE;
  }

  bool is_local() const
  {
    return true;
  }

  void pre_execute()
  { }

  void post_execute()
  { }

  void define_type(typer& t)
  {
    t.member(m_value);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that provides the methods for data
///        localization required for the task.
//////////////////////////////////////////////////////////////////////
template<typename Ref>
struct stub_localization
{
  typedef Ref       subview_type;
  typedef size_t    cid_type;

  Ref               m_ref;

  typedef typename localize::is_static_dontcare<Ref>::type
    task_placement_dontcare;

  stub_localization(Ref const& ref)
    : m_ref(ref)
  { }

  subview_type get_subview(size_t) const
  {
    return m_ref;
  }

  locality_info locality(size_t const&)
  {
    return localize::locality(m_ref);
  }

  void define_type(typer& t)
  {
    t.member(m_ref);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that provides the same interface as
///        @c std::pair.
///
/// This object is used during task creation to specify the data that
/// is used for the task.
/// @tparam Stub type defines the object responsible to determine the
///               localization of the given reference.
//////////////////////////////////////////////////////////////////////
template<typename Ref, typename Stub>
struct localize_object
{
  typedef Stub       first_type;
  typedef size_t     second_type;

  first_type         m_stub;

  first_type*        first;
  second_type        second;

  localize_object(Ref const& ref)
    : m_stub(ref),
      first(&m_stub),
      second(0)
  { }

  localize_object()
    : first(&m_stub),
      second()
  { }

  localize_object(localize_object const& other)
    : m_stub(other.m_stub),
      first(&m_stub),
      second(other.second)
   { }

  void define_type(typer& t)
  {
    t.member(m_stub);
    t.member(second);
    t.pointer_to_member(first, &m_stub);
  }

private:
  localize_object& operator=(localize_object const& rhs);
};

} // namespace loc_ref_detail


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to determine the localization object based on
///        the reference type.
//////////////////////////////////////////////////////////////////////
template<typename Ref, bool isLiteral = false>
struct localize_object
{
  typedef typename loc_ref_detail::localize_object<
    Ref, loc_ref_detail::stub_localization<Ref>
  > type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to determine the localization object based on
///        the reference type.
///
/// Specialization when the reference is a literal.
//////////////////////////////////////////////////////////////////////
template<typename Ref>
struct localize_object<Ref, true>
{
  typedef typename loc_ref_detail::localize_object<
    Ref, loc_ref_detail::stub_literal_localization<Ref>
  > type;
};

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to return an object that provides the
///        required methods to localize the given reference (@p ref).
///
/// This helper method is used during task specifications and the
/// generated object is queried to determine where the task should be
/// executed.
//////////////////////////////////////////////////////////////////////
template<typename Ref>
typename result_of::localize_object<Ref, false>::type
localize_object(Ref const& ref)
{
  typedef loc_ref_detail::stub_localization<Ref> stub_t;

  return loc_ref_detail::localize_object<Ref, stub_t>(ref);
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialized helper function to return an object that
///        provides the required methods to localize the given
///        reference (@p ref) is a literal.
///
/// This helper method is used during task specifications and the
/// generated object is queried to determine where the task should be
/// executed.
//////////////////////////////////////////////////////////////////////
template<bool isLiteral, typename Ref>
typename result_of::localize_object<Ref, isLiteral>::type
localize_object(Ref const& ref)
{
  return typename result_of::localize_object<Ref, isLiteral>::type(ref);
}

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_LOCALIZE_OBJECT_HPP
