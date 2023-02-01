/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_LOCALIZE_ELEMENT_HPP
#define STAPL_LOCALIZE_ELEMENT_HPP

#include <utility>
#include <stapl/views/type_traits/is_native_partition.hpp>
#include <boost/mpl/if.hpp>

//////////////////////////////////////////////////////////////////////
/// @file localize_element.hpp
/// @todo Verify which stubs need to be merged with the ones provided
///       in localize_object.hpp.
//////////////////////////////////////////////////////////////////////

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that provides the methods for data
///        localization required for the task.
//////////////////////////////////////////////////////////////////////
template<typename View, typename GID = typename View::gid_type>
struct stub_localize
{
  typedef View                             view_type;
  typedef typename view_type::reference    subview_type;
  typedef GID                              cid_type;

  subview_type*                            m_sview;
  cid_type                                 m_cid;
  View                                     m_view;

  stub_localize(View const& view, cid_type cid)
    : m_sview(0), m_cid(cid), m_view(view)
  { }

  stub_localize(stub_localize&&)      = default;
  stub_localize(stub_localize const&) = default;

  subview_type get_subview(GID cid) const
  {
    if (m_sview != 0)
      return *m_sview;

    // else
    return m_view[cid];
  }

  locality_info locality(size_t)
  {
    return m_view.container().locality(m_cid);
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_cid);
    t.member(m_view);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines a view that behaves as the given @p View and
///        provides the methods for data localization required for the
///        task.
//////////////////////////////////////////////////////////////////////
template<typename View, typename GID>
struct stub_self_element
  : public View
{
  typedef View              view_type;
  typedef stub_self_element subview_type;
  typedef GID               cid_type;

  stub_self_element(View& view, GID)
    : View(view)
  { }

  stub_self_element& get_subview(GID) const
  {
    return const_cast<stub_self_element&>(*this);
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

  void define_type(stapl::typer &t)
  {
    stapl_assert(false, "this should not be shipped");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that provides the methods for data
///        localization required for the task.
///
/// This class is used to report locality information over literal
/// values.
//////////////////////////////////////////////////////////////////////
template<typename View, typename GID>
struct stub_dontcare
{
  typedef View      view_type;
  typedef View      subview_type;
  typedef GID       cid_type;

  View             m_view;

  stub_dontcare(View const& view, GID)
    : m_view(view)
  { }

  subview_type get_subview(GID) const
  {
    return m_view;
  }

  typedef std::true_type task_placement_dontcare;

  constexpr locality_info locality(size_t) const
  {
    return LQ_DONTCARE;
  }

  rmi_handle::reference nested_locality(size_t)
  {
    return m_view.container().get_rmi_handle();
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_view);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that provides the methods for data
///        localization required for the task.
///
/// This class is used to report locality information over subviews
/// created for a native partitioned view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename GID=typename View::gid_type>
struct stub_localize_nat_view
{
  typedef View                          view_type;
  typedef typename View::value_type     subview_type;
  typedef GID                           cid_type;

  cid_type        m_cid;
  View            m_view;

  stub_localize_nat_view(View const& view, cid_type cid)
    : m_cid(cid), m_view(view)
  { }

  subview_type get_subview(GID cid) const
  {
    return m_view[cid];
  }

  locality_info locality(size_t)
  {
    return LQ_CERTAIN;
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_cid);
    t.member(m_view);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that provides the same interface as
///        @c std::pair.
///
/// This object is used during task creation to specify the data that
/// is used for the task.
/// @tparam Stub Defines the object responsible to determine the
///              localization of the given reference.
//////////////////////////////////////////////////////////////////////
template<typename View,typename GID, typename Stub>
struct localize_element
{
  typedef Stub       view_type;

  typedef view_type  first_type;
  typedef GID        second_type;

  first_type         m_stub;
  first_type*        first;
  second_type        second;

  localize_element(View const& view, const GID& gid)
    : m_stub(view,gid), first(&m_stub), second(gid)
  { }

  localize_element(View& view, const GID& gid)
    : m_stub(view,gid), first(&m_stub), second(gid)
  { }

  localize_element(void) = default;
  localize_element(localize_element const&) = default;
  localize_element(localize_element&&) = default;

  void define_type(stapl::typer &t)
  {
    t.member(first);
    t.member(second);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct the localization object over
///        the element referenced by the index @p i in the given view
///        @p vw.
//////////////////////////////////////////////////////////////////////
template<typename View, typename GID>
localize_element<View,GID,stub_localize<View,GID> >
localize_ref(View const& vw, GID const& i)
{
  return localize_element<View,GID,stub_localize<View,GID> >(vw,i);
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct the localization object over
///        the view @p vw.
//////////////////////////////////////////////////////////////////////
template<typename View>
localize_element<View,size_t,stub_dontcare<View,size_t> >
localize_ref(View const& vw)
{
  return localize_element<View, size_t, stub_dontcare<View,size_t> >(vw,0);
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a view extended with
///        localization information.
//////////////////////////////////////////////////////////////////////
template<typename View>
localize_element<View,size_t,stub_self_element<View,size_t> >
localize_self_element(View& vw)
{
  return localize_element<View,size_t,stub_self_element<View,size_t> >(vw,0);
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper to determine which stub to use based on the type of
///        partition @p PS.
//////////////////////////////////////////////////////////////////////
template <typename View, typename GID,typename PS>
struct select_stub
{
  typedef typename boost::mpl::if_<
    typename is_native_partition<PS>::type,
    stub_localize_nat_view<View,GID>,
    stub_localize<View,GID>
  >::type type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to extract the partition type used for the given
///        partitioned view  (Base case).
//////////////////////////////////////////////////////////////////////
template<typename View>
struct get_partitioner
{
  typedef void type;
};

template <typename C, typename PS, typename MFG, typename SV>
class segmented_view;

template <typename View>
class overlap_view;

//////////////////////////////////////////////////////////////////////
/// @brief Helper to extract the partition type used for the given
///        @ref segmented_view.
//////////////////////////////////////////////////////////////////////
template <typename C, typename PS, typename MFG, typename SVC>
struct get_partitioner<segmented_view<C, PS, MFG, SVC> >
{
  typedef PS type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to extract the partition type used for the given
///        @ref overlap_view.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct get_partitioner<overlap_view<View> >
{
  typedef typename overlap_view<View>::partition_type type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct the localization object when
///        the given view is a @ref segmented_view.
//////////////////////////////////////////////////////////////////////
template <typename C, typename PS, typename MFG, typename SVC, typename GID>
localize_element<
  segmented_view<C, PS, MFG, SVC>, GID,
  typename select_stub<segmented_view<C, PS, MFG, SVC>, GID,
                       PS>::type>
localize_ref(segmented_view<C, PS, MFG, SVC> const& vw,
             GID const& i)
{
  typedef typename select_stub<
    segmented_view<C, PS, MFG, SVC>, GID, PS
  >::type stub_t;

return localize_element<segmented_view<C,PS,MFG,SVC>,
                          GID,
                          stub_t>(vw,i);
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct the localization object when
///        the given view is an @ref overlap_view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename GID>
localize_element<
  overlap_view<View>,
  GID,
  typename select_stub<overlap_view<View>,GID,
    typename overlap_view<View>::partition_type>::type
>
localize_ref(overlap_view<View> const& vw, GID const& i)
{
  typedef typename select_stub<
    overlap_view<View>, GID, typename overlap_view<View>::partition_type
  >::type stub_t;

return localize_element<overlap_view<View>,
                          GID,
                          stub_t>(vw,i);
}

} // namespace stapl

#endif // STAPL_LOCALIZE_ELEMENT_HPP
