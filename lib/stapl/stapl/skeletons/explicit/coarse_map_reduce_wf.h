/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXPLICIT_COARSE_MAP_REDUCE_WF_H
#define STAPL_SKELETONS_EXPLICIT_COARSE_MAP_REDUCE_WF_H

#include <stapl/skeletons/utility/wf_iter_compare.hpp>
#include <stapl/skeletons/transformations/optimizers/utils.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Work function that uses the fine-grain map and reduce operations
/// provided to @ref stapl::map_reduce to implement a coarse-grain operation
/// that aggregates multiple calls with a for loop into a single result.
///
/// @tparam Map Type of the fine-grain map operation.
/// @tparam Reduce Type of the reduce operation.
///
/// @internal Multiple inheritance of empty bases which share a type in their
/// class hierarchy will cause the minimum size of derived to be > 1 as the
/// standard says that same type subobjects cannot have the same address.  In
/// practice, this functor is a base of Task<...> which is plenty big enough to
/// hide this.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Map, typename Reduce>
struct cmr_base
  : private Map,
    private paragraph_impl::identity_wf<Reduce>
{
  cmr_base(Map const& fmap, Reduce const& freduce)
    : Map(fmap),
      paragraph_impl::identity_wf<Reduce>(freduce)
  { }

  void define_type(typer &t)
  {
    t.base<Map>(*this);
    t.base<paragraph_impl::identity_wf<Reduce> >(*this);
  }

  template <typename R, typename IterComp, typename ...Iter>
  R apply(IterComp& iter_compare, Iter... iter)
  {
    using namespace skeletons::optimizers;
    Map&    map_op = static_cast<Map&>(*this);
    Reduce& red_op =
      static_cast<paragraph_impl::identity_wf<Reduce>&>(*this).wf();
    R result = map_op(*(iter++)...);
    for (; iter_compare(iter...); helpers::no_op(++iter...)) {
      helpers::reduce_assign(result, red_op(result, map_op(*(iter)...)));
    }
    return result;
  }

  template <typename R, typename Scheduler, typename IterComp, typename ...Iter>
  R apply(paragraph_impl::paragraph_view<Scheduler>& tgv,
          IterComp& iter_compare, Iter... iter)
  {
    using namespace skeletons::optimizers;
    Map&    map_op = static_cast<Map&>(*this);
    Reduce& red_op =
      static_cast<paragraph_impl::identity_wf<Reduce>&>(*this).wf();
    R result = map_op(tgv, *(iter++)...);
    for (; iter_compare(iter...); helpers::no_op(++iter...)) {
      helpers::reduce_assign(result, red_op(result, map_op(tgv, *(iter)...)));
    }
    return result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator signatures for different number of views.
  ///   The map operation is applied to sets of elements, one from each view,
  ///   where the elements of a set all have the same offset from the start of
  ///   their respective view.
  //////////////////////////////////////////////////////////////////////
  template <typename V0, typename... V>
  typename boost::result_of<
    Reduce(
      typename boost::result_of<Map(typename V0::reference,
                                    typename V::reference...)>::type,
      typename boost::result_of<Map(typename V0::reference,
                                    typename V::reference...)>::type
    )
  >::type
  operator()(V0& view0, V&... view)
  {
    stapl_assert(view0.size() > 0, "empty views cannot be zip-reduced");
    typedef typename
      boost::result_of<
        Reduce(
          typename boost::result_of<Map(typename V0::reference,
                                        typename V::reference...)>::type,
          typename boost::result_of<Map(typename V0::reference,
                                        typename V::reference...)>::type
        )
      >::type result_type;

    wf_iter_compare<V0, V...> iter_compare(view0, view...);
    return apply<result_type>(iter_compare, view0.begin(), view.begin()...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator signatures for different number of views.
  ///   The map operation is applied to sets of elements, one from each view,
  ///   where the elements of a set all have the same offset from the start of
  ///   their respective view.
  //////////////////////////////////////////////////////////////////////
  template <typename Scheduler, typename V0, typename... V>
  typename boost::result_of<
    Reduce(
      typename boost::result_of<
        Map(paragraph_impl::paragraph_view<Scheduler>,
            typename V0::reference, typename V::reference...)>::type,
      typename boost::result_of<
        Map(paragraph_impl::paragraph_view<Scheduler>,
            typename V0::reference, typename V::reference...)>::type
    )
  >::type
  operator()(paragraph_impl::paragraph_view<Scheduler>& tgv,
             V0& view0, V&... view)
  {
    typedef typename
    boost::result_of<
      Reduce(
        typename boost::result_of<
          Map(paragraph_impl::paragraph_view<Scheduler>,
              typename V0::reference, typename V::reference...)>::type,
        typename boost::result_of<
          Map(paragraph_impl::paragraph_view<Scheduler>,
              typename V0::reference, typename V::reference...)>::type
      )
    >::type result_type;

    stapl_assert(view0.size() > 0, "empty views cannot be zip-reduced");
    wf_iter_compare<V0, V...> iter_compare(view0, view...);
    return apply<result_type>(tgv, iter_compare,
                              view0.begin(), view.begin()...);
  }
}; // class cmr_base


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of the coarsened map-reduce operation for
/// @ref logical_and as the reduce operation.  If the fine-grain map operation
/// returns false the coarsened operation returns false immediately without
/// applying the map operation to the remaining sets of elements from the
/// views.
///
/// @tparam Map Type of the fine-grain map operation.
///
/// @internal Multiple inheritance of empty bases which share a type in their
/// class hierarchy will cause the minimum size of derived to be > 1 as the
/// standard says that same type subobjects cannot have the same address.  In
/// practice, this functor is a base of Task<...> which is plenty big enough to
/// hide this.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Map>
struct cmr_base<Map, ::stapl::logical_and<bool> >
  : private Map,
    private paragraph_impl::identity_wf< ::stapl::logical_and<bool> >
{
  cmr_base(Map const& fmap, ::stapl::logical_and<bool> const& freduce)
    : Map(fmap),
      paragraph_impl::identity_wf< ::stapl::logical_and<bool> >(freduce)
  { }

  void define_type(typer &t)
  {
    t.base<Map>(*this);
    t.base<paragraph_impl::identity_wf< ::stapl::logical_and<bool> > >(*this);
  }

  template <typename IterComp, typename ...Iter>
  bool apply(IterComp& iter_compare, Iter... iter)
  {
    using namespace skeletons::optimizers;
    Map&    map_op = static_cast<Map&>(*this);
    ::stapl::logical_and<bool>& red_op =
      static_cast<
        paragraph_impl::identity_wf< ::stapl::logical_and<bool> >&
      >(*this).wf();

    bool result = map_op(*(iter++)...);
    if (!result)
      return result;

    for (; iter_compare(iter...); helpers::no_op(++iter...)) {
      helpers::reduce_assign(result, red_op(result, map_op(*(iter)...)));
      if (!result)
        return result;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator signatures for different number of views.
  /// The map operation is applied to sets of elements, one from each view,
  /// where the elements of a set all have the same offset from the start of
  /// their respective view.
  //////////////////////////////////////////////////////////////////////
  template<typename V0, typename... V>
  bool operator()(V0& view0, V&... view)
  {
    stapl_assert(view0.size() > 0, "empty views cannot be zip-reduced");
    wf_iter_compare<V0, V...> iter_compare(view0, view...);
    return apply(iter_compare, view0.begin(), view.begin()...);
  }

}; // class cmr_base<Map, logical_and<bool> >


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of the coarsened map-reduce operation for
/// @ref stapl::logical_or as the reduce operation.  If the fine-grain
/// map operation returns true the coarsened operation returns true
/// immediately without applying the map operation to the remaining
/// sets of elements from the views.
///
/// @tparam Map Type of the fine-grain map operation.
///
/// @internal Multiple inheritance of empty bases which share a type in their
/// class hierarchy will cause the minimum size of derived to be > 1 as the
/// standard says that same type subobjects cannot have the same address.  In
/// practice, this functor is a base of Task<...> which is plenty big enough to
/// hide this.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Map>
struct cmr_base<Map, ::stapl::logical_or<bool> >
  : private Map,
    private paragraph_impl::identity_wf< ::stapl::logical_or<bool> >
{
  cmr_base(Map const& fmap, ::stapl::logical_or<bool> const& freduce)
    : Map(fmap),
      paragraph_impl::identity_wf< ::stapl::logical_or<bool> >(freduce)
  { }

  void define_type(typer &t)
  {
    t.base<Map>(*this);
    t.base<paragraph_impl::identity_wf< ::stapl::logical_or<bool> > >(*this);
  }

  template <typename IterComp, typename ...Iter>
  bool apply(IterComp& iter_compare, Iter... iter)
  {
    using namespace skeletons::optimizers;
    Map&    map_op = static_cast<Map&>(*this);
    ::stapl::logical_and<bool>& red_op =
      static_cast<
        paragraph_impl::identity_wf< ::stapl::logical_and<bool> >&
      >(*this).wf();

    bool result = map_op(*(iter++)...);
    if (result)
      return result;

    for (; iter_compare(iter...); helpers::no_op(++iter...)) {
      helpers::reduce_assign(result, red_op(result, map_op(*(iter)...)));
      if (result)
        return result;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator signatures for different number of views.
  /// The map operation is applied to sets of elements, one from each view,
  /// where the elements of a set all have the same offset from the start of
  /// their respective view.
  //////////////////////////////////////////////////////////////////////
  template<typename V0, typename... V>
  bool operator()(V0& view0, V&... view)
  {
    stapl_assert(view0.size() > 0, "empty views cannot be zip-reduced");
    wf_iter_compare<V0, V...> iter_compare(view0, view...);
    return apply(iter_compare, view0.begin(), view.begin()...);
  }
}; // class cmr_base<Map, logical_or<bool> >


//////////////////////////////////////////////////////////////////////
/// @brief Work function that implements the coarsened map reduce operation and
/// reflects the result_type defined by the fine-grain reduce operation.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Map, typename Reduce,
         bool R = paragraph_impl::has_result_type<Reduce>::value>
struct coarse_map_reduce_wf
  : public cmr_base<Map, Reduce>
{
public:
  typedef typename Reduce::result_type result_type;

  coarse_map_reduce_wf(Map const& fmap, Reduce const& freduce)
    : cmr_base<Map, Reduce>(fmap, freduce)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that implements the coarsened map reduce operation and
/// uses the struct result to determine the result_type because the fine-grain
/// reduce operation does not reflect a result_type.
///
/// @todo Enable struct result that accepts the parameter list and remove the
/// use of Boost.PP.  See gForge to-do 1015.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename Map, typename Reduce>
struct coarse_map_reduce_wf<Map, Reduce, false>
  : public cmr_base<Map, Reduce>
{
public:
  template<typename Signature>
  struct result;

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the type of result of the operation for the given view
  /// types.
  //////////////////////////////////////////////////////////////////////
  template<typename... V>
  struct result<coarse_map_reduce_wf(V...)>
  {
  private:
    typedef typename
      boost::result_of<Map(typename V::reference...)>::type map_ret_t;
  public:
    typedef typename
      boost::result_of<Reduce(map_ret_t, map_ret_t)>::type  type;
  };

  coarse_map_reduce_wf(Map const& fmap, Reduce const& freduce)
    : cmr_base<Map, Reduce>(fmap, freduce)
  { }
};

} // namespace stapl
#endif
