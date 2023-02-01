/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_OVERLAP_VIEW_HPP
#define STAPL_VIEWS_OVERLAP_VIEW_HPP

#include <stapl/views/segmented_view.hpp>
#include <stapl/containers/partitions/overlap.hpp>

#include <stapl/views/common_view.hpp>

#include <iostream>

namespace stapl {

template <typename View>
class overlap_view;


template <typename View>
struct view_traits<overlap_view<View>>
{
private:
  using base_type =
    segmented_view<View, overlap_partition<typename View::domain_type>>;

  using base_traits_type = view_traits<base_type>;

public:
  STAPL_IMPORT_TYPE(typename base_traits_type, container)
  STAPL_IMPORT_TYPE(typename base_traits_type, map_fun_gen_t)
  STAPL_IMPORT_TYPE(typename base_traits_type, composed_mf_t)
  STAPL_IMPORT_TYPE(typename base_traits_type, subview_type)
  STAPL_IMPORT_TYPE(typename base_traits_type, value_type)
  STAPL_IMPORT_TYPE(typename base_traits_type, reference)
  STAPL_IMPORT_TYPE(typename base_traits_type, map_function)
  STAPL_IMPORT_TYPE(typename base_traits_type, index_type)
  STAPL_IMPORT_TYPE(typename base_traits_type, domain_type)
};


//////////////////////////////////////////////////////////////////////
/// @brief An overlap view is a segmented view whose segments overlap
/// by some number of elements in each dimension as specified by the user.
///
/// The overlap domains are defined specifying the number of elements
/// that are overlapped to the left (@c l), the number of elements that
/// are not overlapped (@c c) and the number of elements overlapped to the
/// right (@c r). Each subdomain has size: l+c+r.
/// @par Example:
///     Domain to partition: [0..8]<br/>
///     left overlap (l): 2<br/>
///     non overlap (c): 3<br/>
///     right overlap (r): 1<br/>
///     Resulting partition: {[0..5],[3..8]}<br/>
//////////////////////////////////////////////////////////////////////
template<typename View>
class overlap_view
  : public segmented_view<View, overlap_partition<typename View::domain_type>>
{
private:
  using base_type =
    segmented_view<View, overlap_partition<typename View::domain_type>>;

  using overlap_storage_t = std::vector<typename View::value_type>;

public:
  STAPL_IMPORT_TYPE(typename base_type, view_type)
  STAPL_IMPORT_TYPE(typename base_type, value_type)
  STAPL_IMPORT_TYPE(typename base_type, view_container_type)
  STAPL_IMPORT_TYPE(typename base_type, domain_type)
  STAPL_IMPORT_TYPE(typename base_type, map_func_type)
  STAPL_IMPORT_TYPE(typename base_type, map_function)
  STAPL_IMPORT_TYPE(typename base_type, partition_type)
  STAPL_IMPORT_TYPE(typename base_type, index_type)
  STAPL_IMPORT_TYPE(typename base_type, dimension_type)

  using map_fun_gen_t = typename view_traits<overlap_view>::map_fun_gen_t;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an overlap_view over the view @c v using
  ///        the partition @c part.
  ///
  /// @param v View to partition into overlapping partitions
  /// @param part partitioner describing the overlap partition to be used
  /// @param mfg mapping function generator
  //////////////////////////////////////////////////////////////////////
  overlap_view(View const& v, partition_type const& part,
               map_fun_gen_t const& mfg = map_fun_gen_t())
    : base_type(v, part, mfg)
   { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an overlap_view from the components of an existing
  /// overlap_view.
  ///
  /// This constructor is invoked during construction of a @ref mix_view
  /// over the overlap view during view coarsening.
  ///
  /// @param c Container of partitioned view components
  /// @param dom Domain of the partitions
  /// @param mf Mapping function generated for the view
  //////////////////////////////////////////////////////////////////////
  overlap_view(view_container_type const& c, domain_type const& dom,
               map_func_type const& mf)
    : base_type(c, dom, mf)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an overlap_view from the components of an existing
  /// overlap_view.
  ///
  /// This constructor is invoked during construction of a @ref mix_view
  /// over the overlap view during view coarsening.
  ///
  /// @param c Container of partitioned view components
  /// @param dom Domain of the partitions
  /// @param mf Mapping function generated for the view
  /// @param other overlap_view instance from which state can be copied
  //////////////////////////////////////////////////////////////////////
  overlap_view(view_container_type const& c, domain_type const& dom,
               map_func_type const& mf, overlap_view const& other)
    : base_type(c, dom, mf, other)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc segmented_view(Other const& other,Container* view)
  //////////////////////////////////////////////////////////////////////
  template <typename Other>
  overlap_view(Other const& other, View* view)
    : base_type(other, view)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  { t.base<base_type>(*this); }
}; // class overlap_view


namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to construct a segmented view using an overlap
///        partitioner.
/// @ingroup overlap_view
//////////////////////////////////////////////////////////////////////
template<typename View>
class overlap_view_builder
{
private:
  using domain_type = typename View::domain_type;
  using pover_type  = overlap_partition<domain_type>;

public:
  using view_type   = overlap_view<View>;

  //////////////////////////////////////////////////////////////////////
  /// @see stapl::overlap_view
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View& v, size_t c = 1, size_t l = 0, size_t r = 0)
  {
    return view_type(
      v,
      pover_type(
        domain_type(v.domain().first(), v.domain().last(), v.domain()),
        c, l, r
      )
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @see stapl::overlap_view
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View const& v, size_t c=1, size_t l=0, size_t r=0)
  {
    return view_type(v, pover_type(domain_type(v.domain().first(),
                                               v.domain().last(),
                                               v.domain()),
                                   c, l, r) );
  }
}; // class overlap_view_builder

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of localized @ref overlap_view which
/// asynchronously fetches elements on other locations, as long
/// as the total size of the view is below a specified threshold.
//////////////////////////////////////////////////////////////////////
template<typename Container, typename Info, typename CID>
struct localized_view<mix_view<overlap_view<array_view<Container>>, Info, CID>>
  : public cast_container_view<
      typename mix_view<
        overlap_view<array_view<Container>>, Info, CID>::base_type,
      typename mix_view<
        overlap_view<array_view<Container>>, Info, CID>::component_type
    >::type
{
private:
  using mix_view_t = mix_view<overlap_view<array_view<Container>>, Info, CID>;

  using base_t = typename cast_container_view<
    typename mix_view_t::base_type,
    typename mix_view_t::component_type
  >::type;

  using component_t = typename mix_view_t::component_type;
  using dom_t       = typename component_t::domain_type;
  using View1       = overlap_view<array_view<Container>>;
  using value_t     = typename array_view<Container>::value_type;
  using async_t     = tuple<size_t, std::function<void (void)>>;

  std::shared_ptr<async_t> m_waiter;

  static dom_t total_domain(mix_view_t const& v)
  {
    return dom_t(
      v.container().partition().compute_first(v.domain().first()),
      v.container().partition().compute_last(v.domain().last()));
  }

  static bool already_local(mix_view_t const& v)
  {
    return view_impl::get_container_location(static_cast<View1 const*>(&v))
             == v.m_md_info.location()
           && v.m_md_info.component() != nullptr
           && get_affinity() == v.m_md_info.affinity();
  }

public:
  /// @brief Tells view coarsening / localization process (i.e., @ref mix_view)
  /// that this view's localization has a custom check for localizability.
  using custom_is_local_check = std::true_type;

  /// @brief Tells paragraph framework that this view's localization may not
  /// be complete after the invocation of the constructor below. Paragraph
  /// will provide a callback for view to asynchronously inform it that
  /// localization is complete.
  using deferred_localizable  = std::true_type;


  //////////////////////////////////////////////////////////////////////
  /// @brief Custom check which checks standard localization condition and
  /// then threshold of size and type to determine if localization should
  /// be forced through remote element fetching into temporary storage.
  //////////////////////////////////////////////////////////////////////
  static bool is_local(mix_view_t const& v)
  {
    if (is_nested_container<typename View1::value_type>::value)
      return false;

    if (already_local(v))
      return true;

    // Check threshold
    return std::is_fundamental<value_t>::value && total_domain(v).size() < 1000;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor for this localized view which initialized base
  /// class with either a view based on the component (base container)
  /// type of the underlying container or a temporary buffer of the base
  /// container type with elements that are fetched on demand.
  ///
  /// @brief v The view to be localized.
  /// @brief callback Functor provided by @ref paragraph framework which
  /// should be invoked when the deferred initialization of this localized
  /// view is completed.
  //////////////////////////////////////////////////////////////////////
  localized_view(mix_view_t const& v, std::function<void (void)> callback)
    : base_t(
        already_local(v) ?
        detail::fast_view_builder<base_t>::create(v, v.get_component())
        : detail::fast_view_builder<base_t>::create(
            v, new component_t(total_domain(v)))),
      m_waiter(std::make_shared<async_t>(0, std::move(callback)))
  {
    // If this view was already natively localizable (i.e., all elements reside
    // on this location, invoke the paragraph callback and return.
    if (already_local(v))
    {
      get<1>(*m_waiter)();
      return;
    }

    // Proceed with deferred localization.  We're below the threshold,
    // otherwise this constructor would not have been called.
    const auto domain = total_domain(v);
    const auto last   = domain.last();

    // Set counter, as promises fulfilled, future callbacks will tick it
    // down, until it reaches zero, when the paragraph callback will be invoked.
    get<0>(*m_waiter) += domain.size();
    async_t* p         = m_waiter.get();
    component_t* ptr   = &this->container().view().container();

    // Iterate over domain, invoking an asynchronous fetch for each element.
    // Hook the future returned for each fetch into lambda which decrements
    // counter.  Last invocation calls paragraph callback.
    //
    // Note this is an O(n) loop over the domain of this overlap view, but
    // this is bounded to relatively small views in the threshold condition
    // in is_local().  Typical use of this optimization post data coarsening
    // will mean the domain is of size left_overlap + right_overlap + 1.
    for (auto idx = domain.first(); idx <= last ; idx = domain.advance(idx, 1))
    {
      v.container().view().container().get_element_split(idx).async_then(
        [p, idx, ptr](future<value_t> fut)
        {
          stapl_assert(get<0>(*p) > 0, "overlap view future counter is zero");

          ptr->operator[](idx) = fut.get();

          if (--get<0>(*p) == 0)
            get<1>(*p)();
        }
      );
    }
  }
}; // struct localized_view<mix_view<overlap_view<View>, Info, CID>>


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct an overlap segmented view.
///
/// The overlap domains are defined specifying the number of elements
/// that are overlap to the left (@c l), the number of elements that
/// are not overlapped (@c c) and the number of elements overlap to the
/// right (@c r). Each subdomain has size: l+c+r.
/// @par Example:
///     Domain to partition: [0..8]<br/>
///     left overlap (l): 2<br/>
///     non overlap (c): 3<br/>
///     right overlap (r): 1<br/>
///     Resulting partition: {[0..5],[3..8]}<br/>
/// @param view View to partition.
/// @param c Number of elements not overlapped.
/// @param l Number of elements overlapped to the left.
/// @param r Number of elements overlapped to the right.
/// @return An overlap segmented view.
//////////////////////////////////////////////////////////////////////
template<typename View>
overlap_view<View>
make_overlap_view(View const& view, size_t c, size_t l = 0, size_t r = 0)
{
  return view_impl::overlap_view_builder<View>()(view, c, l, r);
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for changing the container used for an
///        overlap segmented view.
//////////////////////////////////////////////////////////////////////
template<typename View,
         typename NewC >
struct cast_container_view<overlap_view<View>, NewC>
{
  using part_view_t      = overlap_view<View>;
  using part_container_t = typename part_view_t::view_container_type;

  using type = overlap_view<typename cast_container_view<View, NewC>::type>;
};

} // namespace stapl

#endif // STAPL_VIEWS_OVERLAP_VIEW_HPP
