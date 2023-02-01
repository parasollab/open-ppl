/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_NATIVE_VIEW_HPP
#define STAPL_VIEWS_NATIVE_VIEW_HPP

#include <stapl/views/metadata/extract.hpp>
#include <stapl/views/segmented_view.hpp>
#include <stapl/views/common_view.hpp>

#include <iostream>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Represent a collection of domains that matches how the data
///        in distributed on the underlying container associated with
///        the given view.
///
/// @tparam View given to create the partition.
//////////////////////////////////////////////////////////////////////
template <typename View>
struct native_partition
{
  typedef indexed_domain<size_t>                            domain_type;

private:
  using native_part_type = typename std::remove_pointer<
    typename metadata::extract_metadata<View>::return_type::second_type>::type;

  typedef typename native_part_type::value_type             metadata_entry_type;

  typedef metadata::growable_container<metadata_entry_type> partition_t;
  typedef metadata::view_wrapper<
    metadata_entry_type,
    typename partition_t::domain_type,
    metadata_traits<partition_t>::is_isomorphic::value
  >                                                         partition_type;

private:
  View                m_view;
  partition_type      m_part;

public:
  typedef domain_type::index_type                              index_type;
  typedef typename native_part_type::value_type::domain_type   value_type;
  typedef typename value_type::index_type                      gid_type;

  void define_type(typer& t)
  {
    t.member(m_view);
    t.member(m_part);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that uses the underlying view's container
  ///        distribution information to generate the domain partition
  //////////////////////////////////////////////////////////////////////
  native_partition(View const& view)
    : m_view(view)
    , m_part(metadata::extract_metadata<View const>()(&view).second)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain indexed by @c index.
  ///
  /// All the indexes contained in the returned domain reference
  /// elements that are in the same location.
  /// @param index of the domain to return
  /// @return the domain associated with @c index
  //////////////////////////////////////////////////////////////////////
  value_type operator[](size_t index) const
  {
    if (m_view.domain().is_same_container_domain())
      return m_part[index].domain();
    // Intersection to determine the domain to return
    typename View::domain_type vdom = m_view.domain();
    typename View::domain_type pdom = m_part[index].domain();
    typename View::domain_type res_dom = vdom & pdom;
    if (!res_dom.empty()) {
      return res_dom;
    }
    return typename View::domain_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location partition information extracted from
  ///        the stored view
  //////////////////////////////////////////////////////////////////////
  native_part_type& location_partition() const
  {
    return m_part;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return m_part.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the domain associated with the stored view
  //////////////////////////////////////////////////////////////////////
  typename View::domain_type global_domain() const
  {
    return m_view.domain();
  }

private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g
  ///
  /// @param g gid to find
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type g)
  {
    return m_view.get_container()->distribution().container_manager().within(g);
  }

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Determine which partition has the elements referenced
  ///        for the given domain.
  ///
  /// The returned information is a collection (possibly empty) of
  /// pairs. Each pair contains information about which partitions are
  /// included in the given domain and how they are included (True: if
  /// is fully contained, False: if it is partially included). The
  /// returned collection only has elements if there is at least one
  /// partition that contains elements on the given domain.
  ///
  /// @par Example:
  ///    Partition: [0..3],[4..6],[7..9],[10..13]<br/>
  ///    Given domain: [2..9]<br/>
  ///    Returns:  {([0..0],False),([1..2],True)}<br/>
  ///
  /// @param dom Domain to compare
  /// @param mfg Mapping function generator used to get the associated
  ///            mapping function to each partition. The generated
  ///            mapping function is used to project generated
  ///            partitioned domains into the given domain.
  /// @return a vector of pairs.
  /// @todo This method makes O(p) sync calls.  It also does not distinguish
  /// between partially contained and completely contained domains.  Rewrite it.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(ODom const& dom, MFG const& mfg)
  {
    std::vector<std::pair<domain_type,bool> > doms;
    bool started_range = false;
// this is part of the code guarded below.
#if 0
    bool range_qualifier = false;
#endif
    index_type first(index_bounds<index_type>::invalid());
    index_type last(index_bounds<index_type>::invalid());

    index_type part_size = m_part.size();

    // check the local partition first if the number of partitions equals the
    // number of locations.
    if (m_view.get_num_locations() == part_size)
    {
      index_type i = m_view.get_location_id();
      value_type pdom = m_part[i].domain();
      value_type res  = pdom & dom;

      // The local partition completely contains the domain
      if (!res.empty() && pdom.size() == res.size())
      {
        doms.push_back(std::make_pair(domain_type(i,i),true));
        return doms;
      }
    }

    // else, go through all partitions
    for (index_type i = 0; i != part_size; ++i)
    {
      // This is a synchronous call.
      value_type pdom = m_part[i].domain();
      value_type res  = pdom & dom;
      if (!res.empty())
      {
        if (!started_range) {
          first = i;
          started_range = true;
          if (i == part_size - 1) {
            // The last partition is the first non-empty.
            last = i;
            started_range = false;
            doms.push_back(std::make_pair(domain_type(first,last),true));
          }
        }
// this code will be valid when operator== is provided for domains.
#if 0
        if (!started_range) {
          started_range = true;
          if (res == pdom)
            range_qualifier = true;
          first = i;
        }
        else {
          // switching from partial to full contain.
          if ((res == pdom) && !range_qualifier)
          {
            last = i-1;
            if (last < first)
              last = first;
            doms.push_back(std::make_pair(domain_type(first,last),false));
            range_qualifier = true;
            first = i;
            last = 0;
          }
          else if ((res == pdom) && range_qualifier)
          {
            //continuing the fully contained section.
          }
          else if ((res != pdom) && range_qualifier)
          {
            //finished the fully contained section.
            last = i - 1;
            if (last < first)
              last = first;
            doms.push_back(std::make_pair(domain_type(first,last),true));
            range_qualifier = false;
            first = i;
            last = 0;
          }
          else // res != pdom && !range_qualifier
          {
            //finished the set of non-empty intersections
            last = i;
            doms.push_back(std::make_pair(domain_type(first,last),false));
            return doms;
          }
        }
#endif
      } else {
        if (started_range)
        {
          last = i-1;
          if (last < first)
            last = first;
          doms.push_back(std::make_pair(domain_type(first,last),true));
          started_range = false;
        }
      }
    }
    if (started_range)
    {
      last = part_size-1;
      doms.push_back(std::make_pair(domain_type(first,last),true));
    }
    return doms;
  }
}; // struct native_partition


//////////////////////////////////////////////////////////////////////
/// @brief Static functor to construct a native view using the native
///        partition
/// @ingroup native_view
//////////////////////////////////////////////////////////////////////
template<typename View>
class create_native_view
{
private:
  typedef native_partition<View>                       pnat_type;
  typedef typename pnat_type::domain_type              domain_type;

public:
  typedef segmented_view<View, pnat_type>              view_type;

  static view_type apply(View const& v)
  {
    return view_type(v, pnat_type(v));
  }
};

} // namespace view_impl


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Defines native_view type over View.
///
/// @tparam View to partition
//////////////////////////////////////////////////////////////////////
template <typename View>
struct native_view
{
  typedef typename view_impl::create_native_view<View>::view_type type;
};

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a native_view
///
/// @param view to partition
/// @return a native partitioned view
//////////////////////////////////////////////////////////////////////
template<typename View>
typename result_of::native_view<View>::type
native_view(const View& view)
{
  return view_impl::create_native_view<View>::apply(view);
}

} // namespace stapl

#endif // STAPL_VIEWS_NATIVE_VIEW_HPP
