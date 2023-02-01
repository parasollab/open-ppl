/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_SETTABLE_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_SETTABLE_HPP

#include <stapl/containers/type_traits/distribution_traits.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray_storage.hpp>

namespace stapl {

template<typename Iterator, typename T>
class immutable_range_wrapper;

template<typename View>
struct localized_view;


template<typename View, typename Info, typename CID>
class mix_view;


template<typename T, typename... OptionalNoInitParam>
class static_array;


template<typename T, typename ...OptionalParams>
class array;


template<typename C, typename ...OptionalParams>
class array_view;


template<typename Traits>
class array_base_container;


template<typename T, typename... OptionalNoInitParam>
class basic_array_base_container;


template<typename T>
struct lightweight_vector;


template <typename T>
struct lightweight_multiarray_storage;


namespace operations {

//////////////////////////////////////////////////////////////////////
/// @brief Used to enable use of base container @p set_elements
/// method for servicing request.  Limited to now to small set of
/// array cases.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct use_bc_set_elements
  : std::false_type
{ };


template<typename Iterator, typename T>
struct use_bc_set_elements<immutable_range_wrapper<Iterator, T>>
  : std::true_type
{ };


template<typename T, typename Info, typename CID>
struct use_bc_set_elements<
  localized_view<mix_view<array_view<array<T>>, Info, CID>>>
  : std::true_type
{ };


template<typename T, typename Info, typename CID>
struct use_bc_set_elements<
  localized_view<mix_view<array_view<static_array<T>>, Info, CID>>>
  : std::true_type
{ };


template<typename Traits>
struct use_bc_set_elements<array_view<array_base_container<Traits>>>
  : std::true_type
{ };


template<typename T, typename... OptionalNoInitParam>
struct use_bc_set_elements<
  array_view<basic_array_base_container<T, OptionalNoInitParam...>>
>
  : std::true_type
{ };


template<typename T>
struct use_bc_set_elements<lightweight_vector<T>>
  : std::true_type
{ };


template<typename View>
struct construct_wrapper
{
  template<typename ViewParam>
  static
  auto
  apply(ViewParam&& view, size_t offset, size_t size)
    ->decltype(
        make_immutable_range_n(view.container().container().cbegin(), size))
  {
    auto& bc          = view.container();
    const auto bc_offset =
      bc.local_position(view.domain().advance(view.domain().first(), offset));

    return make_immutable_range_n(bc.container().cbegin() + bc_offset, size);
  }
};


template<typename Iterator>
struct construct_wrapper<immutable_range_wrapper<Iterator>>
{
  static
  immutable_range_wrapper<Iterator>
  apply(immutable_range_wrapper<Iterator> const& view,
        size_t offset, size_t size)
  {
    return immutable_range_wrapper<Iterator>(view.cbegin() + offset, size);
  }
};


template<typename T>
struct construct_wrapper<lightweight_vector<T>>
{
  template<typename ViewParam>
  static
  auto
  apply(ViewParam&& view, size_t offset, size_t size)
    ->decltype(
        make_immutable_range_n(view.cbegin(), size))
  {
    return make_immutable_range_n(view.cbegin() + offset, size);
  }
};


template<typename T>
struct construct_wrapper<lightweight_multiarray_storage<T>>
{
  template<typename ViewParam>
  static
  auto
  apply(ViewParam&& view, size_t offset, size_t size )
    ->decltype(
        make_immutable_range_n(view.cbegin(), size))
  {
    return make_immutable_range_n(view.cbegin() + offset, size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Operations class for container distributions
/// that provides set_element.
///
/// Uses the CRTP pattern. Requires that the base container has a
/// set_element method that takes a single GID.
///
/// @tparam Derived The most derived distribution class
////////////////////////////////////////////////////////////////////////
template<typename Derived>
class settable
{
public:
  STAPL_IMPORT_TYPE(typename distribution_traits<Derived>, value_type)
  STAPL_IMPORT_TYPE(typename distribution_traits<Derived>, gid_type)

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Helper class to call @p Derived::set_element().
  ////////////////////////////////////////////////////////////////////////
  struct set_element_wf
  {
    typedef void result_type;

    value_type m_value;

    explicit set_element_wf(value_type const& val)
      : m_value(val)
    { }

    void operator()(p_object& d, gid_type const& gid) const
    {
      down_cast<Derived&>(d).set_element(gid, m_value);
    }

    void define_type(typer& t)
    {
      t.member(m_value);
    }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Update the element at a GID by replacing it with a given value.
  /// @param gid GID of the element to replace
  /// @param val The new value for the element
  //////////////////////////////////////////////////////////////////////
  void set_element(gid_type const& gid, value_type const& val)
  {
    Derived& derived_ref = static_cast<Derived&>(*this);

    STAPL_IMPORT_TYPE(typename distribution_traits<Derived>,
                      base_container_type)

    using mem_fun_t =
      void (base_container_type::*)(gid_type const&, value_type const&);

    constexpr mem_fun_t mem_fun = &base_container_type::set_element;

    if (derived_ref.container_manager().contains_invoke(gid, mem_fun, gid, val))
      return;

    // else
    derived_ref.directory().invoke_where(set_element_wf(val), gid);
  }

private:
  struct set_elements_wf
  {
    using result_type = void;
    using iterator    = typename std::vector<value_type>::const_iterator;
    using wrapper     = immutable_range_wrapper<iterator>;

    wrapper m_values;

    explicit set_elements_wf(wrapper values)
      : m_values(std::move(values))
    { }

    void operator()(p_object& d, gid_type const& gid) const
    {
      down_cast<Derived&>(d).set_elements(gid, m_values);
    }

    void define_type(typer& t)
    {
      t.member(m_values);
    }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Update a range of contiguous elements in the container,
  /// starting at @p gid.  A total of @p view.size() elements will be
  /// updated with the values in @p view.
  ///
  /// @note Implementation assumes partition::try_get_domain will not fail
  /// if partition::contained_in succeeds.
  //////////////////////////////////////////////////////////////////////
  template<typename SourceView,
           typename std::enable_if<
             use_bc_set_elements<typename std::decay<SourceView>::type>::value,
             int
           >::type = 0>
  void set_elements(gid_type gid, SourceView&& source_view)
  {
    Derived& derived_ref = static_cast<Derived&>(*this);

    STAPL_IMPORT_DTYPE(distribution_traits<Derived>, base_container_type)
    STAPL_IMPORT_DTYPE(base_container_type, domain_type)

    using source_t   = typename std::decay<SourceView>::type;
    using iterator_t = typename source_t::const_iterator;
    using mem_fun_t  =
      void (base_container_type::*)(gid_type const&, iterator_t, iterator_t);

    constexpr mem_fun_t mem_fun = &base_container_type::set_elements;

    const gid_type last_gid =
      derived_ref.domain().advance(gid, source_view.size() - 1);

    const domain_type updated_domain(gid, last_gid);

    auto partitions =
      derived_ref.partition().contained_in(
        updated_domain, identity_map_func_gen<gid_type>());

    // An empty return vector from contained_in() means the partition
    // can't "cheaply" (ie. closed form, no communication) answer
    // the query. Fallback to iterative calls to set_element().
    if (partitions.size() == 0)
    {
      for (auto&& val : source_view)
      {
        this->set_element(gid, val);
        gid = derived_ref.domain().advance(gid, 1);
      }
      return;
    }

    std::vector<size_t> local_partition_ids;

    // Iterate over all intervals of partition ids which have element
    // of the target subdomain.
    for (auto&& partition : partitions)
    {
      auto f = partition.first.first();
      auto l = partition.first.last();

      // Iterate over all partition ids in current interval
      for (; f <= l; )
      {
        location_type loc = derived_ref.mapper().map(f);

        // If current partition id is local, defer processing it
        // until all remote updates have been fired.
        if (loc == derived_ref.get_location_id())
          local_partition_ids.push_back(f);
        else
        {
          // Construct a subrange values the source view to send with remote
          // request.
          auto actual_domain      = derived_ref.partition().try_get_domain(f);
          auto intersected_domain = *actual_domain & updated_domain;
          auto first_gid          = intersected_domain.first();
          auto offset             = first_gid - gid;

          // fire off remote write for this partition
          derived_ref.directory().invoke_where(
            set_elements_wf(
              construct_wrapper<source_t>::apply(
                std::forward<SourceView>(source_view),
                offset, intersected_domain.size()
              )
             ),
            first_gid
          );
        }

        // advance to next partition_id
        f = partition.first.advance(f, 1);
      }
    }

    // Perform updates or all local partitions / base containers.
    for (auto&& partition_id : local_partition_ids)
    {
      auto actual_domain      =
        derived_ref.partition().try_get_domain(partition_id);
      auto intersected_domain = *actual_domain & updated_domain;
      auto first_gid          = intersected_domain.first();
      auto offset             = first_gid - gid;

#ifndef STAPL_NDEBUG
      auto ret_val =
#endif
        derived_ref.container_manager().contains_invoke(
          first_gid, mem_fun, first_gid,
          source_view.begin() + offset,
          source_view.begin() + offset + intersected_domain.size()
        );

      stapl_assert(
        ret_val, "Failed to find subdomain that was expected locally");
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature of set_elements used when underlying base container
  /// is not supported.  Fall back to iterative use of @ref set_element.
  //////////////////////////////////////////////////////////////////////
  template<typename View,
           typename std::enable_if<
             !use_bc_set_elements<typename std::decay<View>::type>::value,
             int
           >::type = 0>
  void set_elements(gid_type gid, View&& view)
  {
    Derived& derived_ref = static_cast<Derived&>(*this);

    for (auto&& val : view)
    {
      this->set_element(gid, val);
      gid = derived_ref.domain().advance(gid, 1);
    }
  }
}; // class settable

} // namespace operations

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_SETTABLE_HPP
