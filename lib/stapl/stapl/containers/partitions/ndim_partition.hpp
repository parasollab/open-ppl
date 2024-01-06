/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PARTITIONS_NDIM_PARTITION_HPP
#define STAPL_VIEWS_PARTITIONS_NDIM_PARTITION_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/containers/type_traits/is_invertible_partition.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include "partitions_generator.hpp"
#include "ndim_partition_find.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a multidimensional partition, where @p Traversal
///        specifies the traversal order defined over the generated
///        partitioned domains.
//////////////////////////////////////////////////////////////////////
template<typename Ps,
          typename Traversal =
            typename stapl::default_traversal<tuple_size<Ps>::value>::type>
class nd_partition;


template<typename P0, typename... Ps,
         typename Traversal>
class nd_partition<stapl::tuple<P0, Ps...>, Traversal>
{
  static constexpr unsigned int num_partitions = sizeof...(Ps) + 1;

public:
  using partitions_type = stapl::tuple<P0, Ps...>;
  using this_type       = nd_partition<partitions_type, Traversal>;
  using domain_type     = indexed_domain<typename P0::domain_type::index_type,
                                         num_partitions, Traversal>;
  using value_type      = indexed_domain<typename P0::value_type::index_type,
                                         num_partitions, Traversal>;
  using index_type      = typename value_type::index_type;
  using gid_type        = typename value_type::index_type;
  using size_type       = typename domain_type::size_type;
  using dimension_type  = std::integral_constant<int, num_partitions>;
  using traversal_type  = Traversal;

protected:
  value_type      m_domain;
  partitions_type m_part;


  template<std::size_t... Dim, typename Part>
  static bool same_container_domain(Part const& part, index_sequence<Dim...>&&)
  {
    return pack_ops::functional::and_(
      get<Dim>(part).global_domain().is_same_container_domain()...
    );
  }

private:
  template <std::size_t... Indices>
  nd_partition(partitions_type const& part,
              stapl::index_sequence<Indices...>&& indices)
    : m_domain(
        index_type(stapl::get<Indices>(part).global_domain().first()...),
        index_type(stapl::get<Indices>(part).global_domain().last()...),
        same_container_domain(part, std::move(indices))
      ),
      m_part(part)
  { }

public:
  nd_partition(this_type const& other) = default;

  nd_partition(P0& p0, Ps&... ps)
    : m_domain(index_type(
                 p0.global_domain().first(), ps.global_domain().first()...),
               index_type(
                 p0.global_domain().last(), ps.global_domain().last()...),
               same_container_domain(
                 std::make_tuple(p0, ps...),
                 make_index_sequence<num_partitions>()
               )),
      m_part(p0, ps...)
  { }


  nd_partition(partitions_type const& part)
    :  nd_partition(part, stapl::make_index_sequence<num_partitions>())
  { }

  void define_type(stapl::typer& t)
  {
    t.member(m_domain);
    t.member(m_part);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs partition of given domains using the specification
  /// of the number of partitions in each dimension provided.
  /// @param dom Multi-dimension domain to be partitioned
  /// @param nparts The number of partitions to be created in each dimension
  //////////////////////////////////////////////////////////////////////
  nd_partition(domain_type const& dom, size_type const& nparts)
    : m_domain(dom),
      m_part(partitions_impl::partitions_generator<
               num_partitions,
               domain_type, partitions_type>(dom)(partitions_type(), nparts))
  { }

  virtual ~nd_partition() = default;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a multidimensional domain where each dimension
  ///        specifies the valid partition's domain for each dimension.
  //////////////////////////////////////////////////////////////////////
  template <std::size_t... Indices>
  domain_type invoke_domain(stapl::index_sequence<Indices...>&&) const
  {
    return domain_type(
      index_type((Indices-Indices)...),
      index_type((stapl::get<Indices>(m_part).size() - 1)...)
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the subdomain indexed by @p idx.
  //////////////////////////////////////////////////////////////////////
  value_type invoke_bracket_operator(index_type idx,
                                    stapl::index_sequence<0>&&) const
  {
    return value_type((get<0>(m_part))[idx]);
  }

  template <std::size_t... Indices>
  value_type invoke_bracket_operator(index_type idx,
                                    stapl::index_sequence<Indices...>&&) const
  {
    return value_type(
             index_type((stapl::get<Indices>(m_part)[
                           stapl::get<Indices>(idx)].first())...),
             index_type((stapl::get<Indices>(m_part)[
                           stapl::get<Indices>(idx)].last())...), true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of generated partitions.
  //////////////////////////////////////////////////////////////////////
  template <std::size_t... Indices>
  size_t invoke_size(stapl::index_sequence<Indices...>&&) const
  {
    std::initializer_list<std::size_t> sizes =
      {stapl::get<Indices>(m_part).size()...};
    return std::accumulate(sizes.begin(), sizes.end(), (std::size_t) 1,
                           std::multiplies<std::size_t>());
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a multidimensional domain where each dimension
  ///        specifies the valid partition's domain for each dimension.
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return invoke_domain(stapl::make_index_sequence<num_partitions>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the associated domain that is partitioned.
  //////////////////////////////////////////////////////////////////////
  value_type const& global_domain() const
  {
    return m_domain;
  }

  value_type* get_global_domain()
  {
    return &m_domain;
  }

  template <std::size_t... Indices>
  void update_domain(value_type const& dom, stapl::index_sequence<Indices...>&&)
  {
    int ignore_me[]={0, (void(
      stapl::get<Indices>(m_part).set_domain(dom.template get_domain<Indices>())
    ),0)... };
    (void)ignore_me;

    m_domain = value_type(get<Indices>(m_part).global_domain()...);
  }
  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the global domain associated with this partitioner.
  //////////////////////////////////////////////////////////////////////
  void update_domain(value_type const& dom)
  {
    invoke_update_domain(dom, stapl::make_index_sequence<num_partitions>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the subdomain indexed by @p idx.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](index_type idx) const
  {
    return invoke_bracket_operator(
             idx,
             stapl::make_index_sequence<num_partitions>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the subdomain indexed by (@p i_0, @p i_1, ..., @p i_n).
  //////////////////////////////////////////////////////////////////////
  value_type operator()(typename P0::index_type i0,
                        typename Ps::index_type... is) const
  {
    return this->operator[](index_type(i0, is...));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of generated partitions.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return invoke_size(stapl::make_index_sequence<num_partitions>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated in each dimension.
  //////////////////////////////////////////////////////////////////////
  template <std::size_t... Indices>
  size_type invoke_dimensions(stapl::index_sequence<Indices...>&&) const
  {
    return size_type(stapl::get<Indices>(m_part).size()...);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated in each dimension.
  //////////////////////////////////////////////////////////////////////
  size_type dimensions() const
  {
    return invoke_dimensions(stapl::make_index_sequence<num_partitions>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the subdomain that contains the
  ///        given index @p g.
  /// @param pos Flag to indicate which index return. If @p pos is
  ///            True, returns the index of the first subdomain that
  ///            contains @p g, otherwise the index of the last
  ///            subdoomain that contains @p g.  @return index_type
  //////////////////////////////////////////////////////////////////////
  index_type invoke_find(index_type const& g, bool pos,
                         stapl::index_sequence<0>&&) const
  {
    return get<0>(m_part).find(g);
  }

  template<std::size_t... Indices>
  index_type invoke_find(index_type const& g, bool pos,
                         stapl::index_sequence<Indices...>&&) const
  {
    return index_type(
             stapl::get<Indices>(m_part).find(stapl::get<Indices>(g))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the subdomain that contains the
  ///        given index @p g.
  /// @param pos Flag to indicate which index return. If @p pos is
  ///            True, returns the index of the first subdomain that
  ///            contains @p g, otherwise the index of the last
  ///            subdoomain that contains @p g.  @return index_type
  //////////////////////////////////////////////////////////////////////
  index_type find(index_type const& g, bool pos=true) const
  {
    return invoke_find(g, pos, stapl::make_index_sequence<num_partitions>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc find
  //////////////////////////////////////////////////////////////////////
  index_type where_is(index_type const& g, bool pos=true) const
  {
    return find(g);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc find
  //////////////////////////////////////////////////////////////////////
  index_type in_domain(index_type const& g, bool pos=true) const
  { return where_is(g, pos); }


  //////////////////////////////////////////////////////////////////////
  /// @brief Determine which partition has the elements referenced
  ///        for the given domain.
  ///
  /// The returned information is a collection (possibly empty) of
  /// pairs. Each pair contains information about which partitions are
  /// included in the given domain and how they are included (True: if
  /// is fully contained, False: if is partially included).The
  /// returned collection only has elements if there is at least one
  /// partition that contains elements on the given domain.
  ///
  /// @param dom Domain to compare
  /// @param mfg Mapping function generator used to get the associated
  ///            mapping function to each partition. The generated
  ///            mapping function is used to project generated
  ///            partitioned domains into the given domain.
  /// @return a vector of pairs.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  find(ODom const& dom, MFG const& mfg)
  {
    return stapl::details::find_in_part<domain_type>(
             m_part, dom, mfg,
             std::integral_constant<int, num_partitions>());
  }
};


template<typename... Ps, typename Traversal>
struct is_invertible_partition<nd_partition<stapl::tuple<Ps...>,Traversal>>
  : public std::integral_constant<bool, true>
{ };

} // namespace stapl

#endif // STAPL_CONTAINERS_PARTITIONS_NDIM_PARTITION_HPP
