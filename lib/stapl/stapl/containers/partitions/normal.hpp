/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_NORMAL_PARTITION_HPP
#define STAPL_CONTAINERS_NORMAL_PARTITION_HPP

#include <stapl/domains/indexed.hpp>
#include <numeric>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <boost/iterator/counting_iterator.hpp>
#include <iomanip>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Creates pairs from const references. This exists to avoid the C++11
/// make_pair, which uses move construction.
/// @param t1 The first value of the pair
/// @param t2 The second value of the pair
/// @return A pair of t1 and t2
//////////////////////////////////////////////////////////////////////
template<typename T1, typename T2>
std::pair<T1, T2>
non_move_make_pair(T1 const& t1, T2 const& t2)
{
  return std::make_pair(t1, t2);
}

}

//////////////////////////////////////////////////////////////////////
/// @brief Partition in which domains created form a standard normal
/// distribution.
///
/// Subdomains that are near the mean index have more elements than others,
/// which have sizes that taper off based on a given standard deviation value.
///
/// @code
///   0.45 ++---------+----------+-----------+----------+----------+---------++
///        +          +          +           +      number of elements ###### +
///    0.4 ++                             ######                             ++
///        |                           ###      ###                           |
///   0.35 ++                         ##          ##                         ++
///        |                         ##            ##                         |
///    0.3 ++                      ##                ##                      ++
///        |                      ##                  ##                      |
///   0.25 ++                    ##                    ##                    ++
///    0.2 ++                   ##                      ##                   ++
///        |                  ##                          ##                  |
///   0.15 ++                ##                            ##                ++
///        |               ###                              ###               |
///    0.1 ++            ###                                  ###            ++
///        |           ###                                      ###           |
///   0.05 ++        ###                                          ###        ++
///        +   ###### +          +           +          +          + ######   +
///      0 #####------+----------+-----------+----------+----------+------#####
///        2          3          4           5          6          7          8
///                                  partition index
/// @endcode
/// @tparam Dom Type of the domain to be partitioned
//////////////////////////////////////////////////////////////////////
template <typename Dom>
struct normal_partition
{
  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  typedef indexed_domain<size_t>           domain_type;
  /// Type used to describe the i'th subdomain
  typedef typename domain_type::index_type index_type;
  /// Type of the subdomains produced by the partition
  typedef Dom                              value_type;
  /// Type of the GIDs in the subdomains
  typedef typename value_type::index_type  gid_type;
  /// Type of the size of the subdomains
  typedef typename value_type::size_type   size_type;

protected:
  /// The global domain
  value_type                     m_domain;
  /// The index that where the normal distribution will be centered
  size_t                         m_mean;
  /// Standard deviation value to dictate the shape of the normal curve
  double                         m_sigma;
  /// The number of partitions to generate
  size_t                         m_parts;
  /// The sizes of each partition
  std::vector<size_t>            m_sizes;
  /// Lower bounds (inclusive) of the subdomains
  std::map<gid_type, index_type> m_offsets;
  /// Upper bounds (exclusive) of the subdomains
  std::map<gid_type, index_type> m_upper_bound;

public:
  normal_partition(normal_partition const& other)
    : m_domain(other.m_domain), m_mean(other.m_mean), m_sigma(other.m_sigma),
      m_parts(other.m_parts), m_sizes(other.m_sizes),
      m_offsets(other.m_offsets), m_upper_bound(other.m_upper_bound)
  { }

  //////////////////////////////////////////////////////////////////////
  /// Create a partition with a domain, the number of partitions and values
  /// describing the shape of the normal distribution.
  /// @param dom The domain to partition
  /// @param parts The number of partitions to generate
  /// @param mean The index of the partition that will be the mean of the
  /// distribution.
  /// @param sigma The standard deviation of the distribution
  //////////////////////////////////////////////////////////////////////
  normal_partition(value_type const& dom, const size_t parts,
                   const size_t mean, const double sigma = 1.0)
    : m_domain(dom), m_mean(mean), m_sigma(sigma), m_parts(parts)
  {
    using boost::math::normal;

    try {
      // Construct a standard normal distribution
      normal ns(m_mean, m_sigma);

      size_t size   = m_domain.size();
      size_t unused = size;
      for (size_t z = 0; z < m_parts; ++z)
      {
        // get the size for the current partition
        size_t s = floor(pdf(ns, z)*size);

        size_t size_to_push = std::min(s, unused);
        m_sizes.push_back(size_to_push);
        unused -= size_to_push;
      }

      // add unused gids to the mean partition
      if (unused > 0)
        m_sizes[mean] += unused;

      // compute offsets (lower bounds) for the partitions
      std::vector<size_t> offsets(m_sizes.size());
      std::partial_sum(m_sizes.begin(), m_sizes.end(), offsets.begin());

      m_offsets[m_domain.first()] = 0;
      std::transform(
        boost::counting_iterator<gid_type>(1),
        boost::counting_iterator<gid_type>(m_sizes.size()), offsets.begin(),
        std::inserter(m_offsets, m_offsets.begin()),
        detail::non_move_make_pair<gid_type, index_type>
      );

      // compute upper bounds for partitions
      typename std::map<gid_type, index_type>::const_iterator cit =
        m_offsets.begin();

      for (; cit != m_offsets.end(); ++cit)
        if (m_sizes[cit->first] > 0)
          m_upper_bound.insert(
            std::make_pair(cit->second + m_sizes[cit->first], cit->first)
          );

    } catch (const std::exception& e) {
        std::cout << std::endl << "Exception thrown with message: "
                  << e.what() << std::endl;
    }

  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1]
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  { return domain_type(m_parts); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the global domain to partition
  /// @param dom The domain to partition
  /// @bug This should recompute the bounds of the partition
  //////////////////////////////////////////////////////////////////////
  void set_domain(value_type const& dom)
  {
    stapl_assert(false, "set_domain not supported");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](index_type idx) const
  {
    // special case for empty domains
    if (m_sizes[idx] == 0)
      return value_type();

    index_type first =
      m_domain.advance(m_domain.first(), m_offsets.find(idx)->second);
    return value_type(first, m_domain.advance(first, m_sizes[idx]) - 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_type size() const
  {
    return m_sizes.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the original domain that was used to create subdomains.
  //////////////////////////////////////////////////////////////////////
  value_type const& global_domain() const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g
  /// @param g gid to find
  /// @return The index of the subdomain that contains @c g
  //////////////////////////////////////////////////////////////////////
  index_type find(index_type g) const
  {
    stapl_assert(m_upper_bound.upper_bound(g) != m_upper_bound.end(),
      "partition index out of bounds");

    return m_upper_bound.upper_bound(g)->second;
  }
};

} // stapl namespace

#endif /* STAPL_CONTAINERS_NORMAL_PARTITION_HPP */
