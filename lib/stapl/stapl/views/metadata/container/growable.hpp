/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_GROWABLE_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_GROWABLE_HPP

#include <stapl/runtime.hpp>

#include <vector>

#include <stapl/views/metadata/container/utility.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>

#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/skeletons/scan.hpp>
#include <stapl/skeletons/reduce.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/functional/reduce_to_locs.hpp>

#include <boost/optional.hpp>

namespace stapl {

namespace metadata {

struct combine_update_wf
{
  using result_type = std::pair<size_t, size_t>;

  template<typename Ref1, typename Ref2>
  result_type operator()(Ref1&& ref1, Ref2&& ref2) const
  {
    return result_type(ref1, ref2);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Container for metadata used when the number of elements
///        (metadata information) is not known at construction.
///
/// This metadata container uses a relaxed consistency over
/// insertion. The elements inserted using @c push_back_here method
/// are locally stored. When the @c update method is invoked the local
/// stored data is copied into the right position into a member
/// pContainer, keeping the data in order guided by the gid order.
///
/// @tparam MD Type of the metadata used to store the metadata
///            information.
//////////////////////////////////////////////////////////////////////
template<typename MD>
class growable_container
  : public p_object
{
private:
  typedef std::vector<MD>                                   local_vec_type;
  typedef static_array<MD>                                  container_t;
  typedef typename container_t::distribution_type           distribution_t;
  typedef typename distribution_traits<
    distribution_t>::base_container_type                    base_container_type;

  boost::optional<container_t> m_dist;
  /// local vector to store the elements before update
  local_vec_type               m_vec;
  size_t                       m_version;

public:
  typedef indexed_domain<size_t>                   domain_type;
  typedef size_t                                   index_type;
  typedef size_t                                   dimensions_type;
  typedef MD                                       value_type;
  typedef typename container_t::reference          reference;
  typedef typename base_container_type::iterator   iterator;

  growable_container(size_t n)
    : m_dist(boost::in_place<container_t>(n)),
      m_version(0)
  { }

  growable_container()
    : m_version(0)
  { }

  growable_container(growable_container const&) = delete;
  growable_container& operator=(growable_container const&) = delete;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current version number
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return m_version;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the given metadata information in to the local storage.
  //////////////////////////////////////////////////////////////////////
  void push_back_here(MD const& md)
  {
    m_vec.push_back(md);
  }

  local_vec_type& vec(void)
  {
    return m_vec;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the beginning of the local base
  ///        container.
  ///
  /// This iterator is used to traverse the local data.
  /// @note There is only one base container per location.
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return m_dist.get().distribution().container_manager().begin()->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the end of the local base container.
  ///
  /// This iterator is used to traverse the local data.
  /// @note There is only one base container per location.
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return m_dist.get().distribution().container_manager().begin()->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global index of the given local @p index.
  /// @todo Needs distribution() to be const.
  //////////////////////////////////////////////////////////////////////
  size_t get_local_vid(size_t index) const
  {
    return const_cast<container_t&>(m_dist.get()).distribution().
      container_manager().begin()->domain().first()+index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of local elements (number of elements
  ///        in the base container).
  /// @todo Needs distribution() to be const.
  //////////////////////////////////////////////////////////////////////
  size_t local_size(void) const
  {
    return const_cast<container_t&>(m_dist.get()).distribution().
      container_manager().begin()->size();
  }

  dimensions_type local_dimensions(void) const
  {
    return local_size();
  }

  typename container_t::reference
  operator[](typename container_t::gid_type index)
  {
    return m_dist.get().operator[](index);
  }

  size_t size() const
  {
    if (m_dist)
      return m_dist.get().size();
    else
      return 0;
  }

  size_t dimensions() const
  {
    return this->size();
  }

  domain_type domain() const
  {
    return m_dist ? m_dist.get().domain() : domain_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location where the metadata indexed by @p index is
  ///        located
  /// @todo Needs distribution() to be const.
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(size_t index) const
  {
    auto loc = const_cast<container_t&>(m_dist.get()).distribution().
      directory().key_mapper()(index);
    stapl_assert(loc.second != LQ_LOOKUP,
      "growable_container::get_location_element instructed to forward request");
    return loc.first;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the total number of elements (metadata
  ///        information) and copies the local stored metadata into
  ///        the distributed container, after using the prefix sum
  ///        algorithm to determine the correct position for each
  ///        element in the distributed container.
  //////////////////////////////////////////////////////////////////////
  void update()
  {
    size_t nlocs = this->get_num_locations();

    if (!m_dist)
    {
      size_t initial_pos;
      size_t nelem;

      if (nlocs != 1)
      {
        static_array<size_t> psum_init(this->get_num_locations());
        psum_init[this->get_location_id()] = m_vec.size();
        psum_init.advance_epoch();
        array_view<static_array<size_t> > vpsum_init(psum_init);

        using wf_t = detail::plus<size_t, typename MD::delay_type>;
        using namespace skeletons;

        DECLARE_INLINE_INPUT_PLACEHOLDERS(1, in)
        DECLARE_INLINE_PLACEHOLDERS(3, x)

        auto skel =
          compose<tags::inline_flow>(
            x0 << reduce_to_locs(wf_t())      | (in0),
            x1 << scan(wf_t(), 0)             | (in0),
            x2 << zip<2>(combine_update_wf(),
                         skeleton_traits<spans::per_location>()) | (x0, x1)
          );

        std::pair<size_t, size_t> p =
          execute(execution_params<std::pair<size_t, size_t>>(null_coarsener()),
            sink_value<std::pair<size_t, size_t>>(skel), vpsum_init);

        nelem       = p.first;
        initial_pos = p.second;
      }
      else
      {
        initial_pos = 0;
        nelem = m_vec.size();
      }

      m_dist = boost::in_place<container_t>(nelem);

      size_t i = 0;

      for (auto&& entry : m_vec)
      {
        entry.set_id(initial_pos+i);
        (*m_dist)[initial_pos+(i++)] = entry;
      }
    }

    if (nlocs != 1)
      block_until(detail::all_info_set<iterator>(begin(), end()));
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_CONTAINER_GROWABLE_HPP
