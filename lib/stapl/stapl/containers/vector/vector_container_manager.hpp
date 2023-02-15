/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_CONTAINER_MANAGER_HPP
#define STAPL_CONTAINERS_VECTOR_CONTAINER_MANAGER_HPP

#include <stapl/containers/distribution/container_manager/container_manager.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Container manager used for the @ref vector container.
/// @tparam BContainer The vector base container type.
/// @ingroup pvectorDist
//////////////////////////////////////////////////////////////////////
template<typename BContainer>
class vector_container_manager
  : public container_manager<BContainer>
{
private:
  typedef container_manager<BContainer>                    base_type;

public:
  typedef BContainer                                       base_container_type;
  typedef typename BContainer::gid_type                    gid_type;
  typedef typename BContainer::cid_type                    cid_type;
  typedef typename BContainer::value_type                  value_type;
  typedef typename base_type::storage_type                 storage_type;

private:
  typedef typename storage_type::storage_type::iterator    storage_iterator;

protected:
  typedef typename base_type::interval_type                interval_type;

public:
  vector_container_manager(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that initializes the initial metadata for the base
  ///        container.
  /// @param partition Partition object that translates elements to CIDs
  /// @param mapper Mapper object that translates CIDs to locations.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  vector_container_manager(Partition const& partition, Mapper const& mapper)
    : base_type(partition, mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a container manager with a specific partition and mapper.
  /// @param partition Partition object that translates elements to CIDs
  /// @param mapper Mapper object that translates CIDs to locations.
  /// @param default_value The default value for all elements in the base
  ///        container.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  vector_container_manager(Partition const& partition,
                           Mapper const& mapper,
                           value_type const& default_value)
    : base_type(partition, mapper, default_value)
  { }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Adjust the domain of a base container in a manner defined
  ///   by the input parameters.  Adjust ICL entry's domain accordingly.
  ///
  /// @param iter An iterator to the ICL entry for the base container.
  /// @param first_delta Direction and magnitude of adjustment to
  ///   the first element of the domain.
  /// @param last_delta Direction and magnitude of adjustment to
  ///   the last element of the domain.
  /// @param bc_update_func A functor to be applied, passing the base
  ///   container and new first / last.  Handles update of the base container
  ///   after necessary adjustments to the ICL domain have been handled.
  ///
  /// Constant time erase() and insert() expected amortized constant
  /// with use of hint.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  storage_iterator
  update_bc_domain(storage_iterator iter, gid_type gid,
                   int first_delta, int last_delta,
                   F bc_update_func)
  {
    size_t first = boost::icl::first(iter->first) + first_delta;
    size_t last  = boost::icl::last(iter->first)  + last_delta;

    // temp of copy base container pointer prior to interval_map erasure.
    base_container_type* bc_ptr = iter->second;

    // If inserting into an empty/invalid base container the resulting domain
    // will be [gid ..gid]. Else if the resulting bounds will be invalid/empty
    // (first > last) use the special bounds of [-1 ..-1]. Else if shifting on
    // empty base container then exit
    if (first > last && last_delta - first_delta == 1)
      first = last = gid;
    else if (first > last || (first == 0 && first == last + 1))
      first = last = -1;
    else if ((first == 0 && first_delta == 1)
        || (first + 2 == 0 && first_delta == -1))
      return ++iter;

    bc_update_func(bc_ptr, first, last);

    // Erase using iterator parameter and increment.
    this->m_intervals.erase(iter++);

    // Insert using current position as hint.
    this->m_intervals.insert(iter, std::make_pair(
      boost::icl::construct<interval_type>(
        first, last, boost::icl::interval_bounds::closed()
      ),
      bc_ptr
    ));

    return iter;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adjust all base container domains affected by insertion or
  ///   deletion of value at specified location.
  ///
  /// @param gid Index of position where mutation occurs.
  /// @param shift Magnitude and direction of change caused by mutation.
  /// @param contains_update_func Update function used when on the base
  ///  container where the mutation occurred.
  ///
  /// Base containers not contained @p gid are updated with
  /// @p set_domain as opposed to @p contains_update_func.
  ///
  /// @todo Use reverse iteration when shift is positive to avoid
  ///   transient creation of overlapping intervals in interval_map.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void adjust_domains(gid_type gid, int shift, cid_type dest,
      F contains_update_func)
  {
    // Find base containers on this location that need to be updated.
    // i.e., the one containing gid or those following it in the
    // domain's ordering.
    auto iter = this->m_intervals.lower_bound(
      boost::icl::construct<interval_type>(
        gid, gid, boost::icl::interval_bounds::closed()
      )
    );

    // Exit if no matching base container on this location.
    if (iter == this->m_intervals.end())
      return;

    // If base container containing gid exists on this location, reduce domain
    // size by one in ICL and correspondingly update the base container. If
    // erasing (shift < 0), match with any base container containing gid. If
    // inserting only match with predetermined base container
    if ((shift < 0 && boost::icl::contains(iter->first, gid))
        || (shift > 0 && iter->second->cid() == dest))
      iter = update_bc_domain(iter, gid, 0, shift, contains_update_func);

    // Update domains of all base containers with domains strictly
    // larger than gid.
    for ( ; iter != this->m_intervals.end() ; )
    {
      typedef typename base_container_type::domain_type domain_t;

      // Continue checking to insert/shift as a location with multiple base
      // containers could potentially have interval map listed out of order if
      // one were to become empty
      if (shift < 0 || (shift > 0 && iter->second->cid() > dest))
        iter = update_bc_domain(iter, gid, shift, shift,
            [](base_container_type* bc_ptr, int n_first, int n_last)
              { bc_ptr->set_domain(domain_t(n_first, n_last)); }
            );
      else if (shift > 0 && iter->second->cid() == dest)
        iter = update_bc_domain(iter, gid, 0, shift, contains_update_func);
      else // Inserting and the base container is empty but nothing to do
        iter++;
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value in the given @p gid position,
  ///   updating the base container metadata accordingly.
  //////////////////////////////////////////////////////////////////////
  void insert(gid_type gid, value_type const& value, cid_type dest)
  {
    // workaround lambda inability to capture by const&
    value_type const* value_ptr = &value;

    adjust_domains(gid, 1, dest,
      [=](base_container_type* bc_ptr, int, int)
        { bc_ptr->insert(gid, *value_ptr); }
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element associated to the given @p gid,
  ///   updating the base container metadata accordingly.
  //////////////////////////////////////////////////////////////////////
  void erase(gid_type gid)
  {
    adjust_domains(gid, -1, -1,
      [=](base_container_type* bc_ptr, int, int)
        { bc_ptr->erase(gid); }
    );
  }

#if 0
  // This is currently dead code whose only call site is the lazy_insert_view,
  // which is unused.  Enable this code if the lazy_insert_view is put into use.
  void add_values(std::vector<value_type> const& values)
  {
    base_container_type* bc = this->m_ordering.last();
    gid_type gid = 0;
    if (bc!=NULL)
      gid = bc->domain().last()+1;
    typename BContainer::domain_type bc_domain(gid,gid+values.size()-1);
    base_container_type* new_bc = new base_container_type(bc_domain,values);

    this->m_ordering.insert_after(bc,new_bc, get_location_id());
    interval_type range = boost::icl::construct<interval_type>(
                              gid,gid+values.size()-1,
                              boost::icl::interval_bounds::closed());
    this->m_intervals.insert(std::make_pair(range,new_bc));
  }
#endif

  //////////////////////////////////////////////////////////////////////
  /// @brief Add an element at the end of the vector.
  /// @param v The element to add.
  //////////////////////////////////////////////////////////////////////
  gid_type push_back(value_type const& value)
  {
    base_container_type* bc =
      down_cast<base_container_type*>(this->m_ordering.last());

    base_container_type* new_bc = bc;
    gid_type new_gid;

    if (new_bc == nullptr)
    {
      typedef typename BContainer::domain_type bc_domain_t;

      new_bc = new base_container_type(bc_domain_t(),
                                       this->m_ordering.get_location_id());

      // CID: should be invalid
      this->m_ordering.insert_after(bc, new_bc);
    }

    new_bc->push_back(value);

    new_gid = new_bc->domain().last();

    interval_type range = boost::icl::construct<interval_type>
      (new_gid, new_gid, boost::icl::interval_bounds::closed());

    this->m_intervals.insert(std::make_pair(range, new_bc));

    return new_gid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the last element of the vector.
  //////////////////////////////////////////////////////////////////////
  gid_type pop_back(void)
  {
    base_container_type* bc =
      down_cast<base_container_type*>(this->m_ordering.last());

    if (bc == nullptr)
      return index_bounds<gid_type>::invalid();

    gid_type gid = bc->pop_back();

    interval_type range = boost::icl::construct<interval_type>
      (gid,gid, boost::icl::interval_bounds::closed());

    this->m_intervals.erase(range);

    if (bc->size() == 0)
    {
      this->m_ordering.remove(bc);
      delete bc;
    }

    return gid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adjusts the associated domain to each base container based
  ///        on the given @p first position.
  ///
  /// This method is invoked during the metadata synchronization (@see
  /// vector_distribution::synchronize_metadata).
  /// @todo Generalize this code to handle arbitrary number of base
  ///   containers with arbitrary domains.  Right now only works for
  ///   for one base container per location, with gid strictly increasing
  ///   per location. See gforge #1240.
  //////////////////////////////////////////////////////////////////////
  void update_domains(size_t first)
  {
    // short circuit out if offset is 0.  No need to update.
    if (first == 0)
    {
      this->m_ordering.m_is_ordered = false;
      return;
    }

    // Reverse iterate through ICL map as we are shifting domains by
    // positive offset.
    auto iter           = this->m_intervals.rbegin();
    const auto end_iter = this->m_intervals.rend();

    for ( ; iter != end_iter ; )
    {
      typename storage_type::value_type val = *iter;

      if (!boost::icl::is_empty(val.first))
      {
        const size_t sz = val.first.upper() - val.first.lower();

        interval_type range = boost::icl::construct<interval_type>
          (first, first+sz, boost::icl::interval_bounds::closed());

        typename base_container_type::domain_type dom(first, first + sz);

        val.second->set_domain(dom);

        // Accessing base iterator of reverse iterator per
        // http://stackoverflow.com/a/1830240/429110
        this->m_intervals.erase((++iter).base());

        auto hint_iter = iter;

        // If we've already reached the end of the reverse iteration, we cannot
        // safely use above idiom.  Just use begin of ICL interval_map.
        if (hint_iter == end_iter)
          this->m_intervals.insert(
            this->m_intervals.begin(), std::make_pair(range, val.second)
          );
        else
          this->m_intervals.insert(
            (++hint_iter).base(), std::make_pair(range, val.second)
          );
      }
      else
        this->m_intervals.erase((++iter).base());
    }

    this->m_ordering.m_is_ordered = false;
  }
}; // class vector_container_manager

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_CONTAINER_MANAGER_HPP
