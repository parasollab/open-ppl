/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_BASE_CONTAINER_ORDERING_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_BASE_CONTAINER_ORDERING_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/runtime.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <boost/bimap/bimap.hpp>
#include <stapl/containers/distribution/container_manager/ordering/aux_info_ranking.hpp>

#include "node.hpp"
#include "ordering_functors.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an object that keeps track of the ordering of base
///        containers in a pContainer.
//////////////////////////////////////////////////////////////////////
struct base_container_ordering
  : public p_object
{
  typedef size_t                                        bcontainer_id_type;
  typedef bc_base                                       base_container_type;
  typedef ordering_impl::ptr_bcontainer                 ptr_bcontainer_type;
  typedef ordering_impl::node                           node_type;
  typedef node_type                                     follow_bcontainers_type;

  typedef std::map<bc_base*, follow_bcontainers_type>   bcontainer_dir_type;
  typedef boost::bimaps::bimap<size_t, bc_base*>        local_indexing_type;
  typedef local_indexing_type::value_type               local_idx_entry_type;

  /// Location on which the first base container is stored.
  location_type            m_first_loc;

  /// Location on which the last base container is stored.
  location_type            m_last_loc;

  /// Keeps track of the links between base containers.
  bcontainer_dir_type      m_follows;

  /// local mapping of base containers.
  local_indexing_type      m_local_indexing;

  /// Local Id of the first local base container.
  bcontainer_id_type       m_first;

  /// Local Id of the last valid local base container.
  bcontainer_id_type       m_last;

  /// Keeps track of the next valid local id.
  size_t                   m_key;

  /// Flag to determine if the base containers are in order
  bool                     m_is_ordered;

  /// Total number of base containers in the pContainer.
  size_t                   m_total_num_bc;

  /// Buffer of base container predecessor information.
  std::map<size_t, ptr_bcontainer_type> m_prev_info;

  base_container_ordering()
    : m_first_loc(index_bounds<location_type>::invalid()),
      m_last_loc(index_bounds<location_type>::invalid()),
      m_first(index_bounds<bcontainer_id_type>::invalid()),
      m_last(index_bounds<bcontainer_id_type>::invalid()),
      m_key(0),
      m_is_ordered(true),
      m_total_num_bc(0)
  {
    this->advance_epoch();
    initialize_link();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all information related to the current ordering.
  /// @bug Access to the container distribution is needed to allow ordering
  ///   to properly represent arbitrary distributions.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    m_local_indexing.clear();
    m_prev_info.clear();
    m_follows.clear();
    m_total_num_bc = 0;
    m_is_ordered = false;
    m_key = 0;
    m_last = index_bounds<bcontainer_id_type>::invalid();
    m_first = index_bounds<bcontainer_id_type>::invalid();
    m_last_loc = index_bounds<location_type>::invalid();
    m_first_loc = index_bounds<location_type>::invalid();
    initialize_link();
  }

private:
  ////////////////////////////////////////////////////////////////////////
  /// @brief Inserts an empty node into the link manager to define an initial
  ///   connection.
  ////////////////////////////////////////////////////////////////////////
  void initialize_link(void)
  {
    const location_type loc = this->get_location_id();

    const location_type prev_loc =
      loc > 0 ? loc-1 : index_bounds<location_type>::invalid();

    const location_type next_loc =
      loc + 1 < this->get_num_locations() ?
        loc + 1 : index_bounds<location_type>::invalid();

    bcontainer_id_type null_id = 0;
    bc_base* p                 = nullptr;

    m_follows.insert(std::make_pair(
      p, node_type(ptr_bcontainer_type(get_rmi_handle(), prev_loc, null_id),
                   ptr_bcontainer_type(get_rmi_handle(), next_loc, null_id),
                   loc)
    ));
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ordering pointer to the previous base
  ///        container of the given base container @p bc.
  //////////////////////////////////////////////////////////////////////
  ptr_bcontainer_type prev(bc_base* bc) const
  {
    return m_follows.find(bc)->second.prev;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ordering pointer to the next base container
  ///        of the given base container @p bc.
  //////////////////////////////////////////////////////////////////////
  ptr_bcontainer_type next(bc_base* bc) const
  {
    return m_follows.find(bc)->second.next;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the pointer to the first local base container
  ///        based on the local order.
  //////////////////////////////////////////////////////////////////////
  bc_base* first() const
  {
    if (m_first != index_bounds<bcontainer_id_type>::invalid())
      return (*this)[m_first];

    return nullptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the pointer to the last local base container
  ///        based on the local order.
  //////////////////////////////////////////////////////////////////////
  bc_base* last() const
  {
    if (m_last!=index_bounds<bcontainer_id_type>::invalid())
      return (*this)[m_last];
    return nullptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the previous link information for the local base
  ///        container referenced for @p id, with the given ordering
  ///        pointer @p new_prev.
  //////////////////////////////////////////////////////////////////////
  void set_prev(bcontainer_id_type id, ptr_bcontainer_type const& new_prev)
  {
    local_indexing_type::left_map::const_iterator bc_iter =
      m_local_indexing.left.find(id);
    if (bc_iter != m_local_indexing.left.end())
    {
      m_follows[bc_iter->second].prev = new_prev;
    }
    else
    {
      m_prev_info[id] = new_prev;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the next link information for the local base
  ///        container referenced for @p id, with the given ordering
  ///        pointer @p new_next.
  //////////////////////////////////////////////////////////////////////
  void set_next(bcontainer_id_type id, ptr_bcontainer_type const& new_next)
  {
    local_indexing_type::left_map::const_iterator bc_iter =
      m_local_indexing.left.find(id);
    if (bc_iter != m_local_indexing.left.end())
    {
      m_follows[bc_iter->second].next = new_next;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the associated local identifier from the given
  ///        base container pointer.
  //////////////////////////////////////////////////////////////////////
  bcontainer_id_type find_id(bc_base* bc) //const
  {
    local_indexing_type::right_iterator it_bc =
                                             m_local_indexing.right.find(bc);
    if (it_bc!=m_local_indexing.right.end())
      return it_bc->second;
    return index_bounds<bcontainer_id_type>::invalid();
  }

  bc_base* operator[](bcontainer_id_type id) const
  {
    local_indexing_type::left_map::const_iterator bc_iter =
      m_local_indexing.left.find(id);
    if (bc_iter != m_local_indexing.left.end())
      return bc_iter->second;
    else
      return nullptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given base container @p bc, before the base
  ///        container @p pos, assigning it the specified rank.
  //////////////////////////////////////////////////////////////////////
  void insert(bc_base* pos, bc_base* bc, size_t rank=0)
  {
    bcontainer_id_type new_id = m_key++;
    m_local_indexing.insert(local_idx_entry_type(new_id,bc));
    follow_bcontainers_type conn = m_follows[pos];
    m_is_ordered = false;

    if (new_id==0) {
      m_follows[bc] = node_type(conn.prev, conn.next, rank);
      m_first = new_id;
      m_last = new_id;
    }
    else {
      location_type this_lid = this->get_location_id();
      bcontainer_id_type pos_id = find_id(pos);
      ptr_bcontainer_type new_prev = conn.prev;
      ptr_bcontainer_type new_next(get_rmi_handle(), this_lid, pos_id);

      m_follows[bc] = node_type(new_prev,new_next,rank);
      if (m_first == pos_id)
        m_first = new_id;

      // setup connections
      m_follows[pos].prev =
        ptr_bcontainer_type(get_rmi_handle(), this_lid, new_id);

      ptr_bcontainer_type tmp_next(get_rmi_handle(),this_lid, new_id);

      if (new_prev.location==this_lid) {
        set_next(new_prev.id,tmp_next);
      }
      else {
        if (new_prev.location!=index_bounds<location_type>::invalid())
          async_rmi(new_prev.location, this->get_rmi_handle(),
                    &base_container_ordering::set_next, new_prev.id, tmp_next);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given base container @p bc, after the base
  ///        container @p pos, assigning it the specified rank.
  //////////////////////////////////////////////////////////////////////
  void insert_after(bc_base* pos, bc_base* bc, size_t rank=0)
  {
    bcontainer_id_type new_id = m_key++;
    m_local_indexing.insert(local_idx_entry_type(new_id,bc));

    follow_bcontainers_type conn = m_follows[pos];
    m_is_ordered = false;

    if (new_id==0) {
      m_follows[bc] = node_type(conn.prev, conn.next, rank);
      m_first = new_id;
      m_last = new_id;
    }
    else {
      location_type this_lid = this->get_location_id();
      bcontainer_id_type pos_id = find_id(pos);
      ptr_bcontainer_type new_prev(get_rmi_handle(), this_lid, pos_id);
      ptr_bcontainer_type new_next = conn.next;

      m_follows[bc] = node_type(new_prev,new_next,rank);
      if (m_last == pos_id)
        m_last = new_id;

      // setup connections
      m_follows[pos].next =
        ptr_bcontainer_type(get_rmi_handle(), this_lid, new_id);

      ptr_bcontainer_type tmp_prev(get_rmi_handle(), this_lid, new_id);

      if (new_next.location==this_lid) {
        set_prev(new_next.id,tmp_prev);
      }
      else {
        if (new_next.location!=index_bounds<location_type>::invalid())
          async_rmi(new_next.location, this->get_rmi_handle(),
                    &base_container_ordering::set_prev, new_next.id, tmp_prev);
      }
    }
  }


protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Records the id of the location that reports itself as storing
  /// the first base container in the ordering.
  /// @param loc Location reporting that it is storing the first base container
  ///
  /// The reporting of the first and last locations in the ordering is an
  /// optimization that allows the linear traversal of base containers in
  /// the ordering to find the first or last element in the container.
  //////////////////////////////////////////////////////////////////////
  void set_first_loc(location_type loc)
  {
    m_first_loc = loc;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Records the id of the location that reports itself as storing
  /// the last base container in the ordering.
  /// @param loc Location reporting that it is storing the last base container
  ///
  /// The reporting of the first and last locations in the ordering is an
  /// optimization that allows the linear traversal of base containers in
  /// the ordering to find the first or last element in the container.
  //////////////////////////////////////////////////////////////////////
  void set_last_loc(location_type loc)
  {
    m_last_loc = loc;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of locally mapped base containers.
  //////////////////////////////////////////////////////////////////////
  size_t local_indexing_size(void) const
  {
    return m_local_indexing.size();
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given base container @p bc locally and sets the
  /// previous base container information for the next base container
  /// specified, as well as the next base container information of
  /// the base container being inserted.
  ///
  /// @param bc Base container to insert in the ordering
  /// @param rank Rank of the base container being inserted
  /// @param next_loc Location of the next base container after @p bc.
  /// @param next_rank Rank of the next base container after @p bc.
  ///
  /// @note If this method is used to construct a pContainer it must be
  /// used exclusively, as it uses the base container rank as the key
  /// while the other insert methods use a local key.
  //////////////////////////////////////////////////////////////////////
  void insert(bc_base* bc, size_t rank, location_type next_loc,
              size_t next_rank)
  {
    stapl_assert(m_key==0,
      "Invalid combination of insert calls on base container ordering");

    location_type lid = this->get_location_id();
    location_type nlocs = this->get_num_locations();

    if (rank == 0)
    {
      m_first_loc = lid;
      if (nlocs != 1)
        unordered::async_rmi(all_locations, this->get_rmi_handle(),
                  &base_container_ordering::set_first_loc, m_first_loc);
    }

    if (next_loc == index_bounds<location_type>::invalid())
    {
      m_last_loc = lid;
      if (nlocs != 1)
        unordered::async_rmi(all_locations, this->get_rmi_handle(),
                  &base_container_ordering::set_last_loc, m_last_loc);
    }

    if (m_local_indexing.size() == 0)
    {
      m_first = rank;
      m_last = rank;
    }
    else
    {
      m_last = rank;
    }
    m_local_indexing.insert(local_idx_entry_type(rank,bc));

    // An async specifying the previous base container of the base container
    // may have arrived.  The base container* should not be in the map however.
    stapl_assert(m_follows.find(bc) == m_follows.end(),
                 "m_follows entry for base container exists.");

    follow_bcontainers_type node;
    node.rank = rank;
    if (next_loc != lid && next_loc != index_bounds<location_type>::invalid())
    {
      node.next = ptr_bcontainer_type(get_rmi_handle(), next_loc, next_rank);
      ptr_bcontainer_type tmp_prev(get_rmi_handle(),lid, rank);
      async_rmi(next_loc, this->get_rmi_handle(),
                    &base_container_ordering::set_prev, next_rank, tmp_prev);
    }
    else if (next_loc != index_bounds<location_type>::invalid())
    {
      local_indexing_type::left_map::const_iterator bc_iter =
        m_local_indexing.left.find(next_rank);
      if (bc_iter != m_local_indexing.left.end())
      {
        bc_base* next_bc = bc_iter->second;
        bcontainer_dir_type::iterator i = m_follows.find(next_bc);
        if (i != m_follows.end())
        {
          i->second.prev = ptr_bcontainer_type(get_rmi_handle(), lid, rank);
          node.next =
            ptr_bcontainer_type(get_rmi_handle(), lid, this->find_id(i->first));
        }
        else
        {
          m_prev_info[next_rank] =
            ptr_bcontainer_type(get_rmi_handle(), lid, rank);
          node.next =
            ptr_bcontainer_type(get_rmi_handle(), next_loc, next_rank);
        }
      }
      else
      {
        m_prev_info[next_rank] =
          ptr_bcontainer_type(get_rmi_handle(), lid, rank);
        node.next =
          ptr_bcontainer_type(get_rmi_handle(), next_loc, next_rank);
      }
    }
    else
    {
      node.next = ptr_bcontainer_type();
    }
    std::map<size_t, ptr_bcontainer_type>::iterator pi = m_prev_info.find(rank);
    if (pi != m_prev_info.end())
    {
      node.prev = pi->second;
      m_prev_info.erase(pi);
    }
    m_follows[bc] = node;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Helper function to invoke the forward traversal visitor
  ///        with the given functor @p func over the local base
  ///        container identified with @p id.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void traverse_fw_helper(Functor func, bcontainer_id_type id) const
  {
    if (id != index_bounds<location_type>::invalid())
      traverse_forward(func, (*this)[id], false);
    else
      traverse_forward(func, nullptr, true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper function to invoke the backward traversal visitor
  ///        with the given functor @p func over the local base
  ///        container identified with @p id.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void traverse_bw_helper(Functor func, bcontainer_id_type id) const
  {
    if (id!=index_bounds<location_type>::invalid())
      traverse_backward(func, (*this)[id], false);
    else
      traverse_backward(func, nullptr, true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Visitor that applies the specified functor over the given
  ///        base container @p bc and continues its forward traversal
  ///        until the stop condition is reached.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void traverse_forward(Functor func, bc_base* bc, bool is_end = false) const
  {
    const location_type this_lid = this->get_location_id();

    if (func(bc, this_lid, is_end))
    {
      ptr_bcontainer_type next_bc = this->next(bc);

      if (next_bc.location == this_lid)
        traverse_fw_helper(func, next_bc.id);
      else
      {
        if (next_bc.location!=index_bounds<location_type>::invalid())
        {
          typedef void (base_container_ordering::*fn)
            (Functor const&, bcontainer_id_type) const;

          async_rmi(next_bc.location, this->get_rmi_handle(),
                    (fn) &base_container_ordering::traverse_fw_helper,
                    func, next_bc.id);
        }
        else
          traverse_fw_helper(func, index_bounds<location_type>::invalid());
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Visitor that applies the specified functor over the given
  ///        base container @p bc and continues its backward traversal
  ///        until the stop condition is reached.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void traverse_backward(Functor func, bc_base* bc, bool is_end=false) const
  {
    const location_type this_lid = this->get_location_id();

    if (func(bc, this_lid, is_end)) {
      ptr_bcontainer_type prev_bc = this->prev(bc);
      if (prev_bc.location==this_lid)
        traverse_bw_helper(func, prev_bc.id);
      else
        if (prev_bc.location!=index_bounds<location_type>::invalid())
        {
          typedef void (base_container_ordering::*fn)
            (Functor const&, bcontainer_id_type) const;

          async_rmi(prev_bc.location, this->get_rmi_handle(),
                    (fn) &base_container_ordering::traverse_bw_helper,
                    func, prev_bc.id);
        }
        else {
          traverse_bw_helper(func, index_bounds<location_type>::invalid());
        }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the first base container in the ordering based on the
  /// ordering information, and then iterate forward through the ordering
  /// to skip over empty base containers.
  ///
  /// This method is used in the implementation of distribution begin()
  /// methods.
  ///
  /// @param func Functor storing the promise that will be set to notify
  /// the original calling location of the result.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void find_first(Functor& func) const
  {
    location_type this_lid = this->get_location_id();
    bc_base* bc = first();
    if (this_lid == m_first_loc)
    {
      // This location was registered as the location containing the first
      // base container.
      stapl_assert(bc != nullptr,
                   "base_container_ordering::find_first invalid first_loc");
      if (func(bc))
      {
        // The base container is empty.  Traverse forward to find non-empty.
        ordering_detail::find_fw<typename Functor::promise_type,
                                 base_container_ordering,
                                 typename Functor::base_container_type>
        findfw(func);
        traverse_backward(findfw, bc, true);
      }
      // else the promise in the functor was set.
    }
    else
    {
      // Find the first base container on the location and follow its prev link.
      // The base container should be on another location, otherwise first() is
      // incorrectly implemented.
      ptr_bcontainer_type prev_info = prev(bc);
      stapl_assert(prev_info.location != this_lid,
        "first base container on location has prev base container on loc too.");

      if (prev_info.location != index_bounds<location_type>::invalid())
      {
        typedef void (base_container_ordering::*fn)(Functor&) const;

        async_rmi(prev_info.location, this->get_rmi_handle(),
                  (fn)&base_container_ordering::find_first, func);

      } else if (func(bc)) {
        // The base container is empty.  Traverse forward to find non-empty.
        ordering_detail::find_fw<typename Functor::promise_type,
                                 base_container_ordering,
                                 typename Functor::base_container_type>
        findfw(func);
        traverse_backward(findfw, bc, true);
      }
      // else the promise in the functor was set.
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the last base container in the ordering based on the
  /// ordering information, and then iterate backward through the ordering
  /// to skip over empty base containers.
  ///
  /// This method is used in the implementation of distribution end()
  /// methods.
  ///
  /// @param func Functor storing the promise that will be set to notify
  /// the original calling location of the result.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void find_last(Functor& func) const
  {
    const location_type this_lid = this->get_location_id();
    bc_base* const bc            = last();

    if (this_lid == m_last_loc)
    {
      // This location was registered as the location containing the last
      // base container.
      stapl_assert(bc != nullptr,
                   "base_container_ordering::find_last invalid last_loc");
      if (func(bc))
      {
        // The base container is empty.  Traverse backward to find non-empty.
        ordering_detail::find_bw<typename Functor::promise_type,
                                 base_container_ordering,
                                 typename Functor::base_container_type>
        findbw(func);
        traverse_backward(findbw, bc, true);
      }
      // else the promise in the functor was set.
    }
    else
    {
      // Find the last base container on the location and follow its next link.
      // The base container should be on another location, otherwise last() is
      // incorrectly implemented.
      ptr_bcontainer_type next_info = next(bc);
      stapl_assert(next_info.location != this_lid,
        "last base container on location has next base container on location.");

      if (next_info.location != index_bounds<location_type>::invalid())
      {
        typedef void (base_container_ordering::*fn)(Functor&) const;
        async_rmi(next_info.location, this->get_rmi_handle(),
                  (fn) &base_container_ordering::find_last, func);

      } else if (func(bc)) {
        // The base container is empty.  Traverse forward to find non-empty.
        ordering_detail::find_bw<typename Functor::promise_type,
                                 base_container_ordering,
                                 typename Functor::base_container_type>
        findbw(func);
        traverse_forward(findbw, bc, true);
      }
      // else the promise in the functor was set.
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the specified base container @p bc from the
  ///        ordering object.
  //////////////////////////////////////////////////////////////////////
  void remove(bc_base* pos)
  {
    location_type this_lid = this->get_location_id();
    follow_bcontainers_type conn = m_follows[pos];

    if (pos==nullptr)
      return;

    if (conn.next.location==this_lid)
      set_prev(conn.next.id, conn.prev);
    else {
      // Last bc on current location is being removed - update the last id
      m_last = conn.prev.id;
      // Link from the first bc on the next location
      if (conn.next.location!=index_bounds<location_type>::invalid())
        async_rmi(conn.next.location, this->get_rmi_handle(),
                  &base_container_ordering::set_prev, conn.next.id, conn.prev);
      else {
        // If current ordering tracks the global index of the location that
        // stores the last base container, update it
        if (m_last_loc != index_bounds<location_type>::invalid()) {
          assert(m_last_loc == this_lid);

          m_last_loc = conn.prev.location;

          if (this->get_num_locations() != 1)
            unordered::async_rmi(all_locations, this->get_rmi_handle(),
                    &base_container_ordering::set_last_loc, m_last_loc);
        }
      }
    }

    if (conn.prev.location==this_lid)
      set_next(conn.prev.id, conn.next);
    else {
      // First bc on current location is being removed - update the first id
      m_first = conn.next.id;
      // Link to last bc on the previous location
      if (conn.prev.location!=index_bounds<location_type>::invalid())
        async_rmi(conn.prev.location, this->get_rmi_handle(),
                  &base_container_ordering::set_next, conn.prev.id, conn.next);
      else {
        // If current ordering tracks the global index of the location that
        // stores the first base container, update it
        if (m_first_loc != index_bounds<location_type>::invalid()) {
          assert(m_first_loc == this_lid);

          m_first_loc = conn.next.location;

          if (this->get_num_locations() != 1)
            unordered::async_rmi(all_locations, this->get_rmi_handle(),
                    &base_container_ordering::set_first_loc, m_first_loc);
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given migrated base container @p mbc and
  ///        updates the links based on the given connection
  ///        information @p conn.
  //////////////////////////////////////////////////////////////////////
  void migrate(bc_base* mbc, follow_bcontainers_type conn)
  {
    // TODO: remove this copy (the base container is created outside)
    bc_base* bc = new bc_base(*mbc);

    bcontainer_id_type new_id = m_key++;
    //    m_local_indexing[new_id] = bc;
    m_local_indexing.insert(local_idx_entry_type(new_id,bc));
    m_follows[bc] = conn;

    ptr_bcontainer_type new_ptr(
      get_rmi_handle(), this->get_location_id(), new_id);

    //set prev
    if (conn.prev.location!=index_bounds<location_type>::invalid())
      async_rmi(conn.prev.location, this->get_rmi_handle(),
                &base_container_ordering::set_next, conn.prev.id, new_ptr);
    // set next
    if (conn.next.location!=index_bounds<location_type>::invalid())
      async_rmi(conn.next.location, this->get_rmi_handle(),
                &base_container_ordering::set_prev, conn.next.id, new_ptr);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Moves the specified base container @p bc to the given
  ///        location @p loc.
  //////////////////////////////////////////////////////////////////////
  void move(bc_base* bc, location_type loc)
  {
    const location_type this_lid = this->get_location_id();
    follow_bcontainers_type conn = m_follows[bc];

    if (loc != this_lid)
    {
      async_rmi(loc, this->get_rmi_handle(), &base_container_ordering::migrate,
                bc, conn);
      bcontainer_id_type pos_id = find_id(bc);
      m_local_indexing.left.erase(pos_id);
      m_follows.erase(bc);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Replaces the given base container @p old_bc with the
  ///        given new base container @p new_bc in the base container
  ///        ordering.
  //////////////////////////////////////////////////////////////////////
  void replace(bc_base* old_bc, bc_base* new_bc)
  {
    if (old_bc == nullptr)
      old_bc = this->first();

    m_follows.insert(std::make_pair(new_bc,m_follows[old_bc]));
    m_follows.erase(old_bc);
    size_t id = find_id(old_bc);
    m_local_indexing.left.erase(id);
    m_local_indexing.insert(local_idx_entry_type(id,new_bc));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the @p rank for the base container associated with the
  ///        local identifier @p id.
  //////////////////////////////////////////////////////////////////////
  void set_rank(size_t id, size_t rank)
  {
    if (m_local_indexing.left.begin()!=m_local_indexing.left.end())
      m_follows[(*this)[id]].rank = rank;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the rank of the base container associated with the
  ///        local identifier @p id.
  //////////////////////////////////////////////////////////////////////
  size_t get_rank(size_t id) // const
  {
    return m_follows[(*this)[id]].rank;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the rank of the base container @p bc.
  //////////////////////////////////////////////////////////////////////
  size_t get_rank(bc_base* bc) //const
  {
    return m_follows[bc].rank;
  }

  /// @todo Could this method be moved into container manager?
  size_t total_number_base_containers() const
  {
    if (this->get_num_locations()==1)
      return local_indexing_size();

    return allreduce_rmi(std::plus<size_t>(),
                         this->get_rmi_handle(),
                         &base_container_ordering::local_indexing_size).get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper function to initialize the auxiliary information
  ///        needed to compute the base container ranking.
  ///
  /// @param steps Number of elements needed to be initialized based
  ///              on log(number of locations). Base container ranking
  ///              uses a pointer jumping approach to compute the
  ///              ranks.
  /// @return A vector of size @p steps with the initialized values
  ///         for each step.
  //////////////////////////////////////////////////////////////////////
  std::vector<aux_info>
  get_init_rank(size_t steps)
  {
    const size_t tmp_sz = (m_key==0) ? 1 : m_key;

    std::vector<aux_info> tmp_aux(tmp_sz);

    local_indexing_type::left_iterator it_b = m_local_indexing.left.begin();

    if (it_b == m_local_indexing.left.end())
    {
      const size_t id = 0;

      aux_info info_node(get_rmi_handle(), this->get_location_id(), id,
                         m_follows[nullptr], steps+1);

      info_node.set_rank(0,0);
      info_node.set_accum_rank(0,0);
      info_node.update_rank(0);
      tmp_aux[id] = info_node;
    }
    else
    {
      for ( ; it_b != m_local_indexing.left.end(); ++it_b)
      {
        const size_t id = it_b->first;

        aux_info info_node(
          get_rmi_handle(), this->get_location_id(),
          id, m_follows[it_b->second], steps + 1);

        info_node.set_rank(0,1);
        info_node.set_accum_rank(0,1);
        info_node.update_rank(0);
        tmp_aux[id] = info_node;
      }
    }

    return tmp_aux;
  }
}; // struct base_container_ordering

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_BASE_CONTAINER_ORDERING_HPP
