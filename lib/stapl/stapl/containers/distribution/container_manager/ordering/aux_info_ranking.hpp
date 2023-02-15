/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_AUX_INFO_RANKING_HPP
#define STAPL_AUX_INFO_RANKING_HPP

#include <stapl/containers/base/bc_base.hpp>
#include "node.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Struct used to as temporary storage for the information
/// required to compute list ranking over the list of base containers.
///
/// @tparam Order Ordering object type.
//////////////////////////////////////////////////////////////////////
struct aux_info
{
  typedef ordering_impl::ptr_bcontainer  ptr_type;
  typedef bc_base                        base_container_type;

  typedef ordering_impl::node            node_type;

  size_t                       rank;
  ptr_type                     local_id;
  std::vector<node_type>       links;
  std::vector<size_t>          accum_rank;

  aux_info()
    : rank(index_bounds<size_t>::invalid()),
      local_id()
  { }

  aux_info(rmi_handle::reference const& handle, size_t loc, size_t id,
           node_type const& node, size_t steps)
    : rank(),
      local_id(handle, loc, id),
      links(steps,node_type(local_id,local_id)),
      accum_rank(steps,0)
  {
    links[0] = node;
  }

  void define_type(typer& t)
  {
    t.member(rank);
    t.member(local_id);
    t.member(links);
    t.member(accum_rank);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Verify if setting the rank can procede for the given @p round.
  //////////////////////////////////////////////////////////////////////
  bool can_update_rank(size_t round)
  {
    return (links[round].rank != index_bounds<size_t>::invalid() &&
           (links[round].prev != local_id) &&
           (links[round].next != local_id));
  }

  size_t get_rank(size_t i)
  {
    return links[i].rank;
  }

  void set_rank(size_t i, size_t r)
  {
    links[i].rank=r;
  }

  ptr_type get_prev(size_t i)
  {
    return links[i].prev;
  }

  void set_prev(size_t i, ptr_type p)
  {
    links[i].prev=p;
  }

  ptr_type get_next(size_t i)
  {
    return links[i].next;
  }

  void set_next(size_t i, ptr_type n)
  {
    links[i].next=n;
  }

  node_type& get_node(size_t i)
  {
    return links[i];
  }

  void set_node(size_t i, node_type n)
  {
    links[i]=n;
  }

  size_t get_accum_rank(size_t i)
  {
    return accum_rank[i];
  }

  void set_accum_rank(size_t i, size_t r)
  {
    accum_rank[i]=r;
  }

  ptr_type get_local_id() const
  {
    return local_id;
  }

  void update_rank(size_t r)
  {
    rank = r;
  }

  size_t current_rank() const
  {
    return rank;
  }

  bool is_local()
  {
    return (get_location_id() == local_id.location);
  }

  void pre_execute()
  { }

  void post_execute()
  { }
}; // struct aux_info


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of a proxy over aux_info.
//////////////////////////////////////////////////////////////////////
template<typename Accessor>
class proxy<aux_info, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef aux_info                                 target_t;

public:
  typedef typename target_t::base_container_type   base_container_type;
  typedef typename target_t::node_type             node_type;
  typedef typename node_type::ptr_type             ptr_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {
    return Accessor::read();
  }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  size_t get_rank(size_t i)
  {
    return Accessor::invoke(&target_t::get_rank,i);
  }

  void set_rank(size_t i, size_t r)
  {
    Accessor::invoke(&target_t::set_rank,i,r);
  }

  ptr_type get_prev(size_t i)
  {
    return Accessor::invoke(&target_t::get_prev,i);
  }

  void set_prev(size_t i, ptr_type const& p)
  {
    Accessor::invoke(&target_t::set_prev,i,p);
  }

  ptr_type get_next(size_t i)
  {
    return Accessor::invoke(&target_t::get_next,i);
  }

  void set_next(size_t i, ptr_type const& n)
  {
    Accessor::invoke(&target_t::set_next,i,n);
  }

  node_type& get_node(size_t i)
  {
    return Accessor::invoke(&target_t::get_node,i);
  }

  void set_node(size_t i, node_type const& n)
  {
    Accessor::invoke(&target_t::set_node,i,n);
  }

  size_t get_accum_rank(size_t i)
  {
    return Accessor::invoke(&target_t::get_accum_rank,i);
  }

  void set_accum_rank(size_t i, size_t r)
  {
    Accessor::invoke(&target_t::set_accum_rank,i,r);
  }

  size_t current_rank() const
  {
    return Accessor::const_invoke(&target_t::current_rank);
  }

  void update_rank(size_t v)
  {
    Accessor::invoke(&target_t::update_rank,v);
  }

  bool can_update_rank(size_t r)
  {
    return Accessor::invoke(&target_t::can_update_rank,r);
  }

  ptr_type get_local_id() const
  {
    return Accessor::invoke(&target_t::get_local_id);
  }
}; //struct proxy

} // namespace stapl

#endif // STAPL_AUX_INFO_RANKING_HPP
