/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HEAP_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_HEAP_DISTRIBUTION_HPP

#include <stapl/domains/list_distributed_domain.hpp>
#include <stapl/containers/distribution/distribution.hpp>
#include <stapl/containers/distribution/list_metadata.hpp>
#include <stapl/containers/distribution/operations/base.hpp>
#include <stapl/containers/iterators/container_accessor.hpp>
#include <stapl/containers/iterators/container_iterator.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/balance_view.hpp>
#include <libstdc++/proxy/pair.h>
#include <stapl/views/proxy.h>
#include <stapl/views/counting_view.hpp>
#include <stapl/array.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief The distribution for @ref heap container.
/// @tparam Container Type of the container managing the distribution.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct heap_distribution
  : public distribution<Container>,
    public operations::base<heap_distribution<Container> >
{

  typedef std::random_access_iterator_tag                 iterator_category;

  typedef distribution<Container>                         base_type;
  typedef typename base_type::directory_type              directory_type;
  typedef typename base_type::
                   container_manager_type         container_manager_type;
  typedef typename container_manager_type::
                   base_container_type                    base_container_type;

  typedef typename container_manager_type::gid_type       gid_type;
  typedef typename base_container_type::comp_type         comp_type;
  typedef gid_type                                        index_type;

  typedef typename container_manager_type::value_type     value_type;
  typedef typename base_type::partition_type              partition_type;
  typedef typename base_type::mapper_type                 mapper_type;

  typedef typename base_type::container_manager_type
                   ::base_container_type::iterator        local_iterator;
  typedef typename container_manager_type::iterator       cm_iterator;

  typedef list_metadata<heap_distribution>                loc_dist_metadata;
  typedef list_distributed_domain<heap_distribution>      domain_type;

  typedef container_accessor<heap_distribution>           accessor_type;
  typedef proxy<value_type, accessor_type>                reference;

  typedef const_container_accessor<heap_distribution>    const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;

  typedef container_iterator<heap_distribution,
                             accessor_type,
                             iterator_category
                            >                             iterator;
  typedef const_container_iterator<heap_distribution,
                             const_accessor_type,
                             iterator_category
                            >                             const_iterator;

protected:
  /// Domain used by the distribution
  domain_type m_domain;

private:
  typedef std::pair<gid_type, value_type>       pair_type;

  heap_distribution(void)
    : m_domain(this)
  { }

public:
  heap_distribution(heap_distribution const& other)
    : base_type(other),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a directory and container manager.
  /// @param directory Directory of the container.
  /// @param bcmangr Container manager of the container.
  //////////////////////////////////////////////////////////////////////
  heap_distribution(directory_type const& directory,
                    container_manager_type const& bcmangr)
    : base_type(directory, bcmangr),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a specific partitioner and mapper.
  /// @param partition Partition of the container.
  /// @param mapper Mapper of the container.
  //////////////////////////////////////////////////////////////////////
  heap_distribution(partition_type const& partition,
                    mapper_type const& mapper)
    : base_type(
        directory_type(
          list_manager<partition_type, mapper_type>(partition, mapper)),
        container_manager_type(partition,mapper)
      ),
      m_domain(*this)
  { }

  // ========================
  // Heap methods
  // ========================

  void push(value_type const& t)
  {
    cm_iterator smallest_bc = this->container_manager().begin();

    //If container_manager is empty
    if (smallest_bc == this->container_manager().end())
    {
      base_container_type* bc = new base_container_type();
      bc->push(t);
      this->container_manager().push_bc(bc);
      return ;
    }

    //Find smallest base container and push in it
    for (cm_iterator it = this->container_manager().begin() ;
         it != this->container_manager().end() ; ++it)
    {
      if ((*it)->size() < (*smallest_bc)->size())
        smallest_bc = it ;
    }
    (*smallest_bc)->push(t);
  }

private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Mapping function to find the k greatest element according to
  ///        the order inferred by the comparator.
  //////////////////////////////////////////////////////////////////////
  struct map_get_top_k
  {
     typedef std::pair<gid_type, value_type>       pair_type;
     typedef std::vector<pair_type>                vec_type;
     typedef vec_type result_type;

     std::vector<pair_type> m_local_top;

     map_get_top_k(std::vector<pair_type> vec)
     {
       m_local_top = vec;
     }

    vec_type operator()(size_t)
    {
      return m_local_top;
    }

    void define_type(typer& t)
    {
      t.member(m_local_top);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Reduce function comparing two elements using their GID
  ///        and returning the GID of the greatest element.
  //////////////////////////////////////////////////////////////////////
  struct red_gid
  {
    typedef std::pair<gid_type, value_type> pair_type;
    typedef pair_type result_type;

    pair_type operator()(pair_type a, pair_type b) const
    {
      if (a.first.m_base_container != nullptr &&  a.second > b.second)
        return a;
      else return b;
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Reduce function for two boolean returning false if at least
  ///        one boolean is false.
  /// @todo Uses of it should be replaced by stapl::logical_and
  ///       and the struct removed.
  //////////////////////////////////////////////////////////////////////
  struct red_bool
  {
    typedef bool result_type;

    bool operator()(bool a, bool b) const
    {
      if (!a)
        return false;
      else
        return b;
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Reduce two vectors holding between 0 and
  ///        k elements to a vector containing the k greatest elements
  ///        of both according to the comparator.
  //////////////////////////////////////////////////////////////////////
  struct red_k
  {
    typedef std::pair<gid_type,value_type> pair_type;
    typedef std::vector<pair_type> vec_type;
    typedef vec_type result_type;

    size_t m_k;

    red_k(size_t const& k)
      : m_k(k)
    { }

    void define_type(typer& t)
    {
      t.member(m_k);
    }

    vec_type operator()(vec_type a, vec_type b) const
    {
      size_t size_a = a.size();
      size_t size_b = b.size();
      size_t output_size = std::min<size_t>(m_k, size_a + size_b);
      size_t index_a = 0;
      size_t index_b = 0;

      stapl::less<value_type> comp;
      vec_type result(output_size);

      for (size_t i = 0; i != output_size ; ++i)
      {
        //If not out of boundaries
        if (index_a != size_a && index_b != size_b)
        {
          if (comp(a[index_a].second,b[index_b].second))
          {
            result[i] = b[index_b];
            ++index_b;
          } else {
           result[i] = a[index_a];
           ++index_a;
          }
        } else {
          if (index_a < size_a)
          {
            result[i] = a[index_a];
            ++index_a;
          } else if (index_b < size_b) {
            result[i] = b[index_b];
            ++index_b;
          }
        }
      }
      return result;
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Distribute the greatest elements found in find_k among locations.
  /// @param k Number of greatest elements to get.
  /// @param global_result vector holding the k greatest elements of the
  ///        distributed container according to the order
  ///        inferred by the comparator.
  /// @param arr_top distributed array where each location will
  ///        insert in its range k/p greatest elements.
  ///
  /// If k < p, only k locations will insert an element
  /// into their @p arr_top local range.
  //////////////////////////////////////////////////////////////////////
  void  distribute_top(size_t const& k,
                       std::vector<std::pair<gid_type, value_type> >
                       const& global_result,
                       array<value_type>& arr_top)
  {
    //Euclidian division after global results
    //q : number of estimated top_k pop
    //r : number of locs that should pop alone after q top_k pop
    size_t num_locs = this->get_num_locations();
    size_t id_loc = this->get_location_id();
    size_t q = size_t(global_result.size()/num_locs);
    size_t r = global_result.size() % num_locs;

    std::vector<value_type> vec_pop;
    vec_pop.reserve(q);

    //Find indices
    size_t index_begin = id_loc * q;
    size_t index_end = index_begin + q;

    if (r != 0)
    {
      index_begin += id_loc;
      index_end += id_loc;
      if (id_loc < r)
        ++index_end;
    }

    for (size_t x = index_begin; x != index_end ; ++x)
      arr_top[x] = global_result[x].second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the k local greatest elements according to the
  ///        order inferred by the comparator.
  /// @param k Number of greatest elements to find.
  /// @return A vector containing k pairs <gid,value> of elements found.
  //////////////////////////////////////////////////////////////////////
  std::vector<pair_type> find_k(size_t const& k)
  {
    std::vector<pair_type> array_top(k);
    std::vector<pair_type> stack(k);
    size_t counter = 0;
    bool value_exist = false;

    if (this->container_manager().get_bcs().size() == 0)
    {
      array_top.resize(0);
      return array_top;
    }

    for (size_t i= 0 ; i < k ; ++i)
    {
      //Find k min among all bc
      if (counter != k)
      {
        //Find min among all bc
        for (size_t bc_iter = 0;
             bc_iter != this->container_manager().get_bcs().size();
             ++bc_iter)
        {
          value_exist = false;
          if ((this->container_manager().get_bcs()[bc_iter]->size() > 0) &&
              ((this->container_manager().get_bcs()[bc_iter])->top())
              >= (array_top[i].second))
          {
            value_exist = true;
            array_top[i].first =
                 (this->container_manager().get_bcs()[bc_iter])->top_gid();
            array_top[i].second = *(array_top[i].first.m_pointer);
          }
        }
        //pop the element and add it into stack to be pushed later
        if (value_exist)
        {
          stack[i] = pair_type(array_top[i].first,
                               array_top[i].first.m_base_container->pop());
          ++counter;
        }
      }
    }

    //Resize if necessary
    if (counter != array_top.size())
      array_top.resize(counter);

    for (size_t i = 0; i != counter ; ++i)
    {
      stack[i].first.m_base_container->push(stack[counter-i-1].second);
    }
    return array_top;
  }

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the greatest element of the container according to the
  ///        order inferred by the comparator.
  /// @return A pair <gid,value> of the element found.
  //////////////////////////////////////////////////////////////////////
  std::pair<gid_type, value_type> get_top(void)
  {
    typedef std::pair<gid_type, value_type>       pair_type;
    array<pair_type> arr_heap(this->get_num_locations());
    array_view<array<pair_type> > view_heap(arr_heap);

    typename container_manager_type::iterator it =
             this->container_manager().begin();
    pair_type val;

    //If empty container_manager
    if (it == this->container_manager().end())
    {
      val.first = gid_type();
      val.second = value_type();
    }
    //Each location find its local top
    else {
      val.first = (*it)->top_gid() ;
      val.second = *(val.first.m_pointer);

      for (++it ; it != this->container_manager().end() ; ++it)
      {
        if (*((*it)->top_gid().m_pointer) > val.second)
        {
          val.first = (*it)->top_gid();
          val.second =  *(val.first.m_pointer);
        }
      }
    }

    view_heap[this->get_location_id()] = val;
    pair_type result = map_reduce(identity<pair_type>(),red_gid(),view_heap);
    return result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator.
  //////////////////////////////////////////////////////////////////////
  value_type top(void)
  {
    return this->get_top().second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the k global greatest elements of the container
  ///        according to the order inferred by the comparator.
  /// @param k Number of greatest elements to find.
  /// @return A vector of pairs <gid,value> of elements found.
  //////////////////////////////////////////////////////////////////////
  std::vector<std::pair<gid_type, value_type> > get_top_k(size_t const& k)
  {
    typedef std::pair<gid_type, value_type>       pair_type;

    //Each location get its top k values
    std::vector<pair_type> local_result;
    std::vector<pair_type> global_result;

    local_result.reserve(k);
    global_result.reserve(k);

    local_result = find_k(k);

    //Find the global top k values
    global_result = map_reduce(map_get_top_k(local_result),
                               red_k(k),counting_view<size_t>
                                        (this->get_num_locations()));
    return global_result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the k greatest elements of the container according to the
  ///        order inferred by the comparator and distribute their
  ///        copies into an array view in a balance way.
  /// @param k Number of greatest elements to find.
  /// @return A view over a copy of the elements found.
  //////////////////////////////////////////////////////////////////////
  stapl::array_view<stapl::array<value_type> > top_k(size_t const& k)
  {
    typedef std::pair<gid_type, value_type>       pair_type;
    typedef stapl::array<value_type>              arr_type;
    typedef stapl::array_view<arr_type>           view_type;

    //q: number of estimated top_k pop
    size_t q = size_t(k/get_num_locations());

    std::vector<pair_type> global_result;
    std::vector<value_type> vec_pop;
    global_result.reserve(k);
    vec_pop.reserve(q);

    global_result = get_top_k(k);

    //If heap size < k
    stapl_assert(global_result.size() >= k,
    "top_k impossible ! heap hold less than k elements ");

    arr_type* arr_top = new arr_type(global_result.size());
    distribute_top(k, global_result, *arr_top);
    return view_type(arr_top);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes k greatest elements of the container according to the
  ///        order inferred by the comparator and distribute their
  ///        copies into an array view in a balance way.
  /// @param k Number of greatest elements to find.
  /// @return A view over a copy of the elements found.
  //////////////////////////////////////////////////////////////////////
  stapl::array_view<stapl::array<value_type> > pop_k(size_t const& k)
  {
    typedef std::pair<gid_type, value_type>       pair_type;
    typedef array<value_type>                     arr_type;
    typedef array_view<arr_type>                  view_type;

    //q: number of estimated top_k pop
    size_t q = size_t(k/get_num_locations());

    std::vector<pair_type> global_result;
    std::vector<value_type> vec_pop;
    global_result.reserve(k);
    vec_pop.reserve(q);

    global_result = get_top_k(k);

    //Check if results size < k
    stapl_assert(global_result.size() >= k,
    "pop_k impossible ! heap hold less than k elements ");

    //Pop local values
    for (size_t i = 0; i < global_result.size() ; ++i)
    {
      if (this->container_manager().contains(global_result[i].first))
      {
        this->pop_local(global_result[i].first);
      }
    }

    //Return values
    arr_type* arr_top = new arr_type(global_result.size());
    distribute_top(k, global_result, *arr_top);
    return view_type(arr_top);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the first element according to the order
  ///        determined by the comparator and removes it from the container.
  //////////////////////////////////////////////////////////////////////
  value_type pop(void)
  {
    std::pair<gid_type, value_type> result = this->get_top();
    const value_type keep = result.second;
    if (this->container_manager().contains(result.first))
    {
      this->pop_local(result.first);
    }
    return keep;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the container follows the ordering
  ///        inferred by the comparator.
  //////////////////////////////////////////////////////////////////////
  bool is_heap(void)
  {
    array<bool> arr_heap(this->get_num_locations());
    array_view<array<bool> > view_heap(arr_heap);

    typename container_manager_type::iterator it =
             this->container_manager().begin();

    //If locally heap ordering is maintain return true, false otherwise
    bool val = (*it)->is_heap();
    for (it = this->container_manager().begin() ;
         it != this->container_manager().end() ; ++it)
    {
      if (!(*it)->is_heap())
        val = false;
    }

    view_heap[this->get_location_id()] = val;
    bool result = stapl::map_reduce(identity<bool>(),red_bool(),view_heap);
    return result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all elements in the container and fill it
  ///        with elements of a view.
  /// @tparam V The view used to populate the container.
  //////////////////////////////////////////////////////////////////////
  template <typename V>
  void make(V v)
  {
    typedef typename result_of::balance_view<V>::type         pbv;

    //FIXME bug in list_distributed_domain
    //GForge [#857] remove methods of container ordering is bugged
    //clear();     //when fixed, uncomment this line

    //Create a balance view
    pbv bv = balance_view<V>(v,get_num_locations());

    //Create new base container and fill it with it a n/p chunck
    base_container_type* bc = new base_container_type();
    bc->make(bv[get_location_id()]);

    //Push it in the container maanger
    this->container_manager().push_bc(bc);

  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->container_manager().clear();
  }

  // ========================
  // Domain Interface
  // ========================

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to an element of the container
  ///        using its GID.
  /// @param gid The GID for which to create a reference.
  /// @return A proxy of the element @p gid.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return reference(accessor_type(this,gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @param gid The GID for which to create the iterator.
  /// @return An iterator to the element @p gid.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(this,gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @tparam Dom The domain type of the GID's element distribution.
  /// @param gid The GID for which to create the iterator.
  /// @return An iterator to the element @p gid.
  //////////////////////////////////////////////////////////////////////
  template <typename Dom>
  iterator make_iterator(Dom, gid_type const& gid)
  {
    return iterator(this,gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @param gid The GID for which to create the iterator.
  /// @return A const iterator to the element @p gid.
  //////////////////////////////////////////////////////////////////////
  const_iterator make_iterator(gid_type const& gid) const
  {
    return iterator(this,gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a domain for the distribution and return it.
  //////////////////////////////////////////////////////////////////////
  domain_type const& domain(void) const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global number of elements in the domain.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return domain().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the local number of elements in the domain.
  //////////////////////////////////////////////////////////////////////
  size_t local_size(void) const
  {
    return domain().local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a global iterator to the first element
  ///        of the domain.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return make_iterator(this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a global iterator to one position past the
  ///        last element of the domain.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return ++(make_iterator(this->domain().last()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a const global iterator to the first element
  ///        of the domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return make_iterator(this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a const global iterator to one
  ///        position past the last element of domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return make_iterator(this->domain().last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @param idx The GID for which to create the iterator.
  /// @return An iterator to the element @p idx.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& idx)
  {
    return make_iterator(idx);
  }


  // ========================
  // Iteration
  // ========================

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the GID that is @p n elements after the specified
  ///        GID.
  /// @param gid The GID beginning the advance operation.
  /// @param n The number of GIDs to advance.
  /// @param globally Indicate if iterating locally or globally.
  //////////////////////////////////////////////////////////////////////
  gid_type advance(gid_type const& gid,
                   long long n, bool globally)
  {
    return domain_type(*this).advance(gid, n, globally);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the GID that is @p n elements after the specified
  ///        GID and set the promise @p p at the GID found.
  /// @param gid The GID beginning the advance operation.
  /// @param n The number of GIDs to advance.
  /// @param globally Indicate if iterating locally or globally.
  /// @param p promise to be set.
  //////////////////////////////////////////////////////////////////////
  void defer_advance(gid_type const& gid,
                     long long n, bool globally,
                     promise<gid_type> p)
  {
    p.set_value(this->advance(gid,n,globally));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the promise to indicate if a GID exists or not
  ///        in the container manager.
  /// @param gid The GID to be used.
  /// @param p promise to be set.
  //////////////////////////////////////////////////////////////////////
  void defer_contains(gid_type const& gid,
                      promise<bool> p)
  {
    bool res =
         this->container_manager().contains(gid);
    p.set_value(res);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set a promise to the distance between two GIDs.
  /// @param gida The first GID.
  /// @param gidb The second GID.
  /// @param p promise to be set.
  //////////////////////////////////////////////////////////////////////
  void defer_distance(gid_type const& gida,
                      gid_type const& gidb,
                      promise<size_t> p)
  {
    this->container_manager().defer_distance(gida, gidb, std::move(p));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the promise to indicate if a GID exists or not
  ///        in a domain.
  /// @param dom The domain to be used.
  /// @param gid The GID to be checked in the domain provided.
  /// @param p promise to be set.
  //////////////////////////////////////////////////////////////////////
  void contains_helper(domain_type const& dom,
                       gid_type const& gid,
                       promise<bool> p) const
  {
    const bool res = dom.contains(gid);
    p.set_value(res);
  }

 // ========================
 // Local methods
 // ========================

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove the element with the specified GID from the local
  ///        base container in which it is currently stored.
  /// @param gid GID of the element to remove.
  /// @bug Cannot remove last element of a base container.
  ///      GForge [#857] remove methods of container ordering is bugged.
  //////////////////////////////////////////////////////////////////////
  void pop_local(gid_type const& gid)
  {
    if (this->container_manager().contains(gid))
    {
      gid.m_base_container->pop();
    }
    //If base container become empty, remove it and modify ordering
    if (gid.m_base_container->empty())
    {
      //FIXME GForge [#857] remove methods of container ordering is bugged
      //this->container_manager().get_ordering().remove(gid.base_container);
      /*cm_iterator erase_me = std::find(this->container_manager().begin(),
                             this->container_manager().end(),
                             gid.base_container);
      this->container_manager().erase(erase_me);*/
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Pops out the greatest element of all elements of the
  ///        local base containers.
  /// @bug Cannot remove last element of a base container.
  ///      GForge [#857] remove methods of container ordering is bugged.
  //////////////////////////////////////////////////////////////////////
  void  pop_local(void)
  {
    gid_type max_value = (*(this->container_manager().begin()))->top_gid();
    cm_iterator keep_bc_max = this->container_manager().begin();

    //If empty container
    if (keep_bc_max == this->container_manager().end())
    {
      abort("No element to pop, empty bc");
      return ;
    }

    //Find biggest element
    //FIXME First iteration already checked
    for (cm_iterator it = (this->container_manager().begin()) ;
         it != this->container_manager().end() ; ++it)
    {
      if ( *(((*it)->top_gid()).m_pointer) > *(max_value.m_pointer) )
      {
        max_value = (*it)->top_gid();
        keep_bc_max = it;
      }
    }

    //Pop it out
    (*keep_bc_max)->pop();


    //If the bc become empty, remove it and modify ordering
    if ((*keep_bc_max)->size() == 0)
    {
      //FIXME remove method of orering not working
      this->container_manager().get_ordering().remove(*keep_bc_max);
      this->container_manager().erase(keep_bc_max);
    }
  }

  // ========================
  // Private Methods
  // ========================

private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Rearranges the distribution elements using the comparator
  ///        provided in such a way that it forms a heap.
  //////////////////////////////////////////////////////////////////////
  void heapify(void)
  {
    for (cm_iterator it = this->container_manager().begin() ;
         it != this->container_manager().end() ; ++it)
    {
      it->heapify();
    }
  }

};

} // stapl namespace

#endif
