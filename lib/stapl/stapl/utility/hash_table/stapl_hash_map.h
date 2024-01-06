/*
 * Copyright (c) 1996
 * Silicon Graphics Computer Systems, Inc.
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Silicon Graphics makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 *
 * Copyright (c) 1994
 * Hewlett-Packard Company
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Hewlett-Packard Company makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */

/* NOTE: This is an internal header file, included by other STL headers.
 *   You should not attempt to use it directly.
 */
#ifndef __STAPL_INTERNAL_HASH_MAP_H
#define __STAPL_INTERNAL_HASH_MAP_H

#include <algorithm> // lower_bound
#include <memory> // allocator

#include <stapl/utility/hash.hpp>
#include "stapl_hashtable_impl.h"

#ifdef _STAPL
# include <stapl/runtime/serialization.hpp>
#endif

namespace stapl
{

template <class _Key, class _Tp,
          class _HashFcn  = stapl::hash<_Key>,
          class _EqualKey = std::equal_to<_Key>,
          class _Alloc    = std::allocator<_Tp> >
class hash_map;

//////////////////////////////////////////////////////////////////////
/// @note Forward declaration of equality operator.
///   needed for friend declaration.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Key, class _Tp, class _HashFn, class _EqKey, class _Alloc>
bool operator==(const hash_map<_Key, _Tp, _HashFn, _EqKey, _Alloc>&,
                       const hash_map<_Key, _Tp, _HashFn, _EqKey, _Alloc>&);

//////////////////////////////////////////////////////////////////////
/// @brief Container that stores key-value pairs by hashing the key to
///   determine the position in the container where the elements will be
///   stored.
/// @tparam _Key The hash_map's key type.
/// @tparam _Tp The hash_map value type.
/// @tparam _HashFcn The hash function used by the hash_map. The default hash
/// function is stapl::hash.
/// @tparam _EqualKey The key equality functor. Binary predicate used to
///   compare key values of elements stored in the container for equality.
///   The default equality functor is std::equal_to<_Key>.
/// @tparam _Alloc The hash_map's allocator used for internal memory
///   management. The default allocator is std::allocator<_Tp>.
/// @ingroup utility
/// @todo Re-evaluate the use of this class in STAPL.  It is provided
///   here to allow bidirectional iteration through the elements in the hash
///   table.  The requirement that bContainers of STAPL containers require
///   bidirectional iteration is something to re-evaluate as well, as we may
///   be able to at least eliminate the requirement on associative and
///   relational containers.
//////////////////////////////////////////////////////////////////////
template <class _Key, class _Tp, class _HashFcn, class _EqualKey,
          class _Alloc>
class hash_map
{
private:
  typedef stapl::hashtable<
    std::pair<const _Key,_Tp>,_Key,_HashFcn,
    std::_Select1st<std::pair<const _Key,_Tp> >,_EqualKey,_Alloc> _Ht;
  _Ht _M_ht;

public:
  typedef typename _Ht::key_type key_type;
  typedef _Tp data_type;
  typedef _Tp mapped_type;
  typedef typename _Ht::value_type value_type;
  typedef typename _Ht::hasher hasher;
  typedef typename _Ht::key_equal key_equal;

  typedef typename _Ht::size_type size_type;
  typedef typename _Ht::difference_type difference_type;
  typedef typename _Ht::pointer pointer;
  typedef typename _Ht::const_pointer const_pointer;
  typedef typename _Ht::reference reference;
  typedef typename _Ht::const_reference const_reference;

  typedef typename _Ht::iterator iterator;
  typedef typename _Ht::const_iterator const_iterator;

  typedef typename _Ht::allocator_type allocator_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the hash functor.
  //////////////////////////////////////////////////////////////////////
  hasher hash_funct() const { return _M_ht.hash_funct(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the comparison functor.
  //////////////////////////////////////////////////////////////////////
  key_equal key_eq() const { return _M_ht.key_eq(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the allocator.
  //////////////////////////////////////////////////////////////////////
  allocator_type get_allocator() const { return _M_ht.get_allocator(); }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a default hash map. By default there's 100 buckets.
  //////////////////////////////////////////////////////////////////////
  hash_map() : _M_ht(100, hasher(), key_equal(), allocator_type()) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets.
  /// @param __n The number of buckets desired.
  //////////////////////////////////////////////////////////////////////
  explicit hash_map(size_type __n)
    : _M_ht(__n, hasher(), key_equal(), allocator_type()) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets and a hasher.
  /// @param __n The number of buckets desired.
  /// @param __hf The hash functor.
  //////////////////////////////////////////////////////////////////////
  hash_map(size_type __n, const hasher& __hf)
    : _M_ht(__n, __hf, key_equal(), allocator_type()) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets, a hasher, a comparison object and an allocator.
  /// @param __n The number of buckets desired.
  /// @param __hf The hash functor.
  /// @param __eql The comparison object.
  /// @param __a The allocator. By default a new one is constructed.
  //////////////////////////////////////////////////////////////////////
  hash_map(size_type __n, const hasher& __hf, const key_equal& __eql,
           const allocator_type& __a = allocator_type())
    : _M_ht(__n, __hf, __eql, __a) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a default hash map. By default there's 100 buckets.
  ///   Inserts all the elements in a range of two input iterators.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l)
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  hash_map(_InputIterator __f, _InputIterator __l)
    : _M_ht(100, hasher(), key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  /// @param __n The number of buckets desired.
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  hash_map(_InputIterator __f, _InputIterator __l, size_type __n)
    : _M_ht(__n, hasher(), key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets and a hasher.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  /// @param __n The number of buckets desired.
  /// @param __hf The hash functor.
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  hash_map(_InputIterator __f, _InputIterator __l, size_type __n,
           const hasher& __hf)
    : _M_ht(__n, __hf, key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets, a hasher, an equality functor and an allocator.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l)
  /// @param __n The number of buckets desired.
  /// @param __hf The hash functor.
  /// @param __eql The equality functor.
  /// @param __a The allocator. By default a new one is constructed.
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  hash_map(_InputIterator __f, _InputIterator __l, size_type __n,
           const hasher& __hf, const key_equal& __eql,
           const allocator_type& __a = allocator_type())
    : _M_ht(__n, __hf, __eql, __a)
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a default hash map. By default there's 100 buckets.
  ///   Inserts all the elements in a range of two input iterators.
  /// @param __f, __l Pointers delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l)
  //////////////////////////////////////////////////////////////////////
  hash_map(const value_type* __f, const value_type* __l)
    : _M_ht(100, hasher(), key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets.
  /// @param __n The number of buckets desired.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  hash_map(const value_type* __f, const value_type* __l, size_type __n)
    : _M_ht(__n, hasher(), key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial
  ///   buckets and a hasher.
  /// @param __f, __l Pointers delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  /// @param __n The number of buckets desired.
  /// @param __hf The hash functor.
  //////////////////////////////////////////////////////////////////////
  hash_map(const value_type* __f, const value_type* __l, size_type __n,
           const hasher& __hf)
    : _M_ht(__n, __hf, key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hash map with a specific number of initial.
  ///   buckets, a hasher, a comparison object, and an allocator.
  /// @param __f, __l Pointers delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  /// @param __n The number of buckets desired.
  /// @param __hf The hash functor.
  /// @param __eql The comparison object.
  /// @param __a The allocator. By default a new one is constructed.
  //////////////////////////////////////////////////////////////////////
  hash_map(const value_type* __f, const value_type* __l, size_type __n,
           const hasher& __hf, const key_equal& __eql,
           const allocator_type& __a = allocator_type())
    : _M_ht(__n, __hf, __eql, __a)
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc hash_map(_InputIterator __f, _InputIterator __l)
  //////////////////////////////////////////////////////////////////////
  hash_map(const_iterator __f, const_iterator __l)
    : _M_ht(100, hasher(), key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc hash_map(_InputIterator __f, _InputIterator __l, size_type __n)
  //////////////////////////////////////////////////////////////////////
  hash_map(const_iterator __f, const_iterator __l, size_type __n)
    : _M_ht(__n, hasher(), key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc hash_map(_InputIterator __f, _InputIterator __l, size_type __n, const hasher& __hf)
  //////////////////////////////////////////////////////////////////////
  hash_map(const_iterator __f, const_iterator __l, size_type __n,
           const hasher& __hf)
    : _M_ht(__n, __hf, key_equal(), allocator_type())
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc hash_map(_InputIterator __f, _InputIterator __l, size_type __n, const hasher& __hf, const key_equal& __eql, const allocator_type& __a )
  //////////////////////////////////////////////////////////////////////
  hash_map(const_iterator __f, const_iterator __l, size_type __n,
           const hasher& __hf, const key_equal& __eql,
           const allocator_type& __a = allocator_type())
    : _M_ht(__n, __hf, __eql, __a)
    { _M_ht.insert_unique(__f, __l); }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the container.
  //////////////////////////////////////////////////////////////////////
  size_type size() const { return _M_ht.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the maximum number of elements that can be held
  ///   in the container.
  //////////////////////////////////////////////////////////////////////
  size_type max_size() const { return _M_ht.max_size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the container is empty or not.
  //////////////////////////////////////////////////////////////////////
  bool empty() const { return _M_ht.empty(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Exchange the content of the container with another one.
  /// @param __hs The container which swaps elements of the same type.
  //////////////////////////////////////////////////////////////////////
  void swap(hash_map& __hs) { _M_ht.swap(__hs._M_ht); }

  template <class _K1, class _T1, class _HF, class _EqK, class _Al>
  friend bool operator== (const hash_map<_K1, _T1, _HF, _EqK, _Al>&,
                          const hash_map<_K1, _T1, _HF, _EqK, _Al>&);


  iterator begin() { return _M_ht.begin(); }
  iterator end() { return _M_ht.end(); }
  const_iterator begin() const { return _M_ht.begin(); }
  const_iterator end() const { return _M_ht.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts a new element in the container.
  /// @param __obj The element to be inserted. It is a pair<key,value>.
  /// @return A pair containing an iterator pointing to the element whose key
  ///   is the same as the key of the element to insert, and a bool value that
  ///   indicates whether the element was successfully inserted or not.
  //////////////////////////////////////////////////////////////////////
  std::pair<iterator,bool> insert(const value_type& __obj)
    { return _M_ht.insert_unique(__obj); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts new elements in the container.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  void insert(_InputIterator __f, _InputIterator __l)
    { _M_ht.insert_unique(__f,__l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts new elements in the container.
  /// @param __f, __l Pointers delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  void insert(const value_type* __f, const value_type* __l) {
    _M_ht.insert_unique(__f,__l);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc insert(_InputIterator __f, _InputIterator __l)
  //////////////////////////////////////////////////////////////////////
  void insert(const_iterator __f, const_iterator __l)
    { _M_ht.insert_unique(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts a new element in the container without increasing the
  ///   number of buckets.
  /// @param __obj The element to be inserted. It is a pair<key,value>.
  /// @return A pair containing an iterator pointing to the element whose key
  ///   is the same as the key of the element to insert, and a bool value that
  ///   indicates whether the element was successfully inserted or not.
  //////////////////////////////////////////////////////////////////////
  std::pair<iterator,bool> insert_noresize(const value_type& __obj)
    { return _M_ht.insert_unique_noresize(__obj); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Search for a specific element in the container.
  /// @param __key The key indexing the element.
  /// @return Iterator to the element, or to the end if no element is found.
  //////////////////////////////////////////////////////////////////////
  iterator find(const key_type& __key) { return _M_ht.find(__key); }
  const_iterator find(const key_type& __key) const
    { return _M_ht.find(__key); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Access an element with a given key. If no element with the key
  ///   is present, it inserts a default element with that key.
  /// @param __key The key indexing the element.
  /// @return Reference to the mapped value of the element
  //////////////////////////////////////////////////////////////////////
  _Tp& operator[](const key_type& __key) {
    return _M_ht.find_or_insert(value_type(__key, _Tp())).second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements with the specified key.
  /// @param __key The key indexing the element.
  //////////////////////////////////////////////////////////////////////
  size_type count(const key_type& __key) const { return _M_ht.count(__key); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a range that includes all the elements with a specific key.
  /// @param __key The key indexing the element.
  /// @return Pair of iterators delimiting a range of elements
  ///   The range of elements is [pair's first, pair's second).
  //////////////////////////////////////////////////////////////////////
  std::pair<iterator, iterator> equal_range(const key_type& __key)
    { return _M_ht.equal_range(__key); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc equal_range(const key_type& __key)
  //////////////////////////////////////////////////////////////////////
  std::pair<const_iterator, const_iterator>
  equal_range(const key_type& __key) const
    { return _M_ht.equal_range(__key); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all elements with the specified key from the container.
  /// @param __key The key indexing the element.
  /// @return The number of elements removed from the container.
  //////////////////////////////////////////////////////////////////////
  size_type erase(const key_type& __key) {return _M_ht.erase(__key); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes an element pointed by an iterator from the container.
  /// @param __it The iterator over the element to delete.
  //////////////////////////////////////////////////////////////////////
  void erase(iterator __it) { _M_ht.erase(__it); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes some elements pointed by an iterator from the container.
  /// @param __f, __l Input iterators delimiting a range of elements to delete.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  void erase(iterator __f, iterator __l) { _M_ht.erase(__f, __l); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all the elements from the container
  //////////////////////////////////////////////////////////////////////
  void clear() { _M_ht.clear(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the bucket count to at least __hint.
  //////////////////////////////////////////////////////////////////////
  void resize(size_type __hint) { _M_ht.resize(__hint); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of buckets used by the container.
  //////////////////////////////////////////////////////////////////////
  size_type bucket_count() const { return _M_ht.bucket_count(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the maximum number of buckets that may be used by
  ///   the container.
  //////////////////////////////////////////////////////////////////////
  size_type max_bucket_count() const { return _M_ht.max_bucket_count(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in a specific bucket.
  //////////////////////////////////////////////////////////////////////
  size_type elems_in_bucket(size_type __n) const
    { return _M_ht.elems_in_bucket(__n); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the size in the memory used by the container.
  //////////////////////////////////////////////////////////////////////
  size_type memory_size(void) const { return _M_ht.memory_size(); }

#ifdef _STAPL
  void define_type(stapl::typer&)
  {
    stapl_assert(false, "define_type() not implemented for hash_map");
  }
#endif
};


template <class _Key, class _Tp, class _HashFcn, class _EqlKey, class _Alloc>
bool operator==(const hash_map<_Key,_Tp,_HashFcn,_EqlKey,_Alloc>& __hm1,
           const hash_map<_Key,_Tp,_HashFcn,_EqlKey,_Alloc>& __hm2)
{
  return __hm1._M_ht == __hm2._M_ht;
}

template <class _Key, class _Tp, class _HashFcn, class _EqlKey, class _Alloc>
bool operator!=(const hash_map<_Key,_Tp,_HashFcn,_EqlKey,_Alloc>& __hm1,
           const hash_map<_Key,_Tp,_HashFcn,_EqlKey,_Alloc>& __hm2) {
  return !(__hm1 == __hm2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Swaps the content of two hash_map.
/// @param __hm1,__hm2 The two hash_map.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Key, class _Tp, class _HashFcn, class _EqlKey, class _Alloc>
void swap(hash_map<_Key,_Tp,_HashFcn,_EqlKey,_Alloc>& __hm1,
     hash_map<_Key,_Tp,_HashFcn,_EqlKey,_Alloc>& __hm2)
{
  __hm1.swap(__hm2);
}
}// STAPL NAMESPACE

#endif /* __STAPL_INTERNAL_HASH_MAP_H */
