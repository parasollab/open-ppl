/*
 * Copyright (c) 1996,1997
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

#ifndef __STAPL_INTERNAL_HASHTABLE_H
#define __STAPL_INTERNAL_HASHTABLE_H
// Hashtable class, used to implement the hashed associative containers
// hash_set, hash_map, hash_multiset, and hash_multimap.


namespace stapl
{
/* Separating node and base so that _M_end can be initialized without
 * needing a default constructor for _Val, the value type
 * It does lead to some nasty lookin static_casts, but they're all safe
 * Thanks Tim, for pointing that one out
*/

struct _Hashtable_node_base {
  _Hashtable_node_base() : _M_next(0), _M_prev(0) {}
  _Hashtable_node_base* _M_next;
  _Hashtable_node_base* _M_prev;
};

template <class _Val>
struct _Hashtable_node: public _Hashtable_node_base {
  _Val _M_val;
};

//=========================================================
// Forward Declarations
//=========================================================

template <class _Val, class _Key, class _HashFcn,
          class _ExtractKey, class _EqualKey,
          class _Alloc >
class hashtable;

template <class _Val, class _Key, class _HashFcn,
          class _ExtractKey, class _EqualKey, class _Alloc>
struct _Hashtable_iterator;

template <class _Val, class _Key, class _HashFcn,
          class _ExtractKey, class _EqualKey, class _Alloc>
struct _Hashtable_const_iterator;

//////////////////////////////////////////////////////////////////////
/// @brief Iterator over the hashtable
/// @tparam _Val The hashtable value type.
/// @tparam _Key The hashtable's key type.
/// @tparam _HashFcn The hash function used by the hashtable.
/// @tparam _ExtractKey Functor that extracts the key from the data type.
/// @tparam _EqualKey The key equality functor. Binary predicate used to
///   compare key values of elements stored in the container for equality.
///   The default equality functor is std::equal_to<_Key>.
/// @tparam _Alloc The hashtable's allocator used for internal memory
///   management. The default allocator is std::allocator<_Tp>.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HashFcn,
          class _ExtractKey, class _EqualKey, class _Alloc>
struct _Hashtable_iterator {
  typedef hashtable<_Val,_Key,_HashFcn,_ExtractKey,_EqualKey,_Alloc>
          _Hashtable;
  typedef _Hashtable_iterator<_Val, _Key, _HashFcn,
                              _ExtractKey, _EqualKey, _Alloc>
          iterator;
  typedef _Hashtable_const_iterator<_Val, _Key, _HashFcn,
                                    _ExtractKey, _EqualKey, _Alloc>
          const_iterator;

  typedef _Hashtable_node_base _Node_base;
  typedef _Hashtable_node<_Val> _Node;


  typedef std::bidirectional_iterator_tag iterator_category;
  typedef _Val value_type;
  typedef ptrdiff_t difference_type;
  typedef size_t size_type;
  typedef _Val& reference;
  typedef _Val* pointer;

  _Node_base* _M_cur_node;
  size_type _M_cur_bkt_num;
  _Hashtable* _M_ht;

  _Hashtable_iterator(_Node_base* __n, _Hashtable* __tab)
    : _M_cur_node(__n), _M_cur_bkt_num(0), _M_ht(__tab)
    {
      if (_M_cur_node)
        _M_cur_bkt_num =
          _M_ht->_M_bkt_num(static_cast<_Node*>(_M_cur_node)->_M_val);
    }
  _Hashtable_iterator() : _M_cur_node(0), _M_cur_bkt_num(0), _M_ht(0) {}
  reference operator*() const {
    return static_cast<_Node* >(_M_cur_node)->_M_val; }
  pointer operator->() const { return &(operator*()); }
  iterator& operator++();
  iterator operator++(int);
  iterator& operator--();
  iterator operator--(int);
  bool operator==(const iterator& __it) const
    { return _M_cur_node == __it._M_cur_node; }
  bool operator!=(const iterator& __it) const
    { return _M_cur_node != __it._M_cur_node; }
};

//////////////////////////////////////////////////////////////////////
/// @copydoc _Hashtable_iterator
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HashFcn,
          class _ExtractKey, class _EqualKey, class _Alloc>
struct _Hashtable_const_iterator {
  typedef hashtable<_Val,_Key,_HashFcn,_ExtractKey,_EqualKey,_Alloc>
          _Hashtable;
  typedef _Hashtable_iterator<_Val,_Key,_HashFcn,
                              _ExtractKey,_EqualKey,_Alloc>
          iterator;
  typedef _Hashtable_const_iterator<_Val, _Key, _HashFcn,
                                    _ExtractKey, _EqualKey, _Alloc>
          const_iterator;
  typedef _Hashtable_node_base _Node_base;
  typedef _Hashtable_node<_Val> _Node;

  typedef std::bidirectional_iterator_tag iterator_category;
  typedef _Val value_type;
  typedef ptrdiff_t difference_type;
  typedef size_t size_type;
  typedef const _Val& reference;
  typedef const _Val* pointer;

  const _Node_base* _M_cur_node;
  size_type _M_cur_bkt_num;
  const _Hashtable* _M_ht;

  _Hashtable_const_iterator(const _Node_base* __n, const _Hashtable* __tab)
    : _M_cur_node(__n), _M_cur_bkt_num(0), _M_ht(__tab)
    {
      if (_M_cur_node)
        _M_cur_bkt_num =
          _M_ht->_M_bkt_num(static_cast<const _Node*>(_M_cur_node)->_M_val);
    }
  _Hashtable_const_iterator() : _M_cur_node(0), _M_cur_bkt_num(0), _M_ht(0)  {}
  _Hashtable_const_iterator(const iterator& __it)
    : _M_cur_node(__it._M_cur_node), _M_cur_bkt_num(__it._M_cur_bkt_num),
      _M_ht(__it._M_ht) {}
  reference operator*() const {
    return static_cast<const _Node* >(_M_cur_node)->_M_val; }
  pointer operator->() const { return &(operator*()); }
  const_iterator& operator++();
  const_iterator operator++(int);
  const_iterator& operator--();
  const_iterator operator--(int);
  bool operator==(const const_iterator& __it) const
    { return _M_cur_node == __it._M_cur_node; }
  bool operator!=(const const_iterator& __it) const
    { return _M_cur_node != __it._M_cur_node; }
};

/// @note assumes long is at least 32 bits.
enum { __stl_num_primes = 28 };

static const unsigned long __stl_prime_list[__stl_num_primes] =
{
  53ul,         97ul,         193ul,       389ul,       769ul,
  1543ul,       3079ul,       6151ul,      12289ul,     24593ul,
  49157ul,      98317ul,      196613ul,    393241ul,    786433ul,
  1572869ul,    3145739ul,    6291469ul,   12582917ul,  25165843ul,
  50331653ul,   100663319ul,  201326611ul, 402653189ul, 805306457ul,
  1610612741ul, 3221225473ul, 4294967291ul
};

//////////////////////////////////////////////////////////////////////
/// @brief Returns the next prime number from a list of prime numbers.
/// @param __n The current prime number
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
inline unsigned long __stl_next_prime(unsigned long __n)
{
  const unsigned long* __first = __stl_prime_list;
  const unsigned long* __last = __stl_prime_list + (int)__stl_num_primes;
  const unsigned long* pos = std::lower_bound(__first, __last, __n);
  return pos == __last ? *(__last - 1) : *pos;
}


template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
class hashtable;

//////////////////////////////////////////////////////////////////////
/// @note Forward declaration of equality operator.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
bool operator==(const hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>& __ht1,
                const hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>& __ht2);


//////////////////////////////////////////////////////////////////////
/// @brief The Hashtable  containing all the elements
///
/// [Changes made from SGI Hashtable:]
/// [_Hashtable_node is now bidirectional.]
/// [ -- Operators added]
/// [_Node** used rather than Vector<_Node*, alloc> for array of buckets]
/// [_M_bucket_allocator used for bucket memory management]
/// [Used in _M_delete/new_buckets functions]
/// @note Hashtables handle allocators a bit differently than other containers
///   do.  If we're using standard-conforming allocators, then a hashtable
///   unconditionally has a member variable to hold its allocator, even if
///   it so happens that all instances of the allocator type are identical.
///   This is because, for hashtables, this extra storage is negligible.
///   Additionally, a base class wouldn't serve any other purposes; it
///   wouldn't, for example, simplify the exception-handling code.
/// @tparam _Val The hashtable value type.
/// @tparam _Key The hashtable's key type.
/// @tparam _HashFcn The hash function used by the hashtable.
/// @tparam _ExtractKey Functor that extracts the key from the data type.
/// @tparam _EqualKey The key equality functor. Binary predicate used to
///   compare key values of elements stored in the container for equality.
///   The default equality functor is std::equal_to<_Key>.
/// @tparam _Alloc The hashtable's allocator used for internal memory
///   management. The default allocator is std::allocator<_Tp>.
/// @ingroup utility
/// @todo Re-evaluate the use of this class in STAPL.  It is provided
///   here to allow bidirectional iteration through the elements in the hash
///   table.  The requirement that bContainers of STAPL containers require
///   bidirectional iteration is something to re-evaluate as well, as we may
///   be able to at least eliminate the requirement on associative and
///   relational containers.
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HashFcn,
          class _ExtractKey, class _EqualKey, class _Alloc>
class hashtable {
public:
  typedef _Key key_type;
  typedef _Val value_type;
  typedef _HashFcn hasher;
  typedef _EqualKey key_equal;

  typedef size_t            size_type;
  typedef ptrdiff_t         difference_type;
  typedef value_type*       pointer;
  typedef const value_type* const_pointer;
  typedef value_type&       reference;
  typedef const value_type& const_reference;

  hasher hash_funct() const { return _M_hash; }
  key_equal key_eq() const { return _M_equals; }

protected:
  typedef _Hashtable_node_base _Node_base;
  typedef _Hashtable_node<_Val> _Node;

public:
  typedef _Alloc allocator_type;
  allocator_type get_allocator() const { return allocator_type(); }

protected:
  hasher                _M_hash;
  key_equal             _M_equals;
  _ExtractKey           _M_get_key;
  _Node_base**          _M_buckets;
  size_type             _M_num_buckets;
  size_type             _M_num_elements;

  typedef typename allocator_type::template rebind<_Node>::other
                                             _M_node_allocator_type;

  _M_node_allocator_type _M_node_allocator;
  _Node* _M_get_node() { return _M_node_allocator.allocate(1); }
  void _M_put_node(_Node* __p) { _M_node_allocator.deallocate(__p, 1); }


  typedef typename _M_node_allocator_type::template rebind<_Node_base*>::other
                                              _M_bucket_allocator_type;

  _M_bucket_allocator_type _M_bucket_allocator;
  _Node_base** _M_get_buckets(size_type __n)
    { return _M_bucket_allocator.allocate(__n); }
  void _M_put_buckets(_Node_base** __p, size_type __n)
    { _M_bucket_allocator.deallocate(__p, __n); }

  _Node_base*               _M_end;

public:
  typedef _Hashtable_iterator<_Val,_Key,_HashFcn,_ExtractKey,_EqualKey,_Alloc>
          iterator;
  typedef _Hashtable_const_iterator<_Val,_Key,_HashFcn,_ExtractKey,_EqualKey,
                                    _Alloc>
          const_iterator;

  friend struct
  _Hashtable_iterator<_Val,_Key,_HashFcn,_ExtractKey,_EqualKey,_Alloc>;
  friend struct
  _Hashtable_const_iterator<_Val,_Key,_HashFcn,_ExtractKey,_EqualKey,_Alloc>;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hashtable with a specific number of buckets,
  ///   a hasher, a comparison object, a key extractor functor
  ///   and an allocator.
  /// @param __n The number of buckets desired.
  /// @param __hf The hasher functor.
  /// @param __eql The comparison object.
  /// @param __ext The key extractor object.
  /// @param __a The allocator. By default a new one is constructed.
  //////////////////////////////////////////////////////////////////////
  hashtable(size_type __n,
            const _HashFcn&    __hf,
            const _EqualKey&   __eql,
            const _ExtractKey& __ext,
            const allocator_type& __a = allocator_type())
    : _M_hash(__hf),
      _M_equals(__eql),
      _M_get_key(__ext),
      _M_node_allocator(__a),
      _M_bucket_allocator(__a),
      _M_num_elements(0),
      _M_end(0)
  {
    _M_initialize_buckets(__n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a hashtable with a specific number of buckets,
  ///   a hasher, a comparison object, a key extractor functor
  ///   and an allocator.
  /// @param __n The number of buckets desired.
  /// @param __hf The hasher functor.
  /// @param __eql The comparison object.
  /// @param __a The allocator. By default a new one is constructed.
  //////////////////////////////////////////////////////////////////////
  hashtable(size_type __n,
            const _HashFcn&    __hf,
            const _EqualKey&   __eql,
            const allocator_type& __a = allocator_type())
    : _M_hash(__hf),
      _M_equals(__eql),
      _M_get_key(_ExtractKey()),
      _M_num_buckets(0),
      _M_num_elements(0),
      _M_node_allocator(__a),
      _M_bucket_allocator(__a),
      _M_end(0)
  {
    _M_initialize_buckets(__n);
  }

  hashtable(const hashtable& __ht)
    : _M_hash(__ht._M_hash),
      _M_equals(__ht._M_equals),
      _M_get_key(__ht._M_get_key),
      _M_num_buckets(0),
      _M_num_elements(0),
      _M_node_allocator(__ht.get_allocator()),
      _M_bucket_allocator(__ht.get_allocator()),
      _M_end(__ht.end())
  {
    _M_copy_from(__ht);
  }

  hashtable& operator= (const hashtable& __ht)
  {
    if (&__ht != this) {
      clear();
      _M_hash = __ht._M_hash;
      _M_equals = __ht._M_equals;
      _M_get_key = __ht._M_get_key;
      _M_copy_from(__ht);
    }
    return *this;
  }

  ~hashtable() {
    clear();
    _M_delete_buckets(_M_buckets, _M_num_buckets);
    _M_put_node(static_cast<_Node*>(_M_end));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the container.
  //////////////////////////////////////////////////////////////////////
  size_type size() const { return _M_num_elements; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the maximum number of elements that can be held
  ///   in the container.
  //////////////////////////////////////////////////////////////////////
  size_type max_size() const { return size_type(-1); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the container is empty or not.
  //////////////////////////////////////////////////////////////////////
  bool empty() const { return size() == 0; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Exchange the content of the container with another one.
  /// @param __ht The container which swaps elements of the same type.
  //////////////////////////////////////////////////////////////////////
  void swap(hashtable& __ht)
  {
    std::swap(_M_hash, __ht._M_hash);
    std::swap(_M_equals, __ht._M_equals);
    std::swap(_M_get_key, __ht._M_get_key);

    std::swap(_M_num_buckets, __ht._M_num_buckets);
    std::swap(_M_buckets, __ht._M_buckets);

    std::swap(_M_end, __ht._M_end);

    std::swap(_M_num_elements, __ht._M_num_elements);
  }

  iterator begin()
  {
    for (size_type __n = 0; __n < _M_num_buckets; ++__n)
      if (_M_buckets[__n])
        return iterator(_M_buckets[__n], this);
    return end();
  }

  iterator end()
  {
    iterator tmp( 0, this);
    tmp._M_cur_node = _M_end;
    tmp._M_cur_bkt_num = _M_num_buckets;
    return tmp;
  }

  const_iterator begin() const
  {
    for (size_type __n = 0; __n < _M_num_buckets; ++__n)
      if (_M_buckets[__n])
        return const_iterator(_M_buckets[__n], this);
    return end();
  }

  const_iterator end() const
  {
    const_iterator tmp( 0, this);
    tmp._M_cur_node = _M_end;
    tmp._M_cur_bkt_num = _M_num_buckets;
    return tmp;
  }
  template <class _Vl, class _Ky, class _HF, class _Ex, class _Eq, class _Al>
  friend bool operator== (const hashtable<_Vl, _Ky, _HF, _Ex, _Eq, _Al>&,
                          const hashtable<_Vl, _Ky, _HF, _Ex, _Eq, _Al>&);

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of buckets used by the container.
  //////////////////////////////////////////////////////////////////////
  size_type bucket_count() const { return _M_num_buckets; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the maximum number of buckets that may be used by
  ///   the container.
  //////////////////////////////////////////////////////////////////////
  size_type max_bucket_count() const
    { return __stl_prime_list[(int)__stl_num_primes - 1]; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in a specific bucket.
  //////////////////////////////////////////////////////////////////////
  size_type elems_in_bucket(size_type __bucket) const
  {
    size_type __result = 0;
    for (_Node_base* __cur =
      _M_buckets[__bucket]; __cur; __cur = __cur->_M_next)
      __result += 1;
    return __result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts a new element in the container.
  /// @param __obj The element to be inserted.
  /// @return A pair containing an iterator pointing to the element whose key
  ///   is the same as the key of the element to insert, and a bool value that
  ///   indicates whether the element was successfully inserted or not.
  //////////////////////////////////////////////////////////////////////
  std::pair<iterator, bool> insert_unique(const value_type& __obj)
  {
    resize(_M_num_elements + 1);
    return insert_unique_noresize(__obj);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts a new element in the container. If other elements with the
  ///   same key already are in the container, it inserts the new element after
  ///   the first element with the same key.
  /// @param __obj The element to be inserted.
  /// @return An iterator pointing to the newly inserted element
  //////////////////////////////////////////////////////////////////////
  iterator insert_equal(const value_type& __obj)
  {
    resize(_M_num_elements + 1);
    return insert_equal_noresize(__obj);
  }

  std::pair<iterator, bool> insert_unique_noresize(const value_type& __obj);
  iterator insert_equal_noresize(const value_type& __obj);

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts new elements in the container.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  void insert_unique(_InputIterator __f, _InputIterator __l)
  {
    insert_unique(__f, __l, __ITERATOR_CATEGORY(__f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts a new element in the container. If other elements with the
  ///   same key already are in the container, it inserts the new element after
  ///   the first element with the same key.
  /// @param __f, __l Input iterators delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  void insert_equal(_InputIterator __f, _InputIterator __l)
  {
    insert_equal(__f, __l, __ITERATOR_CATEGORY(__f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of insert_unique for input iterators.
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  void insert_unique(_InputIterator __f, _InputIterator __l,
                     std::input_iterator_tag)
  {
    for ( ; __f != __l; ++__f)
      insert_unique(*__f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of insert_equal for input iterators.
  //////////////////////////////////////////////////////////////////////
  template <class _InputIterator>
  void insert_equal(_InputIterator __f, _InputIterator __l,
                    std::input_iterator_tag)
  {
    for ( ; __f != __l; ++__f)
      insert_equal(*__f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of insert_unique for bidirectional iterators.
  //////////////////////////////////////////////////////////////////////
  template <class _ForwardIterator>
  void insert_unique(_ForwardIterator __f, _ForwardIterator __l,
                     std::bidirectional_iterator_tag)
  {
    size_type __n = 0;
    distance(__f, __l, __n);
    resize(_M_num_elements + __n);
    for ( ; __n > 0; --__n, ++__f)
      insert_unique_noresize(*__f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of insert_equal for bidirectional iterators.
  //////////////////////////////////////////////////////////////////////
  template <class _ForwardIterator>
  void insert_equal(_ForwardIterator __f, _ForwardIterator __l,
                    std::bidirectional_iterator_tag)
  {
    size_type __n = 0;
    distance(__f, __l, __n);
    resize(_M_num_elements + __n);
    for ( ; __n > 0; --__n, ++__f)
      insert_equal_noresize(*__f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts new elements in the container.
  /// @param __f, __l Pointers delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  void insert_unique(const value_type* __f, const value_type* __l)
  {
    size_type __n = __l - __f;
    resize(_M_num_elements + __n);
    for ( ; __n > 0; --__n, ++__f)
      insert_unique_noresize(*__f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts a new element in the container. If other elements with the
  ///   same key already are in the container, it inserts the new element after
  ///   the first element with the same key.
  /// @param __f, __l Pointers delimiting a range of elements to insert.
  ///   The range of elements is [__f,__l).
  //////////////////////////////////////////////////////////////////////
  void insert_equal(const value_type* __f, const value_type* __l)
  {
    size_type __n = __l - __f;
    resize(_M_num_elements + __n);
    for ( ; __n > 0; --__n, ++__f)
      insert_equal_noresize(*__f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc insert_unique(_InputIterator __f, _InputIterator __l)
  //////////////////////////////////////////////////////////////////////
  void insert_unique(const_iterator __f, const_iterator __l)
  {
    size_type __n = 0;
    distance(__f, __l, __n);
    resize(_M_num_elements + __n);
    for ( ; __n > 0; --__n, ++__f)
      insert_unique_noresize(*__f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc insert_equal(_InputIterator __f, _InputIterator __l)
  //////////////////////////////////////////////////////////////////////
  void insert_equal(const_iterator __f, const_iterator __l)
  {
    size_type __n = 0;
    distance(__f, __l, __n);
    resize(_M_num_elements + __n);
    for ( ; __n > 0; --__n, ++__f)
      insert_equal_noresize(*__f);
  }

  reference find_or_insert(const value_type& __obj);

  //////////////////////////////////////////////////////////////////////
  /// @brief Search for a specific element in the container.
  /// @param __key The key indexing the element.
  /// @return Iterator to the element, or to the end if no element is found.
  //////////////////////////////////////////////////////////////////////
  iterator find(const key_type& __key)
  {
    size_type __n = _M_bkt_num_key(__key);
    _Node_base* __first = _M_buckets[__n];;

    while(__first)
    {
      if(_M_equals(_M_get_key(static_cast<_Node* >(__first)->_M_val), __key))
        return iterator(__first, this);
      __first = __first->_M_next;
    }

    return end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc find(const key_type& __key)
  //////////////////////////////////////////////////////////////////////
  const_iterator find(const key_type& __key) const
  {
    size_type __n = _M_bkt_num_key(__key);
    const _Node_base* __first = _M_buckets[__n];

    while(__first)
    {
      if(_M_equals(
        _M_get_key(static_cast<const _Node* >(__first)->_M_val), __key))
        return const_iterator(__first, this);
      __first = __first->_M_next;
    }

    return end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements with the specified key.
  /// @param __key The key indexing the element.
  //////////////////////////////////////////////////////////////////////
  size_type count(const key_type& __key) const
  {
    const size_type __n = _M_bkt_num_key(__key);
    size_type __result = 0;

    for (const _Node_base* __cur =
      _M_buckets[__n]; __cur; __cur = __cur->_M_next)
      if (_M_equals(
        _M_get_key(static_cast<const _Node* >(__cur)->_M_val), __key))
        ++__result;
    return __result;
  }

  std::pair<iterator, iterator>
  equal_range(const key_type& __key);

  std::pair<const_iterator, const_iterator>
  equal_range(const key_type& __key) const;

  size_type erase(const key_type& __key);
  void erase(const iterator& __it);
  void erase(iterator __first, iterator __last);

  void erase(const const_iterator& __it);
  void erase(const_iterator __first, const_iterator __last);

  void resize(size_type __num_elements_hint);
  void clear();

  size_type memory_size(void) const;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the first prime number following a given number.
  /// @param __n The current number.
  //////////////////////////////////////////////////////////////////////
  size_type _M_next_size(size_type __n) const
    { return __stl_next_prime(__n); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes a given number of buckets. It will initialize
  ///   a prime number of buckets with _M_next_size(size_type __n).
  /// @param __n The desired number of buckets.
  //////////////////////////////////////////////////////////////////////
  void _M_initialize_buckets(size_type __n)
  {
    _M_end = _M_get_node();
    _M_end->_M_next=0;
    _M_end->_M_prev=0;

    _M_num_buckets = _M_next_size(__n);
    _M_buckets = _M_new_buckets(_M_num_buckets);
    _M_num_elements = 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the bucket where a particular element should
  ///   be.
  /// @param __key The key indexing the element.
  //////////////////////////////////////////////////////////////////////
  size_type _M_bkt_num_key(const key_type& __key) const
  {
    return _M_bkt_num_key(__key, _M_num_buckets);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the bucket where a particular element should
  ///   be.
  /// @param __obj The element.
  //////////////////////////////////////////////////////////////////////
  size_type _M_bkt_num(const value_type& __obj) const
  {
    return _M_bkt_num_key(_M_get_key(__obj));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the bucket where a particular element should
  ///   be.
  /// @param __key The key indexing the element.
  /// @param __n The total number of buckets.
  //////////////////////////////////////////////////////////////////////
  size_type _M_bkt_num_key(const key_type& __key, size_t __n) const
  {
    return _M_hash(__key) % __n;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the bucket where a particular element should
  ///   be.
  /// @param __obj The element.
  /// @param __n The total number of buckets.
  //////////////////////////////////////////////////////////////////////
  size_type _M_bkt_num(const value_type& __obj, size_t __n) const
  {
    return _M_bkt_num_key(_M_get_key(__obj), __n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initialize a new node for a given element
  /// @param __obj The element.
  /// @return A pointer to the _Node_base containing the element
  //////////////////////////////////////////////////////////////////////
  _Node_base* _M_new_node(const value_type& __obj)
  {
    _Node_base* __n = _M_get_node();
    __n->_M_next = 0;
    __n->_M_prev = 0;
    try {
      new ((void*)&static_cast<_Node* >(__n)->_M_val) value_type(__obj);
      return __n;
    }
    catch(...) {
      _M_put_node(static_cast<_Node*>(__n));
      throw;
      }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a node.
  /// @param __n A pointer to the node to delete.
  //////////////////////////////////////////////////////////////////////
  void _M_delete_node(_Node_base* __n)
  {
    (&static_cast<_Node* >(__n)->_M_val)->value_type::~value_type();
    _M_put_node(static_cast<_Node*>(__n));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a list of nodes.
  /// @param __p A pointer to the first _Node_base pointer.
  /// @param __num The size of the nodes' list.
  //////////////////////////////////////////////////////////////////////
  void _M_delete_nodes(_Node_base** __p, size_type __num)
  {
    for(size_type i=0; i<__num; i++)
    {
      _Node_base* __n = __p[i];
      while(__n)
      {
        _Node_base* tmp = __n;
        __n = __n->_M_next;
        _M_delete_node(static_cast<_Node*>(tmp));
      }
      __p[i] = 0;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increases the number of buckets by a specified integer.
  /// @param __n The number of buckets to create.
  //////////////////////////////////////////////////////////////////////
  _Node_base** _M_new_buckets(const size_type __n)
  {
    _Node_base** __p = _M_get_buckets(__n + 1);
    std::fill(__p, __p + __n, (_Node_base*) 0);
    __p[__n] = _M_end;
    return __p;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a specified number of buckets
  /// @param __p The pointer to the starting bucket to be deleted.
  /// @param __n The number of buckets to delete.
  //////////////////////////////////////////////////////////////////////
  void _M_delete_buckets(_Node_base** __p, const size_type __n)
  {
    //Delete all nodes but the last (We want to preserve _M_end)
    _M_delete_nodes(__p, __n);
    //Then deallocate memory for all buckets
    _M_put_buckets(__p, __n + 1);
  }

  void _M_erase_bucket(const size_type __n, _Node_base* __first,
    _Node_base* __last);
  void _M_erase_bucket(const size_type __n, _Node_base* __last);

  void _M_copy_from(const hashtable& __ht);
};


template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator++()
{
  _M_cur_node = _M_cur_node->_M_next;
  if (!_M_cur_node) {
    while (!_M_cur_node && ++_M_cur_bkt_num <= _M_ht->_M_num_buckets)
      _M_cur_node = _M_ht->_M_buckets[_M_cur_bkt_num];
  }
  return *this;
}

template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator++(int)
{
  iterator __tmp = *this;
  ++*this;
  return __tmp;
}

template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator++()
{
  _M_cur_node = _M_cur_node->_M_next;
  if (!_M_cur_node) {
    while (!_M_cur_node && ++_M_cur_bkt_num <= _M_ht->_M_num_buckets)
      _M_cur_node = _M_ht->_M_buckets[_M_cur_bkt_num];
  }
  return *this;
}

template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator++(int)
{
  const_iterator __tmp = *this;
  ++*this;
  return __tmp;
}

//////////////////////////////////////////////////////////////////////
/// @todo When we go in the previous bucket, we need to start at the end of
///   the list. The second While loop through a bucket could be made O(1) by
///   making the bidirectional list circular (i.e., head's prev is tail).
///   But means some reworking throughout the entire iterator and the hashtable
///   implementation.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator--()
{
  _M_cur_node = _M_cur_node->_M_prev;
  if (!_M_cur_node) {
    // Find a bucket with elements
    while (!_M_cur_node && _M_cur_bkt_num-1 != size_type(-1))
      _M_cur_node = _M_ht->_M_buckets[--_M_cur_bkt_num];

    // There isn't a bucket with elements.  Return, because we've reached the
    // start of the container.
    if (!_M_cur_node && _M_cur_bkt_num-1 == size_type(-1))
      return *this;

    while (_M_cur_node->_M_next)
      _M_cur_node = _M_cur_node->_M_next;
  }
  return *this;
}

template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>
_Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator--(int)
{
  iterator __tmp = *this;
  --*this;
  return __tmp;
}

//////////////////////////////////////////////////////////////////////
/// @todo When we go in the previous bucket, we need to start at the end of
///   the list. The second While loop through a bucket could be made O(1) by
///   making the bidirectional list circular (i.e., head's prev is tail).
///   But means some reworking throughout the entire iterator and the hashtable
///   implementation.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator--()
{
  _M_cur_node = _M_cur_node->_M_prev;
  if (!_M_cur_node) {
    // Find a bucket with elements
    while (!_M_cur_node && _M_cur_bkt_num-1 != size_type(-1))
      _M_cur_node = _M_ht->_M_buckets[--_M_cur_bkt_num];

    while (_M_cur_node->_M_next)
      _M_cur_node = _M_cur_node->_M_next;
  }
  return *this;
}

template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>
_Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>::operator--(int)
{
  const_iterator __tmp = *this;
  --*this;
  return __tmp;
}


template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
std::bidirectional_iterator_tag
iterator_category(const _Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&)
{
  return std::bidirectional_iterator_tag();
}

//////////////////////////////////////////////////////////////////////
/// @brief Converts an iterator to a null pointer of pointer-to-value_type.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Val* value_type(const _Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&)
{
  return (_Val*) 0;
}

//////////////////////////////////////////////////////////////////////
/// @brief Converts an iterator to a null pointer of
///   pointer-to-difference_type.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
typename hashtable<_Val,_Key,_HF,_ExK,_EqK,_All>::difference_type*
distance_type(const _Hashtable_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&)
{
  return (typename hashtable<_Val,_Key,_HF,_ExK,_EqK,_All>::difference_type*)0;
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns the iterator category tag of the hashtable :
///   std::bidirectional_iterator_tag
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
std::bidirectional_iterator_tag
iterator_category(const _Hashtable_const_iterator<_Val,_Key,_HF,
                                                  _ExK,_EqK,_All>&)
{
  return std::bidirectional_iterator_tag();
}

//////////////////////////////////////////////////////////////////////
/// @brief Converts an iterator to a null pointer of pointer-to-value_type.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
_Val*
value_type(const _Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&)
{
  return (_Val*) 0;
}

//////////////////////////////////////////////////////////////////////
/// @brief Converts an iterator to a null pointer of
///   pointer-to-difference_type.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _ExK, class _EqK,
          class _All>
typename hashtable<_Val,_Key,_HF,_ExK,_EqK,_All>::difference_type*
distance_type(const _Hashtable_const_iterator<_Val,_Key,_HF,_ExK,_EqK,_All>&)
{
  return (typename hashtable<_Val,_Key,_HF,_ExK,_EqK,_All>::difference_type*)0;
}


template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
bool operator==(const hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>& __ht1,
                const hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>& __ht2)
{
  typedef typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::_Node_base
    _Node_base;
  if (__ht1._M_num_buckets != __ht2._M_num_buckets)
    return false;
  for (int __n = 0; __n < __ht1._M_num_buckets; ++__n) {
    _Node_base* __cur1 = __ht1._M_buckets[__n];
    _Node_base* __cur2 = __ht2._M_buckets[__n];
    for ( ; __cur1 && __cur2 && __cur1->_M_val == __cur2->_M_val;
          __cur1 = __cur1->_M_next, __cur2 = __cur2->_M_next)
      {}
    if (__cur1 || __cur2)
      return false;
  }
  return true;
}


template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
bool operator!=(const hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>& __ht1,
                       const hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>& __ht2) {
  return !(__ht1 == __ht2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Swaps the content of two hashtable.
/// @param __ht1,__ht2 The two hashtable.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Extract, class _EqKey,
          class _All>
void swap(hashtable<_Val, _Key, _HF, _Extract, _EqKey, _All>& __ht1,
                 hashtable<_Val, _Key, _HF, _Extract, _EqKey, _All>& __ht2) {
  __ht1.swap(__ht2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Inserts a new element in the container without resizing the buckets.
/// @param __obj The element to be inserted.
/// @return A pair containing an iterator pointing to the element whose key
///   is the same as the key of the element to insert, and a bool value that
///   indicates whether the element was successfully inserted or not.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
std::pair<typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::iterator, bool>
hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::insert_unique_noresize(const value_type& __obj)
{
  const size_type __n = _M_bkt_num(__obj);
  _Node_base* __first = _M_buckets[__n];

  for (_Node_base* __cur = __first; __cur; __cur = __cur->_M_next)
    if (_M_equals(
      _M_get_key(static_cast<_Node* >(__cur)->_M_val), _M_get_key(__obj)))
      return std::pair<iterator, bool>(iterator(__cur, this), false);

  _Node_base* __tmp = _M_new_node(__obj);
  __tmp->_M_next = __first;
  if(__first)
    __first->_M_prev = __tmp;
  _M_buckets[__n] = __tmp;
  ++_M_num_elements;
  return std::pair<iterator, bool>(iterator(__tmp, this), true);
}

//////////////////////////////////////////////////////////////////////
/// @brief Inserts a new element in the container without resizing the buckets.
///   If other elements with the same key already are in the container, it
///   inserts the new element after the first element with the same key.
/// @param __obj The element to be inserted.
/// @return An iterator pointing to the newly inserted element
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::iterator
hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::insert_equal_noresize(const value_type& __obj)
{
  const size_type __n = _M_bkt_num(__obj);
  _Node_base* __first = _M_buckets[__n];

  for (_Node_base* __cur = __first; __cur; __cur = __cur->_M_next)
    if (_M_equals(
      _M_get_key(static_cast<_Node* >(__cur)->_M_val), _M_get_key(__obj))) {
      _Node_base* __tmp = _M_new_node(__obj);
      __tmp->_M_next = __cur->_M_next;
      __tmp->_M_prev = __cur;
      if(__cur->_M_next)
        __cur->_M_next->_M_prev = __tmp;
      __cur->_M_next = __tmp;
      ++_M_num_elements;
      return iterator(__tmp, this);
    }

  _Node_base* __tmp = _M_new_node(__obj);
  __tmp->_M_next = __first;
  if(__first)
    __first->_M_prev = __tmp;
  _M_buckets[__n] = __tmp;
  ++_M_num_elements;
  return iterator(__tmp, this);
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a reference to a searched object. If the object was not
///   found, inserts the object in the hashtable
/// @param __obj The element to be found or inserted.
/// @return A reference to the object that was found or newly inserted.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::reference
hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::find_or_insert(const value_type& __obj)
{
  resize(_M_num_elements + 1);

  size_type __n = _M_bkt_num(__obj);
  _Node_base* __first = _M_buckets[__n];

  for (_Node_base* __cur = __first; __cur; __cur = __cur->_M_next)
    if (_M_equals(
      _M_get_key(static_cast<_Node* >(__cur)->_M_val), _M_get_key(__obj)))
      return static_cast<_Node* >(__cur)->_M_val;

  _Node_base* __tmp = _M_new_node(__obj);
  __tmp->_M_next = __first;
  if(__first)
    __first->_M_prev = __tmp;
  _M_buckets[__n] = __tmp;
  ++_M_num_elements;
  return static_cast<_Node* >(__tmp)->_M_val;
}

//////////////////////////////////////////////////////////////////////
/// @brief Return a range that includes all the elements with a specific key.
/// @param __key The key indexing the element.
/// @return Pair of iterators delimiting a range of elements
///   The range of elements is [pair's first, pair's second).
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
std::pair<typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::iterator,
     typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::iterator>
hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::equal_range(const key_type& __key)
{
  typedef std::pair<iterator, iterator> _Pii;
  const size_type __n = _M_bkt_num_key(__key);

  for (
    _Node_base* __first = _M_buckets[__n]; __first; __first = __first->_M_next)
    if (_M_equals(_M_get_key(static_cast<_Node* >(__first)->_M_val), __key)) {
      for (_Node_base* __cur = __first->_M_next; __cur; __cur = __cur->_M_next)
        if (!_M_equals(_M_get_key(static_cast<_Node* >(__cur)->_M_val), __key))
          return _Pii(iterator(__first, this), iterator(__cur, this));
      for (size_type __m = __n + 1; __m < _M_num_buckets; ++__m)
        if (_M_buckets[__m])
          return _Pii(iterator(__first, this),
                     iterator(_M_buckets[__m], this));
      return _Pii(iterator(__first, this), end());
    }
  return _Pii(end(), end());
}

//////////////////////////////////////////////////////////////////////
/// @copydoc equal_range(const key_type&)
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
std::pair<typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::const_iterator,
     typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::const_iterator>
hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::equal_range(const key_type& __key) const
{
  typedef std::pair<const_iterator, const_iterator> _Pii;
  const size_type __n = _M_bkt_num_key(__key);

  for (const _Node_base* __first = _M_buckets[__n] ;
       __first;
       __first = __first->_M_next) {
    if (_M_equals(
      _M_get_key(static_cast<const _Node* >(__first)->_M_val), __key)) {
      for (const _Node_base* __cur = __first->_M_next;
           __cur;
           __cur = __cur->_M_next)
        if (!_M_equals(
          _M_get_key(static_cast<const _Node* >(__cur)->_M_val), __key))
          return _Pii(const_iterator(__first, this),
                      const_iterator(__cur, this));
      for (size_type __m = __n + 1; __m < _M_num_buckets; ++__m)
        if (_M_buckets[__m])
          return _Pii(const_iterator(__first, this),
                      const_iterator(_M_buckets[__m], this));
      return _Pii(const_iterator(__first, this), end());
    }
  }
  return _Pii(end(), end());
}

//////////////////////////////////////////////////////////////////////
/// @brief Removes all elements with the specified key from the container.
/// @param __key The key indexing the element.
/// @return The number of elements removed from the container.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::size_type
hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::erase(const key_type& __key)
{
  const size_type __n = _M_bkt_num_key(__key);
  _Node_base* __first = _M_buckets[__n];
  size_type __erased = 0;

  if (__first) {
    _Node_base* __cur = __first;
    _Node_base* __next = __cur->_M_next;
    while (__next) {
      if (_M_equals(_M_get_key(static_cast<_Node* >(__next)->_M_val), __key)) {
        __cur->_M_next = __next->_M_next;
        _M_delete_node(__next);
        if(__cur->_M_next)
            __cur->_M_next->_M_prev = __cur;
        __next = __cur->_M_next;
        ++__erased;
        --_M_num_elements;
      }
      else {
        __cur = __next;
        __next = __cur->_M_next;
      }
    }
    if (_M_equals(_M_get_key(static_cast<_Node* >(__first)->_M_val), __key)) {
      _M_buckets[__n] = __first->_M_next;
      if(__first->_M_next)
        __first->_M_next->_M_prev = 0; //nothing comes before the first node
      _M_delete_node(__first);
      ++__erased;
      --_M_num_elements;
    }
  }
  return __erased;
}

//////////////////////////////////////////////////////////////////////
/// @brief Removes an element pointed by an iterator from the container.
/// @param __it The iterator over the element to delete.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::erase(const iterator& __it)
{
  _Node_base* __p = __it._M_cur_node;
  if (__p) {
    const size_type __n = __it._M_cur_bkt_num;
    _Node_base* __cur = _M_buckets[__n];

    if (__cur == __p) {
      _M_buckets[__n] = __cur->_M_next;
      if(__cur->_M_next)
        __cur->_M_next->_M_prev = 0;
      _M_delete_node(__cur);
      --_M_num_elements;
    }
    else {
      _Node_base* __next = __cur->_M_next;
      while (__next) {
        if (__next == __p) {
          __cur->_M_next = __next->_M_next;
          if(__cur->_M_next)
            __cur->_M_next->_M_prev = __cur;
          _M_delete_node(__next);
          --_M_num_elements;
          break;
        }
        else {
          __cur = __next;
          __next = __cur->_M_next;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Removes some elements pointed by an iterator from the container.
/// @param __first, __last Input iterators delimiting a range of elements to
///   delete. The range of elements is [__first,__last).
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::erase(iterator __first, iterator __last)
{
  size_type __f_bucket = __first._M_cur_node ?
    __first._M_cur_bkt_num : _M_num_buckets;
  size_type __l_bucket = __last._M_cur_node ?
    __last._M_cur_bkt_num : _M_num_buckets;

  if (__first._M_cur_node == __last._M_cur_node)
    return;
  else if (__f_bucket == __l_bucket)
    _M_erase_bucket(__f_bucket, __first._M_cur_node, __last._M_cur_node);
  else {
    _M_erase_bucket(__f_bucket, __first._M_cur_node, 0);
    for (size_type __n = __f_bucket + 1; __n < __l_bucket; ++__n)
      _M_erase_bucket(__n, 0);
    if (__l_bucket != _M_num_buckets)
      _M_erase_bucket(__l_bucket, __last._M_cur_node);
  }
}

//////////////////////////////////////////////////////////////////////
/// @copydoc erase(iterator, iterator)
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::erase(const_iterator __first,
                                             const_iterator __last)
{
  erase(iterator(const_cast<_Node_base*>(__first._M_cur_node),
                 const_cast<hashtable*>(__first._M_ht)),
        iterator(const_cast<_Node_base*>(__last._M_cur_node),
                 const_cast<hashtable*>(__last._M_ht)));
}

//////////////////////////////////////////////////////////////////////
/// @copydoc erase(const iterator&)
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::erase(const const_iterator& __it)
{
  erase(iterator(const_cast<_Node_base*>(__it._M_cur_node),
                 const_cast<hashtable*>(__it._M_ht)));
}

//////////////////////////////////////////////////////////////////////
/// @brief Increases the bucket count to at least __hint.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::resize(size_type __num_elements_hint)
{
  const size_type __old_n = _M_num_buckets;
  if (__num_elements_hint > __old_n) {
    _M_num_buckets = _M_next_size(__num_elements_hint);
    if (_M_num_buckets > __old_n) {
      _Node_base** __tmp = _M_new_buckets(_M_num_buckets);
      try {
        //Copy old values over to new buckets with new hash-code indices
        for (size_type __bucket = 0; __bucket < __old_n; ++__bucket) {
          _Node_base* __first = _M_buckets[__bucket];
          while (__first) {
            size_type __new_bucket = _M_bkt_num(
              static_cast<_Node* >(__first)->_M_val, _M_num_buckets);
            _M_buckets[__bucket] = __first->_M_next;
            __first->_M_prev = 0;
            __first->_M_next = __tmp[__new_bucket];
            if(__tmp[__new_bucket])
              __tmp[__new_bucket]->_M_prev = __first;
            __tmp[__new_bucket] = __first;
            __first = _M_buckets[__bucket];
          }
        }
        _M_delete_buckets(_M_buckets, __old_n);
        _M_buckets = __tmp;
        _M_buckets[_M_num_buckets] = _M_end;
      }
      catch(...) {
        for (size_type __bucket = 0; __bucket < _M_num_buckets; ++__bucket) {
          while (__tmp[__bucket]) {
            _Node_base* __next = __tmp[__bucket]->_M_next;
            _M_delete_node(__tmp[__bucket]);
            __tmp[__bucket] = __next;
          }
        }
        _M_delete_buckets(__tmp, _M_num_buckets);
        throw;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Removes all the elements in a range from a specific bucket
/// @param __n The index of the bucket where the deletions will occur.
/// @param __first,__last Pointers delimiting a range of elements to delete.
///   The range of elements is [__first,__last).
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::_M_erase_bucket(
  const size_type __n, _Node_base* __first, _Node_base* __last)
{
  _Node_base* __cur = _M_buckets[__n];
  if (__cur == __first)
    _M_erase_bucket(__n, __last);
  else {
    _Node_base* __next;
    for (__next = __cur->_M_next;
         __next != __first;
         __cur = __next, __next = __cur->_M_next)
      ;
    while (__next != __last) {
      __cur->_M_next = __next->_M_next;
      _M_delete_node(__next);
      __next = __cur->_M_next;
      --_M_num_elements;
    }
    if(__next)
      __next->_M_prev = __cur;
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Removes all the elements in a range from a specific bucket
/// @param __n The index of the bucket where the deletions will occur.
/// @param __last Pointer delimiting the end of the range of elements to
///   delete. The range of elements is [begin of the hashtable,__last).
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::_M_erase_bucket(const size_type __n, _Node_base* __last)
{
  _Node_base* __cur = _M_buckets[__n];
  while (__cur != __last) {
    _Node_base* __next = __cur->_M_next;
    _M_delete_node(__cur);
    __cur = __next;
    _M_buckets[__n] = __cur;
    --_M_num_elements;
  }
  if(_M_buckets[__n])
    _M_buckets[__n]->_M_prev = 0;
}

//////////////////////////////////////////////////////////////////////
/// @brief Removes all the elements from the container.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::clear()
{
  _M_delete_nodes(_M_buckets, _M_num_buckets);
  _M_num_elements = 0;
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy all the elements from another hashtable.
/// @param __ht The hashtable to copy from.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
void hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::_M_copy_from(const hashtable& __ht)
{
  _M_num_buckets = __ht._M_num_buckets;
  _M_buckets = _M_new_buckets(_M_num_buckets);

  try {
    for (size_type __i = 0; __i < __ht._M_num_buckets; ++__i) {
      const _Node_base* __cur = __ht._M_buckets[__i];
      if (__cur) {
        _Node_base* __copy =
          _M_new_node(static_cast<const _Node* >(__cur)->_M_val);
        _M_buckets[__i] = __copy;

        for (_Node_base* __next = __cur->_M_next;
             __next;
             __cur = __next, __next = __cur->_M_next)
        {
          __copy->_M_next = _M_new_node(static_cast<_Node* >(__next)->_M_val);
          __copy->_M_next->_M_prev = __copy;
          __copy = __copy->_M_next;
        }
      }
    }
    _M_num_elements = __ht._M_num_elements;
  }
  catch(...) {
    clear();
    throw;
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns the size in the memory used by the container.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <class _Val, class _Key, class _HF, class _Ex, class _Eq, class _All>
typename hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>::size_type
hashtable<_Val,_Key,_HF,_Ex,_Eq,_All>
  ::memory_size(void) const
{
  size_type __size = 0;
  // size of Table itself
  __size += _M_num_buckets * sizeof(_Node_base*);
  // size of all elements in table (each node has pointers to prev and next,
  //   and the value)
  __size += _M_num_elements * (sizeof(_Node_base*) * 2 + sizeof(value_type));

  return __size;
  // All in one
  //return (sizeof(_Node_base*) * (_M_num_buckets + _M_num_elements*2)
  //  + sizeof(value_type) * _M_num_elements);
}


} // STAPL NAMESPACE
#endif /* __STAPL_INTERNAL_HASHTABLE_H */
