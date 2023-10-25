/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TEST_TEST_CONTAINERS_H
#define STAPL_RUNTIME_TEST_TEST_CONTAINERS_H

#include <stapl/runtime.hpp>
#include <algorithm>

// test_vector is a simple vector class.
template<typename T>
class test_vector
{
private:
  T* m_begin;
  T* m_end;

public:
  test_vector(void)
  : m_begin(0),
    m_end(0)
  { }

  test_vector(const int size)
  : m_begin(new T[size]),
    m_end(m_begin + size)
  { }

  test_vector(test_vector const& v)
  : m_begin(0),
    m_end(0)
  {
    if (v.size()!=0) {
      m_begin = new T[v.size()];
      m_end = m_begin + v.size();
      std::copy(v.m_begin, v.m_end, m_begin);
    }
  }

  virtual ~test_vector(void)
  { delete[] m_begin; }

  test_vector& operator=(test_vector const& v)
  {
    if (&v!=this) {
      delete[] m_begin;
      if( v.size() != 0 ) {
        m_begin = new T[v.size()];
        for( int i=0; i<v.size(); i++ )
          m_begin[i] = v.m_begin[i];
        m_end = m_begin + v.size();
      }
    }
    return *this;
  }

  void define_type(stapl::typer& t)
  {
    const int sz = size();
    t.member(m_begin, sz);
    t.pointer_to_member(m_end, m_begin, sz);
  }

  int size(void) const
  { return (m_end - m_begin); }

  void resize(const int s)
  {
    m_begin = new T[s];
    m_end = m_begin + s;
  }

  T& operator[](const int index)
  { return m_begin[index]; }

  T const& operator[](const int index) const
  { return m_begin[index]; }

  bool operator==(test_vector const& v) const
  {
    if (v.size()!=size())
      return false;
    for (int i=0; i<size(); ++i)
      if (m_begin[i] != v.m_begin[i])
        return false;
    return true;
  }

  bool operator!=(test_vector const& v) const
  { return !(*this==v); }
};


// test_vector2 extends test_vector for testing purposes.  It uses the dynamic
// offsets typical of a real vector, as well as the local types of offsets.
template<typename T>
class test_vector2
: public test_vector<T>
{
private:
  typedef test_vector<T> base_type;

  double* m_ptr_d;
  double  m_d;
  int     m_i[3];
  int*    m_i_ptr;

public:
  test_vector2(void)
  : m_ptr_d(&m_d),
    m_i_ptr(&m_i[1])
  { }

  test_vector2(const int size)
  : base_type(size),
    m_ptr_d(&m_d),
    m_i_ptr(&m_i[1])
  { }

  test_vector2(test_vector2 const& v)
  : base_type(v),
    m_ptr_d(&m_d),
    m_i_ptr(&m_i[1])
  { }

  test_vector2& operator=(test_vector2 const& v)
  {
    if (this!=&v) {
      base_type::operator=(v);
      std::copy(v.m_i, v.m_i+3, m_i);
    }
    return *this;
  }

  void define_type(stapl::typer& t)
  {
    t.base<base_type>(*this);
    t.member( m_d );
    t.member( m_i );
    t.pointer_to_member( m_ptr_d, &m_d );
    t.pointer_to_member( m_i_ptr, m_i, 1 );
  }

  bool check(const int s) const
  { return ((m_ptr_d==&m_d) && (m_i_ptr==&m_i[1]) && (base_type::size()==s)); }
};


// test_list is a simple doubly-linked list class.
template<typename T>
class test_list
{
private:
  struct node
  {
    T     m_elem;
    node* m_next;
    node* m_prev;

    node(const T& element, node* const next, node* const prev)
    : m_elem(element),
      m_next(next),
      m_prev(prev)
    { }

    void define_type(stapl::typer& t)
    {
      t.member(m_elem);
      // recursively define the rest of the list, until m_next == 0
      t.member(m_next);
      // as the recursion unwinds, the m_prev offsets will be valid
      t.pointer_to_member( m_prev, m_prev );
    }
  };
  node* m_head;

  struct list_iterator_impl
  {
    node* m_cur;

    explicit list_iterator_impl(node* const pos)
    : m_cur(pos)
    { }

    void operator++(void)
    { m_cur = m_cur->m_next; }

    void operator++(int)
    { m_cur = m_cur->m_next; }

    void operator--(void)
    { m_cur = m_cur->m_prev; }

    void operator--(int)
    { m_cur = m_cur->m_prev; }

    T& operator*(void)
    { return m_cur->m_elem; }

    bool operator!=(list_iterator_impl const& l) const
    { return (m_cur!=l.m_cur); }
  };

  struct const_list_iterator_impl
  {
    node* m_cur;

    explicit const_list_iterator_impl(node* const pos)
    : m_cur(pos)
    { }

    void operator++(void)
    { m_cur = m_cur->m_next; }

    void operator++(int)
    { m_cur = m_cur->m_next; }

    void operator--(void)
    { m_cur = m_cur->m_prev; }

    void operator--(int)
    { m_cur = m_cur->m_prev; }

    const T& operator*(void) const
    { return m_cur->m_elem; }

    bool operator!=(const_list_iterator_impl const& l) const
    { return (m_cur!=l.m_cur); }
  };

public:
  typedef list_iterator_impl       iterator;
  typedef const_list_iterator_impl const_iterator;

  iterator begin(void)
  { return list_iterator_impl(m_head); }

  iterator end(void)
  { return list_iterator_impl(0); }

  const_iterator begin(void) const
  { return const_list_iterator_impl(m_head); }

  const_iterator end(void) const
  { return const_list_iterator_impl(0); }

  test_list(void)
  : m_head(0)
  { }

  test_list(test_list const& l)
  : m_head(0)
  {
    for (const_iterator i=l.begin(); i!=l.end(); ++i)
      insert_back(*i);
  }

  ~test_list(void)
  {
    while (m_head!=0)
      remove_front();
  }

  test_list& operator=(test_list const& l)
  {
    if (this!=&l) {
      while (m_head!=0)
        remove_front();
      for (const_iterator i=l.begin(); i!=l.end(); ++i)
        insert_back( *i );
    }
    return *this;
  }

  void define_type(stapl::typer& t)
  { t.member(m_head); }

  int size(void) const
  {
    int sz = 0;
    for (const_iterator i=begin(); i!=end(); ++i)
      sz++;
    return sz;
  }

  void insert_front(T const& element)
  {
    if (m_head == 0) {
      m_head = new node(element, 0, 0);
      m_head->m_prev = m_head;
    }
    else {
      m_head = new node(element, m_head, m_head->m_prev);
      m_head->m_next->m_prev = m_head;
    }
  }

  void insert_back(T const& element)
  {
    if (m_head == 0) {
      m_head = new node(element, 0, 0);
      m_head->m_prev = m_head;
    }
    else {
      m_head->m_prev = new node(element, 0, m_head->m_prev);
      m_head->m_prev->m_prev->m_next = m_head->m_prev;
    }
  }

  void remove_front(void)
  {
    stapl_assert( m_head!=0, "empty, can't remove_front" );
    node* const tmp = m_head;
    m_head = m_head->m_next;
    if (m_head != 0)
      m_head->m_prev = tmp->m_prev;
    delete tmp;
  }

  bool operator==(test_list const& l) const
  {
    if (l.size() != size())
      return false;
    const_iterator i = begin();
    const_iterator iL = l.begin();
    for ( ; i!=end(); ++i, ++iL)
      if (*i != *iL)
        return false;
    return true;
  }

  bool operator!=(test_list const& l) const
  { return !(*this == l); }
};

#endif
