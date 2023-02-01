/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TEST_TEST_CLASSES_H
#define STAPL_RUNTIME_TEST_TEST_CLASSES_H

#include <stapl/runtime/serialization.hpp>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <limits>
#include <ostream>
#include <tuple>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/array.hpp>

#define COMPARISON_OPERATORS(c)                                     \
 bool operator!=(c const& rhs) const noexcept                       \
 { return !operator==(rhs); }                                       \
                                                                    \
 bool operator==(c const volatile& rhs) const volatile noexcept     \
 { return const_cast<c const&>(*this)==const_cast<c const&>(rhs); } \
                                                                    \
 bool operator!=(c const volatile& rhs) const volatile noexcept     \
 { return !operator==(rhs); }


//
// PODs with qualifier conversions
//

struct normal_struct
{
  int i;

  normal_struct(int i = 0)
  : i(i)
  { }

  normal_struct(normal_struct const&) = default;
  normal_struct(normal_struct const volatile& other)
    : i(other.i)
  { }

  bool operator==(normal_struct const& rhs) const noexcept
  { return (i==rhs.i); }

  COMPARISON_OPERATORS(normal_struct)

  void define_type(stapl::typer& t)
  { t.member(i); }
};


struct cv_struct
{
  int i;

  cv_struct(int i = 0)
  : i(i)
  { }

  cv_struct(cv_struct const&) = default;
  cv_struct(cv_struct&&) = default;

  cv_struct(cv_struct const volatile& other)
  : i(other.i)
  { }

  cv_struct(cv_struct volatile&& other)
  : i(other.i)
  { }

  bool operator==(cv_struct const& rhs) const noexcept
  { return (i==rhs.i); }

  COMPARISON_OPERATORS(cv_struct)

  void define_type(stapl::typer& t)
  { t.member(i); }
};



//
// PODs
//
class pod_stapl
{
private:
  bool   b1;
  char   c1;
  int    i1;
  bool   b2;
  float  f1;
  double d1;
  bool   b3;
  char   c2;
  double d2;
  float  f2;

public:
  pod_stapl(void) noexcept
  : b1(false), c1('\0'), i1(0), b2(false), f1(0.0f),
    d1(0.0f), b3(false), c2('\0'),  d2(0.0f), f2(0.0f)
  { }

  explicit pod_stapl(const int) noexcept
  : b1(true), c1('S'), i1(-1), b2(false), f1(42.0f),
    d1(42.42f), b3(true), c2('0'),  d2(1.0f), f2(0.1f)
  { }

  pod_stapl(pod_stapl const&) = default;

  pod_stapl(pod_stapl const volatile& o) noexcept
  : b1(o.b1), c1(o.c1), i1(o.i1), b2(o.b2), f1(o.f1),
    d1(o.d1), b3(o.b3), c2(o.c2), d2(o.d2), f2(o.f2)
  { }

  ~pod_stapl(void)
  {
    b1 = false;
    c1 = 0;
    i1 = 0;
    b2 = false;
    f1 = 0.0;
    d1 = 0.0;
    b3 = false;
    c2 = 0;
    d2 = 0.0;
    f2 = 0.0;
  }

  bool operator==(pod_stapl const& rhs) const noexcept
  {
    return (b1==rhs.b1 &&
            c1==rhs.c1 &&
            i1==rhs.i1 &&
            b2==rhs.b2 &&
            (f1-rhs.f1) < std::numeric_limits<float>::epsilon() &&
            (d1-rhs.d1) < std::numeric_limits<double>::epsilon() &&
            b3==rhs.b3 &&
            c2==rhs.c2 &&
            (d2-rhs.d2) < std::numeric_limits<double>::epsilon() &&
            (f2-rhs.f2) < std::numeric_limits<float>::epsilon());
  }

  COMPARISON_OPERATORS(pod_stapl)

  void define_type(stapl::typer& t)
  {
    t.member(b1);
    t.member(c1);
    t.member(i1);
    t.member(b2);
    t.member(f1);
    t.member(d1);
    t.member(b3);
    t.member(c2);
    t.member(d2);
    t.member(f2);
  }

  friend std::ostream& operator<<(std::ostream& os, pod_stapl const& t)
  {
    return os << '{'
              << t.b1 << ',' << t.c1 << ',' << t.i1 << ',' << t.b2 << ','
              << t.f1 << ',' << t.d1 << ',' << t.b3 << ',' << t.c2 << ','
              << t.d2 << ',' << t.f2
              << '}';
  }
};


class pod_boost
{
private:
  bool   b1;
  char   c1;
  int    i1;
  bool   b2;
  float  f1;
  double d1;
  bool   b3;
  char   c2;
  double d2;
  float  f2;

  friend class boost::serialization::access;

  template<typename Archive>
  void serialize(Archive& ar, const unsigned int)
  {
    ar & b1;
    ar & c1;
    ar & i1;
    ar & b2;
    ar & f1;
    ar & d1;
    ar & b3;
    ar & c2;
    ar & d2;
    ar & f2;
  }

public:
  pod_boost(void) noexcept
  : b1(false), c1('\0'), i1(0), b2(false), f1(0.0f),
    d1(0.0f), b3(false), c2('\0'),  d2(0.0f), f2(0.0f)
  { }

  explicit pod_boost(const int) noexcept
  : b1(true), c1('S'), i1(-1), b2(false), f1(42.0f),
    d1(42.42f), b3(true), c2('0'),  d2(1.0f), f2(0.1f)
  { }

  pod_boost(pod_boost const&) = default;
  pod_boost& operator=(pod_boost const&) = default;

  pod_boost(pod_boost const volatile& o) noexcept
  : b1(o.b1), c1(o.c1), i1(o.i1), b2(o.b2), f1(o.f1),
    d1(o.d1), b3(o.b3), c2(o.c2), d2(o.d2), f2(o.f2)
  { }

  ~pod_boost(void)
  {
    b1 = false;
    c1 = 0;
    i1 = 0;
    b2 = false;
    f1 = 0.0;
    d1 = 0.0;
    b3 = false;
    c2 = 0;
    d2 = 0.0;
    f2 = 0.0;
  }

  bool operator==(pod_boost const& rhs) const noexcept
  {
    return (b1==rhs.b1 &&
            c1==rhs.c1 &&
            i1==rhs.i1 &&
            b2==rhs.b2 &&
            (f1-rhs.f1) < std::numeric_limits<float>::epsilon() &&
            (d1-rhs.d1) < std::numeric_limits<double>::epsilon() &&
            b3==rhs.b3 &&
            c2==rhs.c2 &&
            (d2-rhs.d2) < std::numeric_limits<double>::epsilon() &&
            (f2-rhs.f2) < std::numeric_limits<float>::epsilon());
  }

  COMPARISON_OPERATORS(pod_boost)

  friend std::ostream& operator<<(std::ostream& os, pod_boost const& t)
  {
    return os << '{'
              << t.b1 << ',' << t.c1 << ',' << t.i1 << ',' << t.b2 << ','
              << t.f1 << ',' << t.d1 << ',' << t.b3 << ',' << t.c2 << ','
              << t.d2 << ',' << t.f2
              << '}';
  }
};


//
// Empty
//
struct empty
{
  bool operator==(empty const&) const noexcept
  { return true; }

  COMPARISON_OPERATORS(empty)

  friend std::ostream& operator<<(std::ostream& os, empty const&)
  {
    return os << "{}";
  }
};


//
// Class with transient member
//
class transient_type
{
private:
  int   m_i;
  void* m_anchor;

public:
  explicit transient_type(int i = 1) noexcept
  : m_i(i),
    m_anchor(this)
  { }

  transient_type(transient_type const& other) noexcept
  : m_i(other.m_i),
    m_anchor(this)
  { }

  transient_type& operator=(transient_type const& other) noexcept
  {
    m_i = other.m_i;
    return *this;
  }

  ~transient_type(void)
  {
    m_i      = 0;
    m_anchor = nullptr;
  }

  bool operator==(transient_type const& other) const noexcept
  {
    if (m_anchor!=this && m_i!=1234)
      return false;

    if (other.m_anchor!=&other && m_i!=1234)
      return false;

    return true;
  }

  COMPARISON_OPERATORS(transient_type)

  void define_type(stapl::typer& t)
  {
    t.transient(m_i, 1234);
    t.member(stapl::bitwise(m_anchor));
  }

  friend std::ostream& operator<<(std::ostream& os, transient_type const& t)
  {
    return os << '{' << t.m_i << '}';
  }
};


//
// complex stack object
//
struct stack_abstract_base
{
  friend std::ostream& operator<<(std::ostream& os, stack_abstract_base const&)
  {
    return os << "{}";
  }
};


struct stack_base1
: public stack_abstract_base
{
  int i;

  explicit stack_base1(int v = 0)
  : i(v)
  { }

  ~stack_base1(void)
  { i = 0; }

  bool operator==(stack_base1 const& rhs) const noexcept
  { return (i==rhs.i); }

  COMPARISON_OPERATORS(stack_base1)

  void define_type(stapl::typer& t)
  {
    t.base<stack_abstract_base>(*this);
    t.member(i);
  }

  friend std::ostream& operator<<(std::ostream& os, stack_base1 const& t)
  {
    return os << '{'
              << static_cast<stack_abstract_base const&>(t) << '|'
              << t.i
              << '}';
  }
};


struct stack_base2
{
  double d[2];

  explicit stack_base2(int i = 0)
  : d{double(i), double(i)}
  { }

  ~stack_base2(void)
  {
    d[0] = 0.0;
    d[1] = 0.0;
  }

  bool operator==(stack_base2 const& rhs) const noexcept
  {
    return ((d[0]-rhs.d[0]) <= std::numeric_limits<double>::epsilon()) &&
           ((d[1]-rhs.d[1]) <= std::numeric_limits<double>::epsilon());
  }

  COMPARISON_OPERATORS(stack_base2)

  void define_type(stapl::typer& t)
  { t.member(d); }

  friend std::ostream& operator<<(std::ostream& os, stack_base2 const& t)
  {
    return os << '{'
              << '[' << t.d[0] << ',' << t.d[1] << ']'
              << '}';
  }
};


struct stack_object
: public stack_base1,
  public stack_base2
{
  pod_stapl o;

  stack_object(void)
  : o(0)
  { }

  explicit stack_object(int i)
  : stack_base1(i),
    stack_base2(i),
    o(i)
  { }

  bool operator==(stack_object const& rhs) const noexcept
  {
    return ( (o==rhs.o) &&
             (static_cast<const stack_base1&>(*this)==
               static_cast<const stack_base1&>(rhs)) &&
             (static_cast<const stack_base2&>(*this)==
               static_cast<const stack_base2&>(rhs)) );
  }

  COMPARISON_OPERATORS(stack_object)

  void define_type(stapl::typer& t)
  {
    t.base<stack_base1>(*this);
    t.base<stack_base2>(*this);
    t.member(o);
  }

  friend std::ostream& operator<<(std::ostream& os, stack_object const& t)
  {
    return os << '{'
              << static_cast<stack_base1 const&>(t) << ','
              << static_cast<stack_base2 const&>(t) << ':'
              << t.o
              << '}';
  }
};


//
// complex dynamic object
//
struct dynamic_base
{
  float f;
  int*  i;

  dynamic_base(int v = -1)
  : f(float(v)),
    i(new int(v))
  { }

  dynamic_base(dynamic_base const& other)
  : f(other.f),
    i(new int(*other.i))
  { }

  dynamic_base& operator=(int v)
  {
    f  = float(v);
    *i = v;
    return *this;
  }

  dynamic_base& operator=(dynamic_base const& other)
  {
    if (this!=&other) {
      f  = other.f;
      *i = *other.i;
    }
    return *this;
  }

  dynamic_base& operator=(dynamic_base const volatile& other)
  {
    if (this!=&other) {
      f  = other.f;
      *i = *other.i;
    }
    return *this;
  }

  ~dynamic_base(void)
  {
    f = 0.0;
    delete i;
    i = nullptr;
  }

  void set(float v) noexcept
  {
    f  = v;
    *i = int(v);
  }

  bool operator==(dynamic_base const& rhs) const noexcept
  {
    return ( ((f-rhs.f)<std::numeric_limits<float>::epsilon()) &&
             (*i==*rhs.i) );
  }

  COMPARISON_OPERATORS(dynamic_base)

  bool operator<(dynamic_base const& rhs) const noexcept
  {
    if (f<rhs.f)
      return true;
    if (f>rhs.f)
      return false;
    return (*i<*(rhs.i));
  }

  void define_type(stapl::typer& t)
  {
    t.member(f);
    t.member(i);
  }

  friend std::ostream& operator<<(std::ostream& os, dynamic_base const& t)
  {
    return os << '{'
              << t.f << ','
              << *(t.i)
              << '}';
  }
};


struct dynamic_object
{
private:
  int*          i;
  dynamic_base  d[2];
  dynamic_base* d2;
  int*          p;
  dynamic_base* p2;

public:
  dynamic_object(const int v = 0)
  : i(new int(v)),
    d2(new dynamic_base[2]),
    p(i),
    p2(&d[1])
  {
    d[0].set(v);
    d[1].set(v+1);
    d2[0].set(v);
    d2[1].set(v+1);
  }

  dynamic_object(dynamic_object const& other)
  : i(new int(*other.i)),
    d2(new dynamic_base[2]),
    p(i),
    p2(&d[1])
  {
    d[0].set(*i);
    d[1].set(*i+1);
    d2[0].set(*i);
    d2[1].set(*i+1);
  }

  dynamic_object& operator=(dynamic_object const& other)
  {
    if (this!=&other) {
      *i    = *other.i;
      d[0]  = other.d[0];
      d[1]  = other.d[1];
      d2[0] = other.d2[0];
      d2[1] = other.d2[1];
    }
    return *this;
  }

  ~dynamic_object(void)
  {
    delete i;
    i = nullptr;
    d[0] = 0.0;
    d[1] = 0.0;
    delete[] d2;
    d2 = nullptr;
    p  = nullptr;
    p2 = nullptr;
  }

  bool operator==(dynamic_object const& rhs) const noexcept
  {
    return *i==*rhs.i       &&
           d[0]==rhs.d[0]   &&
           d[1]==rhs.d[1]   &&
           d2[0]==rhs.d2[0] &&
           d2[1]==rhs.d2[1] &&
           *p==*rhs.p       &&
           *p2==*rhs.p2;
  }

  COMPARISON_OPERATORS(dynamic_object)

  void define_type(stapl::typer& t)
  {
    t.member(i);
    t.member(d);
    t.member(d2, 2);
    t.pointer_to_member(p, i);
    t.pointer_to_member(p2, d, 1);
  }

  friend std::ostream& operator<<(std::ostream& os, dynamic_object const& t)
  {
    return os << '{'
              << *(t.i) << ','
              << '[' << t.d[0] << ',' << t.d[1] << "],"
              << '[' << t.d2[0] << ',' << t.d2[1] << "],"
              << *(t.p) << ','
              << *(t.p2)
              << '}';
  }
};


//
// multiple inheritance
//
class multiple_inheritance
: public stack_object,
  public dynamic_object,
  private empty
{
public:
  multiple_inheritance(void) = default;

  multiple_inheritance(int i)
  : stack_object(i),
    dynamic_object(i)
  { }

  bool operator==(multiple_inheritance const& rhs) const noexcept
  {
    return ( (static_cast<stack_object const&>(*this)==
                static_cast<stack_object const&>(rhs)) &&
             (static_cast<dynamic_object const&>(*this)==
                static_cast<dynamic_object const&>(rhs)) );
  }

  COMPARISON_OPERATORS(multiple_inheritance)

  void define_type(stapl::typer& t)
  {
    t.base<stack_object>(*this);
    t.base<dynamic_object>(*this);
    t.base<empty>(*this);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  multiple_inheritance const& t)
  {
    return os << '{'
              << static_cast<stack_object const&>(t) << ','
              << static_cast<dynamic_object const&>(t) << ','
              << static_cast<empty const&>(t)
              << '}';
  }
};


//
// static allocation
//
template<typename T, unsigned int Size>
class array_stapl
{
private:
  T data[Size];

public:
  array_stapl(void) noexcept
  { std::fill(data, data + Size, T()); }

  explicit array_stapl(const int i) noexcept
  { std::iota(data, data + Size, i); }

  ~array_stapl(void)
  { std::fill(data, data + Size, T()); }

  unsigned int size(void) const volatile noexcept
  { return Size; }

  T const& operator[](const unsigned int index) const noexcept
  { return data[index]; }

  T& operator[](const unsigned int index) noexcept
  { return data[index]; }

  bool operator==(array_stapl const& rhs) const noexcept
  { return (std::equal(data, data + Size, rhs.data)); }

  COMPARISON_OPERATORS(array_stapl)

  void define_type(stapl::typer& t)
  { t.member(data); }

  friend std::ostream& operator<<(std::ostream& os, array_stapl const& t)
  {
    os << '{';
    for (auto const& i : t.data)
      os << i << ',';
    return os << '}';
  }
};


template<typename T, unsigned int Size>
class array_boost
{
private:
  T data[Size];

  friend class boost::serialization::access;

  template<typename Archive>
  void serialize(Archive& ar, const unsigned int)
  { ar & boost::serialization::make_array(data, Size); }

public:
  array_boost(void) noexcept
  { std::fill(data, data + Size, T()); }

  explicit array_boost(const int i) noexcept
  { std::iota(data, data + Size, i); }

  ~array_boost(void)
  { std::fill(data, data + Size, T()); }

  unsigned int size(void) const volatile noexcept
  { return Size; }

  T const& operator[](const unsigned int index) const noexcept
  { return data[index]; }

  T& operator[](const unsigned int index) noexcept
  { return data[index]; }

  bool operator==(array_boost const& rhs) const noexcept
  { return (std::equal(data, data + Size, rhs.data)); }

  COMPARISON_OPERATORS(array_boost)

  friend std::ostream& operator<<(std::ostream& os, array_boost const& t)
  {
    os << '{';
    for (auto const& i : t.data)
      os << i << ',';
    return os << '}';
  }
};


//
// dynamic allocation
//
template<typename T>
class vector_stapl
{
private:
  T* m_begin;
  T* m_end;

public:
  vector_stapl(void) noexcept
  : m_begin(nullptr),
    m_end(nullptr)
  { }

  explicit vector_stapl(const unsigned int size)
  : m_begin(new T[size]),
    m_end(m_begin + size)
  { std::fill(m_begin, m_end, T()); }

  vector_stapl(vector_stapl const& other)
  : m_begin(nullptr),
    m_end(nullptr)
  {
    if (other.size()!=0) {
      m_begin = new T[other.size()];
      m_end   = m_begin + other.size();
      std::copy(other.m_begin, other.m_end, m_begin);
    }
  }

  ~vector_stapl(void)
  {
    delete[] m_begin;
    m_begin = nullptr;
    m_end   = nullptr;
  }

  vector_stapl& operator=(vector_stapl const& other)
  {
    if (&other!=this)
    {
      delete[] m_begin;
      m_begin = nullptr;
      m_end   = nullptr;
      if (other.size()!=0) {
        m_begin = new T[other.size()];
        m_end   = m_begin + other.size();
        std::copy(other.m_begin, other.m_end, m_begin);
      }
    }
    return *this;
  }

  unsigned int size(void) const volatile noexcept
  { return m_end - m_begin; };

  T const& operator[](const unsigned int index) const noexcept
  { return m_begin[index]; }

  T& operator[](const unsigned int index) noexcept
  { return m_begin[index]; }

  bool operator==(vector_stapl const& rhs) const noexcept
  {
    if (rhs.size()!=size()) return false;
    return std::equal(m_begin, m_end, rhs.m_begin);
  }

  COMPARISON_OPERATORS(vector_stapl)

  void define_type(stapl::typer& t)
  {
    const unsigned int sz = size();
    t.member(m_begin, sz);
    t.pointer_to_member(m_end, m_begin, sz);
  }

  friend std::ostream& operator<<(std::ostream& os, vector_stapl const& t)
  {
    os << '{';
    for (T const* p=t.m_begin; p!=t.m_end; ++p)
      os << *p << ',';
    return os << '}';
  }
};


template<typename T>
class vector_boost
{
private:
  unsigned int m_size;
  T*           m_begin;

  friend class boost::serialization::access;

  template<typename Archive>
  void serialize(Archive& ar, const unsigned int)
  {
    ar & m_size;
    if (!m_begin) {
      m_begin = new T[m_size];
    }
    ar & boost::serialization::make_array(m_begin, m_size);
  }

public:
  vector_boost(void) noexcept
  : m_size(0),
    m_begin(nullptr)
  { }

  vector_boost(const unsigned int size)
  : m_size(size),
    m_begin(new T[size])
  { std::fill(m_begin, m_begin + m_size, T()); }

  vector_boost(vector_boost const& other)
  : m_size(0),
    m_begin(nullptr)
  {
    if (other.size()!=0) {
      m_begin = new T[other.size()];
      m_size  = other.m_size;
      std::copy(other.m_begin, other.m_begin + other.m_size, m_begin);
    }
  }

  ~vector_boost(void)
  {
    delete[] m_begin;
    m_begin = nullptr;
    m_size  = 0;
  }

  vector_boost& operator=(vector_boost const& other)
  {
    if (&other!=this)
    {
      delete[] m_begin;
      m_begin = nullptr;
      m_size  = nullptr;
      if (other.size()!=0) {
        m_begin = new T[other.size()];
        m_size  = other.m_size;
        std::copy(other.m_begin, other.m_begin + other.m_size, m_begin);
      }
    }
    return *this;
  }

  unsigned int size(void) const volatile noexcept
  { return m_size; };

  T const& operator[](const unsigned int index) const noexcept
  { return m_begin[index]; }

  T& operator[](const unsigned int index) noexcept
  { return m_begin[index]; }

  bool operator==(vector_boost const& rhs) const noexcept
  {
    if (rhs.size()!=size()) return false;
    return std::equal(m_begin, m_begin + m_size, rhs.m_begin);
  }

  COMPARISON_OPERATORS(vector_boost)

  friend std::ostream& operator<<(std::ostream& os, vector_boost const& t)
  {
    os << '{';
    for (unsigned int i=0; i<t.size(); ++i)
      os << t[i] << ',';
    return os << '}';
  }
};


//
// pointers to pointers
//
template<typename T>
class ptr_ptr_container
{
private:
  T** m_data;

public:
  ptr_ptr_container(void) noexcept
  : m_data(nullptr)
  { }

  ptr_ptr_container(T const& i)
  : m_data(new T*)
  { *m_data = new T(i); }

  ptr_ptr_container(ptr_ptr_container const& other)
  : m_data(new T*)
  { *m_data = new T(**other.m_data); }

  ~ptr_ptr_container(void)
  {
    if (m_data) {
      delete *m_data;
      *m_data = nullptr;
      delete m_data;
      m_data  = nullptr;
    }
  }

  ptr_ptr_container& operator=(ptr_ptr_container const& other)
  {
    if (this!=&other && other.m_data) {
      if (!m_data) {
        m_data = new T*;
        *m_data = new T(**other.m_data);
      }
      else {
        **m_data = **other.m_data;
      }
    }
    return *this;
  }

  bool operator==(ptr_ptr_container const& rhs) const noexcept
  {
    // if both are nullptr, then equal
    if (!m_data && !rhs.m_data)
      return true;

    // if only one of them is nullptr, then not equal
    if (!m_data || !rhs.m_data)
      return false;

    // both of them are not nullptr

    // if they both store pointers to nullptr, then equal
    if (!(*m_data) && !(*rhs.m_data))
      return true;

    // if only one of the stored pointers is nullptr, then not equal
    if (!(*m_data) || !(*rhs.m_data))
      return false;

    // both of them are not nullptr, check the underlying object
    return (**m_data==**rhs.m_data);
  }

  COMPARISON_OPERATORS(ptr_ptr_container)

  void define_type(stapl::typer& t)
  {
    t.member(m_data);
  }

  friend std::ostream& operator<<(std::ostream& os, ptr_ptr_container const& t)
  {
    return os << '{' << **t.m_data << '}';
  }
};

//
// pointers to members
//
template<typename Array, typename T>
class mem_ptr_container
{
private:
  Array data;

  T*                p1;
  const T*          p2;
  volatile T*       p3;
  const volatile T* p4;
  T* const                cp1;
  const T* const          cp2;
  volatile T* const       cp3;
  const volatile T* const cp4;
  T* volatile                vp1;
  const T* volatile          vp2;
  volatile T* volatile       vp3;
  const volatile T* volatile vp4;
  T* const volatile                cvp1;
  const T* const volatile          cvp2;
  volatile T* const volatile       cvp3;
  const volatile T* const volatile cvp4;

  unsigned int get_index(unsigned int N) noexcept
  { return (N%data.size()); }

public:
  mem_ptr_container(void) noexcept
  :   p1(nullptr),   p2(nullptr),
      p3(nullptr),   p4(nullptr),
     cp1(nullptr),  cp2(nullptr),
     cp3(nullptr),  cp4(nullptr),
     vp1(nullptr),  vp2(nullptr),
     vp3(nullptr),  vp4(nullptr),
    cvp1(nullptr), cvp2(nullptr),
    cvp3(nullptr), cvp4(nullptr)
  { }

  explicit mem_ptr_container(const unsigned int size) noexcept
  : data(size),
      p1( &(data[get_index(1)]) ),    p2( &(data[get_index(3)]) ),
      p3( &(data[get_index(5)]) ),    p4( &(data[get_index(7)]) ),
     cp1( &(data[get_index(9)]) ),   cp2( &(data[get_index(11)]) ),
     cp3( &(data[get_index(13)]) ),  cp4( &(data[get_index(15)]) ),
     vp1( &(data[get_index(17)]) ),  vp2( &(data[get_index(19)]) ),
     vp3( &(data[get_index(21)]) ),  vp4( &(data[get_index(23)]) ),
    cvp1( &(data[get_index(25)]) ), cvp2( &(data[get_index(27)]) ),
    cvp3( &(data[get_index(29)]) ), cvp4( &(data[get_index(31)]) )
  { }

  mem_ptr_container(mem_ptr_container const& other) noexcept
  : data(other.data),
      p1( &(data[get_index(1)]) ),    p2( &(data[get_index(3)]) ),
      p3( &(data[get_index(5)]) ),    p4( &(data[get_index(7)]) ),
     cp1( &(data[get_index(9)]) ),   cp2( &(data[get_index(11)]) ),
     cp3( &(data[get_index(13)]) ),  cp4( &(data[get_index(15)]) ),
     vp1( &(data[get_index(17)]) ),  vp2( &(data[get_index(19)]) ),
     vp3( &(data[get_index(21)]) ),  vp4( &(data[get_index(23)]) ),
    cvp1( &(data[get_index(25)]) ), cvp2( &(data[get_index(27)]) ),
    cvp3( &(data[get_index(29)]) ), cvp4( &(data[get_index(31)]) )
  { }

  mem_ptr_container& operator=(mem_ptr_container const& other) noexcept
  {
    if (&other!=this) {
      data = other.data;
      p1   = &(data[get_index(1)]);
      p2   = &(data[get_index(3)]);
      p3   = &(data[get_index(5)]);
      p4   = &(data[get_index(7)]);
      cp1  = &(data[get_index(9)]);
      cp2  = &(data[get_index(11)]);
      cp3  = &(data[get_index(13)]);
      cp4  = &(data[get_index(15)]);
      vp1  = &(data[get_index(17)]);
      vp2  = &(data[get_index(19)]);
      vp3  = &(data[get_index(21)]);
      vp4  = &(data[get_index(23)]);
      cvp1 = &(data[get_index(25)]);
      cvp2 = &(data[get_index(27)]);
      cvp3 = &(data[get_index(29)]);
      cvp4 = &(data[get_index(31)]);
    }
    return *this;
  }

  ~mem_ptr_container(void)
  {
    p1   = nullptr;
    p2   = nullptr;
    p3   = nullptr;
    p4   = nullptr;
    vp1  = nullptr;
    vp2  = nullptr;
    vp3  = nullptr;
    vp4  = nullptr;
  }

  bool operator==(mem_ptr_container const& rhs) const noexcept
  {
    if (data!=rhs.data) return false;
    if (*(p1)!=*(rhs.p1)) return false;
    if (*(p2)!=*(rhs.p2)) return false;
    if (*(p3)!=*(rhs.p3)) return false;
    if (*(p4)!=*(rhs.p4)) return false;
    if (*(cp1)!=*(rhs.cp1)) return false;
    if (*(cp2)!=*(rhs.cp2)) return false;
    if (*(cp3)!=*(rhs.cp3)) return false;
    if (*(cp4)!=*(rhs.cp4)) return false;
    if (*(vp1)!=*(rhs.vp1)) return false;
    if (*(vp2)!=*(rhs.vp2)) return false;
    if (*(vp3)!=*(rhs.vp3)) return false;
    if (*(vp4)!=*(rhs.vp4)) return false;
    if (*(cvp1)!=*(rhs.cvp1)) return false;
    if (*(cvp2)!=*(rhs.cvp2)) return false;
    if (*(cvp3)!=*(rhs.cvp3)) return false;
    if (*(cvp4)!=*(rhs.cvp4)) return false;
    return true;
  }

  COMPARISON_OPERATORS(mem_ptr_container)

  void define_type(stapl::typer& t)
  {
    t.member(data);

    t.pointer_to_member(p1, &data[0], get_index(1));
    t.pointer_to_member(p2, &data[0], get_index(3));
    t.pointer_to_member(p3, &data[0], get_index(5));
    t.pointer_to_member(p4, &data[0], get_index(7));

    t.pointer_to_member(cp1, &data[0], get_index(9));
    t.pointer_to_member(cp2, &data[0], get_index(11));
    t.pointer_to_member(cp3, &data[0], get_index(13));
    t.pointer_to_member(cp4, &data[0], get_index(15));

    t.pointer_to_member(vp1, &data[0], get_index(17));
    t.pointer_to_member(vp2, &data[0], get_index(19));
    t.pointer_to_member(vp3, &data[0], get_index(21));
    t.pointer_to_member(vp4, &data[0], get_index(23));

    t.pointer_to_member(cvp1, &data[0], get_index(25));
    t.pointer_to_member(cvp2, &data[0], get_index(27));
    t.pointer_to_member(cvp3, &data[0], get_index(29));
    t.pointer_to_member(cvp4, &data[0], get_index(31));
  }

  friend std::ostream& operator<<(std::ostream& os, mem_ptr_container const& t)
  {
    return os << '{' << *t.p1 << '}';
  }
};


//
// cv-qualifiers
//
template<typename T>
class cv_class_stapl
{
private:
  T                t;
  const T          ct;
  volatile T       vt;
  const volatile T cvt;

public:
  cv_class_stapl(void)
  : t(0),
    ct(0),
    vt(0),
    cvt(0)
  { }

  explicit cv_class_stapl(const int i)
  : t(i),
    ct(i + 1),
    vt(i + 2),
    cvt(i + 3)
  { }

  bool operator==(cv_class_stapl const& rhs) const noexcept
  { return (t==rhs.t && ct==rhs.ct && vt==rhs.vt && cvt==rhs.cvt); }

  COMPARISON_OPERATORS(cv_class_stapl)

  void define_type(stapl::typer& st)
  {
    st.member(t);
    st.member(ct);
    st.member(vt);
    st.member(cvt);
  }

  friend std::ostream& operator<<(std::ostream& os, cv_class_stapl const& t)
  {
    return os << '{'
              << t.t << ',' << t.ct << ',' << t.vt << ',' << t.cvt
              << '}';
  }
};


template<typename T>
class cv_class_boost
{
private:
  T                t;
  const T          ct;
  volatile T       vt;
  const volatile T cvt;

  friend class boost::serialization::access;

  template<typename Archive>
  void serialize(Archive& ar, const unsigned int)
  {
    ar & t;
    ar & const_cast<T&>(ct);
    ar & const_cast<T&>(vt);
    ar & const_cast<T&>(cvt);
  }

public:
  cv_class_boost(void)
  : t(0),
    ct(0),
    vt(0),
    cvt(0)
  { }

  explicit cv_class_boost(const int i)
  : t(i),
    ct(i + 1),
    vt(i + 2),
    cvt(i + 3)
  { }

  bool operator==(cv_class_boost const& rhs) const noexcept
  { return (t==rhs.t && ct==rhs.ct && vt==rhs.vt && cvt==rhs.cvt); }

  COMPARISON_OPERATORS(cv_class_boost)

  friend std::ostream& operator<<(std::ostream& os, cv_class_boost const& t)
  {
    return os << '{'
              << t.t << ',' << t.ct << ',' << t.vt << ',' << t.cvt
              << '}';
  }
};


//
// polymorphic classes
//
class poly_base
{
public:
  poly_base(void) noexcept = default;

  virtual ~poly_base(void) = default;

  STAPL_POLYMORPHIC_TYPE()

  virtual bool compare(const poly_base*) const noexcept = 0;

  bool operator==(poly_base const& rhs) const noexcept
  { return compare(&rhs); }

  COMPARISON_OPERATORS(poly_base)

  void define_type(stapl::typer&)
  { }

  friend std::ostream& operator<<(std::ostream& os, poly_base const&)
  {
    return os << "{}";
  }
};


class poly_type1
: public poly_base
{
private:
  stack_object o;

public:
  poly_type1(int i = 0)
  : o(i)
  { }

  STAPL_POLYMORPHIC_TYPE()

  bool operator==(poly_type1 const& rhs) const noexcept
  { return (o==rhs.o); }

  COMPARISON_OPERATORS(poly_type1)

  virtual bool compare(const poly_base* rhs) const noexcept
  {
    const poly_type1* ot = dynamic_cast<const poly_type1*>(rhs);
    return (*this==*ot);
  }

  void define_type(stapl::typer& t)
  {
    t.base<poly_base>(*this);
    t.member(o);
  }

  friend std::ostream& operator<<(std::ostream& os, poly_type1 const& t)
  {
    return os << '{' << t.o << '}';
  }
};


class poly_type2
: public poly_base
{
private:
  dynamic_object o;

public:
  poly_type2(const int i = 0)
  : o(i)
  { }

  STAPL_POLYMORPHIC_TYPE()

  bool operator==(poly_type2 const& rhs) const noexcept
  { return (o==rhs.o); }

  COMPARISON_OPERATORS(poly_type2)

  bool compare(const poly_base* rhs) const noexcept
  {
    const poly_type2* ot = dynamic_cast<const poly_type2*>(rhs);
    return (*this==*ot);
  }

  void define_type(stapl::typer& t)
  {
    t.base<poly_base>(*this);
    t.member(o);
  }

  friend std::ostream& operator<<(std::ostream& os, poly_type2 const& t)
  {
    return os << '{' << static_cast<poly_base const&>(t) << ',' << t.o << '}';
  }
};


namespace stapl {

template<>
struct derived_types<poly_base>
{
  typedef std::tuple<poly_type1, poly_type2> typelist_type;
};

} // namespace stapl


class poly_test
{
private:
  poly_base* p1;
  poly_base* p2;

public:
  poly_test(void)
  : p1(new poly_type1),
    p2(new poly_type2)
  { }

  poly_test(const int i)
  : p1(new poly_type1(i)),
    p2(new poly_type2(i))
  { }

  poly_test(poly_test const& other)
  : p1(new poly_type1(dynamic_cast<poly_type1&>(*other.p1))),
    p2(new poly_type2(dynamic_cast<poly_type2&>(*other.p2)))
  { }

  ~poly_test(void)
  {
    delete p1;
    p1 = nullptr;
    delete p2;
    p2 = nullptr;
  }

  poly_test& operator=(poly_test const& other)
  {
    if (this!=&other) {
      dynamic_cast<poly_type1&>(*p1) = dynamic_cast<poly_type1&>(*other.p1);
      dynamic_cast<poly_type2&>(*p2) = dynamic_cast<poly_type2&>(*other.p2);
    }
    return *this;
  }

  bool operator==(poly_test const& rhs) const noexcept
  { return (p1->compare(rhs.p1) && p2->compare(rhs.p2)); }

  COMPARISON_OPERATORS(poly_test)

  void define_type(stapl::typer& t)
  {
    t.member(p1);
    t.member(p2);
  }

  friend std::ostream& operator<<(std::ostream& os, poly_test const& t)
  {
    return os << '{' << *t.p1 << ',' << *t.p2 << '}';
  }
};


//
// polymorphic classes 2
//
class poly2_base
{
private:
  int o;

public:
  poly2_base(int i = 0)
  : o(i)
  { }

  virtual ~poly2_base(void)
  { o = 0; }

  STAPL_POLYMORPHIC_TYPE()

  virtual bool compare(const poly2_base*) const noexcept = 0;

  bool operator==(poly2_base const& rhs) const noexcept
  { return (o==rhs.o && compare(&rhs)); }

  COMPARISON_OPERATORS(poly2_base)

  void define_type(stapl::typer& t)
  { t.member(o); }

  friend std::ostream& operator<<(std::ostream& os, poly2_base const& t)
  {
    return os << '{' << t.o << '}';
  }
};


class poly2_type1
: public poly2_base
{
private:
  int o;

public:
  poly2_type1(void)
  : o(0)
  { }

  poly2_type1(int i)
  : poly2_base(i),
    o(i+1)
  { }

  ~poly2_type1(void)
  { o = 0; }

  STAPL_POLYMORPHIC_TYPE()

  bool operator==(poly2_type1 const& rhs) const noexcept
  { return (o==rhs.o); }

  COMPARISON_OPERATORS(poly2_type1)

  bool compare(const poly2_base* rhs) const noexcept
  {
    const poly2_type1* ot = dynamic_cast<const poly2_type1*>(rhs);
    return (*this==*ot);
  }

  void define_type(stapl::typer& t)
  {
    t.base<poly2_base>(*this);
    t.member(o);
  }

  friend std::ostream& operator<<(std::ostream& os, poly2_type1 const& t)
  {
    return os << '{' << static_cast<poly2_base const&>(t) << ',' << t.o << '}';
  }
};


class poly2_type2
: public poly2_base
{
private:
  double d;

public:
  poly2_type2(void)
  : d(0.0)
  { }

  poly2_type2(const int i)
  : poly2_base(i),
    d(1.0 + i)
  { }

  ~poly2_type2(void)
  { d = 0.0; }

  STAPL_POLYMORPHIC_TYPE()

  bool operator==(poly2_type2 const& rhs) const noexcept
  { return std::equal_to<double>()(d, rhs.d); }

  COMPARISON_OPERATORS(poly2_type2)

  bool compare(const poly2_base* rhs) const noexcept
  {
    const poly2_type2* ot = dynamic_cast<const poly2_type2*>(rhs);
    return (*this==*ot);
  }

  void define_type(stapl::typer& t)
  { t.base<poly2_base>(*this); }

  friend std::ostream& operator<<(std::ostream& os, poly2_type2 const& t)
  {
    return os << '{' << static_cast<poly2_base const&>(t) << ',' << t.d << '}';
  }
};


namespace stapl {

template<>
struct derived_types<poly2_base>
{
  typedef std::tuple<poly2_type1, poly2_type2> typelist_type;
};

} // namespace stapl


class poly2_test
{
private:
  boost::shared_ptr<poly2_base> p1;
  boost::shared_ptr<poly2_base> p2;

public:
  poly2_test(void)
  : p1(new poly2_type1),
    p2(new poly2_type2)
  { }

  poly2_test(const int i)
  : p1(new poly2_type1(i)),
    p2(new poly2_type2(i))
  { }

  poly2_test(poly2_test const& other) = default;

  poly2_test& operator=(poly2_test const& other)
  {
    if (this!=&other) {
      p1 = other.p1;
      p2 = other.p2;
    }
    return *this;
  }

  bool operator==(poly2_test const& rhs) const noexcept
  { return (*p1==*rhs.p1 && *p2==*rhs.p2); }

  COMPARISON_OPERATORS(poly2_test)

  friend std::ostream& operator<<(std::ostream& os, poly2_test const& t)
  {
    return os << '{' << *t.p1 << ',' << *t.p2 << '}';
  }

  void define_type(stapl::typer& t)
  {
    t.member(p1);
    t.member(p2);
  }
};

#endif
