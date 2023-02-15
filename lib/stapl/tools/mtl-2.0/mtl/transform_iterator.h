// -*- c++ -*-
//
/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


// Copyright 1997, 1998, 1999 University of Notre Dame.
// Authors: Andrew Lumsdaine, Jeremy G. Siek, Lie-Quan Lee
//
// This file is part of the Matrix Template Library
//
// You should have received a copy of the License Agreement for the
// Matrix Template Library along with the software;  see the
// file LICENSE.  If not, contact Office of Research, University of Notre
// Dame, Notre Dame, IN  46556.
//
// Permission to modify the code and to distribute modified code is
// granted, provided the text of this NOTICE is retained, a notice that
// the code was modified is included with the above COPYRIGHT NOTICE and
// with the COPYRIGHT NOTICE in the LICENSE file, and that the LICENSE
// file is distributed with the modified code.
//
// LICENSOR MAKES NO REPRESENTATIONS OR WARRANTIES, EXPRESS OR IMPLIED.
// By way of example, but not limitation, Licensor MAKES NO
// REPRESENTATIONS OR WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
// PARTICULAR PURPOSE OR THAT THE USE OF THE LICENSED SOFTWARE COMPONENTS
// OR DOCUMENTATION WILL NOT INFRINGE ANY PATENTS, COPYRIGHTS, TRADEMARKS
// OR OTHER RIGHTS.
//
//===========================================================================

#include "mtl/iterator_adaptor.h"

#if 0
/* KCC can't handle this version */

//: Tranforming Iterator
// This iterator adaptor applies some function during the dereference
//!category: iterators, adaptors
//!component: type
//!tparam: Iterator - The underlying iterator type
//!tparam: UnaryFunction - A function that takes one argument of value type
template <class Iterator, class UnaryFunction>
class transform_iterator
  : public iterator_adaptor< transform_iterator<Iterator,
                                                   UnaryFunction>,
                             Iterator >
{
  typedef iterator_adaptor< transform_iterator<Iterator,
                                                   UnaryFunction>,
                             Iterator > super;
public:
  /* for old broken compilers */
  //: The value type
  typedef typename UnaryFunction::result_type value_type;
  //: The difference type
  typedef typename super::difference_type difference_type;
  //: The iterator category
  typedef typename super::iterator_category iterator_category;
  //: The pointer type
  typedef typename super::pointer pointer;
  //: The reference type
  typedef value_type reference;

  //: Normal Constructor
  inline transform_iterator(Iterator i, UnaryFunction op) 
    : super(*this, i), f(op) { }
  //: Copy Constructor
  inline transform_iterator(const transform_iterator& x)
    : super(*this, x.iter), f(x.f) { }
  //: Assignment Operator
  inline transform_iterator& operator=(const transform_iterator& x) {
    f = x.f; me = this; super::operator=(x); return *this; }
  //: Dereference Operator (applies the function here)  
  inline reference operator*() const { return f(*iter); }

protected:
  UnaryFunction f;
};
#else
template <class RandomAccessIterator, class UnaryFunction>
class transform_iterator {
  typedef transform_iterator self;
public:
  //: The value type
  typedef typename UnaryFunction::result_type value_type;

#if !defined ( _MSVCPP_ )
  //: The difference type  
  typedef typename std::iterator_traits<RandomAccessIterator>::difference_type
    difference_type;
  //: The pointer type
  typedef typename std::iterator_traits<RandomAccessIterator>::pointer 
    pointer;          
#else
  typedef typename std::iterator_traits<RandomAccessIterator>::distance_type
    difference_type;
  typedef difference_type distance_type;
  typedef value_type* pointer;
#endif
  //: The iterator category
  typedef typename std::iterator_traits<RandomAccessIterator>::iterator_category 
                               iterator_category;

  typedef difference_type      Distance;
  typedef RandomAccessIterator iterator_type;
  //: The reference type
  typedef value_type reference;
  typedef value_type const_reference;

  //: The default constructor
  //!wheredef: Trivial Iterator
  inline transform_iterator() { }

  //: Normal constructor
  //!wheredef: transform_iterator
  inline transform_iterator(const RandomAccessIterator& x, UnaryFunction op)
    : current(x), f(op) { }

  //: Copy constructor
  //!wheredef: Trivial Iterator
  inline transform_iterator(const self& x)
    : current(x.current), f(x.f) { }

  //: MTL index method
  //!wheredef: Indexible Iterator
  inline int index() const { return current.index(); }
  
  //: Convert to base iterator
  //!wheredef: transform_iterator
  inline operator RandomAccessIterator() { return current; }

  //: Access base iterator
  //!wheredef: transform_iterator
  inline RandomAccessIterator base() const { return current; }

  //: Dereference (and scale)
  //!wheredef: Trivial Iterator
  inline value_type operator*() const { return f(*current); }

  //: Preincrement
  //!wheredef: Forward Iterator
  inline self& operator++ () { ++current; return *this; }

  //: Postincrement
  //!wheredef: Forward Iterator
  inline self operator++ (int) { self tmp = *this; ++current; return tmp; }

  //: Preincrement
  //!wheredef: Bidirectional Iterator
  inline self& operator-- () { --current; return *this; }

  //: Postincrement
  //!wheredef: Bidirectional Iterator
  inline self operator-- (int) { self tmp = *this; --current; return tmp; }

  //: Iterator addition
  //!wheredef: Random Access Iterator
  inline self operator+ (Distance n) const {
    self c = current;
    c += n;
    return self (c, f);
  }

  //: Advance a distance
  //!wheredef: Random Access Iterator
  inline self& operator+= (Distance n) { current += n; return *this; }

  //: Subtract a distance
  //!wheredef: Random Access Iterator
  inline self operator- (Distance n) const {
    return self (current - n, f);
  }

  inline difference_type operator- (const self& x) const {
    return current - x.current;
  }

  //: Retreat a distance
  //!wheredef: Random Access Iterator
  inline self& operator-= (Distance n) { current -= n; return *this; }

  //: Access at an offset
  inline value_type operator[] (Distance n) const {
    return f(*(current + n));
  }
  //: Equality
  //!wheredef: Trivial Iterator
  inline bool operator==(const self& x) const { return current == x.current; }

  //: Inequality
  //!wheredef: Trivial Iterator
  inline bool operator!=(const self& x) const { return current != x.current; }

  //: Less than
  //!wheredef: Random Access Iterator
  inline bool operator<(const self& x) const { return current < x.current; }  

protected:
  RandomAccessIterator current;
  UnaryFunction f;
};
#endif

//: Helper function for creating a transforming iterator
//!category: iterators
//!component: function
template <class Iterator, class UnaryFunction> inline
transform_iterator<Iterator,UnaryFunction>
trans_iter(Iterator i, UnaryFunction op) {
  return transform_iterator<Iterator,UnaryFunction>(i, op);
}


#if 0
/* this probably doesn't work, the me pointer all goofed up */
template <class Iterator, class T>
class scale_iterator 
  : public transform_iterator<Iterator, 
                                 std::binder1st< std::multiplies<T> > >
{
  typedef transform_iterator<Iterator,
                                std::binder1st< std::multiplies<T> > >
      super;
public:
  inline scale_iterator(Iterator i, T a) 
    : super(i, bind1st(multiplies<T>(),a)) { }
};
#endif


template <class RandomAccessIterator>
class constant_stride_generator {
  typedef constant_stride_generator<RandomAccessIterator> self;
  typedef typename std::iterator_traits<RandomAccessIterator>::difference_type
     Distance;
public:
  inline constant_stride_generator(Distance s) : stride_(s) { }
  inline constant_stride_generator(const self& x) : stride_(x.stride_) { }
  inline void inc(RandomAccessIterator& i) { i += stride_; }
  inline void dec(RandomAccessIterator& i) { i -= stride_; }
  inline void advance(RandomAccessIterator& i, int n) { i += n * stride_; }
  /* JGS this interfance to diff may need to be changed */
  inline Distance diff(const RandomAccessIterator& x,
		const RandomAccessIterator& y,
		const self& /* y_stride_gen */) const {
    return (x - y) / stride_;
  }
private:
  Distance stride_;
};

/*
  StrideGen is a stride generator!
  The simple version just creates constant strides
 */
template <class RandomAccessIterator, class StrideGen>
class general_stride_iterator 
  : public iterator_adaptor< general_stride_iterator<RandomAccessIterator,
                                                      StrideGen>,
			     RandomAccessIterator >
{
  typedef general_stride_iterator<RandomAccessIterator, StrideGen> self;
  typedef iterator_adaptor< self, RandomAccessIterator> super;
public:
  typedef typename super::Distance Distance;

  inline general_stride_iterator(RandomAccessIterator i, StrideGen s) 
    : super(*this, i), stride_gen(s) { }
  inline self& operator++() {
    stride_gen.inc(this->iter);
    return *this; 
  }
  inline self& operator--() { 
    stride_gen.dec(this->iter);
    return *this; 
  }
  inline self& operator+=(Distance n) { 
    stride_gen.advance(this->iter, n);
    return *this; 
  }
  inline self& operator-=(Distance n) {
    stride_gen.advance(this->iter, -n);
    return *this; 
  }

  inline Distance diff(const self& y) const {
    return stride_gen.diff(this->iter, y.iter, y.stride_gen);
  }

  inline friend Distance operator-(const self& x, const self& y) {
    return x.diff(y);
  }

protected:
  StrideGen stride_gen;
};


template <class RandomAccessIterator>
class stride_iterator 
  : public general_stride_iterator<RandomAccessIterator,
                        constant_stride_generator<RandomAccessIterator> >

{
  typedef general_stride_iterator<RandomAccessIterator,
                     constant_stride_generator<RandomAccessIterator> > super;
public:
  typedef typename super::difference_type difference_type;
  inline stride_iterator(RandomAccessIterator i, difference_type s)
    : super(i, constant_stride_generator<RandomAccessIterator>(s)) { }
};

