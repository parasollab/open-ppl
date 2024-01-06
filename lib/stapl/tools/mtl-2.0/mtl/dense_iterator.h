/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//
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
//                                                                           
//
//===========================================================================

#ifndef MTL_DENSE_ITERATOR_H
#define MTL_DENSE_ITERATOR_H

#include "mtl/mtl_iterator.h"
#include "mtl/meta_if.h"
#include "mtl/mtl_config.h"

namespace mtl {


//: dense iterator
//
// An iterator for dense contiguous container that keeps track of the index.
//
//!category: iterators, adaptors
//!component: type
//!definition:dense_iterator.h
//!tparam: RandomAccessIterator - the base iterator
//!models: RandomAccessIterator? (with index())

#if defined ( _MSVCPP_ )

struct _bogus { };

/* The inheritance from Ranit is a VC++ workaround for not having
  a working iterator traits */
template <class T, int isConst, int IND_OFFSET=0, class SizeType=int>
class dense_iterator : public std::_Ranit<T,SizeType> {
  typedef dense_iterator self;
  typedef typename IF<isConst, const T*, T*>::RET RandomAccessIterator;
public:

  //: The value type
  typedef T value_type;
  //: This is a random access iterator
  typedef std::random_access_iterator_tag iterator_category;
  //: The type for differences between iterators
  typedef int difference_type;
  typedef int distance_type;
  //: The type for pointers to the value type
  typedef IF<isConst, const T*, T*>::RET pointer;
  //: The type for references to the value type
  typedef IF<isConst, const T&, T&>::RET reference;

  typedef difference_type Distance;

  typedef SizeType size_type;
  
  /*
protected:
*/

  RandomAccessIterator start;
  size_type pos;
  size_type start_index;

public:
  //: Return the index of the current element
  //!wheredef: IndexedIterator
  inline size_type index() const { 
    return pos + start_index + IND_OFFSET; 
  }
  //: Default Constructor  
  inline dense_iterator() : pos(0), start_index(0) {}

  //: Constructor from underlying iterator
  inline dense_iterator(RandomAccessIterator s, size_type i, size_type first_index = 0)
    : start(s), pos(i), start_index(first_index) { }

  //: Copy Constructor  
  inline dense_iterator (const self& x) 
    : start(x.start), pos(x.pos), start_index(x.start_index) {}

  typedef typename IF<isConst, dense_iterator<T,0,IND_OFFSET,SizeType>,
	  _bogus >::RET NonConst;

  inline dense_iterator(const NonConst& x)
   : start(x.start), pos(x.pos), start_index(x.start_index) {}

  //: Assignment operator
  inline self& operator=(const self& x) {
    start = x.start;
    pos = x.pos;
    start_index = x.start_index;
    return *this;
  }
  //: Destructor
  inline ~dense_iterator () { }

  //: Access the underlying iterator  
  inline RandomAccessIterator base() const { return start + pos; }

  inline operator RandomAccessIterator() const { return start + pos; }
  //: Dereference operator  
  inline reference operator*() const { return *(start + pos);  }
  //: Member access operator
  inline pointer operator-> () const { return start + pos; }
  //: Pre-increment operator  
  inline self& operator++ () { ++pos; return *this; }
  //: Post-increment operator
  inline self operator++ (int) { self tmp = *this; ++pos; return tmp; }
  //: Pre-decrement operator
  inline self& operator-- () { --pos; return *this; }
  //: Post-decrement operator
  inline self operator-- (int) { self tmp = *this; --pos; return tmp; }
  //: Add iterator and distance n
  inline self operator+ (Distance n) const { return self(start, pos + n); }
  //: Add distance n to this iterator
  inline self& operator+= (Distance n) { pos += n; return *this; }
  //: Subtract iterator and distance n
  inline self operator- (Distance n) const { return self(start, pos - n); }
  //: Return the difference between two iterators
  inline difference_type operator- (const self& x) const { 
    return base() - x.base(); }
  //: Subtract distance n from this iterator
  inline self& operator-= (Distance n) { pos -= n; return *this; }
  //: Return whether this iterator is not equal to iterator x
  inline bool operator!= (const self& x) const { return pos != x.pos; }
  //: Return whether this iterator is less than iterator x
  inline bool operator < (const self& x) const { return pos < x.pos; }
  //: Return whether this iterator is greater than iterator x
  inline bool operator > (const self& x) const { return pos > x.pos; }
  //: Return whether this iterator is equal to iterator x
  inline bool operator== (const self& x) const { return pos == x.pos; }
  //: Return whether this iterator is less than or equal to iterator x
  inline bool operator<= (const self& x) const { return pos <= x.pos; }
  //: Return whether this iterator is greater than or equal to iterator x
  inline bool operator>= (const self& x) const { return pos >= x.pos; }
  //: Equivalent to *(i + n)
#if 0 /* caused ambiguity with + op for VC++ */
  inline reference operator[] (Distance n) const { 
    return *(start + pos + n); 
  }
#endif
};


template <class T, int isConst,int OS, class ST>
inline
dense_iterator<T,isConst,OS,ST>
operator+ (typename dense_iterator<T,isConst,OS,ST>::size_type n, 
	   const dense_iterator<T,isConst,OS,ST> &x)
{
  return dense_iterator<T,isConst,OS,ST>(x.base(), n);
}


#elif defined( _MSVCPP7_ )

// while the iterator_traits behaviour has been fixed with MSVC 7, it still has
// a problem deducing template arguments in some instances. It works if dense_iterator
// is derived from rather than containing RandomAccessIterator. - BEL

template <class RandomAccessIterator, int IND_OFFSET=0, class SizeType=int>
class dense_iterator : public RandomAccessIterator {
  typedef dense_iterator self;
public:

  //: The value type
  typedef typename std::iterator_traits<RandomAccessIterator>::value_type value_type;
  //: This is a random access iterator
  typedef typename std::iterator_traits<RandomAccessIterator>::iterator_category iterator_category;
  //: The type for differences between iterators
  typedef typename std::iterator_traits<RandomAccessIterator>::difference_type difference_type;
  //: The type for pointers to the value type
  typedef typename std::iterator_traits<RandomAccessIterator>::pointer pointer;
  //: The type for references to the value type
  typedef typename std::iterator_traits<RandomAccessIterator>::reference reference;

  typedef difference_type Distance;
  typedef SizeType size_type;
/*  
protected:
*/
	// if we derive from RandomAccessIterator we shouldn't embed another
	// I implemented start() to make this easier to compare to the non-MSVC version - BEL
	//
	inline RandomAccessIterator&		start()			{ return *this; }
	inline const RandomAccessIterator&	start() const	{ return *this; }

	size_type pos;
	size_type start_index;

public:
  //: Return the index of the current element
  //!wheredef: IndexedIterator
  inline size_type index() const { 
    return pos + start_index + IND_OFFSET; 
  }
  //: Default Constructor  
  inline dense_iterator() : pos(0), start_index(0) {}

  //: Constructor from underlying iterator
  inline dense_iterator(RandomAccessIterator s, 
			size_type i, size_type first_index = 0)
    : RandomAccessIterator(s), pos(i), start_index(first_index) { }
  //: Copy Constructor  
  inline dense_iterator (const self& x) 
    : RandomAccessIterator(x), pos(x.pos), start_index(x.start_index) {}

  template <class SELF>
  inline dense_iterator (const SELF& x)
    : RandomAccessIterator(x), pos(x.pos), start_index(x.start_index) {}

  //: Assignment operator
  inline self& operator=(const self& x) {
	  RandomAccessIterator::operator=( x );
	  pos = x.pos;
	  start_index = x.start_index;
      return *this;
  }
  //: Destructor
  inline ~dense_iterator () { }

  //: Access the underlying iterator  
  inline RandomAccessIterator base() const { return start() + pos; }

  inline operator RandomAccessIterator() const { return start() + pos; }
  //: Dereference operator  
  inline reference operator*() const { return *(start() + pos);  }
  //: Member access operator
  inline pointer operator-> () const { return start() + pos; }
  //: Pre-increment operator  
  inline self& operator++ () { ++pos; return *this; }
  //: Post-increment operator
  inline self operator++ (int) { self tmp = *this; ++pos; return tmp; }
  //: Pre-decrement operator
  inline self& operator-- () { --pos; return *this; }
  //: Post-decrement operator
  inline self operator-- (int) { self tmp = *this; --pos; return tmp; }
  //: Add iterator and distance n
  inline self operator+ (Distance n) const { return self(start(), pos + n); }
  //: Add distance n to this iterator
  inline self& operator+= (Distance n) { pos += n; return *this; }
  //: Subtract iterator and distance n
  inline self operator- (Distance n) const { return self(start(), pos - n); }
  //: Return the difference between two iterators
  inline difference_type operator- (const self& x) const { 
    return base() - x.base(); }
  //: Subtract distance n from this iterator
  inline self& operator-= (Distance n) { pos -= n; return *this; }
  //: Return whether this iterator is not equal to iterator x
  inline bool operator!= (const self& x) const { return pos != x.pos; }
  //: Return whether this iterator is less than iterator x
  inline bool operator < (const self& x) const { return pos < x.pos; }
  //: Return whether this iterator is greater than iterator x
  inline bool operator > (const self& x) const { return pos > x.pos; }
  //: Return whether this iterator is equal to iterator x
  inline bool operator== (const self& x) const { return pos == x.pos; }
  //: Return whether this iterator is less than or equal to iterator x
  inline bool operator<= (const self& x) const { return pos <= x.pos; }
  //: Return whether this iterator is greater than or equal to iterator x
  inline bool operator>= (const self& x) const { return pos >= x.pos; }
  //: Equivalent to *(i + n)
  inline reference operator[] (Distance n) const { 
    return *(start() + pos + n); 
  }
};


template <class T, int OS, class ST>
inline
dense_iterator<T>
operator+ (typename dense_iterator<T,OS,ST>::size_type n, 
	   const dense_iterator<T,OS,ST> &x)
{
  return dense_iterator<T,OS,ST>(x.base(), n);
}

#else // other compilers

template <class RandomAccessIterator, int IND_OFFSET=0, class SizeType=int>
class dense_iterator {
  typedef dense_iterator self;
public:

  //: The value type
  typedef typename std::iterator_traits<RandomAccessIterator>::value_type value_type;
  //: This is a random access iterator
  typedef typename std::iterator_traits<RandomAccessIterator>::iterator_category iterator_category;
  //: The type for differences between iterators
  typedef typename std::iterator_traits<RandomAccessIterator>::difference_type difference_type;
  //: The type for pointers to the value type
  typedef typename std::iterator_traits<RandomAccessIterator>::pointer pointer;
  //: The type for references to the value type
  typedef typename std::iterator_traits<RandomAccessIterator>::reference reference;

  typedef difference_type Distance;

  typedef SizeType size_type;
  
  /*
protected:
*/

  RandomAccessIterator start;
  size_type pos;
  size_type start_index;

public:
  //: Return the index of the current element
  //!wheredef: IndexedIterator
  inline size_type index() const { 
    return pos + start_index + IND_OFFSET; 
  }
  //: Default Constructor  
  inline dense_iterator() : pos(0), start_index(0) {}

  //: Constructor from underlying iterator
  inline dense_iterator(RandomAccessIterator s, 
			size_type i, size_type first_index = 0)
    : start(s), pos(i), start_index(first_index) { }
  //: Copy Constructor  
  inline dense_iterator (const self& x) 
    : start(x.start), pos(x.pos), start_index(x.start_index) {}

  template <class SELF>
  inline dense_iterator (const SELF& x)
    : start(x.start), pos(x.pos), start_index(x.start_index) {}

  //: Assignment operator
  inline self& operator=(const self& x) {
    start = x.start;
    pos = x.pos;
    start_index = x.start_index;
    return *this;
  }
  //: Destructor
  inline ~dense_iterator () { }

  //: Access the underlying iterator  
  inline RandomAccessIterator base() const { return start + pos; }

  inline operator RandomAccessIterator() const { return start + pos; }
  //: Dereference operator  
  inline reference operator*() const { return *(start + pos);  }
  //: Member access operator
  inline pointer operator-> () const { return start + pos; }
  //: Pre-increment operator  
  inline self& operator++ () { ++pos; return *this; }
  //: Post-increment operator
  inline self operator++ (int) { self tmp = *this; ++pos; return tmp; }
  //: Pre-decrement operator
  inline self& operator-- () { --pos; return *this; }
  //: Post-decrement operator
  inline self operator-- (int) { self tmp = *this; --pos; return tmp; }
  //: Add iterator and distance n
  inline self operator+ (Distance n) const { return self(start, pos + n); }
  //: Add distance n to this iterator
  inline self& operator+= (Distance n) { pos += n; return *this; }
  //: Subtract iterator and distance n
  inline self operator- (Distance n) const { return self(start, pos - n); }
  //: Return the difference between two iterators
  inline difference_type operator- (const self& x) const { 
    return base() - x.base(); }
  //: Subtract distance n from this iterator
  inline self& operator-= (Distance n) { pos -= n; return *this; }
  //: Return whether this iterator is not equal to iterator x
  inline bool operator!= (const self& x) const { return pos != x.pos; }
  //: Return whether this iterator is less than iterator x
  inline bool operator < (const self& x) const { return pos < x.pos; }
  //: Return whether this iterator is greater than iterator x
  inline bool operator > (const self& x) const { return pos > x.pos; }
  //: Return whether this iterator is equal to iterator x
  inline bool operator== (const self& x) const { return pos == x.pos; }
  //: Return whether this iterator is less than or equal to iterator x
  inline bool operator<= (const self& x) const { return pos <= x.pos; }
  //: Return whether this iterator is greater than or equal to iterator x
  inline bool operator>= (const self& x) const { return pos >= x.pos; }
  //: Equivalent to *(i + n)
  inline reference operator[] (Distance n) const { 
    return *(start + pos + n); 
  }
};


template <class T, int OS, class ST>
inline
dense_iterator<T>
operator+ (typename dense_iterator<T,OS,ST>::size_type n, 
	   const dense_iterator<T,OS,ST> &x)
{
  return dense_iterator<T,OS,ST>(x.base(), n);
}


#endif


} /* namespace mtl */



#endif


