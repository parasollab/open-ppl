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
//                                                                           
//
//===========================================================================

#ifndef MTL_SCALED1D_H
#define MTL_SCALED1D_H

#include "mtl/scale_iterator.h"
#include "mtl/reverse_iter.h"
#include "mtl/matrix_traits.h"

namespace mtl {

  //: Scaled Container
  //!category: containers, adaptors
  //!component: type
  //
  //  This class in not meant to be used directly. Instead it is
  //  created automatically when the <TT>scaled(x,alpha)</TT> function is
  //  invoked. See the documentation for "Shortcut for Creating A
  //  Scaled Vector".  This vector type is READ ONLY therefore there
  //  are only const versions of things ie. there is no iterator
  //  typedef, just const_iterator.
  //  
  //!definition: scaled1D.h
  //!tparam: RandomAccessContainerRef - The type of underlying container
  //!models: RandomAccessRefContainerRef
template <class RandomAccessContainerRef>
class scaled1D {
  typedef RandomAccessContainerRef Vector;
  typedef scaled1D<Vector> self;
public:
  /**@name Type Definitions */

  //: Static size, 0 if dynamic size
  enum { N = RandomAccessContainerRef::N };
  
  //: The value type
  typedef typename Vector::value_type value_type;
  //: The unsigned integral type for dimensions and indices
  typedef typename Vector::size_type size_type;

  //: The dimension, should be 1D
  typedef typename Vector::dimension_tag dimension_tag;

  //: The iterator type (do not use this)
  typedef scale_iterator<typename Vector::iterator, value_type> iterator;

  //: The const iterator type
  typedef scale_iterator<typename Vector::const_iterator, value_type> const_iterator;
  //: The const reverse iterator type
  typedef reverse_iter<const_iterator> const_reverse_iterator;

  //: The pointer to the value type
  typedef typename const_iterator::pointer pointer;

  //: The reference type
  typedef typename const_iterator::reference reference;

  /* not really a reference, scale_iterator uses the value_type */
  //: The const reference type
  typedef typename const_iterator::reference const_reference;
  //: The difference type
  typedef typename const_iterator::difference_type difference_type;


  //: The scaled type
  typedef scaled1D< self > scaled_type;

  //: The sparsity tag (dense_tag or sparse_tag)
  typedef typename Vector::sparsity sparsity;

  typedef scaled1D< typename Vector::subrange_type > subrange_type;

  //: The type for the index array
  typedef typename Vector::IndexArray IndexArray;
  //: The reference type to the index array
  typedef typename Vector::IndexArrayRef IndexArrayRef;

  
  /**@name Constructors */
  //: Default constructor
  inline scaled1D() { }

  //: Normal constructor
  inline scaled1D(const Vector& r, value_type scale_) 
    : rep(r), scale(scale_) { }

  inline scaled1D(const Vector& r, value_type scale_, do_scaled) 
    : rep(r), scale(scale_) { }

  //: Copy constructor
  inline scaled1D(const self& x) : rep(x.rep), scale(x.scale) { }

  //: Assignment operator
  inline self& operator=(const self& x) {
    rep = x.rep; scale = x.scale; return *this;
  }
  //: Destructor
  inline ~scaled1D() { }

  /**@name Access Methods */
  
  //: Access base containers
  inline operator Vector&() { return rep; }

  //: Return a const iterator pointing to the beginning of the vector
  //!wheredef: Container
  inline const_iterator begin() const {
    return const_iterator(rep.begin(), scale); 
  }
  //: Return a const iterator pointing past the end of the vector
  //!wheredef: Container
  inline const_iterator end() const {
    return const_iterator(rep.end(), scale); 
  }
  //: Return a const reverse iterator pointing to the last element of the vector
  //!wheredef: Reversible Container
  inline const_reverse_iterator rbegin() const {
    return const_reverse_iterator(end());
  }
  //: Return a const reverse iterator pointing past the end of the vector
  //!wheredef: Reversible Container
  inline const_reverse_iterator rend() const {
    return const_reverse_iterator(begin());
  }
  //: Return a const reference to the element with index i
  inline const_reference operator[](size_type i) const { 
    return rep[i] * scale; 
  }
  /* JGS, this changes the behaviour of the indexing for submatrices
   * inline const_reference operator[](int n) const { return *(begin() + n); }
   */

  inline subrange_type operator()(size_type s, size_type f) const {
    return subrange_type(rep(s,f), scale);
  }

  //: Return the size of the vector
  //!wheredef: Container  
  inline size_type size() const { return rep.size(); }

  //: Return the number of non-zeroes
  //!wheredef: Vector  
  inline size_type nnz() const { return rep.nnz(); }

  inline self& adjust_index(size_type i) { rep.adjust_index(i); return *this; }
    
protected:
  Vector rep;
  value_type scale;
};

 
//: Shortcut for Creating a Scaled Argument
//   This function can be used to scale arguments in MTL
//   functions. For example, to perform the vector addition operation
//   z <- a x + b y one would do the following:
//   <pre>
//   mtl::add(scaled(x, alpha), scaled(y, beta), z);
//   </pre>
//   The actual multiplication by alpha and beta is done within the
//   algorithm, so the performance is the same as if the add()
//   function had parameters for alpha and beta.  The
//   <TT>scaled()</TT> function can be used with any vector or matrix
//   argument in MTL functions.  Do not confuse this function with
//   <TT>mtl::scale()</TT> which are stand-alone functions.
//!category: containers
//!component: function
//!definition: scaled1D.h
//!typereqs: T must be convertible to Scalable's value_type
//!typereqs: Scalable's value_type must be a model of Ring
//!complexity: compile time and adds a single multiplication
//!complexity: to each element access inside of any algorithm
//!example: y_ax_y.cc
template <class Scalable, class T> inline
typename Scalable::scaled_type
scaled(const Scalable& A, const T& alpha)
{
  typedef typename Scalable::scaled_type scaled_type;
  return scaled_type(A, alpha, do_scaled());
}


} /* namespace mtl */


#endif
