/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef MTL_DIMENSION_H
#define MTL_DIMENSION_H

#include <utility>

namespace mtl {

  //: The Dimension Class
  //
  // This is similar to the std::pair class except that it can have static
  // parameters, and only deals with size types. The purpose of this
  // class is to transparently hide whether the dimensions of a matrix
  // are specified statically or dynamically.
  //
  //!category: utilities
  //!component: type
  //
template <class sizet, int MM = 0, int NN = 0>
class dimension {
	typedef dimension<sizet, MM, NN> self;
public:
  typedef dimension<sizet, NN, MM> transpose_type;
  typedef sizet size_type;
  enum { M = MM, N = NN };
  inline dimension() : m(0), n(0) { }
  inline dimension(const self& x) : m(x.m), n(x.n) { }
  template <class ST>
  inline dimension(const std::pair<ST,ST>& x) : m(x.first), n(x.second) { }
#if !defined( _MSVCPP_ )
	//template <class Dim>
	//inline dimension(const Dim& x) : m(x.m), n(x.n) { }
  template <class S, int MMM, int NNN>
  inline dimension(const dimension<S, MMM, NNN>& x) : m(x.m), n(x.n) {}
#endif
  inline dimension(size_type m_, size_type n_) : m(m_), n(n_) { }
  inline dimension& operator=(const dimension& x) {
    m = x.m; n = x.n; return *this; }
  inline size_type first() const { return M ? M : m; }
  inline size_type second() const { return N ? N : n; }
  inline bool is_static() const { return M != 0; }
  inline transpose_type transpose() const { return transpose_type(n, m); }
  /* protected: */
  size_type m, n;
};

} /* namespace mtl */

#endif /* MTL_DIMENSION_H */
