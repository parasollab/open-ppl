/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef MTL_ELT_H
#define MTL_ELT_H



template <class Tnum, class Index>
class elt {
  typedef elt<Tnum,Index> self;
public:
  inline elt(const self& x) 
    : value_(x.value_), row_(x.row_), column_(x.column_) { }
  inline elt(Tnum& v, const Index& i, const Index& j)
    : value_(v), row_(i), column_(j) { }
  inline self& operator=(const self& x) {
    value_ = x.value_; row_ = x.row_; column_ = x.column_;
    return *this;
  }
  inline self& operator=(const Tnum& v) { value_ = v; return *this; }
  inline operator Tnum&() { return value_; }
  inline Index row() { return row_; }
  inline Index column() { return column_; }

protected:
  Tnum& value_;
  Index row_;
  Index column_;
};


#endif
