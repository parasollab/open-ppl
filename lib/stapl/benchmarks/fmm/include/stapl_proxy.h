/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_STAPL_PROXY_H
#define STAPL_BENCHMARKS_FMM_STAPL_PROXY_H

#include "stapl_tp.h"
namespace stapl {

STAPL_PROXY_HEADER(Body)
{
  STAPL_PROXY_TYPES(STAPL_PROXY_CONCAT(Body), Accessor)
  STAPL_PROXY_METHODS(STAPL_PROXY_CONCAT(Body), Accessor)
  STAPL_PROXY_MEMBER(IBODY,int)
  STAPL_PROXY_MEMBER(IRANK,int)
  STAPL_PROXY_MEMBER(ICELL,uint64_t)
  STAPL_PROXY_MEMBER(WEIGHT,real_t)
  STAPL_PROXY_MEMBER(X,vec3)
  STAPL_PROXY_MEMBER(SRC,real_t)
  STAPL_PROXY_MEMBER(TRG,kvec4)
  explicit proxy(Accessor const& acc)
   : Accessor(acc),
     IBODY(member_referencer<IBODY_accessor>()(acc)),
     IRANK(member_referencer<IRANK_accessor>()(acc)),
     ICELL(member_referencer<ICELL_accessor>()(acc)),
     WEIGHT(member_referencer<WEIGHT_accessor>()(acc)),
     X(member_referencer<X_accessor>()(acc)),
     SRC(member_referencer<SRC_accessor>()(acc)),
     TRG(member_referencer<TRG_accessor>()(acc))
  { }
};

STAPL_PROXY_HEADER(Bounds)
{
  STAPL_PROXY_TYPES(STAPL_PROXY_CONCAT(Bounds), Accessor)
  STAPL_PROXY_METHODS(STAPL_PROXY_CONCAT(Bounds), Accessor)

  STAPL_PROXY_MEMBER(Xmin, vec3)
  STAPL_PROXY_MEMBER(Xmax, vec3)

  explicit proxy(Accessor const& acc)
    : Accessor(acc),
      Xmin(member_referencer<Xmin_accessor>()(acc)),
      Xmax(member_referencer<Xmax_accessor>()(acc))
  { }
};


STAPL_PROXY_HEADER(Cell)
{
  STAPL_PROXY_TYPES(STAPL_PROXY_CONCAT(Cell), Accessor)
  STAPL_PROXY_METHODS(STAPL_PROXY_CONCAT(Cell), Accessor)
  STAPL_PROXY_MEMBER(IPARENT,int)
  STAPL_PROXY_MEMBER(ICHILD,int)
  STAPL_PROXY_MEMBER(NCHILD,int)
  STAPL_PROXY_MEMBER(IBODY,int)
  STAPL_PROXY_MEMBER(NBODY,int)
  STAPL_PROXY_MEMBER(ICELL,uint64_t)
  STAPL_PROXY_MEMBER(WEIGHT,real_t)
  STAPL_PROXY_MEMBER(R,real_t)
  STAPL_PROXY_MEMBER(M,vecP)
  STAPL_PROXY_MEMBER(L,vecP)
  STAPL_PROXY_MEMBER(X,vec3)
  STAPL_PROXY_MEMBER(BODY,B_iter)

  explicit proxy(Accessor const& acc)
    : Accessor(acc),
      IPARENT(member_referencer<IPARENT_accessor>()(acc)),
      ICHILD(member_referencer<ICHILD_accessor>()(acc)),
      NCHILD(member_referencer<NCHILD_accessor>()(acc)),
      IBODY(member_referencer<IBODY_accessor>()(acc)),
      NBODY(member_referencer<NBODY_accessor>()(acc)),
      ICELL(member_referencer<ICELL_accessor>()(acc)),
      WEIGHT(member_referencer<WEIGHT_accessor>()(acc)),
      R(member_referencer<R_accessor>()(acc)),
      M(member_referencer<M_accessor>()(acc)),
      L(member_referencer<L_accessor>()(acc)),
      X(member_referencer<X_accessor>()(acc)),
      BODY(member_referencer<BODY_accessor>()(acc))
  { }
};

STAPL_PROXY_HEADER(TreeSTAPL)
{
  STAPL_PROXY_TYPES(STAPL_PROXY_CONCAT(TreeSTAPL), Accessor)
  STAPL_PROXY_METHODS(STAPL_PROXY_CONCAT(TreeSTAPL), Accessor)
  STAPL_PROXY_MEMBER(mpirank,int)
  STAPL_PROXY_MEMBER(mpisize,int)
  STAPL_PROXY_MEMBER(images,int)
  STAPL_PROXY_MEMBER(minMaxBounds,std::vector<Bounds>)
  STAPL_PROXY_MEMBER(sendBodies,std::vector<Bodies>)
  STAPL_PROXY_MEMBER(recvBodies,std::vector<Bodies>)
  STAPL_PROXY_MEMBER(nBodies,Bodies)
  STAPL_PROXY_MEMBER(jBodies,std::vector<Body>)
  STAPL_PROXY_MEMBER(sendCells,std::vector<Cells>)
  STAPL_PROXY_MEMBER(recvCells,std::vector<Cells>)
  STAPL_PROXY_MEMBER(sendBodyCount,std::vector<int>)
  STAPL_PROXY_MEMBER(sendBodyDispl,std::vector<int>)
  STAPL_PROXY_MEMBER(recvBodyCount,std::vector<int>)
  STAPL_PROXY_MEMBER(recvBodyDispl,std::vector<int>)
  STAPL_PROXY_MEMBER(sendCellCount,std::vector<int>)
  STAPL_PROXY_MEMBER(sendCellDispl,std::vector<int>)
  STAPL_PROXY_MEMBER(recvCellCount,std::vector<int>)
  STAPL_PROXY_MEMBER(recvCellDispl,std::vector<int>)

  explicit proxy(Accessor const& acc)
    : Accessor(acc),
      mpirank(member_referencer<mpirank_accessor>()(acc)),
      mpisize(member_referencer<mpisize_accessor>()(acc)),
      images(member_referencer<images_accessor>()(acc)),
      minMaxBounds(member_referencer<minMaxBounds_accessor>()(acc)),
      sendBodies(member_referencer<sendBodies_accessor>()(acc)),
      recvBodies(member_referencer<recvBodies_accessor>()(acc)),
      nBodies(member_referencer<nBodies_accessor>()(acc)),
      jBodies(member_referencer<jBodies_accessor>()(acc)),
      sendCells(member_referencer<sendCells_accessor>()(acc)),
      recvCells(member_referencer<recvCells_accessor>()(acc)),
      sendBodyCount(member_referencer<sendBodyCount_accessor>()(acc)),
      sendBodyDispl(member_referencer<sendBodyDispl_accessor>()(acc)),
      recvBodyCount(member_referencer<recvBodyCount_accessor>()(acc)),
      recvBodyDispl(member_referencer<recvBodyDispl_accessor>()(acc)),
      sendCellCount(member_referencer<sendCellCount_accessor>()(acc)),
      sendCellDispl(member_referencer<sendCellDispl_accessor>()(acc)),
      recvCellCount(member_referencer<recvCellCount_accessor>()(acc)),
      recvCellDispl(member_referencer<recvCellDispl_accessor>()(acc))
  { }

  Cells setLET(Cells ref1, real_t ref2)
  {
    return Accessor::invoke(&target_t::setLET,ref1,ref2);
  }

  void getLET(Cells ref1, int ref2)
  {
    Accessor::invoke(&target_t::getLET,ref1,ref2);
  }

  void setBodyCount(int rank, int count)
  {
    Accessor::invoke(&target_t::setBodyCount,rank,count);
  }
};

template <typename Accessor, int N, typename Val>
class proxy<vec<N,Val>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef vec<N,Val> target_t;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {
    return Accessor::read();
  }

  proxy & operator=(proxy const& rhs)
  {
    Accessor::write(rhs); return *this;
  }

  proxy & operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  STAPL_PROXY_REFERENCE_METHOD_1(inner, operator[], Val, int)

  /// Vector arithmetic (subtract)
  target_t operator-(target_t const& v) const
  {
    target_t temp;
    for (int i=0; i<N; i++)
      temp[i] = v[i] -Accessor::invoke(&target_t::value_at, i);
    return temp;
  }

  friend std::ostream& operator<<(std::ostream& os,
    proxy<target_t, Accessor> const& x )
  {
      for (int i=0; i<N; i++) os << x[i] << ' ';
      os << endl;
      return os;
  }
};

} //namespace stapl

#endif // STAPL_BENCHMARKS_FMM_STAPL_PROXY_H
