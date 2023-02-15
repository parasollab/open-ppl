/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_DOMAINS_HPP
#define STAPL_VIEWS_DOMAINS_HPP

#include <stapl/utility/tuple.hpp>
#include <algorithm>
#include <stapl/domains/intersect.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>


namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief One dimensional domain.
///
/// @tparam Index_Type define the type of the \e gids in the domain.
//////////////////////////////////////////////////////////////////////
template <typename Index_Type>
struct ndom1D
{
  typedef Index_Type          index_type;
  typedef Index_Type          gid_type;
  typedef size_t              size_type;

private:
  /// lower bound
  index_type lower;

  /// upper bound
  index_type upper;

  /// This domain represent the domain of the container
  bool       m_cont_dom;

public:

  ndom1D()
  {
    lower = index_type(1);
    upper = index_type(0);
    m_cont_dom = false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain constructor [0..@p size-1]
  //////////////////////////////////////////////////////////////////////
  ndom1D(size_t size,bool is_cont_dom=false)
  {
    if (size>0) {
      lower = index_type(0);
      upper = index_type(size-1);
    }
    else {
      lower = index_type(1);
      upper = index_type(0);
    }
    m_cont_dom = is_cont_dom;
  }

  ndom1D(ndom1D const& other)
  {
    lower = other.lower;
    upper = other.upper;
    m_cont_dom = other.m_cont_dom;
  }

  ndom1D(ndom1D const* other)
  {
    lower = other->lower;
    upper = other->upper;
    m_cont_dom = other->m_cont_dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain constructor [lower .. upper]
  ///
  /// @param lwr Lower index in the domain.
  /// @param uppr Upper index in the domain.
  //////////////////////////////////////////////////////////////////////
  ndom1D(index_type lwr, index_type uppr, bool is_cont_dom=false)
    : lower(lwr), upper(uppr), m_cont_dom(is_cont_dom)
  { }

  ndom1D(index_type lwr, index_type uppr, bool is_cont_dom, size_t)
    : lower(lwr), upper(uppr), m_cont_dom(is_cont_dom)
  { }

  ndom1D(ndom1D const&, index_type lwr, index_type uppr,
         bool is_cont_dom=false)
    : lower(lwr), upper(uppr), m_cont_dom(is_cont_dom)
  { }

  ndom1D(index_type lwr, index_type uppr, ndom1D const&)
    : lower(lwr), upper(uppr), m_cont_dom(false)
  { }

  size_type size() const
  {
    if (upper==index_bounds<index_type>::highest()) return upper;
    if (lower > upper) return 0;
    return upper-lower+1;
  }

  gid_type first(void) const
  {
    return lower;
  }

  gid_type last() const
  {
    return upper;
  }

  gid_type open_last() const
  {
    return upper+1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true or false if the gid is in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type idx) const
  {
    return ((lower<=idx)&&(idx<=upper));
  }

  bool empty() const
  {
    return lower>upper;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an index that is @p n positions after the
  ///        provided @p idx, based on the domain order.
  //////////////////////////////////////////////////////////////////////
  index_type advance(index_type idx,long n) const
  {
    return (idx+n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the positive distance between two indices based on
  /// the domain order.
  //////////////////////////////////////////////////////////////////////
  size_t distance(index_type const& i0,index_type const& i1) const
  {
    if (i0>=i1) return (i0-i1);
    else return (i1-i0);
  }

  bool less_than(index_type const& i0,index_type const& i1) const
  {
    return i0<i1;
  }

  bool is_same_container_domain() const
  {
    return m_cont_dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain intersection.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom>
  ndom1D operator&(ODom const& other) const
  {
    if (!m_cont_dom) {
      typename ODom::index_type olower = other.first();
      typename ODom::index_type oupper = other.last();

      if (!this->empty()) {
        if (other.contains(lower)) {
          olower = lower;
        }
        if (other.contains(upper)) {
          oupper = upper;
        }
      }
      if (!(other.contains(lower) || other.contains(upper)) )
        if (!(this->contains(olower) || this->contains(oupper)))
          return ndom1D();
      return ndom1D(olower,oupper);
    }
    else
      return ndom1D( other.first(), other.last());
  }


  bool is_contiguous() const
  {
    return true;
  }


  void define_type(stapl::typer &t)
  {
    t.member(lower);
    t.member(upper);
  }

  template<typename IT>
  friend std::ostream & operator << (std::ostream &os, ndom1D<IT> const& d);
};

template<typename T>
std::ostream & operator<<(std::ostream &os, ndom1D<T> const& d)
{
  if (d.empty())
    os << "empty";
  else {
    os << "[" << d.lower << "..";
    os << d.upper << "]";
  }
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief One dimensional block-cyclic domain
/// @tparam Index_Type define the type of the \e gids in the domain
//////////////////////////////////////////////////////////////////////
template<typename Index_Type>
struct dom1Dbc
{
  typedef Index_Type index_type;
  typedef Index_Type gid_type;
  typedef size_t size_type;

private:

  size_t p; // which partition
  size_t blksz; // block size
  size_t step; // distance to the next block
  size_type dsize; // number of elements


public:

  dom1Dbc()
    : p(0), blksz(1), step(1), dsize(1)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain constructor [0..size-1] without cyclicity.
  //////////////////////////////////////////////////////////////////////
  dom1Dbc(size_type size)
    : p(0), blksz(size), step(size), dsize(size)
  {
    stapl_assert(size > 0, "Block size has to >0");
  }

  dom1Dbc(dom1Dbc const& other)
  {
    p = other.p;
    blksz = other.blksz;
    step = other.step;
    dsize = other.dsize;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain constructor using a specific block size and
  ///        cyclicity.
  ///
  /// @param blksz Block size.
  /// @param step Cyclicity.
  /// @param size Number of elements in the domain.
  /// @param p Partition where this domain belongs to.
  //////////////////////////////////////////////////////////////////////
  dom1Dbc(size_t blksz, size_t step, size_type size, size_t p = 0)
    : p(p), blksz(blksz), step(step), dsize(size)
  { }

  size_type size() const
  {
    return dsize;
  }

  gid_type first() const
  {
    return logic_to_global(0);
  }

  gid_type last() const
  {
    return logic_to_global(dsize - 1);
  }

  index_type advance(index_type idx, long n) const
  {
    stapl_assert(contains(idx), "Index not in domain");
    if (n == 0)
      return idx;
    size_t temp = blksz * step;
    size_t t_begin = (idx / temp) * temp + p;
    size_t t_diff = idx - t_begin;
    n += t_diff;
    idx -= t_diff;
    size_t jump = n / blksz;
    idx += jump * temp;
    n -= jump * blksz;
    //stapl_assert(contains(idx + n) , " gid out of bounds");
    return idx + n;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the positive distance between two indices based
  ///        on the domain order.
  //////////////////////////////////////////////////////////////////////
  size_t distance(index_type const& i0, index_type const& i1) const
  {
    // stapl_assert(false,"Not implemented yed\n");
    // stapl assert if i0 or i1 is not contained in this domain
    stapl_assert(contains(i0), "Index not in domain");
    stapl_assert(contains(i1), "Index not in domain");
    index_type temp_i0, temp_i1;
    if (i0 == i1)
      return 0;
    else if (i0 < i1)
      {
        temp_i0 = i0;
        temp_i1 = i1;
      }
    else
      {
        temp_i0 = i1;
        temp_i1 = i0;
      }
    size_t temp = blksz * step;
    size_t t_begin0 = (temp_i0 / temp) * temp + p ;
    size_t t_diff0 = temp_i0 - t_begin0;
    size_t t_begin1 = (temp_i1 / temp) * (temp) + p ;
    size_t t_diff1 = temp_i1 - t_begin1;
    size_t t_diff2 = (t_begin1 - t_begin0) / step;
    return t_diff2 + t_diff1 - t_diff0;
  }

  bool empty() const
  {
    return dsize == 0;
  }

  void define_type(stapl::typer &t)
  {
    t.member(p);
    t.member(blksz);
    t.member(step);
    t.member(dsize);
  }

  bool contains(index_type idx) const
  {
    size_t temp = step * blksz;
    if (idx < p)
      return false;
    size_t t = ((idx - p) / temp) * temp + p;
    return (idx - t < blksz && idx <= last() &&
            idx >= first()) ? true : false;
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Maps indices from the logical (local) position into a
  ///        global position.
  //////////////////////////////////////////////////////////////////////
  gid_type
  logic_to_global(size_t i) const
  {
    size_t b = i / blksz;
    return p + b * step * blksz + i % blksz;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Maps indices from global position into a logic (local) position
  //////////////////////////////////////////////////////////////////////
  size_t
  global_to_logic(gid_type g) const
  {
    g -= logic_to_global(0);
    size_t b = g / step;
    size_t i = g % blksz;
    return b % blksz + i;
  }

  template<typename T1>
  friend std::ostream &
  operator <<(std::ostream &os, dom1Dbc<T1> const& d);
};

template<typename T>
std::ostream &
operator <<(std::ostream &o, dom1Dbc<T> const& d)
{
  if (d.empty()) {
    o << "Empty" << std::endl;
    return o;
  }
  typename dom1Dbc<T>::index_type val=d.first();
  o<<val<<" ";
  for (size_t i=1;i<d.dsize;i++) {
    if (i%d.blksz == 0)
      o << std::endl;
    o << d.advance(val,i) << " ";
  }
  return o;
}


//////////////////////////////////////////////////////////////////////
/// @brief One dimensional domain.
/// @tparam T Index type of the domain.
/// @tparam Dom Storage type of the domain.
/// @todo replace usage of this with indexed or other newer domain.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Dom = ndom1D<T> >
struct dom1D
{
  typedef Dom                                      domain_t;
  typedef tuple<domain_t>                          domain_type;
  typedef typename domain_t::index_type            index_type;
  typedef typename domain_t::gid_type              gid_type;
  typedef typename domain_t::size_type             size_type;

public:
  domain_type m_domain;

  static const gid_type null_gid;

  dom1D()
    : m_domain(domain_t())
  { }

  dom1D(domain_t dom)
  {
    m_domain = domain_type(dom);
  }

  dom1D(dom1D const& dom)
  {
    m_domain = dom.m_domain;
  }

  template <typename KDom>
  dom1D(dom1D<T,KDom> const& dom)
  {
    m_domain = domain_type(domain_t(get<0>(dom.m_domain)));
  }

  dom1D(index_type lower, index_type upper, bool is_full_dom=false)
    : m_domain(domain_t(lower,upper,is_full_dom))
  { }

  dom1D(size_t n)
    : m_domain(domain_t(n,true))
  { }

  dom1D(index_type lower, index_type upper, bool is_full_dom, size_t sz)
    : m_domain(domain_t(lower, upper, is_full_dom, sz))
  { }

  // Restricted domain based on other domain
  dom1D(index_type lower, index_type upper, dom1D const& other)
    : m_domain(domain_t(lower,upper,get<0>(other.m_domain)))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the domain.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return get<0>(m_domain).size();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the first index in the domain.
  //////////////////////////////////////////////////////////////////////
  gid_type first() const
  {
    return gid_type(get<0>(this->m_domain).first());
  }



  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last index in the domain.
  //////////////////////////////////////////////////////////////////////
  gid_type last() const
  {
    return gid_type(get<0>(this->m_domain).last());
  }

  gid_type open_last() const
  {
    return gid_type(get<0>(this->m_domain).open_last());
  }

  gid_type end() const
  {
    return open_last();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the specified index is in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type idx) const
  {
    return get<0>(this->m_domain).contains(idx);
  }


  bool empty() const
  {
    return (get<0>(this->m_domain).empty());
  }

  index_type advance(index_type idx,long long n) const
  {
    return (get<0>(this->m_domain).advance(idx,n));
  }

  bool less_than(index_type const& i0,index_type const& i1) const
  {
    return (get<0>(this->m_domain).less_than(i0,i1));
  }

  size_t distance(index_type const& i0,index_type const& i1) const
  {
    return (get<0>(this->m_domain).distance(i0,i1));
  }

  bool is_same_container_domain() const
  {
    return (get<0>(this->m_domain).is_same_container_domain());
  }

  dom1D operator&(dom1D const& other) const
  {
    return dom1D(get<0>(this->m_domain) & get<0>(other.m_domain));
  }

  template <typename ODom>
  dom1D operator&(ODom const& other)
  {
    return intersect(get<0>(this->m_domain), other.template get_domain<0>());
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_domain);
  }

  dom1D operator&(dom1D const& other)
  {
    return dom1D(get<0>(this->m_domain) & get<0>(other.m_domain));
  }

  bool is_contiguous() const
  {
    return get<0>(this->m_domain).is_contiguous();
  }

  template<int N>
  typename tuple_element<N,domain_type>::type get_domain() const
  { return get<N>(m_domain); }

  template<typename T1>
  friend std::ostream & operator << (std::ostream &os, dom1D<T1> const& d);
};

template<typename T>
std::ostream & operator<<(std::ostream &os, dom1D<T> const& d)
{
  os << get<0>(d.m_domain);
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Two dimensional domain implemented as a tuple of one-dimensional
/// domains.
///
/// @tparam OneD1 type of the domain of the first dimension.
/// @tparam OneD2 type of the domain of the second dimension.
//////////////////////////////////////////////////////////////////////
template <typename OneD1,typename OneD2=OneD1>
struct dom2D
{
  typedef tuple<typename OneD1::index_type,
                               typename OneD2::index_type>   index_type;

  typedef tuple<OneD1,OneD2>                  domain_type;

  typedef tuple<typename OneD1::gid_type,
                               typename OneD2::gid_type>     gid_type;
  typedef tuple<typename OneD1::size_type,
                               typename OneD2::size_type>    size_type;

public:
  domain_type m_domain;
  static const gid_type null_gid;

  dom2D()
    : m_domain(OneD1(),OneD2())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the domain given the number of elements in each
  /// dimension.
  /// @param m number of rows
  /// @param n number of columns
  //////////////////////////////////////////////////////////////////////
  dom2D(size_t m,size_t n)
  {
    m_domain = domain_type(OneD1(m),OneD2(n));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the domain given the one-dimensional domains for each
  /// dimension.
  /// @param fdom domain of the first dimension.
  /// @param sdom domain of the second dimension.
  //////////////////////////////////////////////////////////////////////
  dom2D(OneD1 fdom, OneD2 sdom)
  {
    m_domain = domain_type(fdom,sdom);
  }

  dom2D(domain_type& dom)
  {
    m_domain = dom.m_domain;
  }

  dom2D(index_type const& fst, index_type const& snd)
  {
    m_domain = domain_type(OneD1(get<0>(fst),get<0>(snd)),
                           OneD2(get<1>(fst),get<1>(snd)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the domain.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return get<0>(m_domain).size() * get<1>(m_domain).size();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in each dimension.
  //////////////////////////////////////////////////////////////////////
  size_type size_twod() const
  {
    return size_type(get<0>(m_domain).size(), get<1>(m_domain).size());
  }


  gid_type first() const
  {
    return gid_type(get<0>(this->m_domain).first(),
                    get<1>(this->m_domain).first());
  }

  gid_type last() const
  {
    return gid_type(get<0>(this->m_domain).last(),
                    get<1>(this->m_domain).last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the specified index is in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type idx) const
  {
    return (get<0>(this->m_domain).contains(get<0>(idx)) &&
            get<1>(this->m_domain).contains(get<1>(idx))) ;
  }


  bool empty() const
  {
    return (get<0>(this->m_domain).empty() ||
            get<1>(this->m_domain).empty());
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_domain);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the domain associated with a specific dimension.
  /// @tparam N dimension to get the domain
  //////////////////////////////////////////////////////////////////////
  template<int N>
  typename tuple_element<N,domain_type>::type get_domain() const
  { return get<N>(m_domain); }

  template<typename T1,typename T2 >
  friend std::ostream & operator << (std::ostream &os, dom2D<T1,T2> const& d);

};


template<typename T1,typename T2>
std::ostream & operator<<(std::ostream &os, dom2D<T1,T2> const& d)
{
  os << get<0>(d.m_domain) << "x" << get<1>(d.m_domain);
  return os;
}


// Domain based on iterators
template <typename Iterator>
class seqDom
{
  Iterator    m_begin;
  Iterator    m_end;
  size_t      m_sz;
  /// Is True if this domain define the same domain as the container.
  bool        m_cont_dom;

public:
  typedef size_t                          size_type;
  typedef Iterator                        index_type;
  typedef Iterator                        gid_type;

  seqDom()
    : m_sz(0)
  {
    m_begin = m_end;
    m_cont_dom=false;
  }

  seqDom(Iterator begin, Iterator end,  bool is_cont_dom=false,
         long long sz=-1)
    : m_begin(begin), m_end(end), m_cont_dom(is_cont_dom)
  {
    const_cast<seqDom*>(this)->m_sz = size_t(sz);
  }

  seqDom(Iterator begin, Iterator end,  seqDom const& other,
         bool is_cont_dom=false, long long sz=-1)
    : m_begin(begin), m_end(end), m_cont_dom(is_cont_dom)
  {
    const_cast<seqDom*>(this)->m_sz = size_t(sz);
  }

  index_type first() const
  {
    return m_begin;
  }

  index_type last() const
  {
    return m_end;
  }

  index_type open_last() const
  {
    index_type tmp = m_end;
    return ++tmp;
  }

  size_t size() const
  {
    if (m_sz== size_t(-1)) {
      const_cast<seqDom*>(this)->m_sz = std::distance(m_begin,m_end) +
        (m_begin!=Iterator()? 1 : 0);
    }
    return m_sz;
  }

  bool contains(index_type idx) const
  {
    if ((m_begin==idx)||(m_end==idx))
      return true;
    else {
      Iterator tmp=m_begin;
      while (tmp!=m_end) {
        if (tmp==idx) return true;
        ++tmp;
      }
    }
    return false;
  }

  bool empty() const
  {
    if (m_sz == size_t(-1))
      return m_begin!=Iterator() ? false : true;
    else
      return m_sz==0;
  }

  index_type advance(index_type idx,long n) const
  {
    std::advance(idx,n);
    return idx;
  }

  size_t distance(index_type const& a,index_type const& b) const
  {
    auto dist = std::distance(a,b);
    return (dist < 0) ? -dist : dist;
  }

  bool is_same_container_domain() const
  {
    return m_cont_dom;
  }

  // Domain intersection
  template <typename ODom>
  ODom operator&(ODom const& other) const
  {
    //    std::cout << "ndom1D intersection called\n";
    if (!m_cont_dom) {
      typename ODom::index_type olower = other.first();
      typename ODom::index_type oupper = other.last();

      if (!this->empty()) {
        if (other.contains(m_begin)) {
          olower = m_begin;
        }
        if (other.contains(m_end)) {
          oupper = m_end;
        }
      }
      if (!(other.contains(m_begin) || other.contains(m_end)) )
        if (!(this->contains(olower) || this->contains(oupper)))
          return ODom();
      return ODom(olower,oupper);
    }
    else
      return other;
  }

  bool is_contiguous() const
  {
    return true;
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_begin);
    t.member(m_end);
    t.member(m_sz);
    t.member(m_cont_dom);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Two dimensional domain with ovelap information support
/// @tparam Index_Type define the type of the \e gids in the domain
//////////////////////////////////////////////////////////////////////
template <typename Index_Type>
struct ndom2D
{
  typedef Index_Type          index_type;
  typedef Index_Type          gid_type;
  typedef size_t              size_type;

private:
  /// lower bound
  index_type lower;
  /// upper bound
  index_type upper;
  /// Whether the domain represents the entire domain of the container
  bool  m_cont_dom;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructor (empty domain)
  //////////////////////////////////////////////////////////////////////
  ndom2D()
  {
    lower = index_type(1,0);
    upper = index_type(0,0);
    m_cont_dom = false;
  }

  //////////////////////////////////////////////////////////////////////
  /// \brief Copy constructor
  //////////////////////////////////////////////////////////////////////
  ndom2D(ndom2D const& other)
  {
    lower = other.lower;
    upper = other.upper;
    m_cont_dom = other.m_cont_dom;
  }


  //////////////////////////////////////////////////////////////////////
  /// \brief Copy constructor based on a pointer
  //////////////////////////////////////////////////////////////////////
  ndom2D(ndom2D const* other)
  {
    lower = other->lower;
    upper = other->upper;
    m_cont_dom = other->m_cont_dom;
  }


  //////////////////////////////////////////////////////////////////////
  /// \brief Domain constructor [lower .. upper]
  ///
  /// @param lower lower index in the domain
  /// @param upper upper index in the domain
  /// @param left how many elements are overlap to the left (default: 0)
  /// @param right how many elements are overlap to the right (default: 0)
  //////////////////////////////////////////////////////////////////////
  ndom2D(index_type lower, index_type upper, bool is_cont_dom=false)
    : lower(lower), upper(upper), m_cont_dom(is_cont_dom)
  { }


  //////////////////////////////////////////////////////////////////////
  /// \brief Return how many element the domain is indexing
  //////////////////////////////////////////////////////////////////////
  size_type size() const
  {
    if (upper==index_bounds<index_type>::highest()) return upper;
    return upper-lower+1;
  }


  //////////////////////////////////////////////////////////////////////
  /// \brief Returns the first index (\e gid) in the domain
  //////////////////////////////////////////////////////////////////////
  gid_type first() const
  {
    return lower;
  }


  //////////////////////////////////////////////////////////////////////
  /// \brief Returns the last index (\e gid) in the domain
  //////////////////////////////////////////////////////////////////////
  gid_type last() const
  {
    return upper;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the specified index is in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type idx) const
  {
    return ((lower<=idx)&&(idx<=upper));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether the domain is empty
  //////////////////////////////////////////////////////////////////////
  bool empty() const
  {
    return lower>upper;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return an the result of advancing the index provided n positions
  /// for the domain ordering provided at instantiation.
  //////////////////////////////////////////////////////////////////////
  index_type advance(index_type idx,long n) const
  {
    return (idx+n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the positive distance between two indices based on
  /// the domain order
  //////////////////////////////////////////////////////////////////////
  size_t distance(index_type const& i0,index_type const& i1) const
  {
    if (i0>=i1) return (i0-i1);
    else return (i1-i0);
  }

  bool is_same_container_domain() const
  {
    return m_cont_dom;
  }

  void define_type(stapl::typer &t)
  {
    t.member(lower);
    t.member(upper);
  }

  template<typename IT>
  friend std::ostream & operator << (std::ostream &os, ndom2D<IT> const& d);

};

template<typename T>
std::ostream & operator<<(std::ostream &os, ndom2D<T> const& d)
{
  if (d.empty())
    os << "empty";
  else {
    os << "[" << d.lower << "..";
    os << d.upper << "]";
  }
  return os;
}

} // namespace stapl

#endif // STAPL_VIEWS_DOMAINS_HPP
