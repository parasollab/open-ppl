/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_VERTEX_ITERATOR_ADAPTOR_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_VERTEX_ITERATOR_ADAPTOR_HPP

#include <boost/iterator/iterator_adaptor.hpp>

namespace stapl {

namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex descriptor iterator.
/// Dereferencing provides the vertex descriptor.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator>
class vd_iterator
  : public boost::iterator_adaptor<vd_iterator<VertexIterator>,
    VertexIterator>
{
private:
  typedef boost::iterator_adaptor<vd_iterator<VertexIterator>,
                                  VertexIterator> base_type;
public:
  typedef typename VertexIterator::vertex_descriptor value_type;
  vd_iterator() = default;

  vd_iterator(VertexIterator iterator)
      : base_type(iterator)
  {}

  value_type operator*() const
  {
    return this->base_reference()->descriptor();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex data iterator to iterate over the vertex properties.
/// Dereferencing provides the vertex property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator>
class vdata_iterator
  : public boost::iterator_adaptor<vdata_iterator<VertexIterator>,
                                   VertexIterator>
{
private:
  typedef boost::iterator_adaptor<vdata_iterator<VertexIterator>,
                                  VertexIterator> base_type;
public:
  typedef typename VertexIterator::property_type value_type;
  vdata_iterator() = default;

  vdata_iterator(VertexIterator iterator)
      : base_type(iterator)
  {}

  value_type& operator*()
  {
    return this->base_reference()->property();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Const vertex data iterator to iterate over the vertex properties.
/// Dereferencing provides the vertex property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator>
class const_vdata_iterator
  : public boost::iterator_adaptor<
             const_vdata_iterator<VertexIterator>, VertexIterator>
{
private:
  typedef boost::iterator_adaptor<const_vdata_iterator<VertexIterator>,
      VertexIterator> base_type;
public:
  typedef typename VertexIterator::property_type value_type;
  const_vdata_iterator() = default;

  const_vdata_iterator(VertexIterator iterator)
      : base_type(iterator)
  { }

  value_type const& operator*() const
  {
    return this->base_reference()->property();
  }
};

} //end namespace sequential
} //end namespace stapl

#endif
