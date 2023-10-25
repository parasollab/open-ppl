/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_MESH_IMPLICIT_REGULAR_MESH_DOMAIN_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_IMPLICIT_REGULAR_MESH_DOMAIN_HPP

#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/type_traits/is_domain_sparse.hpp>
#include <stapl/utility/tuple.hpp>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/mpl/bool.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor comparing 2 tuples pair-wise.
/// @tparam Op operator type for comparison.
/// @tparam N Last index of the tuple type.
//////////////////////////////////////////////////////////////////////
template <typename Op, int N>
struct pair_wise_tuple_comparison
{
  //////////////////////////////////////////////////////////////////////
  /// @param operator for comparison.
  //////////////////////////////////////////////////////////////////////
  pair_wise_tuple_comparison(Op operation)
   : op(operation)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param t1 first tuple.
  /// @param t2 second tuple.
  /// @return true if all pair-wise comparisons return true
  //////////////////////////////////////////////////////////////////////
  template <typename T1, typename T2>
  bool operator()(T1 const& t1, T2 const& t2)
  {
    return op(get<N>(t1), get<N>(t2)) && pair_wise_tuple_comparison<
                                                 Op,N-1>(op)(t1, t2);
  }
private:
  Op op;
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor comparing 2 tuples pair-wise.
/// @tparam Op operator type for comparison.
///
/// Template specialization to stop the recursion on the tuple elements
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct pair_wise_tuple_comparison<Op,-1>
{
  pair_wise_tuple_comparison(Op)
  { }
  template <typename T1, typename T2>
  bool operator()(T1 const& t1, T2 const& t2)
  {
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Implicit domain representing the cells of a regular mesh.
/// @tparam COORDINATES coordinates type of domain element.
/// @tparam TRAVERSAL traversal type for the linearization of mesh cells.
//////////////////////////////////////////////////////////////////////
template<class COORDINATES,
         class TRAVERSAL = typename default_traversal
                           <tuple_size<COORDINATES>::value>::type>
class implicit_regular_mesh_domain
{
public:
  typedef COORDINATES   tuple_type;
  typedef size_t        gid_type;
  typedef gid_type      index_type;
  typedef size_t        size_type;

private:
  typedef nd_linearize< tuple_type,TRAVERSAL>           linearize_fct;
  typedef nd_reverse_linearize< tuple_type,TRAVERSAL>   reverse_linearize_fct;

  tuple_type m_local_sizes, m_global_sizes;
  tuple_type m_global_origin;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an empty regular mesh domain.
  //////////////////////////////////////////////////////////////////////
  implicit_regular_mesh_domain()
   : m_local_sizes(tuple_type()), m_global_sizes(tuple_type()),
                                  m_global_origin(tuple_type())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param first_elt position of the first cell of the mesh.
  /// @param l_sizes size in each dimension of the mesh.
  /// @param g_sizes size in each dimension of the whole mesh
  ///        if this domain represents a partition.
  //////////////////////////////////////////////////////////////////////
  implicit_regular_mesh_domain(tuple_type const& first_elt,
                               tuple_type const& l_sizes,
                               tuple_type const& g_sizes)
   : m_local_sizes(l_sizes), m_global_sizes(g_sizes),
                             m_global_origin(first_elt)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor taking the size of the mesh in each dimension,
  ///        the coordinates of the origin are set to 0.
  /// @param l_sizes size in each dimension of the mesh.
  //////////////////////////////////////////////////////////////////////
  implicit_regular_mesh_domain(tuple_type const& l_sizes)
   : m_local_sizes(l_sizes), m_global_sizes(l_sizes)
  {
    m_global_origin = reverse_linearize_fct(m_global_sizes)(0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor creating a restricted domain given a first
  ///        element @ref first_elt, a last element @ref last_elt and
  ///        another domain @ref other.
  /// @param first_elt first element.
  /// @param last_elt last element.
  /// @param other other domain.
  //////////////////////////////////////////////////////////////////////
  implicit_regular_mesh_domain(tuple_type const& first_elt,
                               tuple_type const& last_elt,
                               implicit_regular_mesh_domain const& other)
   : m_global_sizes(other.m_global_sizes), m_global_origin(first_elt)
  {
    tuple_type diff = tuple_ops::transform(last_elt, first_elt,
                                           minus<gid_type>());
    m_local_sizes = tuple_ops::transform(diff,
                                         boost::bind(plus<gid_type>(), 1, _1));
  }

  implicit_regular_mesh_domain(index_type const& first_elt,
                               index_type const& last_elt,
                               implicit_regular_mesh_domain const& other)
   : m_global_sizes(other.m_global_sizes),
     m_global_origin(reverse_linearize_fct(m_global_sizes)(first_elt))
  {
    tuple_type first_elt_pos = reverse_linearize_fct(m_global_sizes)(first_elt);
    tuple_type last_elt_pos = reverse_linearize_fct(m_global_sizes)(last_elt);
    tuple_type diff = tuple_ops::transform(last_elt_pos, first_elt_pos,
                                           minus<gid_type>());
    m_local_sizes = tuple_ops::transform(diff,
                                         boost::bind(plus<gid_type>(), 1, _1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the domain.
  //////////////////////////////////////////////////////////////////////
  size_type size() const
  {
    return tuple_ops::fold(m_local_sizes, 1, multiplies<gid_type>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a tuple with the number of elements in each dimension.
  //////////////////////////////////////////////////////////////////////
  tuple_type get_local_sizes() const
  {
    return m_local_sizes;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the first index in the domain.
  //////////////////////////////////////////////////////////////////////
  gid_type first() const
  {
    return linearize_fct(m_global_sizes)(m_global_origin);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last index in the domain.
  //////////////////////////////////////////////////////////////////////
  gid_type last() const
  {
    return advance(first(),size()-1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index after the last one.
  //////////////////////////////////////////////////////////////////////
  gid_type open_last() const
  {
    return advance(first(),size());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if the index @ref idx is in the domain.
  /// @param idx index.
  /// @return true if @ref idx is in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& idx) const
  {
    tuple_type elt_pos = reverse_linearize_fct(m_global_sizes)(idx);
    return contains(elt_pos);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if the tuple index @ref pos is in the domain.
  /// @param pos index tuple.
  /// @return true if @ref pos is in the domain.
  //////////////////////////////////////////////////////////////////////
  bool contains(tuple_type const& pos) const
  {
    tuple_type last_pos = reverse_linearize_fct(m_global_sizes)(last());

    bool sup = pair_wise_tuple_comparison
                <boost::function<bool (gid_type, gid_type)>,
                tuple_size<tuple_type>::value-1>
                (less_equal<gid_type>())(m_global_origin, pos);

    bool inf = pair_wise_tuple_comparison
                <boost::function<bool (gid_type, gid_type)>,
                tuple_size<tuple_type>::value-1>
                (less_equal<gid_type>())(pos, last_pos);

    return (sup && inf);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advance an index @ref idx by @ref n positions in the domain.
  /// @param idx index.
  /// @param n number of positions.
  /// @return advanced index.
  //////////////////////////////////////////////////////////////////////
  gid_type advance(gid_type const& idx,long const& n) const
  {
    tuple_type global_pos = reverse_linearize_fct(m_global_sizes)(idx);

    // Use local coordinate system to advance in the sub_domain
    tuple_type local_pos =
      tuple_ops::transform(global_pos, m_global_origin, minus<gid_type>());

    size_t tmp1 = linearize_fct(m_local_sizes)(local_pos);

    tmp1 += n;

    tuple_type new_local_pos  = reverse_linearize_fct(m_local_sizes)(tmp1);

    tuple_type new_global_pos =
      tuple_ops::transform(new_local_pos, m_global_origin, plus<gid_type>());

    return linearize_fct(m_global_sizes)(new_global_pos);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the positive distance between two indices based
  ///        on the domain order.
  /// @param i0 first index.
  /// @param i1 second index.
  /// @return distance between @ref i0 and @ref i1.
  //////////////////////////////////////////////////////////////////////
  size_t distance(gid_type const& i0, gid_type const& i1) const
  {
    tuple_type l_i0_pos =
      tuple_ops::transform(
        reverse_linearize_fct(m_global_sizes)(i0), m_global_origin,
        minus<gid_type>()
      );

    tuple_type l_i1_pos =
      tuple_ops::transform(
        reverse_linearize_fct(m_global_sizes)(i1), m_global_origin,
        minus<gid_type>()
      );

    gid_type local_id0 = linearize_fct(m_local_sizes)(l_i0_pos);
    gid_type local_id1 = linearize_fct(m_local_sizes)(l_i1_pos);

    return local_id1-local_id0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Linearize a tuple index based on the domain.
  /// @param t1 tuple index.
  /// @return linearized index.
  //////////////////////////////////////////////////////////////////////
  gid_type linearize(tuple_type const& t1) const
  {
    return linearize_fct(m_global_sizes)(t1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reverse linearize an index based on the domain.
  /// @param id index.
  /// @return tuple index.
  //////////////////////////////////////////////////////////////////////
  tuple_type reverse_linearize(gid_type const& id) const
  {
    return reverse_linearize_fct(m_global_sizes)(id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if domain is empty.
  //////////////////////////////////////////////////////////////////////
  bool empty() const
  {
    return (size() == 0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if domain is the same as the container domain.
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain() const
  {
    return (m_local_sizes == m_global_sizes);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Intersect the domain with another domain.
  /// @param other other domain.
  /// @return domain resulting from the intersection.
  //////////////////////////////////////////////////////////////////////
  implicit_regular_mesh_domain operator&
                                  (implicit_regular_mesh_domain const& other)
  {
    if (is_same_container_domain())
      return other;
    else
      return *this;
  }

  void define_type(typer &t)
  {
    t.member(m_local_sizes);
    t.member(m_global_sizes);
    t.member(m_global_origin);
  }

}; // end regular mesh domain class


//////////////////////////////////////////////////////////////////////
/// @brief Output the domain @ref d
///        to an output stream @ref os.
/// @param os output stream.
/// @param d domain.
/// @return the output stream @ref os.
//////////////////////////////////////////////////////////////////////
template<typename T1,typename T2>
std::ostream & operator<<(std::ostream &os,
                          implicit_regular_mesh_domain<T1,T2> const& d)
{
  os << "[";
  for (size_t i=0;i<d.size();++i){
    os << d.advance(d.first(),i);
    os << ", ";
  }
  os << "]";
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Struct helper to select tuple type with the dimension.
/// @tparam DIM dimension.
//////////////////////////////////////////////////////////////////////
template <int DIM>
struct size_tuple
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Struct helper to select tuple type with the dimension.
///
/// Template specialization for dimension 2.
//////////////////////////////////////////////////////////////////////
template<>
struct size_tuple<2>
{
  typedef tuple<size_t, size_t> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Struct helper to select tuple type with the dimension.
///
/// Template specialization for dimension 3.
//////////////////////////////////////////////////////////////////////
template<>
struct size_tuple<3>
{
  typedef tuple<size_t, size_t, size_t> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Make implicit regular mesh domain a sparse domain.
//////////////////////////////////////////////////////////////////////
template <typename Coordinates, typename Traversal>
struct is_domain_sparse<implicit_regular_mesh_domain<
                            Coordinates, Traversal> >
  : boost::mpl::true_
{ };

} //end namespace stapl

#endif /* STAPL_CONTAINERS_GRAPH_MESH_IMPLICIT_REGULAR_MESH_DOMAIN_HPP */
