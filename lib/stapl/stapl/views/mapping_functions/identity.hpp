/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_MAPFUNC_IDENTITY_H
#define STAPL_MAPFUNC_IDENTITY_H


namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an identity mapping function.
///
/// @tparam T Type of index
//////////////////////////////////////////////////////////////////////
template<typename T>
struct f_ident
{
  typedef T          index_type;
  typedef T          gid_type;
  typedef f_ident<T> inverse;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the mapping function type of a view after
  ///        deep slice on the view
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  struct slice_type
  {
    using new_gid_type =
      homogeneous_tuple_type_t<tuple_size<T>::value - tuple_size<Slices>::value,
                               size_t>;

    using type = f_ident<new_gid_type>;
  };

  gid_type operator()(index_type const& x) const
  {
    return x;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief  Applies the mapping on the input indices and calls the
  ///   @c get_reference functor with the mapped indices.
  ///
  /// @tparam RefGetter   Functor specified by an underlying container, that
  ///   returns a reference to the container's element at given position.
  /// @tparam Indices     Indices to be mapped.
  /// @return Reference to the element at the mapped position.
  //////////////////////////////////////////////////////////////////////
  template<typename RefGetter, typename... Indices>
  typename std::result_of<RefGetter(Indices...)>::type
  apply_get(RefGetter const& get_reference, Indices... indices) const
  {
    return get_reference(indices...);
  }
};

} // namespace stapl

#endif
