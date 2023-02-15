/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_UTILITY_INVOKER_HPP
#define STAPL_VIEWS_METADATA_UTILITY_INVOKER_HPP

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/pack_ops.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Invokes an arbitrary functor with a @ref p_object as a parameter.
///
/// @tparam O The p_object type
/// @tparam F The function object type
//////////////////////////////////////////////////////////////////////
template<typename O, typename F>
class invoker
 : public p_object, private F
{
  using object_type = O;

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the invoker.
  //////////////////////////////////////////////////////////////////////
  struct destroy
  {
    invoker* const m_invoker;

    explicit destroy(invoker& i) noexcept
    : m_invoker(&i)
    { }

    ~destroy(void)
    { delete m_invoker; }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the function on the resolved @ref p_object.
  //////////////////////////////////////////////////////////////////////
  typename std::result_of<F(object_type const&)>::type
  operator()(object_type const& obj)
  {
    return static_cast<F const&>(*this)(obj);
  }



public:
  invoker(F const& f = F()) : F(f) { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resolve the handle of the @ref p_object and call the function
  ///        object on the pointer to the resolved @ref p_object.
  //////////////////////////////////////////////////////////////////////
  auto invoke(rmi_handle::reference const& handle)
    -> decltype(this->operator()(std::declval<object_type const&>()))
  {
    auto obj = resolve_handle<object_type>(handle);

    stapl_assert(obj != nullptr, "Handle of object not resolved in invoker");

    return this->operator()(*obj);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resolve the handle of the @ref p_object and call the function
  ///        object on the pointer to the resolved @ref p_object and delete the
  ///        invoker after that.
  //////////////////////////////////////////////////////////////////////
  auto invoke_and_delete(rmi_handle::reference const& handle)
    -> decltype(this->operator()(std::declval<object_type const&>()))
  {
    destroy d{*this};
    return invoke(handle);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Create an rmi_handle reference to an invoker object that has
///        been constructed on the same set of locations as a given
///        p_object.
///
/// @param obj The p_object whose gang will be used to create the invoker
///            object.
/// @param f   A unary function whose argument is a p_object.
///
/// @return A reference to an rmi_handle of an @see invoker object.
//////////////////////////////////////////////////////////////////////
template<typename O, typename F>
rmi_handle::reference make_invoker(O const& obj, F&& f)
{
  using invoker_type = invoker<O, typename std::decay<F>::type>;

  auto handle_future = construct<invoker_type>(
    obj.get_rmi_handle(), all_locations, std::forward<F>(f)
  );

  return handle_future.get();
}

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_UTILITY_INVOKER_HPP
