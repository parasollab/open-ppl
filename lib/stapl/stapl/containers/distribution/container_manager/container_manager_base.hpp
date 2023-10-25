/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_MANAGER_BASE_HPP
#define STAPL_CONTAINERS_CONTAINER_MANAGER_BASE_HPP

#include <utility>

#include <stapl/utility/invoke_arg.hpp>

namespace stapl {

template<typename Registry>
class container_manager_base
  : public Registry
{
protected:
  typedef Registry storage_type;

public:
  STAPL_IMPORT_TYPE(typename storage_type, base_container_type)
  STAPL_IMPORT_TYPE(typename storage_type, iterator)
  STAPL_IMPORT_TYPE(typename storage_type, const_iterator)

  STAPL_IMPORT_TYPE(typename base_container_type, gid_type)
  STAPL_IMPORT_TYPE(typename base_container_type, value_type)

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this base container manager is
  /// responsible for an element.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& gid) const
  {
    return this->find(gid) != this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns number of elements in all base containers owned
  /// by this base container manager.
  //////////////////////////////////////////////////////////////////////
  size_t num_elements(void) const
  {
    return std::accumulate(
      this->begin(), this->end(), 0,
      [](size_t sum, base_container_type const& bc)
        { return sum + bc.size(); }
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid), "invoking a function on gid not in container");

    this->find(gid)->apply_set(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the element specified by the GID.
  /// @param gid The GID of the element.
  /// @param f The Functor to apply on the element.
  /// @todo remove either this method or apply_set (redundant).
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply(gid_type const& gid, Functor const& f)
  {
    this->apply_set(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  ///   if the element exists at this location.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  bool contains_apply_set(gid_type const& gid, Functor const& f)
  {
    const iterator iter = this->find(gid);

    if (iter == this->end())
      return false;

    iter->apply_set(gid, f);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID
  /// and returns the result.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename boost::result_of<Functor(value_type&)>::type
  apply_get(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid), "base container for gid not found");

    return this->find(gid)->apply_get(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID
  /// and returns the result.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f) const
  {
    stapl_assert(contains(gid), "base container for gid not found");

    return this->find(gid)->apply_get(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  ///   if the element exists at this location.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename Functor::result_type>
  contains_apply_get(gid_type const& gid, Functor const& f)
  {
    typedef boost::optional<typename Functor::result_type> return_t;

    const iterator iter = this->find(gid);

    if (iter == this->end())
      return return_t();

    return return_t(iter->apply_get(gid, f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  ///   if the element exists at this location.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename boost::result_of<Functor(value_type&)>::type>
  contains_apply_get(gid_type const& gid, Functor const& f) const
  {
    typedef boost::optional<
      typename boost::result_of<Functor(value_type&)>::type
    > return_t;

    const const_iterator iter = this->find(gid);

    if (iter == this->end())
      return return_t();

    return return_t(iter->apply_get(gid, f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  /// @return True if gid was found and functor applied, otherwise return false.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename... PMFArgs, typename... Args>
  bool contains_invoke(gid_type const& gid,
                       void (Class::* pmf)(PMFArgs...),
                       Args&&... args)
  {
    const iterator iter = this->find(gid);

    if (iter == this->end())
      return false;

    ((*iter).*pmf)(std::forward<Args>(args)...);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  /// @return boost::optional with result of invocation if element was found.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Rtn, typename... PMFArgs, typename... Args>
  boost::optional<Rtn>
  contains_invoke(gid_type const& gid,
                  Rtn (Class::* pmf)(PMFArgs...),
                  Args&&... args)
  {
    const iterator iter = this->find(gid);

    if (iter == this->end())
      return boost::optional<Rtn>();

    stapl_assert(contains(gid), "base container for gid not found");

    return boost::optional<Rtn>(((*iter).*pmf)(std::forward<Args>(args)...));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on an element at a certain GID.
  ///   The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename... PMFArgs, typename... Args>
  void invoke(gid_type const& gid,
              void (Class::* const pmf)(PMFArgs...),
              Args&&... args)
  {
    stapl_assert(contains(gid), "base container for gid not found");

    ((*this->find(gid)).*pmf)(std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on an element at a certain GID. The
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  /// @return The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Rtn, typename... PMFArgs, typename... Args>
  Rtn invoke(gid_type const& gid,
             Rtn (Class::* pmf)(PMFArgs...),
             Args&&... args)
  {
    stapl_assert(contains(gid), "base container for gid not found");

    return ((*this->find(gid)).*pmf)(std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on an element at a certain GID. The
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  /// @return The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Rtn, typename... PMFArgs, typename... Args>
  Rtn const_invoke(gid_type const& gid,
                   Rtn (Class::* const pmf)(PMFArgs...),
                   Args&&... args) const
  {
    stapl_assert(contains(gid), "base container for gid not found");

    return ((*this->find(gid)).*pmf)(std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method returning void on the base container
  /// that minimizes a user-defined comparator over the base containers.
  /// @param comp Comparator that receives two base containers and returns
  /// the more optimal of the two.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  //////////////////////////////////////////////////////////////////////
  template<typename Comparator, typename Class,
           typename... PMFArgs, typename... Args>
  void min_invoke(Comparator const& comp,
                  void (Class::* pmf)(PMFArgs...),
                  Args&&... args)
  {
    const iterator iter = std::min_element(this->begin(), this->end(), comp);

    ((*iter).*pmf)(std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method returning void on the base container
  /// that minimizes a user-defined comparator over the base containers.
  /// @param comp Comparator that receives two base containers and returns
  /// the more optimal of the two.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  /// @return The result of invoking the function pointer
  //////////////////////////////////////////////////////////////////////
  template<typename Comparator, typename Class, typename Rtn,
           typename... PMFArgs, typename... Args>
  Rtn min_invoke(Comparator const& comp,
                 Rtn (Class::* pmf)(PMFArgs...),
                 Args&&... args)
  {
    const iterator iter = std::min_element(this->begin(), this->end(), comp);

    return ((*iter).*pmf)(std::forward<Args>(args)...);
  }
}; // class container_manager_base

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINER_MANAGER_BASE_HPP
