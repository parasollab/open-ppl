/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ITERATOR_REFERENCE_ITERATOR_H
#define STAPL_ITERATOR_REFERENCE_ITERATOR_H

#include <stapl/views/proxy.h>
#include <stapl/views/iterator/iterator_facade.h>
#include <stapl/utility/invoke_arg.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a read only accessor that references a local element.
//////////////////////////////////////////////////////////////////////
template<typename T>
class cref_accessor
{
public:
  /// @todo This should be private.
  const T * t;

public:
  typedef T value_type;

  cref_accessor(T const& t_)
    : t(&t_)
  { }

  cref_accessor(null_reference const&)
    : t(NULL)
  { }

  cref_accessor(cref_accessor const& other)
    : t(other.t)
  { }

  bool is_null() const
  {
    return t == NULL;
  };

  T read() const
  {
    return *t;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Get rid of typename Class by specialization of descriptor
  ///       for is_class.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Arg1>
  void invoke(void (Class::* memberFuncPtr)(Arg1),
              typename invoke_arg<Arg1>::type const& arg1) const
  {
    (t->*memberFuncPtr)(arg1);
  }

  template<typename Class, typename Arg1, typename Arg2>
  void invoke(void (Class::* memberFuncPtr)(Arg1, Arg2),
                  const typename invoke_arg<Arg1>::type& arg1,
                  const typename invoke_arg<Arg2>::type& arg2) const
  {
    return (t->*memberFuncPtr)(arg1, arg2);
  }


  //////////////////////////////////////////////////////////////////////
  /// @todo Get rid of typename Class by specialization of descriptor
  ///       for is_class.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Rtn>
  Rtn invoke(Rtn (Class::* /*const*/ memberFuncPtr)(void) /*const*/) const
  {
    return (t->*memberFuncPtr)();
  }

  template<typename Class, typename Rtn, typename Arg1, typename Arg2>
  Rtn invoke(Rtn (Class::* memberFuncPtr)(Arg1, Arg2),
                  typename invoke_arg<Arg1>::type const& arg1,
                  typename invoke_arg<Arg2>::type const& arg2) const
  {
    return (t->*memberFuncPtr)(arg1, arg2);
  }

  //fixme get rid of typename Class by specialization of descriptor for is_class
  template<typename Class, typename Rtn>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(void) const) const
  {
    return (t->*memberFuncPtr)();
  }

private:
  void define_type(typer&);
}; //struct cref_accessor



//////////////////////////////////////////////////////////////////////
/// @brief Defines the accessor that references a local element.
//////////////////////////////////////////////////////////////////////
template<typename T>
class ref_accessor
{
  friend class accessor_core_access;

public:
  /// @todo This should be private.
  T* t;

public:
  typedef T value_type;

  ref_accessor(T& t_)
    : t(&t_)
  { }

  ref_accessor(null_reference const&)
    : t(NULL)
  { }

  ref_accessor(ref_accessor const& other)
    : t(other.t)
  { }

  bool is_null() const
  {
    return t == NULL;
  };

  T read() const
  {
    return *t;
  }

  void write(T const& val) const
  {
    *t = val;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Get rid of typename Class by specialization of descriptor
  ///       for is_class.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Arg1>
  void invoke(void (Class::* memberFuncPtr)(Arg1),
              typename invoke_arg<Arg1>::type const& arg1) const
  {
    (t->*memberFuncPtr)(arg1);
  }

  template<typename Class, typename Arg1, typename Arg2>
  void invoke(void (Class::* memberFuncPtr)(Arg1, Arg2),
                    typename invoke_arg<Arg1>::type const& arg1,
                    typename invoke_arg<Arg2>::type const& arg2) const
  {
    return (t->*memberFuncPtr)(arg1, arg2);
  }

  template<typename Class, typename Rtn>
  Rtn invoke(Rtn (Class::* /*const*/ memberFuncPtr)(void) /*const*/) const
  {
    return (t->*memberFuncPtr)();
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Get rid of typename Class by specialization of descriptor
  ///       for is_class.
  //////////////////////////////////////////////////////////////////////
   template<typename Class, typename Rtn>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(void) const) const
  {
    return (t->*memberFuncPtr)();
  }

  template<typename Class, typename Rtn, typename Arg1>
  Rtn invoke(Rtn (Class::* memberFuncPtr)(Arg1),
                  typename invoke_arg<Arg1>::type const& arg1) const
  {
    return (t->*memberFuncPtr)(arg1);
  }

  template<typename Class, typename Rtn, typename Arg1>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(Arg1) const ,
                   typename invoke_arg<Arg1>::type const& arg1) const
  {
    return (t->*memberFuncPtr)(arg1);
  }


  template<typename Class, typename Rtn, typename Arg1, typename Arg2>
  Rtn invoke(Rtn (Class::* memberFuncPtr)(Arg1, Arg2),
                  typename invoke_arg<Arg1>::type const& arg1,
                  typename invoke_arg<Arg2>::type const& arg2) const
  {
    return (t->*memberFuncPtr)(arg1, arg2);
  }

private:
  void define_type(typer&);

  template<typename F>
  void apply_set(F const& f) const
  {
    f(*t);
  }

  template<typename F>
  typename boost::result_of<F(value_type)>::type
  apply_get(F const& f) const
  {
    return f(*t);
  }

}; //struct ref_accessor


template<typename T>
proxy<T, ref_accessor<T> >
make_ref(T const& t)
{
  return proxy<T, ref_accessor<T> >(ref_accessor<T>(t));
}


template<typename T>
proxy<T, cref_accessor<T> >
make_cref(T const& t)
{
  return proxy<T, cref_accessor<T> >(cref_accessor<T>(t));
}

} //namespace stapl

#endif //STAPL_ITERATOR_REFERENCE_ITERATOR_H
