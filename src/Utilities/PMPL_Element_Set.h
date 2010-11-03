#ifndef _PMPL_Element_Set_H_
#define _PMPL_Element_Set_H_

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/mpl/begin.hpp>
#include <boost/mpl/end.hpp>
#include <boost/mpl/next_prior.hpp>

#include <map>
#include <string>
using namespace std;

#include "MPProblem.h"


//example: Element = SamplerMethod, ElementTypeList is mpl list of available sampler methods


template<typename Element>
struct element_factory
{
  boost::shared_ptr<Element> operator()(XMLNodeReader& in_Node, MPProblem* in_pProblem) const
  {
    return boost::shared_ptr<Element>(new Element(in_Node, in_pProblem));
  }
};


template<typename Element>
class element_set
{
 protected:
  typedef boost::function<boost::shared_ptr<Element> (XMLNodeReader&, MPProblem*)> factory_type;

  map<string, factory_type> m_universe;
  map<string, boost::shared_ptr<Element> > m_elements;

 public:
  typedef boost::shared_ptr<Element> method_pointer;

  template <typename ElementTypeList>
  element_set(ElementTypeList const& etl)
  {
    add_to_universe(typename boost::mpl::begin<ElementTypeList>::type(), 
                    typename boost::mpl::end<ElementTypeList>::type());
  }

  bool add_element(string const& str, XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
    if(m_universe.find(str) != m_universe.end())
    {
      boost::shared_ptr<Element> e = m_universe[str](in_Node, in_pProblem);
      return add_element(e->GetObjectLabel(), e);
    }
    return false;
  }

  bool add_element(string const& str, boost::shared_ptr<Element> e) 
  {
    if(m_elements.find(e->GetObjectLabel()) == m_elements.end())
      m_elements[e->GetObjectLabel()] = e;
    else
      cerr << "\nWarning, method list already has a pointer associated with \"" << e->GetObjectLabel() << "\", not added\n";
    return true;
  }

  boost::shared_ptr<Element> get_element(string const& name)
  {
    return m_elements[name];
  }

  typename map<string, boost::shared_ptr<Element> >::const_iterator elements_begin() const { return m_elements.begin(); }
  typename map<string, boost::shared_ptr<Element> >::const_iterator elements_end() const { return m_elements.end(); }

 protected:
  template <typename Last>
  void
  add_to_universe(Last, Last)
  { }

  template <typename First, typename Last>
  void
  add_to_universe(First, Last)
  {
    typename boost::mpl::deref<First>::type first;
    m_universe[first.name()] = element_factory<typename boost::mpl::deref<First>::type>();
    add_to_universe(typename boost::mpl::next<First>::type(), Last());
  }
};

#endif
