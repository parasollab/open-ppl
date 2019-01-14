#ifndef METHOD_SET_H_
#define METHOD_SET_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <boost/mpl/list.hpp>
#include <boost/mpl/next_prior.hpp>

#include "IOUtils.h"
#include "XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// @brief Creates new method instances from an XML node.
////////////////////////////////////////////////////////////////////////////////
template <typename Method>
struct MethodFactory final {

  ///@name Local Types
  ///@{

  typedef std::shared_ptr<Method> MethodPointer;

  ///@}
  ///@name Operator
  ///@{

  MethodPointer operator()(XMLNode& _node) const {
    return MethodPointer(new Method(_node));
  }

  ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// Defines basic method container class to hold methods (for classes like
/// DistanceMetricMethod, LocalPlannerMethod, etc).
///
/// MethodTypeList must be defined within templated class of MPTraits
///   e.g., Method = NeighborhoodFinderMethod
///         MethodTypeList = boost::mpl::list<BruteForceNF, BandsNF, ...>
///   e.g., Method = LocalPlannerMethod
///         MethodTypeList = boost::mpl::list<Straightline, RotateAtS, ...>
///
/// MethodSet first parses the MethodTypeList and adds all enumerated methods to
/// its universe of method types. Then, specific instantiations of those types
/// can be added to its map of available methods by calling 'AddMethod'.
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits, typename Method>
class MethodSet final {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::MPLibrary                  MPLibrary;

    typedef std::shared_ptr<Method>                       MethodPointer;
    typedef typename std::map<std::string, MethodPointer> MethodMap;

    typedef std::function<MethodPointer(XMLNode&)>        FactoryType;
    typedef typename std::map<std::string, FactoryType>   FactoryMap;

    typedef typename MethodMap::iterator                  iterator;
    typedef typename MethodMap::const_iterator            const_iterator;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a MethodSet from a MethodTypeList defined in the MPTraits class.
    /// @param _mtl An instance of the method type list.
    /// @param _name The name of this method set.
    template <typename MethodTypeList>
    MethodSet(MPLibrary* const _p, const MethodTypeList& _mtl,
        const std::string& _name);

    ///@}
    ///@name Method Accessors
    ///@{

    /// Add the appropriate methods from an XML node.
    void ParseXML(XMLNode& _node);

    void AddMethod(XMLNode& _node);

    void AddMethod(MethodPointer _e, const std::string& _label);

    /// Get a method by label.
    /// @param _label The method label.
    /// @return The corresponding method pointer.
    MethodPointer GetMethod(const std::string& _label);

    /// Find a method iterator.
    /// @param _label The method label.
    /// @return The iterator to that method, or nullptr if it is not found.
    iterator FindMethod(std::string _label);

    /// Prepare all methods in this set for execution on the owning MPLibrary's
    /// current MPProblem.
    void Initialize();

    /// Display the instantiated methods.
    void Print(std::ostream& _os) const;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the instantiated methods.

    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    ///@}

  private:

    ///@name Initialization Helpers
    ///@{
    /// Add a set of methods to the factory map recursively using template
    /// metaprogramming.

    /// Add the range [first, last) to the universe.
    template <typename First, typename Last>
    void AddToUniverse(First, Last);

    /// Base case for terminating recursion.
    template <typename Last>
    void AddToUniverse(Last, Last);

    ///@}
    ///@name Internal State
    ///@{

    MPLibrary* const m_library; ///< The owning planning library.

    std::string m_name;     ///< The name of this set of methods.
    std::string m_default;  ///< The name of the default method in this set.

    FactoryMap m_universe;  ///< The set of allowed methods.
    MethodMap m_elements;   ///< The set of instantiated methods.

    ///@}

};

/*------------------------------- Method Set ---------------------------------*/

template <typename MPTraits, typename Method>
template <typename MethodTypeList>
MethodSet<MPTraits, Method>::
MethodSet(MPLibrary* const _p, const MethodTypeList& _mtl,
    const std::string& _name) : m_library(_p), m_name(_name) {
  AddToUniverse(typename boost::mpl::begin<MethodTypeList>::type(),
                typename boost::mpl::end<MethodTypeList>::type());
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    AddMethod(child);
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
AddMethod(XMLNode& _node) {
  auto iter = m_universe.find(_node.Name());

  // Skip if method isn't in universe.
  if(iter == m_universe.end())
    return;

  MethodPointer e = iter->second(_node);
  AddMethod(e, e->m_label);
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
AddMethod(MethodPointer _e, const std::string& _label) {
  auto iter = m_universe.find(_e->m_name);

  // Throw exception if method isn't in universe.
  if(iter == m_universe.end())
    throw ParseException(WHERE, "Method '" + _e->m_name +
        "' is not contained within the motion planning universe.");

  _e->SetMPLibrary(m_library);
  _e->SetLabel(_label);
  if(m_elements.empty())
    m_default = _label;
  if(m_elements.find(_label) == m_elements.end())
    m_elements[_label] = _e;
  else
    cerr << "\nWarning, method list already has a pointer associated with "
         << "\"" << _label << "\", not added\n";
}


template <typename MPTraits, typename Method>
typename MethodSet<MPTraits, Method>::MethodPointer
MethodSet<MPTraits, Method>::
GetMethod(const std::string& _label) {
  MethodPointer element = (_label == "") ? m_elements[m_default] :
                                           m_elements[_label];
  if(element.get() == nullptr) {
    std::string err = "Element '" + _label + "' does not exist in " + m_name +
        ". Choices are: ";
    for(auto& elem : m_elements)
      if(elem.second.get())
        err += " '" + elem.first + "',";
    err.pop_back();
    throw RunTimeException(WHERE, err);
  }
  return element;
}


template <typename MPTraits, typename Method>
typename MethodSet<MPTraits, Method>::iterator
MethodSet<MPTraits, Method>::
FindMethod(std::string _label) {
  if(_label == "")
    _label = m_default;
  return m_elements.find(_label);
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
Initialize() {
  for(auto& elem : m_elements)
    elem.second->Initialize();
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
Print(ostream& _os) const {
  size_t count = 0;

  _os << "\n" << m_name << " has these methods available::\n\n";

  for(auto& elem : m_elements) {
    _os << ++count << ") \"" << elem.first << "\" (" << elem.second->m_name
        << ")\n";
    elem.second->Print(_os);
    _os << std::endl;
  }
  _os << std::endl;
}

/*--------------------------------- Iteration --------------------------------*/

template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::iterator
MethodSet<MPTraits, Method>::
begin() noexcept {
  return m_elements.begin();
}


template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::iterator
MethodSet<MPTraits, Method>::
end() noexcept {
  return m_elements.end();
}


template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::const_iterator
MethodSet<MPTraits, Method>::
begin() const noexcept {
  return m_elements.begin();
}


template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::const_iterator
MethodSet<MPTraits, Method>::
end() const noexcept {
  return m_elements.end();
}

/*-------------------------- Initialization Helpers --------------------------*/

template <typename MPTraits, typename Method>
template <typename First, typename Last>
void
MethodSet<MPTraits, Method>::
AddToUniverse(First, Last) {
  using FirstType = typename boost::mpl::deref<First>::type;
  FirstType first;
  m_universe[first.m_name] = MethodFactory<FirstType>();
  AddToUniverse(typename boost::mpl::next<First>::type(), Last());
}


template <typename MPTraits, typename Method>
template <typename Last>
void
MethodSet<MPTraits, Method>::
AddToUniverse(Last, Last) {}

/*----------------------------------------------------------------------------*/

#endif
