#ifndef NONSTD_COLLECTION_H_
#define NONSTD_COLLECTION_H_

#include <cstdlib>
#include <unordered_set>
#include <vector>

#include "nonstd/runtime.h"

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// A self-managing container for dynamically allocated objects that provides a
  /// serial number or index for every element.
  ///
  /// Removing elements does not change the indexes: instead, the now-free index
  /// from the removed element is returned to an unordered set of available
  /// indexes.
  /// @ingroup Containers
  //////////////////////////////////////////////////////////////////////////////
  template <typename element_type>
  class collection
  {

    ///@name Internal State
    ///@{

    std::unordered_set<size_t> m_free_indexes;  ///< Indexes that can be reused.
    std::vector<element_type*> m_elements;      ///< The element pointers.

    ///@}

    public:

      ///@name Construction
      ///@{

      collection() = default;

      virtual ~collection();

      ///@}
      ///@name Element Accessors
      ///@{

      size_t add(element_type*);   ///< Add an element and receive its index.
      element_type* take(size_t);  ///< Remove an element by index.

      element_type* operator[](size_t);             ///< Get an element by index.
      const element_type* operator[](size_t) const; ///< Get an element by index.

      std::vector<element_type*> get_all();             ///< Get all elements.
      std::vector<const element_type*> get_all() const; ///< Get all elements.

      ///@}
      ///@name Index Accessors
      ///@{

      size_t size() const noexcept;            ///< Get the number of elements.
      std::vector<size_t> get_indexes() const; ///< Get all used indexes.

      ///@}
  };

  /*--------------------------- Construction ---------------------------------*/

  template <typename element_type>
  collection<element_type>::
  ~collection()
  {
    for(auto& elem : m_elements)
      delete elem;
  }

  /*------------------------- Element Functions ------------------------------*/

  template <typename element_type>
  size_t
  collection<element_type>::
  add(element_type* _e)
  {
    // Guard against null insert
    assert_msg(_e, "nonstd::collection error: requested insertion of null "
        "object");

    // Add _e at the next unused index.
    size_t index;
    if(m_free_indexes.size()) {
      auto iter = m_free_indexes.begin();
      index = *iter;
      m_free_indexes.erase(iter);
      m_elements[index] = _e;
    }
    else {
      index = m_elements.size();
      m_elements.push_back(_e);
    }

    return index;
  }


  template <typename element_type>
  element_type*
  collection<element_type>::
  take(size_t _i)
  {
    auto elem = this->operator[](_i);
    m_elements[_i] = nullptr;
    m_free_indexes.insert(_i);

    return elem;
  }


  template <typename element_type>
  element_type*
  collection<element_type>::
  operator[](size_t _i)
  {
    assert_msg(_i < m_elements.size(), "nonstd::collection error: out-of-bounds "
        "request for index " + std::to_string(_i) + " (max index is " +
        std::to_string(m_elements.size() - 1) + ")");

    return m_elements[_i];
  }


  template <typename element_type>
  const element_type*
  collection<element_type>::
  operator[](size_t _i) const
  {
    return operator[](_i);
  }


  template <typename element_type>
  std::vector<element_type*>
  collection<element_type>::
  get_all()
  {
    std::vector<element_type*> out;
    out.reserve(size());
    for(auto& elem : m_elements)
      if(elem)
        out.push_back(elem);
    return out;
  }


  template <typename element_type>
  std::vector<const element_type*>
  collection<element_type>::
  get_all() const
  {
    std::vector<const element_type*> out;
    out.reserve(size());
    for(const auto& elem : m_elements)
      if(elem)
        out.push_back(elem);
    return out;
  }

  /*---------------------------- Index Functions -----------------------------*/

  template <typename element_type>
  inline
  size_t
  collection<element_type>::
  size() const noexcept
  {
    return m_elements.size() - m_free_indexes.size();
  }


  template <typename element_type>
  std::vector<size_t>
  collection<element_type>::
  get_indexes() const
  {
    std::vector<size_t> indexes;
    indexes.reserve(size());
    for(size_t i = 0; i < m_elements.size(); ++i)
      if(m_elements[i])
        indexes.push_back(i);
    return indexes;
  }

  /*--------------------------------------------------------------------------*/

}

#endif
