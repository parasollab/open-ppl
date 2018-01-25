#ifndef DYNAMIC_WEIGHT_MAP_H_
#define DYNAMIC_WEIGHT_MAP_H_

#include <map>
#include <unordered_map>

#include "containers/sequential/graph/graph_util.h"
#include "containers/sequential/graph/algorithms/graph_algo_util.h"

#include "Utilities/PMPLExceptions.h"


////////////////////////////////////////////////////////////////////////////////
/// Defines a dynamic property map for use in STAPL graph search operations.
/// Maps vertex or edge ID's to a 'ValueType'.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType, typename ValueType>
struct DynamicWeightMap
  : public stapl::sequential::graph_external_property_map<
        GraphType,
        std::unordered_map<typename GraphType::vertex_descriptor, ValueType>,
        stapl::ident_prop_func<ValueType>
    >
{

  ///@name Local Types
  ///@{

  typedef stapl::sequential::graph_external_property_map<
        GraphType,
        std::unordered_map<typename GraphType::vertex_descriptor, ValueType>,
        stapl::ident_prop_func<ValueType>
  > base_type;

  using base_type::property_value_type;

  typedef typename GraphType::vertex_descriptor VID;

  ///@name Construction
  ///@{

  DynamicWeightMap();

  ///@}
  ///@name Map Interface
  ///@{

  /// Map a weight for a vertex or edge..
  template <typename KeyType>
  void put(KeyType _key, const ValueType& _v);

  /// Get a weight for a vertex or edge.
  template <typename KeyType>
  ValueType get(KeyType _key);

  ///@}

  private:

    ///@name Internal State
    ///@{

    std::unordered_map<VID, ValueType> m_map; ///< Property map.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename GraphType, typename ValueType>
DynamicWeightMap<GraphType, ValueType>::
DynamicWeightMap() : base_type(m_map) {}

/*------------------------------ Map Interface -------------------------------*/

template <typename GraphType, typename ValueType>
template <typename KeyType>
void
DynamicWeightMap<GraphType, ValueType>::
put(KeyType _key, const ValueType& _v)
{
  base_type::put(_key, _v);
}


template <typename GraphType, typename ValueType>
template <typename KeyType>
ValueType
DynamicWeightMap<GraphType, ValueType>::
get(KeyType _key)
{
  return base_type::get(_key);
}

/*----------------------------------------------------------------------------*/

#endif
