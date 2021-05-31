#ifndef _PPL_HYPERGRAPH_H_
#define _PPL_HYPERGRAPH_H_

#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"

#include <set>
#include <unordered_map>

template <typename VertexType, typename HyperarcType>
class Hypergraph {

  public:

    ///@name Local Types
    ///@{

    struct Vertex {
      size_t vid;
      VertexType property;
    };

    struct Hyperarc {
      size_t hid;
      std::set<size_t> head;
      std::set<size_t> tail;
      HyperarcType property;
    };

    ///@}
    ///@name Construction
    ///@{

    Hypergraph();

    ~Hypergraph();

    ///@}
    ///@name Vertex Accessor Functions
    ///@{

    size_t AddVertex(VertexType _vertex);

    Vertex& GetVertex(size_t _vid);

    VertexType& GetVertexType(size_t _vid);

    size_t GetVID(VertexType _vertex) const;

    const std::unordered_map<size_t,Vertex>& GetVertexMap() const;

    ///@}
    ///@name Hyperarc Accessor Functions
    ///@{

    size_t AddHyperarc(std::set<size_t> _head, std::set<size_t> _tail,
                       HyperarcType _hyperarc);

    Hyperarc& GetHyperarc(size_t _hid);

    HyperarcType& GetHyperarcType(size_t _hid);

    const std::set<size_t>& GetIncomingHyperarcs(size_t _vid) const;

    const std::set<size_t>& GetOutgoingHyperarcs(size_t _vid) const;

    const std::unordered_map<size_t,Hyperarc>& GetHyperarcMap() const;

    ///@}

    void Print() const;

  private: 

    ///@name Helper Functions
    ///@{
    
    ///@}
    ///@name Internal State
    ///@{

    std::unordered_map<size_t,Vertex> m_vertexMap;

    std::unordered_map<size_t,Hyperarc> m_hyperarcMap;

    std::unordered_map<size_t,std::set<size_t>> m_incomingHyperarcs;

    std::unordered_map<size_t,std::set<size_t>> m_outgoingHyperarcs;

    size_t m_vertexCounter{0};

    size_t m_hyperarcCounter{0};

    ///@}
};

/*-------------------------- Construction --------------------------*/

template <typename VertexType, typename HyperarcType>
Hypergraph<VertexType,HyperarcType>::
Hypergraph() { }

template <typename VertexType, typename HyperarcType>
Hypergraph<VertexType,HyperarcType>::
~Hypergraph() { 
  std::cout << "DESTRUCTING HYERPGRAPH\n"
            << "Num Vertices: " << m_vertexCounter
            << "Num Hyperarcs: " << m_hyperarcCounter
            << std::endl;
}

/*------------------------- Vertex Accessors ------------------------*/

template <typename VertexType, typename HyperarcType>
size_t 
Hypergraph<VertexType,HyperarcType>::
AddVertex(VertexType _vertex) {

  Vertex v;
  v.property = _vertex;
  v.vid = m_vertexCounter++;

  m_vertexMap[v.vid] = v;
  m_incomingHyperarcs[v.vid] = {};
  m_outgoingHyperarcs[v.vid] = {};

  return v.vid;
}

template <typename VertexType, typename HyperarcType>
typename Hypergraph<VertexType,HyperarcType>::Vertex& 
Hypergraph<VertexType,HyperarcType>::
GetVertex(size_t _vid) {

  auto iter = m_vertexMap.find(_vid);
  if(iter == m_vertexMap.end())
    throw RunTimeException(WHERE) << "Requested vertex "
                                  << _vid
                                  << " that does not exist."
                                  << std::endl;

  return iter->second;
}

template <typename VertexType, typename HyperarcType>
VertexType& 
Hypergraph<VertexType,HyperarcType>::
GetVertexType(size_t _vid) {

  auto iter = m_vertexMap.find(_vid);
  if(iter == m_vertexMap.end())
    throw RunTimeException(WHERE) << "Requested vertex "
                                  << _vid
                                  << " that does not exist."
                                  << std::endl;

  return iter->second.property;
}

template <typename VertexType, typename HyperarcType>
size_t 
Hypergraph<VertexType,HyperarcType>::
GetVID(VertexType _vertex) const {

  for(auto iter = m_vertexMap.begin(); iter != m_vertexMap.end(); iter++) {
    if(iter->second.property == _vertex)
      return iter->first;
  }

  return MAX_INT;
}
    
template <typename VertexType, typename HyperarcType>
const std::unordered_map<size_t,
      typename Hypergraph<VertexType,HyperarcType>::Vertex>& 
Hypergraph<VertexType,HyperarcType>::
GetVertexMap() const {
  return m_vertexMap;
}

/*------------------------- Hyperarc Accessors ----------------------*/

template <typename VertexType, typename HyperarcType>
size_t 
Hypergraph<VertexType,HyperarcType>::
AddHyperarc(std::set<size_t> _head, std::set<size_t> _tail,
            HyperarcType _hyperarc) {

  Hyperarc h;
  h.property = _hyperarc;
  h.head = _head;
  h.tail = _tail;
  h.hid = m_hyperarcCounter++;

  m_hyperarcMap[h.hid] = h;

  for(auto vid : h.head) {
    m_incomingHyperarcs[vid].insert(h.hid);
  }

  for(auto vid : h.tail) {
    m_outgoingHyperarcs[vid].insert(h.hid);
  }

  return h.hid;
}

template <typename VertexType, typename HyperarcType>
typename Hypergraph<VertexType,HyperarcType>::Hyperarc&
Hypergraph<VertexType,HyperarcType>::
GetHyperarc(size_t _hid) {

  auto iter = m_hyperarcMap.find(_hid);
  if(iter == m_hyperarcMap.end()) 
    throw RunTimeException(WHERE) << "The requested hyperarc "
                                  << _hid
                                  << " does not exists."
                                  << std::endl;

  return iter->second;
}

template <typename VertexType, typename HyperarcType>
HyperarcType&
Hypergraph<VertexType,HyperarcType>::
GetHyperarcType(size_t _hid) {

  auto iter = m_hyperarcMap.find(_hid);
  if(iter == m_hyperarcMap.end()) 
    throw RunTimeException(WHERE) << "The requested hyperarc "
                                  << _hid
                                  << " does not exists."
                                  << std::endl;

  return iter->second.property;
}

template <typename VertexType, typename HyperarcType>
const std::set<size_t>&
Hypergraph<VertexType,HyperarcType>::
GetIncomingHyperarcs(size_t _vid) const {
  
  auto iter = m_vertexMap.find(_vid);
  if(iter == m_vertexMap.end())
    throw RunTimeException(WHERE) << "Requested vertex "
                                  << _vid
                                  << " that does not exist."
                                  <<std::endl;

  return m_incomingHyperarcs.at(_vid);
}

template <typename VertexType, typename HyperarcType>
const std::set<size_t>&
Hypergraph<VertexType,HyperarcType>::
GetOutgoingHyperarcs(size_t _vid) const {

  auto iter = m_vertexMap.find(_vid);
  if(iter == m_vertexMap.end())
    throw RunTimeException(WHERE) << "Requested vertex "
                                  << _vid
                                  << " that does not exist."
                                  <<std::endl;

  return m_outgoingHyperarcs.at(_vid);
}

template <typename VertexType, typename HyperarcType>
const std::unordered_map<size_t,
    typename Hypergraph<VertexType,HyperarcType>::Hyperarc>& 
Hypergraph<VertexType,HyperarcType>::
GetHyperarcMap() const {
  return m_hyperarcMap;
}

/*-------------------------------------------------------------------*/

template <typename VertexType, typename HyperarcType>
void
Hypergraph<VertexType,HyperarcType>::
Print() const {

  std::cout << "PRINTING HYPERGRAPH" << std::endl;

  std::cout << "Hyperarcs" << std::endl;
  for(auto kv : m_hyperarcMap) {
    auto hid = kv.first;
    auto arc = kv.second;

    std::cout << hid << " : {Tail:[";

    for(auto iter = arc.tail.begin(); iter != arc.tail.end(); iter++) {
      std::cout << *iter;
      auto next = iter;
      next++;
      if(next != arc.tail.end()) {
        std::cout << ",";
      }
    }

    std::cout << "], Head:[";

    for(auto iter = arc.head.begin(); iter != arc.head.end(); iter++) {
      std::cout << *iter;
      auto next = iter;
      next++;
      if(next != arc.head.end()) {
        std::cout << ",";
      }
    }
    std::cout << "]}" << std::endl;
  }

  std::cout << "END OF HYPERGRAPH" << std::endl;
}

/*-------------------------------------------------------------------*/

#endif
