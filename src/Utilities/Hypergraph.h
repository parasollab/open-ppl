#ifndef _PPL_HYPERGRAPH_H_
#define _PPL_HYPERGRAPH_H_

#include "ConfigurationSpace/GenericStateGraph.h"

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

    typedef GenericStateGraph<VertexType,HyperarcType> GraphType;

    ///@}
    ///@name Construction
    ///@{

    Hypergraph();

    ~Hypergraph();

    ///@}
    ///@name Vertex Accessor Functions
    ///@{

    virtual size_t AddVertex(VertexType _vertex);

    Vertex& GetVertex(size_t _vid);

    VertexType& GetVertexType(size_t _vid);

    size_t GetVID(VertexType _vertex) const;

    const std::unordered_map<size_t,Vertex>& GetVertexMap() const;

    const size_t Size() const;

    ///@}
    ///@name Hyperarc Accessor Functions
    ///@{

    virtual size_t AddHyperarc(std::set<size_t> _head, std::set<size_t> _tail,
                       HyperarcType _hyperarc, bool _overWrite = false, 
                       bool _returnExisting = false);

    Hyperarc& GetHyperarc(size_t _hid);

    size_t GetHID(std::set<size_t> _head, std::set<size_t> _tail);

    HyperarcType& GetHyperarcType(size_t _hid);

    const std::set<size_t>& GetIncomingHyperarcs(size_t _vid) const;

    const std::set<size_t>& GetOutgoingHyperarcs(size_t _vid) const;

    const std::unordered_map<size_t,Hyperarc>& GetHyperarcMap() const;

    ///@}

    void Print() const;

    void Print(std::string _filename) const;

    GraphType* GetGraph();
    GraphType* GetReverseGraph();

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

    GraphType m_graph;
    GraphType m_reverseGraph;

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

  // Check if vertex exists already
  auto vid = GetVID(_vertex);
  if(vid != MAX_UINT)
    return vid;

  Vertex v;
  v.property = _vertex;
  v.vid = m_vertexCounter++;

  m_vertexMap[v.vid] = v;
  m_incomingHyperarcs[v.vid] = {};
  m_outgoingHyperarcs[v.vid] = {};

  // Add to underlying graph
  m_graph.AddVertex(_vertex);
  m_reverseGraph.AddVertex(_vertex);

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

  return MAX_UINT;
}
    
template <typename VertexType, typename HyperarcType>
const size_t 
Hypergraph<VertexType,HyperarcType>::
Size() const {
  return m_vertexMap.size();
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
            HyperarcType _hyperarc, bool _overWrite, bool _returnExisting) {

  // Check if hyperarc already exists
  if(!_head.empty()) {
    auto vid = *(_head.begin());
    auto incomingHIDs = GetIncomingHyperarcs(vid);
    for(auto hid : incomingHIDs) {
      auto hyperarc = GetHyperarc(hid);
      if(hyperarc.head == _head and hyperarc.tail == _tail) {
        // Hyperarc already exists
        // Make sure there isn't a new property
        if(hyperarc.property != _hyperarc) {
          if(_overWrite)
            m_hyperarcMap[hid].property = _hyperarc;
          else {
            if(_returnExisting)
              return hid;

            throw RunTimeException(WHERE) << "Tried to add a different value for the hyperarc: "
                                        << hyperarc.hid;
          }
        }
        return hyperarc.hid;
      }
    }
  }

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

  // Add to underlying graph
  for(auto source : _tail) {
    for(auto target : _head) {
      m_graph.AddEdge(source,target,_hyperarc);
      m_reverseGraph.AddEdge(target,source,_hyperarc);
    }
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
size_t
Hypergraph<VertexType,HyperarcType>::
GetHID(std::set<size_t> _head, std::set<size_t> _tail) {
  
  if(!_head.empty()) {
    auto vid = *(_head.begin());
    auto incomingHIDs = GetIncomingHyperarcs(vid);
    for(auto hid : incomingHIDs) {
      auto hyperarc = GetHyperarc(hid);
      if(hyperarc.head == _head and hyperarc.tail == _tail) {
        return hyperarc.hid;
      }
    }
  }

  return MAX_UINT;
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

template <typename VertexType, typename HyperarcType>
void
Hypergraph<VertexType,HyperarcType>::
Print(std::string _filename) const {

  std::ofstream ofs(_filename);

  ofs << "PRINTING HYPERGRAPH" << std::endl;

  ofs << "Hyperarcs" << std::endl;
  for(auto kv : m_hyperarcMap) {
    auto hid = kv.first;
    auto arc = kv.second;

    ofs << hid << " : {Tail:[";

    for(auto iter = arc.tail.begin(); iter != arc.tail.end(); iter++) {
      ofs << *iter;
      auto next = iter;
      next++;
      if(next != arc.tail.end()) {
        ofs << ",";
      }
    }

    ofs << "], Head:[";

    for(auto iter = arc.head.begin(); iter != arc.head.end(); iter++) {
      ofs << *iter;
      auto next = iter;
      next++;
      if(next != arc.head.end()) {
        ofs << ",";
      }
    }
    ofs << "]}" << std::endl;
  }

  ofs << "END OF HYPERGRAPH" << std::endl;
  ofs.close();
}

template <typename VertexType, typename HyperarcType>
typename Hypergraph<VertexType,HyperarcType>::GraphType*
Hypergraph<VertexType,HyperarcType>::
GetGraph() {
  return &m_graph;
}

template <typename VertexType, typename HyperarcType>
typename Hypergraph<VertexType,HyperarcType>::GraphType*
Hypergraph<VertexType,HyperarcType>::
GetReverseGraph() {
  return &m_reverseGraph;
}

/*-------------------------------------------------------------------*/

#endif
