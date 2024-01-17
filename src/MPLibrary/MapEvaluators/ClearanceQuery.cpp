#include "ClearanceQuery.h"

#include "MPLibrary/MPLibrary.h"

#include <map>
#include <set>

/*------------------------------- Construction -------------------------------*/

ClearanceQuery::
ClearanceQuery() : QueryMethod() {
  this->SetName("ClearanceQuery");
  m_cachedEdges = new std::map<std::pair<VID, VID>, double>;
}


ClearanceQuery::
ClearanceQuery(XMLNode& _node) : MapEvaluatorMethod(_node), QueryMethod(_node) {
  this->SetName("ClearanceQuery");

  m_intermediateEdgeVCLabel = _node.Read("ievcLabel", true, "",
         "the edge intermediate VC label for weighted clearance checking");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
ClearanceQuery::
Print(std::ostream& _os) const {
  QueryMethod::Print(_os);
  _os << std::endl;
}


void
ClearanceQuery::
Initialize() {
  QueryMethod::Initialize();
}

/*--------------------------- QueryMethod Overrides --------------------------*/

void
ClearanceQuery::
Reset(RoadmapType* const _r) {
  QueryMethod::Reset(_r);
  m_cachedEdges = new std::map<std::pair<VID, VID>, double>;
  m_cachedEdges->clear();
}


double
ClearanceQuery::
StaticPathWeight(EI& _ei, 
  const double _sourceDistance, const double _targetDistance) const {
    auto vc = this->GetMPLibrary()->GetEdgeValidityChecker(m_intermediateEdgeVCLabel);

    VID source = _ei->source();
    VID target = _ei->target();

    // This calculation can be any other weight as user desires.
    double clearance = 0;

    std::pair<VID, VID> st(source, target);
    std::pair<VID, VID> ts(target, source);
    if (m_cachedEdges->count(st) == 1 || m_cachedEdges->count(ts) == 1) {
        if (this->m_debug) {
            std::cout << "Edge " << source << ", " << target << " already cached." <<std::endl;
        }
        
        clearance = m_cachedEdges->at(st);
    } else {
        clearance = vc->EdgeWeightedClearance(source, target); 
		    CacheEdge(source, target, clearance);
    }
    
    double edgeWeight = 1./clearance;

    if (clearance == 0) {
        edgeWeight = MAX_DBL;       
    }

    if(this->m_debug) {
        std::cout << "Edge weighted clearance " << clearance;
        std::cout << "; vids " << source << " " << target << std::endl;
    }

    return std::max(_sourceDistance, edgeWeight);
}

/*----------------------------------------------------------------------------*/

void
ClearanceQuery::
CacheEdge(VID _u, VID _v, double _value) const {
    std::pair<VID, VID> uv (_u, _v);
    std::pair<VID, VID> vu (_v, _u);

    m_cachedEdges->emplace(uv, _value);
    m_cachedEdges->emplace(vu, _value);

    if (this->m_debug) {
        std::cout << "Cached edge " << _u << ", " << _v << ". " ;
        std::cout << m_cachedEdges->size() << " edges cached." << std::endl;
    }

}
