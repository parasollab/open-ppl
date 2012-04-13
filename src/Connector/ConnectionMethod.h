#ifndef CONNECTIONMETHOD_H_
#define CONNECTIONMETHOD_H_

#include "RoadmapGraph.h"
#include "LocalPlanners.h"
#include "MetricUtils.h"
#include "GraphAlgo.h"
#include "NeighborhoodFinder.h"
#include "MPStrategy.h"
#include "MPUtils.h"
#include <cmath>

template <typename CFG, typename WEIGHT>
class ConnectionMethod : public MPBaseObject {
  public:
    /////////////////////////////////////////////
    // Constructors
    ConnectionMethod();
    ConnectionMethod(XMLNodeReader& _node, MPProblem* _problem);

    /////////////////////////////////////////////
    // Destructor
    virtual ~ConnectionMethod(){}

    /////////////////////////////////////////////
    // Connection Methods
    template<typename OutputIterator, typename ColorMap>
      void Connect( Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats, ColorMap cmap,
          OutputIterator _collision){
        cout << "ConnectionMethod<CFG,WEIGHT>::Connect(..) base method is not available!" << endl;
        exit(-1);
      }

    template<typename InputIterator, typename OutputIterator, typename ColorMap>
      void Connect( Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats, ColorMap cmap,
          InputIterator _itr1First, InputIterator _itr1Last, 
          OutputIterator _collision){
        cout << "ConnectionMethod<CFG,WEIGHT>::Connect(..) base method is not available!" << endl;
        exit(-1);

      }

    template<typename InputIterator, typename OutputIterator, typename ColorMap>
      void Connect( Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats, ColorMap cmap,
          InputIterator _itr1First, InputIterator _itr1Last, 
          InputIterator _itr2First, InputIterator _itr2Last, 
          OutputIterator _collision){
        cout << "ConnectionMethod<CFG,WEIGHT>::Connect(..) base method is not available!" << endl;
        exit(-1);
      }

  public:
    /////////////////////////////////////////////
    // Utility Methods
    typedef typename RoadmapGraph<CFG, WEIGHT>::vertex_descriptor VID; 
    typename vector<pair<pair<VID, VID>, bool> >::const_iterator ConnectionAttemptsBegin() const { return m_connectionAttempts.begin(); }
    typename vector<pair<pair<VID, VID>, bool> >::const_iterator ConnectionAttemptsEnd() const { return m_connectionAttempts.end(); }
    void ClearConnectionAttempts() { m_connectionAttempts.clear(); }
    virtual void PrintOptions(ostream& _os){
      _os << "Name: " << this->GetName() << " ";
      _os << "connPosRes: " << m_connectionPosRes << " ";
      _os << "connOriRes: " << m_connectionOriRes << " ";
      _os << "addPartialEdge: " << m_addPartialEdge << " ";
      _os << "addAllEdges: " << m_addAllEdges << " ";
      _os << "nf: " << m_nfMethod << " ";
      _os << "lp: " << m_lpMethod << " ";
    }

  protected:
    vector<pair<pair<VID, VID>, bool> > m_connectionAttempts;
    string  m_lpMethod;
    string  m_nfMethod;
    bool    m_addPartialEdge;
    bool    m_addAllEdges; 
    double  m_connectionPosRes;
    double  m_connectionOriRes;

};

/* MOVE THIS TO THE MPUtils.h

/////////////////////////////////////////////
// Utility Comparison Struct
template <typename CFG>
struct CfgDistCompare : public binary_function<pair<CFG,double>, pair<CFG,double>, bool> {
bool operator()(const pair<CFG,double>& _p1,
const pair<CFG,double>& _p2) const {
return (_p1.second < _p2.second);
}
};
 */

template <typename CFG, typename WEIGHT>
ConnectionMethod<CFG,WEIGHT>::
ConnectionMethod(){
  this->SetName("ConnectionMethod");
  m_connectionPosRes = 0.05;
  m_connectionOriRes = 0.05;
}

template <typename CFG, typename WEIGHT>
ConnectionMethod<CFG,WEIGHT>::
ConnectionMethod(XMLNodeReader& _node, MPProblem* _problem) : MPBaseObject(_node,_problem){
  this->SetName("ConnectionMethod");
  m_connectionPosRes = _problem->GetEnvironment()->GetPositionRes();
  m_connectionOriRes = _problem->GetEnvironment()->GetOrientationRes();     
  m_nfMethod = _node.stringXMLParameter("nf", true, "", "nf");
  m_lpMethod = _node.stringXMLParameter("lp_method", true, "", "Local Planner");
}

#endif 

