#ifndef CONNECTIONMETHOD_H_
#define CONNECTIONMETHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>
#include "RoadmapGraph.h"
#include "LocalPlanners.h"
#include "MetricUtils.h"
#include "GraphAlgo.h"
#include "NeighborhoodFinder.h"
#include "MPStrategy.h"
#include "MPUtils.h"
#include <cmath>
#include "Connector.h"

template<class CFG, class WEIGHT>
class NeighborhoodConnection;
template<class CFG, class WEIGHT>
class ConnectCCs;
template<class CFG, class WEIGHT>
class PreferentialAttachment;
template<class CFG, class WEIGHT>
class OptimalConnection;
template<class CFG, class WEIGHT>
class OptimalRewire;

namespace pmpl_detail{
  typedef boost::mpl::list<NeighborhoodConnection<CfgType,WeightType>, 
          ConnectCCs<CfgType,WeightType>, 
          PreferentialAttachment<CfgType,WeightType>, 
          OptimalConnection<CfgType,WeightType>,
          OptimalRewire<CfgType,WeightType>
            > ConnectorMethodList;

  template<typename CM, typename RDMP, typename STATS, typename CMAP,
    typename I, typename O>
      struct VirtualConnect{
        public:
          VirtualConnect(CM* _v, RDMP* _r, STATS& _s, CMAP& _c,
              I _i1f, I _i1l, I _i2f, I _i2l, O _o) : 
            m_memory(_v), m_rdmp(_r), m_stats(_s), m_cmap(_c), m_i1first(_i1f),
            m_i1last(_i1l), m_i2first(_i2f), m_i2last(_i2l), m_output(_o){
            }

          template<typename T>
            void operator()(T& _t) {
              T* tptr = dynamic_cast<T*>(m_memory);
              if(tptr != NULL){
                tptr->Connect(m_rdmp, m_stats, m_cmap, m_i1first, m_i1last, 
                    m_i2first, m_i2last, m_output);
              }
            }
        private:
          CM* m_memory;
          RDMP* m_rdmp;
          STATS& m_stats;
          CMAP& m_cmap; 
          I m_i1first, m_i1last, m_i2first, m_i2last;
          O m_output;
      };
}


template <class CFG, class WEIGHT>
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
    template<typename ColorMap>
    void Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& _cmap){
      vector<CFG> collision;
      Connect(_rm, _stats, _cmap, back_inserter(collision));
    }
    
    template<typename ColorMap, typename OutputIterator>
      void Connect(Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats, ColorMap& _cmap,
          OutputIterator _collision){
        vector<VID> vertices;
        _rm->m_pRoadmap->GetVerticesVID(vertices);
        Connect(_rm, _stats, _cmap, vertices.begin(), vertices.end(), vertices.begin(), vertices.end(), _collision);
      }

    template<typename ColorMap, typename InputIterator>
      void Connect(Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator _itr1First, InputIterator _itr1Last, 
          InputIterator _itr2First, InputIterator _itr2Last){
        vector<CFG> collision;
        Connect(_rm, _stats, _cmap, _itr1First, _itr1Last, _itr2First, _itr2Last, back_inserter(collision));
      }

    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator _itr1First, InputIterator _itr1Last, 
          InputIterator _itr2First, InputIterator _itr2Last, 
          OutputIterator _collision){
        typedef pmpl_detail::ConnectorMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualConnect<
            ConnectionMethod<CFG, WEIGHT>, Roadmap<CFG, WEIGHT>, StatClass, ColorMap,
            InputIterator, OutputIterator>(this, _rm, _stats, _cmap, _itr1First, _itr1Last,
              _itr2First, _itr2Last, _collision));
      }

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
    CDInfo* m_cdInfo;
    string  m_lpMethod;
    string  m_nfMethod;
    bool    m_addPartialEdge;
    bool    m_addAllEdges; 
    double  m_connectionPosRes;
    double  m_connectionOriRes;
};

template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>::
ConnectionMethod(){
  this->SetName("ConnectionMethod");
  m_connectionPosRes = 0.05;
  m_connectionOriRes = 0.05;
}

template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>::
ConnectionMethod(XMLNodeReader& _node, MPProblem* _problem) : MPBaseObject(_node,_problem){
  this->SetName("ConnectionMethod");
  m_connectionPosRes = _problem->GetEnvironment()->GetPositionRes();
  m_connectionOriRes = _problem->GetEnvironment()->GetOrientationRes();     
  m_nfMethod = _node.stringXMLParameter("nf", true, "", "nf");
  m_lpMethod = _node.stringXMLParameter("lp_method", true, "", "Local Planner");
}

#endif 

