#ifndef CONNECTIONMETHOD_H_
#define CONNECTIONMETHOD_H_

#include <boost/mpl/for_each.hpp>
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"
#include "MPProblem/RoadmapGraph.h"

namespace pmpl_detail{
  template<typename CM, typename RDMP, typename STATS, typename CMAP,
    typename I1, typename I2, typename O>
      struct VirtualConnect{
        public:
          VirtualConnect(CM* _v, RDMP* _r, STATS& _s, CMAP& _c,
              I1 _i1f, I1 _i1l, I2 _i2f, I2 _i2l, O _o) : 
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
          I1 m_i1first, m_i1last;
          I2 m_i2first, m_i2last;
          O m_output;
      };
}

template<class MPTraits>
#ifdef _PARALLEL
class ConnectorMethod : public MPBaseObject<MPTraits>, public stapl::p_object {
#else
class ConnectorMethod : public MPBaseObject<MPTraits> {
#endif
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID; 

    ConnectorMethod();
    ConnectorMethod(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~ConnectorMethod(){}

    /////////////////////////////////////////////
    // Connection Methods
    template<typename ColorMap>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, back_inserter(collision));
      }

    template<typename ColorMap, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          OutputIterator _collision){
        Connect(_rm, _stats, _cmap, 
            _rm->GetGraph()->begin(), _rm->GetGraph()->end(), 
            _rm->GetGraph()->begin(), _rm->GetGraph()->end(), 
            _collision);
      }
    
    template<typename ColorMap>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap, VID _vid){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _vid, back_inserter(collision));
      }
    
    template<typename ColorMap, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          VID _vid, OutputIterator _collision){
        Connect(_rm, _stats, _cmap, _vid, _collision);
      }

    template<typename ColorMap, typename InputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator _itrFirst, InputIterator _itrLast){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _itrFirst, _itrLast, back_inserter(collision));
      }
    
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator _itrFirst, InputIterator _itrLast, 
          OutputIterator _collision){
        Connect(_rm, _stats, _cmap, 
            _itrFirst, _itrLast, 
            _rm->GetGraph()->begin(), _rm->GetGraph()->end(), 
            _collision);
      }

    template<typename ColorMap, typename InputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          VID _vid, 
          InputIterator _itrFirst, InputIterator _itrLast){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _vid, _itrFirst, _itrLast, back_inserter(collision));
      }
    
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          VID _vid, 
          InputIterator _itrFirst, InputIterator _itrLast, 
          OutputIterator _collision){
        vector<VID> vids(1, _vid);
        Connect(_rm, _stats, _cmap, vids.begin(), vids.end(), _itrFirst, _itrLast, back_inserter(_collision));
      }

    template<typename ColorMap, typename InputIterator1, typename InputIterator2>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator1 _itr1First, InputIterator1 _itr1Last, 
          InputIterator2 _itr2First, InputIterator2 _itr2Last){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _itr1First, _itr1Last, _itr2First, _itr2Last, back_inserter(collision));
      }

    template<typename ColorMap, typename InputIterator1, typename InputIterator2, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator1 _itr1First, InputIterator1 _itr1Last, 
          InputIterator2 _itr2First, InputIterator2 _itr2Last, 
          OutputIterator _collision){
        typedef typename MPTraits::ConnectorMethodList MethodList;
        boost::mpl::for_each<MethodList>(
            pmpl_detail::VirtualConnect<
            ConnectorMethod<MPTraits>, RoadmapType, StatClass, ColorMap,
            InputIterator1, InputIterator2, OutputIterator>(
              this, _rm, _stats, _cmap, _itr1First, _itr1Last,
              _itr2First, _itr2Last, _collision)
            );
      }


    void SetPositionResolution(double _posRes) { m_connectionPosRes=_posRes; }
    double GetPositionResolution() { return m_connectionPosRes; }
    void SetOrientationResolution(double _oriRes) { m_connectionOriRes=_oriRes; }
    double GetOrientationResolution() { return m_connectionOriRes; }

    /////////////////////////////////////////////
    // Utility Methods
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

template<class MPTraits>
ConnectorMethod<MPTraits>::ConnectorMethod(){
  this->SetName("ConnectorMethod");
  m_connectionPosRes = 0.05;
  m_connectionOriRes = 0.05;
  m_addPartialEdge = false;
  m_addAllEdges = false;
}

template<class MPTraits>
ConnectorMethod<MPTraits>::ConnectorMethod(MPProblemType* _problem, XMLNodeReader& _node) 
  : MPBaseObject<MPTraits>(_problem, _node){
    this->SetName("ConnectorMethod");
    m_connectionPosRes = _problem->GetEnvironment()->GetPositionRes();
    m_connectionOriRes = _problem->GetEnvironment()->GetOrientationRes();     
    m_nfMethod = _node.stringXMLParameter("nf", true, "", "nf");
    m_lpMethod = _node.stringXMLParameter("lp_method", true, "", "Local Planner");
    m_addPartialEdge = false;
    m_addAllEdges = false;
  }

#endif 

