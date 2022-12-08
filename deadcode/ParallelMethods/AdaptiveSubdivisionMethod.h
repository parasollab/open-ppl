/////////////////////////////////////
// HEADER AdaptiveSubdivisionMethod.h
////////////////////////////////////
#ifndef ADAPTIVESUBDIVISIONMETHOD_H_
#define ADAPTIVESUBDIVISIONMETHOD_H_

#include "MPStrategyMethod.h"
#include "AdaptiveRegion.h"


typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, AdaptiveRegion<CfgType>, WeightType> RGraph;
typedef typename RGraph::vertex_descriptor PVID;
typedef typename RGraph::vertex_iterator PVI;

class XMLNode;
class MPProblem;
template<class CFG, class WEIGHT> class MPRegion;
template<class CFG, class WEIGHT> class ConnectCCs;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class AdaptiveSubdivisionMethod : public MPStrategyMethod {
  public:
    AdaptiveSubdivisionMethod(XMLNode& _node, MPProblem* _problem);
    virtual ~AdaptiveSubdivisionMethod();

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize(int _regionID);
    virtual void Run(int _regionID);
    virtual void Finalize(int _regionID);
    virtual void Print(ostream& _os) const;

  private:
    vector<string> m_regionConnectionLabels;
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    MPRegion<CfgType,WeightType>* m_region;
    RGraph* m_adaptiveRegion;
    vector<string> m_strategiesLabels;
    int m_meshRow,m_meshCol,n_runs, m_k1, m_k2, m_tk;
    double m_xEpsilon, m_yEpsilon, m_zEpsilon, m_tr;
    string m_lp,m_nf, m_ccc,m_vcm, m_dmm;
    std::tr1::tuple<double,int, int> m_classifierParam;
    bool m_useOuterBB;
    ConnectCCs<CfgType, WeightType>* m_ccConnector;
};

#endif
