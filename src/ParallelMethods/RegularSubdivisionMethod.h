//////////////////////////////////
//HEADER RegularSubdivisionMethod.h
/////////////////////////////////

#ifndef REGULARSUBDIVISIONMETHOD_H_
#define REGULARSUBDIVISIONMETHOD_H_

#include "MPStrategyMethod.h"
#include "Region.h"
#include "BoundingBox.h"
//#include "ConnectCCs.h"


typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region<BoundingBox>, WeightType> RRGraph;
typedef typename RRGraph::vertex_descriptor RVID; 
typedef typename RRGraph::vertex_iterator RVI;

class XMLNodeReader;
class MPProblem;
template<class CFG, class WEIGHT> class ConnectCCs;

class RegularSubdivisionMethod : public MPStrategyMethod {
  public:
    RegularSubdivisionMethod(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~RegularSubdivisionMethod();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

  private:
    vector<string> m_regionConnectionLabels;
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_strategiesLabels;
    int m_row,m_col,m_runs, m_k1, m_k2;
    double m_xEpsilon, m_yEpsilon, m_zEpsilon;
    string m_nf, m_ccc, m_lp;
    ConnectCCs<CfgType, WeightType>* m_ccConnector;
    MPProblem* m_problem; 
};

#endif 
