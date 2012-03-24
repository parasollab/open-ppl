#ifndef REGULARSUBDIVISIONMETHOD_H
#define REGULARSUBDIVISIONMETHOD_H

#include "MPStrategyMethod.h"
#include "Region.h"




typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region, WeightType> RRGraph;
typedef typename RRGraph::vertex_descriptor RVID; 
typedef typename RRGraph::vertex_iterator RVI;

class XMLNodeReader;
class MPProblem;
template<class CFG, class WEIGHT> class MPRegion;



class RegularSubdivisionMethod : public MPStrategyMethod {
  public:
    RegularSubdivisionMethod(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~RegularSubdivisionMethod();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize(int _regionID);
    virtual void Run(int _regionID);
    virtual void Finalize(int _regionID);
    virtual void PrintOptions(ostream& _os);

  private:
    vector<string> m_ComponentConnectionLabels;
    MPRegion<CfgType,WeightType>* m_region;
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_strategiesLabels;
    int m_row,m_col,m_runs, m_k1, m_k2;
    double m_xepsilon, m_yepsilon, m_zepsilon;
    string m_nf, m_ccc;
};

#endif 
