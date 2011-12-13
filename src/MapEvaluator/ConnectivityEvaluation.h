#ifndef _CONNECTIVITY_EVALUATION_H
#define _CONNECTIVITY_EVALUATION_H

#include "CoverageEvaluation.h"
#include "RoadmapGraph.h"
#include "Roadmap.h"
#include "CfgTypes.h"

template <class CFG, class WEIGHT>
class ConnectivityMetric : public CoverageMetric<CFG, WEIGHT>
{
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  public:
    ConnectivityMetric(vector<CFG> s = vector<CFG>(), bool computeAllCCs = false)
      :CoverageMetric<CFG, WEIGHT>(s,computeAllCCs) {
      }
    ~ConnectivityMetric(){}

    double operator()(int _regionID, MPProblem* _mp, vector<string>& _nodeConnectionLabels){
      CoverageMetric<CFG, WEIGHT>::operator()(_regionID, _mp, _nodeConnectionLabels);

      int numQueries = 0;
      for(size_t i=0; i<this->m_connections.size(); ++i)
        sort(this->m_connections[i].begin(), this->m_connections[i].end());
      for(size_t i=0; i<this->m_connections.size()-1; ++i)
        for(size_t j=i+1; j<this->m_connections.size(); ++j) {
          vector<VID> intersection;
          set_intersection(this->m_connections[i].begin(),
              this->m_connections[i].end(),
              this->m_connections[j].begin(),
              this->m_connections[j].end(),
              back_insert_iterator<vector<VID> >(intersection));
          if(!(intersection.empty()))
            numQueries ++; 
        }   
      double pQueries = ((double)numQueries) / 
        ((double)(this->m_connections.size()*(this->m_connections.size()-1))/2.0);
      //cout << "% Queries Solved: " << pQueries << endl;
      return pQueries;
    }
};

template <class CFG, class WEIGHT>
class ConnectivityEvaluation : public CoverageEvaluation<CFG, WEIGHT> 
{
  public:
    ConnectivityEvaluation(vector<CFG>& _s, double _t, vector<string> _nodeConnection, bool _computeAllCCs = true)
      : CoverageEvaluation<CFG, WEIGHT>(_s, _t, _nodeConnection, _computeAllCCs) {
        this->SetName("ConnectivityEvaluator");
      }

    ConnectivityEvaluation(XMLNodeReader& _node, MPProblem* _problem, bool _defaultAllData = true)
      : CoverageEvaluation<CFG, WEIGHT>(_node, _problem, _defaultAllData) {
        this->SetName("ConnectivityEvaluator");
      }

    virtual ~ConnectivityEvaluation(){} 

    virtual bool operator()(){
      return operator()(this->GetMPProblem()->CreateMPRegion());
    }

    virtual bool operator()(int _regionID){
      ConnectivityMetric<CFG, WEIGHT> connectivity(this->m_samples, this->m_allData);
      return (connectivity(_regionID, this->GetMPProblem(), this->m_nodeConnectionLabels) >= this->m_threshold);
    }
};

#endif
