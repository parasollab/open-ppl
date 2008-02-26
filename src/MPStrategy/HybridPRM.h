#ifndef HybridPRM_h
#define HybridPRM_h


#include "SwitchDefines.h"
#include <sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "Input.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"

/* util.h defines EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"

#include "MapEvaluator.h"

#include "MPStrategy/MPStrategyMethod.h"


class HybridPRM : public MPStrategyMethod {
  public:

  HybridPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    LOG_DEBUG_MSG("HybridPRM::HybridPRM()");
    ParseXML(in_pNode);
   // m_percentage_random = double(0.5);
    LOG_DEBUG_MSG("~HybridPRM::HybridPRM()");
    };
  virtual ~HybridPRM() {}

  virtual void PrintOptions(ostream& out_os) { };

  virtual void ParseXML(TiXmlNode* in_pNode);

  virtual void operator()(int in_RegionID);

  string GetNextNodeGenStr(bool learning) {
    double pro_sum(0.0);
    map<string,pair<double, double> > map_pro_range;
    double max_probability=0.0;
    string max_gen = "";
    static string last_gen = "";
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      string gen = m_vecStrNodeGenLabels[i];
      
      double gen_prob;
      if (m_sampler_selection_distribution == "window_uniform" && learning) {
	gen_prob = m_mapStrNodeGenProbabilityUniform[gen];
      } else if (m_sampler_selection_distribution == "nowindow_adaptive") { 
	gen_prob = m_mapStrNodeGenProbabilityUse[gen];
      } else { //if (m_sampler_selection_distribution == "window_hybrid" {
	gen_prob = m_mapStrNodeGenProbability[gen];
      }
      if (gen_prob >= max_probability) {
	max_probability = gen_prob;
	max_gen = gen;
      }
      double upper_bound;
      if (i != m_vecStrNodeGenLabels.size()-1)
	upper_bound = pro_sum + gen_prob;
      else
	upper_bound = 1.0; // do this only in the last number
      pair<double,double> new_range(pro_sum,upper_bound);
      map_pro_range[gen] = new_range;
      pro_sum+= gen_prob;
    }
    cout << "m_sampler=" << m_sampler_selection_distribution << ";learning="<<learning <<";"<< endl;

    if (!learning) {
      if (m_sampler_selection_distribution == "highest") {
	cout << "***	Highest sampler after learning :: " << max_gen << endl;
	return max_gen;
      } /*else if (m_sampler_selection_distribution == "window_hybrid") {
	cout << "***    Last method used (during learning) :: " << last_gen << endl;
	return last_gen;
	}*/
    }

    // it gets here when learning or when not learning and the sampler
    // distribution is neither of the tested in the previous if.
    double random_num = OBPRM_drand();
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      string gen = m_vecStrNodeGenLabels[i];
      pair<double,double> range = map_pro_range[gen];
      if(range.first < random_num && random_num < range.second) { //we have a winner
        cout << "***   The next node generator is::  " << gen << endl;
	last_gen = gen;
        return gen;
      }
    }
    cerr << endl << endl << "This can't be good";
    cerr << endl;
    for (int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      string gen = m_vecStrNodeGenLabels[i];
      pair<double,double> range = map_pro_range[gen];
      cerr << gen << ":: [" << range.first << ", " << range.second << "]" << endl;
    }
    cerr << "Random Number:: " << random_num << endl;
    exit(-1);
  };
  
  double sum_all_weights() {
    double to_return(0.0);
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      to_return += m_mapStrNodeGenWeight[m_vecStrNodeGenLabels[i]];
    }
    return to_return;
  };

  double sum_all_pro_costs() {
    double to_return(0.0);
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      double pro_no_cost = m_mapStrNodeGenProbabilityNoCost[m_vecStrNodeGenLabels[i]];
      unsigned long int cost = m_mapStrNodeGenCost[m_vecStrNodeGenLabels[i]];
      to_return += (pro_no_cost / double(cost));
    }
    //cerr << "Sum_all_pro_costs() = " << to_return << endl;
    return to_return;
  }

  void RewardAndRecalcWeight(string _method, double _rew,unsigned long int _cost) {
    int K = m_vecStrNodeGenLabels.size();
    int num_samp_for_method = m_mapStrNodeGenNumSamp[_method];
    int avg_cost_for_method = m_mapStrNodeGenCost[_method];
    int new_cost_for_method((avg_cost_for_method * num_samp_for_method + _cost) / (num_samp_for_method +1));
    if (m_fixed_cost != 1)
      m_mapStrNodeGenCost[_method] = new_cost_for_method; 
    m_mapStrNodeGenNumSamp[_method]++;
    //cerr << "COST::   " << _method << " = " << _cost << endl;
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      if(m_vecStrNodeGenLabels[i] != _method) { // set reward to 0 for all others
        string gen = m_vecStrNodeGenLabels[i];
        double rew_rew = double(0.0) / m_mapStrNodeGenProbabilityNoCost[gen];
        double new_weight = m_mapStrNodeGenWeight[gen] * exp(double((m_percentage_random) * rew_rew / double(K)));
        m_mapStrNodeGenWeight[gen] = new_weight;
      } else { // reward this one
        string gen = m_vecStrNodeGenLabels[i];
        double rew_rew = double(_rew) / m_mapStrNodeGenProbabilityNoCost[gen];
        double new_weight = m_mapStrNodeGenWeight[gen] * exp(double((m_percentage_random) * rew_rew / double(K)));
        m_mapStrNodeGenWeight[gen] = new_weight;
      }
    }

   //Calculate new probs from weights
    double smallest_weight=-1; // for normalization
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      string gen = m_vecStrNodeGenLabels[i];
      double weight = m_mapStrNodeGenWeight[gen];
      double pro_no_cost = (1 - m_percentage_random) * (weight / sum_all_weights()) + (m_percentage_random / K);
      m_mapStrNodeGenProbabilityNoCost[gen] = pro_no_cost;
      unsigned long int cost = m_mapStrNodeGenCost[m_vecStrNodeGenLabels[i]];
      if(m_count_cost == 0) {
        m_mapStrNodeGenProbability[gen] = m_mapStrNodeGenProbabilityNoCost[gen];
      } else {
        m_mapStrNodeGenProbability[gen] = ((pro_no_cost / double(cost)) / sum_all_pro_costs() );
      }
      if (weight < smallest_weight || smallest_weight == -1)
	smallest_weight = weight;
    }

    // Normalize weights (not needed in original algorithm, but prevents infinite weights)
    for (int i = 0; i < m_vecStrNodeGenLabels.size(); ++i) {
      string gen = m_vecStrNodeGenLabels[i];
      m_mapStrNodeGenWeight[gen] = m_mapStrNodeGenWeight[gen]/smallest_weight;
    }

  };

   pair < unsigned int, unsigned int >
    ConnectionsWitnessToRoadmap(vector < CfgType > & witness_cfgs, Roadmap< CfgType, WeightType > *rdmp, Stat_Class&);


  bool CanConnectComponents(vector < CfgType > & cc_a, vector < CfgType > & cc_b, Stat_Class&);

   void initializeWeightProb() {
    m_mapStrNodeGenWeight.clear();
    m_mapStrNodeGenProbability.clear();
    m_mapStrNodeGenProbabilityUniform.clear();
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      m_mapStrNodeGenWeight[m_vecStrNodeGenLabels[i]] = double(1.0);
      m_mapStrNodeGenProbability[m_vecStrNodeGenLabels[i]] = double(1.0/m_vecStrNodeGenLabels.size());
      m_mapStrNodeGenProbabilityUniform[m_vecStrNodeGenLabels[i]] = double(1.0/m_vecStrNodeGenLabels.size());
      m_mapStrNodeGenProbabilityNoCost[m_vecStrNodeGenLabels[i]] = double(1.0/m_vecStrNodeGenLabels.size());
      if (m_fixed_cost != 1)
	m_mapStrNodeGenCost[m_vecStrNodeGenLabels[i]] = 1;
    }
  };


   void CopyPlearnPuse() {
    m_mapStrNodeGenProbabilityUse.clear();
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      m_mapStrNodeGenProbabilityUse[m_vecStrNodeGenLabels[i]] = m_mapStrNodeGenProbability[m_vecStrNodeGenLabels[i]];
    }
  };

  void outputWeightMatrix(std::ostream& _out) {
      cout << endl;
    for(int i=0; i<m_vecStrNodeGenLabels.size(); ++i) {
      cout << m_vecStrNodeGenLabels[i] << ":: ";
      cout << "Weight = " << m_mapStrNodeGenWeight[m_vecStrNodeGenLabels[i]] << ", ";
      cout << "ProNoCost = " << m_mapStrNodeGenProbabilityNoCost[m_vecStrNodeGenLabels[i]] << ", ";
      cout << "Pro = " << m_mapStrNodeGenProbability[m_vecStrNodeGenLabels[i]] << ",";
      cout << "Cost = " << m_mapStrNodeGenCost[m_vecStrNodeGenLabels[i]] << endl;
    }
  }

  virtual void operator()() {
    int newRegionId = GetMPProblem()->CreateMPRegion();
    (*this)(newRegionId);
  };

  double cc_diamater(RoadmapGraph<CfgType,WeightType>* pGraph, VID _cc);

private:
  vector<string> m_vecStrNodeGenLabels;
  vector<string> m_vecStrNodeConnectionLabels;
  vector<string> m_vecStrComponentConnectionLabels;
  vector<string> m_vecNodeCharacterizerLabels;
  map<string,double> m_mapStrNodeGenWeight;
  map<string,double> m_mapStrNodeGenProbability;
  map<string,double> m_mapStrNodeGenProbabilityUniform;
  map<string,double> m_mapStrNodeGenProbabilityUse;
  map<string,double> m_mapStrNodeGenProbabilityNoCost;
  map<string, unsigned long int> m_mapStrNodeGenCost;
  map<string,int> m_mapStrNodeGenNumSamp;
  map<string,int> m_mapStrNodeGenNumOversamples;

  Stat_Class m_nodeOverheadStat, m_queryStat;
  string m_strBaseFilename;

  string m_strWitnessFilename;
  vector<CfgType> m_vecWitnessNodes;

  double m_percentage_random;
  double m_window_percent;

  int m_totalSamples;
  int m_count_cost;
  int m_fixed_cost;
  int m_resetting_learning;
  int m_bin_size;

  string m_sampler_selection_distribution;
};



#endif
