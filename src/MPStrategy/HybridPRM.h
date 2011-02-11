#ifndef HybridPRM_h
#define HybridPRM_h


#include "MPStrategyMethod.h"
#include "MPProblem.h"
#include "Roadmap.h"
#include "Stat_Class.h"


struct Visibility
{ 
  int attempts, connections;
  
  Visibility(int a = 0, int c = 0) : attempts(a), connections(c) {}
  ~Visibility() {}

  double ratio() const
  {
    if(attempts == 0)
      return 0.0;
    else
      return (double)connections / (double)attempts;
  }
};


struct NodeTypeCounts
{
  int num_cc_create, num_cc_merge, num_cc_expand, num_cc_oversample;

  NodeTypeCounts() : num_cc_create(0), num_cc_merge(0), num_cc_expand(0), num_cc_oversample(0) {}
  ~NodeTypeCounts() {}

  friend ostream& operator<<(ostream& os, const NodeTypeCounts& nt);
};


class HybridPRM : public MPStrategyMethod 
{
 public:

  HybridPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~HybridPRM();

  virtual void PrintOptions(ostream& out_os);
  virtual void ParseXML(XMLNodeReader& in_Node);

   virtual void Initialize(int in_RegionID);
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID);

  void initialize_weights_probabilities_costs();
  void copy_learned_prob_to_prob_use();
  void print_weights_probabilities_costs(ostream& _out);
  
  string select_next_sampling_method(bool learning);
  
  double compute_visibility_reward(string next_node_gen, double visibility, double threshold, int prev_cc_count, int curr_cc_count, NodeTypeCounts& node_types);
  
  bool in_learning_window(int totalSamples) const;
  
  void reward_and_update_weights_probabilities(string _method, double _rew,unsigned long int _cost);
  
  //pair<unsigned int, unsigned int> ConnectionsWitnessToRoadmap(vector < CfgType > & witness_cfgs, Roadmap< CfgType, WeightType > *rdmp, Stat_Class&);
  //bool CanConnectComponents(vector < CfgType > & cc_a, vector < CfgType > & cc_b, Stat_Class&);

  //double cc_diamater(RoadmapGraph<CfgType,WeightType>* pGraph, VID _cc);

  bool evaluate_map(int in_RegionID);

 protected:
  vector<string> m_node_gen_labels;
  vector<string> m_node_conn_labels;
  //vector<string> m_component_conn_labels;
  vector<string> m_evaluator_labels;

  map<string,double> m_node_gen_weights;
  
  map<string,double> m_node_gen_probabilities;
  map<string,double> m_node_gen_probabilities_uniform;
  map<string,double> m_node_gen_probabilities_use;
  map<string,double> m_node_gen_probabilities_no_cost;
  
  map<string, unsigned long int> m_node_gen_costs;
  
  map<string,int> m_node_gen_num_sampled;
  map<string,int> m_node_gen_num_oversampled;

  //Stat_Class m_query_stat;
  //vector<CfgType> m_witness_nodes;

  double m_percentage_random; //lambda
  double m_window_percent;

  bool m_count_cost;
  bool m_fixed_cost;
  bool m_resetting_learning;
  int m_bin_size;

  string m_sampler_selection_distribution;

 private:
   string base_filename;
   ofstream char_ofstream;
   Clock_Class Allstuff;
   stapl::vector_property_map< GRAPH,size_t > cmap;
   map<VID, Visibility> vis_map;
   NodeTypeCounts node_types;
  int totalSamples;
  bool map_passed_evaluation;
  double NodeGenTotalTime;
  static int instanceNumber;
  string nf_label;
};

#endif
