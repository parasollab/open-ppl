#include "HybridPRM.h"
#include "MPRegion.h"
#include "MPStrategy.h"

#include "boost/lambda/lambda.hpp"

int HybridPRM::instanceNumber=0;

ostream& operator<<(ostream& os, const NodeTypeCounts& nt)
{
  os << ":" << nt.num_cc_create << ":" << nt.num_cc_merge << ":" << nt.num_cc_expand << ":" << nt.num_cc_oversample;
  return os;
}


HybridPRM::
HybridPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem) : MPStrategyMethod(in_Node,in_pProblem) 
{
  LOG_DEBUG_MSG("HybridPRM::HybridPRM()");
  ParseXML(in_Node);
  LOG_DEBUG_MSG("~HybridPRM::HybridPRM()");
}


HybridPRM::
~HybridPRM() 
{}


void 
HybridPRM::
ParseXML(XMLNodeReader& in_Node) 
{
  LOG_DEBUG_MSG("HybridPRM::ParseXML()");
  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) 
  {
    if(citr->getName() == "node_generation_method") 
    {
      string node_gen_method = citr->stringXMLParameter("Method",true,"","Method");
      m_node_gen_labels.push_back(node_gen_method);
      int initial_cost = citr->numberXMLParameter("initial_cost",false,1,1,MAX_INT,"initial_cost");
      m_node_gen_costs[node_gen_method] = initial_cost;
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "node_connection_method") 
    {
      string connect_method = citr->stringXMLParameter("Method",true,"","Method");
      m_node_conn_labels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "evaluation_method")
    {
      string evalMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Evaluation Method"));
      m_evaluator_labels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "stat_dm_method") 
    {
      dm_label = citr->stringXMLParameter(string("Method"),true, string(""),string("Distance Metric Method for Stats computations"));
      citr->warnUnrequestedAttributes();
    } 
    else 
      citr->warnUnknownNode();
  }
  
  m_percentage_random = in_Node.numberXMLParameter("percent_random",true,double(0.5),double(0),double(1),"percent_random");
  
  m_bin_size = in_Node.numberXMLParameter("bin_size",true,5,1,MAX_INT,"bin_size");

  m_window_percent = in_Node.numberXMLParameter("window_percent",true,double(0.5),double(0),double(1),"window_percent");

  m_count_cost = in_Node.boolXMLParameter("Count_Cost",true,true,"Count_Cost");

  m_fixed_cost = in_Node.boolXMLParameter("fixed_cost",true,false,"fixed_cost");

  m_resetting_learning = in_Node.boolXMLParameter("resetting_learning",true,false,"resetting_learning");

  m_sampler_selection_distribution = in_Node.stringXMLParameter("sampler_selection_distribution",false,"","sampler_selection_distribution");
  if (m_sampler_selection_distribution == "nowindow")
    m_window_percent = 1.0; // 100% of the time learning

  LOG_DEBUG_MSG("~HybridPRM::ParseXML()");
}


void
HybridPRM::
PrintOptions(ostream& out_os) 
{
  using boost::lambda::_1;

  out_os << "HybridPRM::\n";
  out_os << "\tpercent_random = " << m_percentage_random << endl;
  out_os << "\tbin_size = " << m_bin_size << endl;
  out_os << "\twindow_percent = " << m_window_percent << endl;
  out_os << "\tcount_cost = " << m_count_cost << endl;
  out_os << "\tfixed_cost = " << m_fixed_cost << endl;
  out_os << "\tresetting_learning = " << m_resetting_learning << endl;
  out_os << "\tsampler_selection_distribution = " << m_sampler_selection_distribution << endl;

  out_os << "\tnode_generation_methods: "; for_each(m_node_gen_labels.begin(), m_node_gen_labels.end(), out_os << _1 << " "); out_os << endl;
  out_os << "\tnode_connection_methods: "; for_each(m_node_conn_labels.begin(), m_node_conn_labels.end(), out_os << _1 << " "); out_os << endl;
  //out_os << "\tcomponent_connection_methods: "; for_each(m_component_conn_labels.begin(), m_component_conn_labels.end(), out_os << _1 << " "); out_os << endl;
  out_os << "\tevaluator_methods: "; for_each(m_evaluator_labels.begin(), m_evaluator_labels.end(), out_os << _1 << " "); out_os << endl;
  
  //out_os << "\twitness_queries:\n"; for_each(m_witness_nodes.begin(), m_witness_nodes.end(), out_os << constant("\t\t") << _1 << " "); out_os << endl;
}

void HybridPRM::Initialize(int in_RegionID){
   ostringstream oss;
   oss<<"Hybrid PRM Initializing in region "<<in_RegionID<<endl;
   LOG_DEBUG_MSG(oss.str());

   PrintOptions(cout);

   OBPRM_srand(getSeed()); 
   //set up base_filename for output filese
  stringstream ssRandomSeed;
  ssRandomSeed << instanceNumber << ".";
  ssRandomSeed << getSeed();
  base_filename = getBaseFilename() + "." + ssRandomSeed.str();
  instanceNumber++;
 
  //setup output of statistics
  string outCharname = base_filename + ".char";
  char_ofstream.open(outCharname.c_str());
  char_ofstream << "#env_file_name:seed:num_node_gen:node_gen_methods" << endl;
  char_ofstream << GetMPProblem()->GetEnvironment()->GetEnvFileName() << ":" << getSeed() << ":" << "HybridPRM" << ":" << *m_node_conn_labels.begin() << ":" <<  endl;
  char_ofstream << "#num_nodes:num_cd_calls:num_ccs";//:diameter_largest:diameter_sum";
  char_ofstream << ":num_cc_create:num_cc_merge:num_cc_expand:num_cc_oversample";
  //char_ofstream << ":query_solved";
  //char_ofstream << ":num_visibility_low:num_visibility_medium:num_visibility_high";
  char_ofstream << ":this_node_type";
  using boost::lambda::constant;
  using boost::lambda::_1;
  for_each(m_node_gen_labels.begin(), m_node_gen_labels.end(), char_ofstream << constant(":") << _1 << "_probability");
  for_each(m_node_gen_labels.begin(), m_node_gen_labels.end(), char_ofstream << constant(":") << _1 << "_num_nodes");
  for_each(m_node_gen_labels.begin(), m_node_gen_labels.end(), char_ofstream << constant(":") << _1 << "_num_cc_oversample");
  char_ofstream << endl;

   Allstuff.StartClock("Everything");
  
   //initialize weights, probabilities, costs, set m_node_gen_probabilities_use = m_node_gen_probabilities
   initialize_weights_probabilities_costs();
   copy_learned_prob_to_prob_use();
  //initialize visibility maps for calculating rewards for cc_expand nodes
  //map<VID, Visibility> vis_map;
  
  //initialize node_types accounting for stats
  //NodeTypeCounts node_types;
  //int num_vis_low(0), num_vis_medium(0), num_vis_high(0);
  
  //initialize diameter accounting for stats
  //double total_dia_time(0),largest_cc_dia(0),sum_cc_dia(0);
    
  //double witness_connectivity(0), witness_coverage(0), total_query_time(0);
  //unsigned int witness_queries = m_witness_nodes.size()*(m_witness_nodes.size()-1)/2;

 

   LOG_DEBUG_MSG("End Hybrid PRM Initializing");
}

void HybridPRM::Run(int in_RegionID){
   ostringstream oss;
   oss<<"Hybrid PRM Running in region "<<in_RegionID<<endl;
   LOG_DEBUG_MSG(oss.str());
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* pStatClass = region->GetStatClass();
   totalSamples = 0;
   map_passed_evaluation = false;
   NodeGenTotalTime = 0;
  while(!map_passed_evaluation)
  {
    cout << "\n\nStart of bin.\n";
    print_weights_probabilities_costs(cout);
    
    do 
    {
      if((totalSamples % m_bin_size == 0) || (in_learning_window(totalSamples) != in_learning_window(totalSamples - 1)))
      {
        if(in_learning_window(totalSamples))
          cout << "\n --> In learning window <--\n\n";
        else
          cout << "\n --> Outside learning window <--\n\n";
      }

      //select next sampler based on probabilities
      string next_node_gen = select_next_sampling_method(in_learning_window(totalSamples));
      cout << "selecting sampler \"" << next_node_gen << "\"\n";

      //generate 1 sample
      Clock_Class NodeGenClock;
      NodeGenClock.StartClock("Node Generation");
      unsigned long int num_cd_before_gen = pStatClass->GetIsCollTotal();
      vector<CfgType> vectorCfgs, in_nodes(1);
      Sampler<CfgType>::SamplerPointer pNodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(next_node_gen);
      pNodeGen->Sample(GetMPProblem()->GetEnvironment(), *pStatClass, in_nodes.begin(), in_nodes.end(), 1, back_inserter(vectorCfgs));
      unsigned long int num_cd_after_gen = pStatClass->GetIsCollTotal();

      //for each valid sampled node, connect it to the roadmap, record reward and cost, update sampler probabilities
      for(vector<CfgType>::iterator C = vectorCfgs.begin(); C != vectorCfgs.end(); ++C)
      { 
        if(C->IsLabel("VALID") && C->GetLabel("VALID")) 
        {
          cmap.reset();
          int nNumPrevCCs = get_cc_count(*(region->roadmap.m_pRoadmap), cmap);
	          
          //add node to roadmap
          VID newVID = region->roadmap.m_pRoadmap->AddVertex(*C);
	    
          //connect node to roadmap
          unsigned long int num_cd_before_conn = pStatClass->GetIsCollTotal();
          vector<pair<pair<VID,VID>,bool> > connection_attempts;
    	  for(vector<string>::iterator itr = m_node_conn_labels.begin(); itr != m_node_conn_labels.end(); ++itr) 
          {
            vector<VID> new_free_vid(1, newVID);
            vector<VID> map_vids;
	    region->roadmap.m_pRoadmap->GetVerticesVID(map_vids);

            ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
            connectmap->GetNodeMethod(*itr)->clear_connection_attempts();
            connectmap->ConnectNodes(connectmap->GetNodeMethod(*itr), region->GetRoadmap(), *pStatClass, 
                                     GetMPProblem()->GetMPStrategy()->GetLocalPlanners(), 
                                     GetMPProblem()->GetMPStrategy()->addPartialEdge,
                                     GetMPProblem()->GetMPStrategy()->addAllEdges,
                                     new_free_vid.begin(), new_free_vid.end(),
                                     map_vids.begin(), map_vids.end()); 
            connection_attempts.insert(connection_attempts.end(), 
                                       connectmap->GetNodeMethod(*itr)->connection_attempts_begin(), 
                                       connectmap->GetNodeMethod(*itr)->connection_attempts_end());
    	  }
          unsigned long int num_cd_after_conn = pStatClass->GetIsCollTotal();
          
          //update visibilities based on connection attempts
          for(vector<pair<pair<VID,VID>,bool> >::const_iterator CA = connection_attempts.begin(); CA != connection_attempts.end(); ++CA) 
          {
            vis_map[CA->first.first].attempts++;
            vis_map[CA->first.second].attempts++;
            if(CA->second) 
            {
              vis_map[CA->first.first].connections++;
              vis_map[CA->first.second].connections++;
            }
          }

          /*
          //update visibility bin counts for stats output
          if(vis_map[newVID].ratio() < 0.3)
            num_vis_low++;
          else if(vis_map[newVID].ratio() < 0.6)
            num_vis_medium++;
          else
            num_vis_high++;
          */
    	  
          //compute sampler cost
          //unsigned long int cost = (double)(num_cd_after_conn - num_cd_before_gen) / (double)vectorCfgs.size();
          //unsigned long int cost = (double)(num_cd_after_gen - num_cd_before_gen) / (double)vectorCfgs.size() + (num_cd_after_conn - num_cd_before_conn);
          unsigned long int cost = (double)(num_cd_after_gen - num_cd_before_gen) / (double)vectorCfgs.size();
          cout << "avg node gen cost = " << (double)(num_cd_after_gen - num_cd_before_gen) / (double)vectorCfgs.size() << endl;
          cout << "connection cost = " << num_cd_after_conn - num_cd_before_conn << endl;
          cout << "cost used = " << cost << endl;

          //compute reward
          cmap.reset();
          int nNumCurrCCs = get_cc_count(*(region->roadmap.m_pRoadmap), cmap);
          double reward = compute_visibility_reward(next_node_gen, vis_map[newVID].ratio(), 0.3, nNumPrevCCs, nNumCurrCCs, node_types);
          cout << "reward = " << reward << endl;

          //update sampler probabilities
          if(in_learning_window(totalSamples))
          {
    	    reward_and_update_weights_probabilities(next_node_gen, reward, cost);
            cout << "new weights and probabilities:";
            print_weights_probabilities_costs(cout);
          }
          else
           cout << endl;
          m_node_gen_num_sampled[next_node_gen]++;

          //update node count
          ++totalSamples;

    	  //if at the end of the bin, update probabilities, reinitialize weights
          if(totalSamples % m_bin_size == 0) 
          {
            cout << "\nEnd of bin.\n";
            if(m_sampler_selection_distribution == "nowindow")
            {
              copy_learned_prob_to_prob_use();
              cout << "no learning window, copying learned probabilities to use probabilities:";
              print_weights_probabilities_costs(cout);
            }
    	    if(m_resetting_learning)
            {
              initialize_weights_probabilities_costs();
              cout << "learning reset, re-initialiing weights and probabilities:";
              print_weights_probabilities_costs(cout);
            }
    	  }

          /*
          //Compute Witness Coverage and Connectivity
          Clock_Class query_time;
          query_time.StartClock("query_time");
          if(witness_connectivity != 100) 
          { //all queries not already solved
            pair<unsigned int, unsigned int> witness_qry = ConnectionsWitnessToRoadmap(m_witness_nodes,&(region->roadmap),m_query_stat);
            witness_coverage = 100*witness_qry.first/m_witness_nodes.size();
            witness_connectivity = 100*witness_qry.second/witness_queries;
          }
          query_time.StopClock();
          total_query_time += query_time.GetClock_SEC();
          cout << "witness_coverage = " << witness_coverage << endl;
          cout << "witness_connectivity = " << witness_connectivity << endl;
          */

          /*
          //Compute Diameters
          Clock_Class dia_time;
          dia_time.StartClock("dia_time");
          if((totalSamples % 50 == 0) || (totalSamples == 1)) 
          { //only every 50 samples
            vector<pair<size_t, VID> > cc;
            cmap.reset();
            get_cc_stats(*(region->roadmap.m_pRoadmap), cmap, cc);
            largest_cc_dia = sum_cc_dia = 0;
            for(vector<pair<size_t, VID> >::const_iterator CC = cc.begin(); CC != cc.end(); ++CC) 
            {
              double _cc_dia = cc_diamater(region->roadmap.m_pRoadmap, CC->second);
              sum_cc_dia += _cc_dia;
              largest_cc_dia = max(largest_cc_dia, _cc_dia);
    	    }
          }
          dia_time.StopClock();
          total_dia_time += dia_time.GetClock_SEC();
          */
   
          //Print out node stats
          cmap.reset();
          char_ofstream << totalSamples
                        << ":" << pStatClass->GetIsCollTotal()
                        << ":" << get_cc_count(*(region->roadmap.m_pRoadmap), cmap) 
                        //<< ":" << largest_cc_dia << ":" << sum_cc_dia
                        << node_types
                        //<< ":" << witness_connectivity
                        //<< ":" << num_vis_low << ":" << num_vis_medium << ":" << num_vis_high;
                        ;
          char_ofstream << ":" << next_node_gen;
          for(vector<string>::const_iterator S = m_node_gen_labels.begin(); S != m_node_gen_labels.end(); ++S) 
            char_ofstream << ":" << m_node_gen_probabilities[*S];
          for(vector<string>::const_iterator S = m_node_gen_labels.begin(); S != m_node_gen_labels.end(); ++S) 
            char_ofstream << ":" << m_node_gen_num_sampled[*S];
          for(vector<string>::const_iterator S = m_node_gen_labels.begin(); S != m_node_gen_labels.end(); ++S) 
            char_ofstream << ":" << m_node_gen_num_oversampled[*S];
          char_ofstream << endl;

      	} //endif GetLabel && IsLabel
      } //endfor vectorCfgs

      NodeGenClock.StopClock();
      NodeGenTotalTime += NodeGenClock.GetClock_SEC();

    } while((totalSamples % m_bin_size) > 0); // (totalSamples % m_bin_size) > (m_bin_size * m_window_percent));

    map_passed_evaluation = evaluate_map(in_RegionID);
  } //end while !map_passed_evaluation

   LOG_DEBUG_MSG("End Hybrid PRM Running");
}

void HybridPRM::Finalize(int in_RegionID){
   ostringstream oss;
   oss<<"Hybrid PRM Finalizing in region "<<in_RegionID<<endl;
   LOG_DEBUG_MSG(oss.str());
   char_ofstream.close();
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* pStatClass = region->GetStatClass();

   //output map
  string outputFilename = base_filename+ ".map";
  ofstream myofstream(outputFilename.c_str());
  if(!myofstream) 
  {
    LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
    exit(-1);
  }
  region->WriteRoadmapForVizmo(myofstream);
  myofstream.close();

  //output stats
  pStatClass->ComputeInterCCFeatures(region->GetRoadmap(), GetMPProblem()->GetDistanceMetric()->GetDMMethod(dm_label));
  pStatClass->ComputeIntraCCFeatures(region->GetRoadmap(), GetMPProblem()->GetDistanceMetric()->GetDMMethod(dm_label));
  string outStatname = base_filename+ ".stat";
  std::ofstream  stat_ofstream(outStatname.c_str());
  std::streambuf* sbuf = std::cout.rdbuf(); // to be restored later
  std::cout.rdbuf(stat_ofstream.rdbuf());   // redirect destination of std::cout
  cout << "NodeGen+Connection Stats" << endl;
  pStatClass->PrintAllStats(region->GetRoadmap());
  cout << "Node Gen = " << NodeGenTotalTime << endl;
  Allstuff.StopPrintClock();
  pStatClass->PrintFeatures();
  /*
  cout << "Query Stats" << endl;
  m_query_stat.PrintAllStats(region->GetRoadmap());
  */
  std::cout.rdbuf(sbuf);  // restore original stream buffer 
  stat_ofstream.close();

  cout << "!!ALL FINISHED!!"<< endl;
 
   LOG_DEBUG_MSG("End Hybrid PRM Finalizing");
}

void
HybridPRM::
initialize_weights_probabilities_costs() 
{
  m_node_gen_weights.clear();
  m_node_gen_probabilities.clear();
  m_node_gen_probabilities_uniform.clear();

  double uniform_probability = double(1.0 / m_node_gen_labels.size());
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
  {
    m_node_gen_weights[*NG] = 1.0;
    m_node_gen_probabilities[*NG] = uniform_probability;
    m_node_gen_probabilities_uniform[*NG] = uniform_probability;
    m_node_gen_probabilities_no_cost[*NG] = uniform_probability;
    /*
    if(!m_fixed_cost)
      m_node_gen_costs[*NG] = 1;
    */
  }
}


void
HybridPRM::
copy_learned_prob_to_prob_use() 
{
  m_node_gen_probabilities_use.clear();
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
    m_node_gen_probabilities_use[*NG] = m_node_gen_probabilities[*NG];
}


void
HybridPRM::
print_weights_probabilities_costs(ostream& _out) 
{
  _out << endl;
  _out << "Sampler::\tWeight\tProNoCost\tPro\tCost\n";
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
  {
    _out << *NG << "::";
    _out << "\t" << m_node_gen_weights[*NG];
    _out << "\t" << m_node_gen_probabilities_no_cost[*NG];
    _out << "\t" << m_node_gen_probabilities[*NG];
    _out << "\t" << m_node_gen_costs[*NG];
    _out << endl;
  }
  _out << endl;
}


string
HybridPRM::
select_next_sampling_method(bool learning) 
{
  //create probability ranges for each sampler
  double pro_sum = 0;
  map<string, pair<double, double> > map_pro_range;
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
  {
    double gen_prob = 0;
    if(learning && m_sampler_selection_distribution == "window_uniform") 
      gen_prob = m_node_gen_probabilities_uniform[*NG];
    else if(m_sampler_selection_distribution == "nowindow") 
      gen_prob = m_node_gen_probabilities_use[*NG];
    else
      gen_prob = m_node_gen_probabilities[*NG];

    double upper_bound = 0;
    if(NG+1 != m_node_gen_labels.end())
      upper_bound = pro_sum + gen_prob;
    else
      upper_bound = 1.0; // do this only in the last number

    map_pro_range[*NG] = make_pair(pro_sum, upper_bound);
    pro_sum += gen_prob;
  }

  //return the sampler with the highest probability if outside learning window and "window_hybrid_outsidewindow_highest" selected
  if(!learning && m_sampler_selection_distribution == "window_hybrid_outsidewindow_highest") 
  {
    double max_probability = 0;
    string max_gen = "";
    for(map<string, pair<double, double> >::const_iterator MPR = map_pro_range.begin(); MPR != map_pro_range.end(); ++MPR)
      if((MPR->second.second - MPR->second.first) > max_probability)
      {
        max_probability = MPR->second.second - MPR->second.first;
        max_gen = MPR->first;
      }
    //cout << "***\tHighest sampler after learning :: " << max_gen << endl;
    return max_gen;
  } 
  else 
  {
    //select sampler based on probability ranges
    double random_num = OBPRM_drand();
    for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
      if(map_pro_range[*NG].first < random_num && random_num < map_pro_range[*NG].second) 
      { //we have a winner
        //cout << "***   The next node generator is::  " << *NG << endl;
        return *NG;
      }

    //error checking, proram flow should not end up here
    cerr << endl << endl << "This can't be good, exiting.";
    cerr << endl;
    for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
      cerr << *NG << ":: [" << map_pro_range[*NG].first << ", " << map_pro_range[*NG].second << "]" << endl;
    cerr << "Random Number:: " << random_num << endl;
    exit(-1);
  }
}


double
HybridPRM::
compute_visibility_reward(string next_node_gen, double visibility, double threshold, int prev_cc_count, int curr_cc_count, NodeTypeCounts& node_types)
{
  if(curr_cc_count > prev_cc_count) //CC Create 
  {
    node_types.num_cc_create++;
    return 1.0;
  } 
  else if(curr_cc_count < prev_cc_count) //CC Merge
  {
    node_types.num_cc_merge++;
    return 1.0;
  } 
  else 
  { //Reward based on visibility
    if(visibility < threshold) //CC Expand 
      node_types.num_cc_expand++;
    else //CC Oversample 
    {
      node_types.num_cc_oversample++;
      m_node_gen_num_oversampled[next_node_gen]++;
    }
    return exp(-4 * pow(visibility, 2));
  }
}


bool
HybridPRM::
in_learning_window(int totalSamples) const
{
  //note, when put all on 1 line w/o temp variables d1 and d2, results are not correct
  double d1 = totalSamples % m_bin_size;
  double d2 = double(m_bin_size) * m_window_percent;
  return d1 < d2;
}


void
HybridPRM::
reward_and_update_weights_probabilities(string node_gen_selected, double reward, unsigned long int cost) 
{
  int K = m_node_gen_labels.size();

  //update costs:
  if(!m_fixed_cost)
  {
    int num_sampled = m_node_gen_num_sampled[node_gen_selected];
    int prev_avg_cost = m_node_gen_costs[node_gen_selected];
    int new_avg_cost = (prev_avg_cost * num_sampled + cost) / (num_sampled + 1);
    m_node_gen_costs[node_gen_selected] = new_avg_cost; 
  }

  //update weights:
  // weight_i(t+1) = weight_i(t) * exp (lambda * adjusted_reward_i / K)
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
  {
    double adjusted_reward = 0; //default reward is 0
    if(*NG == node_gen_selected) //reward this sampler: adjusted_reward_i = reward_i / prob_nocost_i
      adjusted_reward = reward / m_node_gen_probabilities_no_cost[*NG];

    double new_weight = m_node_gen_weights[*NG] * exp(double((m_percentage_random) * adjusted_reward / double(K)));
    m_node_gen_weights[*NG] = new_weight;
  }

  //update probabilities:
  double weight_total = 0;
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
    weight_total += m_node_gen_weights[*NG];
  // p_i_nocost = (1-lambda) * (w_i(t) / w_total(t)) + lambda/K
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
    m_node_gen_probabilities_no_cost[*NG] = (1 - m_percentage_random) * (m_node_gen_weights[*NG] / weight_total) + (m_percentage_random / K);
  // p_i = (p_i_nocost/cost_i) / total sum of (p_j_nocost/cost_j)
  double prob_nocost_total = 0;
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
    prob_nocost_total += (m_node_gen_probabilities_no_cost[*NG] / double(m_node_gen_costs[*NG]));
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
  {
    if(!m_count_cost) 
      m_node_gen_probabilities[*NG] = m_node_gen_probabilities_no_cost[*NG];
    else
      m_node_gen_probabilities[*NG] = (m_node_gen_probabilities_no_cost[*NG] / double(m_node_gen_costs[*NG])) / prob_nocost_total;
  }
 
  //normalize weights (not in original algorithm, but prevents infinite weights
  double smallest_weight = -1;
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
  {
    double weight = m_node_gen_weights[*NG];
    if(weight < smallest_weight || smallest_weight == -1)
      smallest_weight = weight;
  }
  for(vector<string>::const_iterator NG = m_node_gen_labels.begin(); NG != m_node_gen_labels.end(); ++NG)
    m_node_gen_weights[*NG] = m_node_gen_weights[*NG] / smallest_weight;
}



/*
bool 
HybridPRM::
CanConnectComponents(vector<CfgType>& cc_a, vector<CfgType>& cc_b, Stat_Class& stat) 
{
  // variables needed for the local planner call in loop
  LocalPlanners <CfgType, WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  LPOutput<CfgType, WeightType> lp_output; 
  Environment* env = GetMPProblem()->GetEnvironment();
  CollisionDetection* cd = GetMPProblem()->GetCollisionDetection();
  DistanceMetric* dm = GetMPProblem()->GetDistanceMetric();
  double pos_res = GetMPProblem()->GetEnvironment()->GetPositionRes();
  double ori_res = GetMPProblem()->GetEnvironment()->GetOrientationRes();
  //Stat_Class Stats;

  typedef vector< CfgType >::iterator CFG_ITRTR;
  for(CFG_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) 
  {
    sort(cc_b.begin(), cc_b.end(), CFG_CFG_DIST_COMPARE<CfgType>(*i_cc_a,dm,env));
    for(CFG_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) 
      if(lp->IsConnected(env, stat, dm, (*i_cc_a), (*i_cc_b), &lp_output, pos_res, ori_res, true)) 
        return true; // st  op as soon as one cc in a can connect to a node in b
  }
  return false;
}
*/


/*
double 
HybridPRM::
cc_diamater(RoadmapGraph<CfgType,WeightType>* pGraph, VID _cc) 
{
  // vector<VID> cc_vids;
  // GetCC ( *pGraph, _cc, cc_vids);
  double return_val = 0.0;
//  for(int i=0; i<cc_vids.size(); ++i) {
//  //  RoadmapGraph<CfgType,WeightType> *result; 
//  //  result = new RoadmapGraph<CfgType,WeightType>;
//    //double length = DijkstraSSSP(*(pGraph),*result,cc_vids[i]);
//    double length = DijkstraSSSP(*(pGraph),cc_vids[i]);
//    //result->EraseGraph();
//    //delete result;
//    if(length > return_val) { return_val = length;}
//  }
//  return return_val;

  VID farVID, tmpVID;
  double length = ComponentDiameter(*(pGraph),_cc, &farVID);
  return_val = ComponentDiameter(*(pGraph),farVID, &tmpVID);
  return return_val;
}
*/


/*
pair<unsigned int, unsigned int>
HybridPRM::
ConnectionsWitnessToRoadmap(vector<CfgType>& witness_cfgs, Roadmap<CfgType, WeightType>* rdmp, Stat_Class& stat) 
{
  int small_cc_size = 0;
  
  stapl::vector_property_map<stapl::stapl_color<size_t> > cmap;
  cmap.reset();
  vector<pair<size_t, VID> > cc;
  get_cc_stats(*(rdmp->m_pRoadmap), cmap, cc);
	
  // small_cc_size = int(double(cc[0].first) * double(0.01));
  vector<vector<unsigned int> > connected_to_cc;
  vector<unsigned int> tmp_vector;
  for(unsigned int i=0; i< cc.size(); i++)
    connected_to_cc.push_back(tmp_vector);
  
  unsigned int possible_connections = 0;
  vector<CfgType> witness_test_cfg;
  typedef vector<CfgType>::iterator CFG_ITRTR;
  for(unsigned int i=0; i < witness_cfgs.size(); i++) 
  {
    witness_test_cfg.clear();
    witness_test_cfg.push_back(witness_cfgs[i]);

    typedef vector < pair< int, VID > >::iterator CC_ITRTR;
    unsigned int j=0;
    bool i_witness_can_connect = false;
    for(unsigned int j=0; j < cc.size(); j++) 
    {
      CfgType* tmp_pointer = new CfgType((*(rdmp->m_pRoadmap->find_vertex(cc[j].second))).property());
      vector<VID> cc_cfgs_aux;
      cmap.reset();
      get_cc(*(rdmp->m_pRoadmap), cmap, rdmp->m_pRoadmap->GetVID(*(tmp_pointer)), cc_cfgs_aux);
      vector<CfgType> cc_cfgs = rdmp->m_pRoadmap->ConvertVIDs2Vertices(cc_cfgs_aux);
      delete tmp_pointer;
      if(cc_cfgs.size() >= small_cc_size) 
        if(CanConnectComponents(witness_test_cfg, cc_cfgs, stat)) 
        {
          i_witness_can_connect = true;
          connected_to_cc[j].push_back(i);
        }
    }
    if(i_witness_can_connect)
      possible_connections++; 
  }

  unsigned int possible_queries = 0;
  typedef vector<unsigned int>::iterator INT_ITRTR;
  typedef vector<vector<unsigned int> >::iterator DINT_ITRTR;
  for(DINT_ITRTR i_ccs=connected_to_cc.begin(); i_ccs < connected_to_cc.end(); i_ccs++) 
  {
    bool i_in_j = false;
    for(DINT_ITRTR i_ccs_other= i_ccs+1; i_ccs_other < connected_to_cc.end(); i_ccs_other++) 
      for(INT_ITRTR i_el_i = i_ccs->begin(); i_el_i < i_ccs->end(); i_el_i++) 
      {
        //test whether *i_ccs is in *(i_ccs+1)  
        for(INT_ITRTR i_el_j = (i_ccs_other)->begin(); i_el_j < (i_ccs_other)->end(); i_el_j++) 
          if((*i_el_i) == (*i_el_j)) 
          {
            i_ccs_other->insert(i_ccs_other->end(),i_ccs->begin(),i_ccs->end());
            i_in_j = true;
            break;
          }
        if (i_in_j)
          i_ccs->clear();
      }
  }

  for(DINT_ITRTR i_con=connected_to_cc.begin(); i_con < connected_to_cc.end(); i_con++) 
  {
    // count whether *i_witness_cfg can connect to this cc in rdmp  
    // count the number of queries that could be done through this cc
    sort(i_con->begin(),i_con->end());
    INT_ITRTR newEnd;
    newEnd = unique(i_con->begin(),i_con->end());
    int i_con_size = newEnd - i_con->begin();
    possible_queries += (i_con_size)*(i_con_size-1); //remember to divide by 2 at the end
  }

  return pair<unsigned int, unsigned int>(possible_connections, possible_queries/2);
}
*/


bool 
HybridPRM::
evaluate_map(int in_RegionID)
{
  if(m_evaluator_labels.empty())
    return true;
  else
  {
    Clock_Class EvalClock;
    EvalClock.StartClock("Map Evaluation");

    bool mapPassedEvaluation = true;
    for(vector<string>::iterator I = m_evaluator_labels.begin(); I != m_evaluator_labels.end(); ++I)
    {
      MapEvaluator<CfgType, WeightType>::conditional_type pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      Clock_Class EvalSubClock;
      EvalSubClock.StartClock(pEvaluator->GetName());

      cout << "\n\t";
      mapPassedEvaluation = pEvaluator->operator()(in_RegionID);

      cout << "\t";
      EvalSubClock.StopPrintClock();
      if(mapPassedEvaluation)
        cout << "\t  (passed)\n";
      else
        cout << "\t  (failed)\n";
      if(!mapPassedEvaluation)
        break;
    }
    EvalClock.StopPrintClock();
    return mapPassedEvaluation;
  }
}


