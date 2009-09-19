#ifndef _MPREGIONCOMPARERMETHOD_H_
#define _MPREGIONCOMPARERMETHOD_H_

#include "util.h"
class MPProblem;
#include "Roadmap.h"
#include "Stat_Class.h"
#include "LocalPlanners.h"
#include "MPStrategy.h"

template <class CFG, class WEIGHT>
class MPRegionComparerMethod: public MPBaseObject {
 public:
  MPRegionComparerMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPBaseObject(in_Node, in_pProblem) { 
    m_pProblem = in_pProblem;
    
    string filename = in_Node.stringXMLParameter(string("witness_file"),false,
                                    string(""),string("Witness Filename"));
    if (filename.length() > 0) {
      Roadmap< CFG, WEIGHT > tmp_roadmap;
      tmp_roadmap.ReadRoadmapGRAPHONLY(filename.c_str());
      tmp_roadmap.m_pRoadmap->GetVerticesData(m_witness_cfgs);
      LOG_DEBUG_MSG("MPRegionComparerMethod:: input cfgs: " << m_witness_cfgs.size());
      
    }
    
  }

  virtual ~MPRegionComparerMethod() {}
  
  bool CfgIsVisible(CFG& in_cfg, vector< CFG >& in_vec_cfg) {
    LocalPlanners < CFG, WEIGHT > * lp = m_pProblem->GetMPStrategy()->GetLocalPlanners();
    LPOutput< CFG, WEIGHT > lp_output; 
    Environment * env = m_pProblem->GetEnvironment();
//    CollisionDetection * cd = m_pProblem->GetCollisionDetection();
    DistanceMetric * dm = m_pProblem->GetDistanceMetric();
    double pos_res = m_pProblem->GetEnvironment()->GetPositionRes();
    double ori_res = m_pProblem->GetEnvironment()->GetOrientationRes();
    Stat_Class Stats;

    //CFG_CFG_DIST_COMPARE<CFG> mydistcompare(in_cfg, dm, env);
   
    
    typedef typename vector< CFG >::iterator CFG_ITRTR;
    for(CFG_ITRTR i_vec_cfg = in_vec_cfg.begin(); i_vec_cfg < in_vec_cfg.end(); i_vec_cfg++) {
      if (in_cfg == (*i_vec_cfg)) {
	return true; //   in_cfg is actually inside in_vec_cfg
      }
    }
   
   sort(in_vec_cfg.begin(), in_vec_cfg.end(), CFG_CFG_DIST_COMPARE<CFG>(in_cfg, dm, env));
   for(CFG_ITRTR i_vec_cfg = in_vec_cfg.begin(); i_vec_cfg < in_vec_cfg.end(); i_vec_cfg++) {
    if (lp->IsConnected(env, Stats, dm, in_cfg, (*i_vec_cfg), 
			  &lp_output, pos_res, ori_res, true)) {
	return true; // 	stop as soon in_cfg can connect to one cfg in in_vec_cfg
      }
   }
    return false;  //in_cfg is not visible to anything in_vec_cfg
  }
  
  bool CanConnectComponents(vector < CFG > & cc_a, vector < CFG > & cc_b) {
    // variables needed for the local planner call in loop
    LocalPlanners < CFG, WEIGHT > * lp = m_pProblem->GetMPStrategy()->GetLocalPlanners();
    LPOutput< CFG, WEIGHT > lp_output; 
    Environment * env = m_pProblem->GetEnvironment();
//    CollisionDetection * cd = m_pProblem->GetCollisionDetection();
    DistanceMetric * dm = m_pProblem->GetDistanceMetric();
    double pos_res = m_pProblem->GetEnvironment()->GetPositionRes();
    double ori_res = m_pProblem->GetEnvironment()->GetOrientationRes();
    Stat_Class Stats;
    
    typedef typename vector< CFG >::iterator CFG_ITRTR;
    for(CFG_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
      for (CFG_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
	if (lp->IsConnected(env, Stats, dm, (*i_cc_a), (*i_cc_b), 
			    &lp_output, pos_res, ori_res, true)) {
	  return true; // st	op as soon as one cc in a can connect to a node in b
	}
      }
    }
    return false;
  }
 

  pair < unsigned int, unsigned int > 
  ComponentsASpanningInB(Roadmap<CFG, WEIGHT> *rdmp_a, Roadmap<CFG, WEIGHT> *rdmp_b) {
    
    int a_small_cc_size =0;
    int b_small_cc_size =0;
    
    vector < pair< int, VID > > cc_a;
    GetCCStats(*(rdmp_a->m_pRoadmap), cc_a);
    a_small_cc_size = int(double(cc_a[0].first) * double(0.01));
    cout << " Components in Roadmap A " << cc_a.size() << " small_cc_size " << a_small_cc_size << endl;
    vector < pair< int, VID > > cc_b;
    GetCCStats(*(rdmp_b->m_pRoadmap), cc_b);
    b_small_cc_size = int(double(cc_b[0].first) * double(0.01));
    cout << " Components in Roadmap B " << cc_b.size() << " small_cc_size " << b_small_cc_size << endl;
    
    typedef typename vector < pair< int, VID > >::iterator CC_ITRTR;
    
    int spanning_cc_a = 0; // components in A that can connect to B
    int not_small_cc_as = 0;
    // for each connected component cc_a in A
    for (CC_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
      vector < CFG > cc_a_cfgs;
      GetCC(*(rdmp_a->m_pRoadmap), *(rdmp_a->m_pRoadmap->GetReferenceofData(i_cc_a->second)), cc_a_cfgs);
      
      if (cc_a_cfgs.size() >= a_small_cc_size) {
	not_small_cc_as++;
	int connectable_cc_a_to_b = 0;
	// for each connected component cc_b in B
	for (CC_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
	  vector <CFG > cc_b_cfgs;
	  GetCC(*(rdmp_b->m_pRoadmap), *(rdmp_b->m_pRoadmap->GetReferenceofData(i_cc_b->second)), cc_b_cfgs);
	  
	  if (cc_b_cfgs.size() >= b_small_cc_size) {
	    // try to connect component A to component B
	    if (CanConnectComponents(cc_a_cfgs, cc_b_cfgs)) {
	      connectable_cc_a_to_b++;
	      if (connectable_cc_a_to_b > 1) {
		spanning_cc_a++;
		break;
	      }
	    }
	  }
	}
      }
    }
    return pair < unsigned int, unsigned int>(spanning_cc_a, not_small_cc_as);
  }
  
  pair < unsigned int, unsigned int >
  ConnectionsWitnessToRoadmap(vector < CFG > & witness_cfgs, Roadmap< CFG, WEIGHT > *rdmp) {
    int small_cc_size =0;
    vector < pair< int, VID > > cc;
    GetCCStats(*(rdmp->m_pRoadmap), cc);
    small_cc_size = int(double(cc[0].first) * double(0.01));
    vector < vector< unsigned int > > connected_to_cc;
    vector < unsigned int > tmp_vector;
    for(unsigned int i=0; i< cc.size(); i++)
      connected_to_cc.push_back(tmp_vector);
    
    unsigned int possible_connections = 0;
    vector< CFG > witness_test_cfg;
    typedef typename vector< CFG >::iterator CFG_ITRTR;
    for (unsigned int i=0; i < witness_cfgs.size(); i++) {
      witness_test_cfg.clear();
      witness_test_cfg.push_back(witness_cfgs[i]);

      typedef typename vector < pair< int, VID > >::iterator CC_ITRTR;
      unsigned int j=0;
      bool i_witness_can_connect = false;
      for (unsigned int j=0; j < cc.size(); j++) {
	vector < CFG > cc_cfgs;
	GetCC(*(rdmp->m_pRoadmap), *(new CFG(rdmp->m_pRoadmap->GetData(cc[j].second))), cc_cfgs);
	if (cc_cfgs.size() >= small_cc_size) {
	  if ( CanConnectComponents(witness_test_cfg, cc_cfgs) ) {
	    i_witness_can_connect = true;
	    connected_to_cc[j].push_back(i);
	  }
	}
      }
      if (i_witness_can_connect)
      	  possible_connections++; 
    }

    unsigned int possible_queries = 0;
    typedef typename vector< unsigned int >::iterator INT_ITRTR;
    typedef typename vector< vector < unsigned int > >::iterator DINT_ITRTR;
    for (DINT_ITRTR i_ccs=connected_to_cc.begin(); i_ccs < connected_to_cc.end(); i_ccs++) {
      bool i_in_j = false;
      for(DINT_ITRTR i_ccs_other= i_ccs+1; i_ccs_other < connected_to_cc.end(); i_ccs_other++) {
	for (INT_ITRTR i_el_i = i_ccs->begin(); i_el_i < i_ccs->end(); i_el_i++) {
	  //test whether *i_ccs is in *(i_ccs+1)	
	  for (INT_ITRTR i_el_j = (i_ccs_other)->begin(); i_el_j < (i_ccs_other)->end(); i_el_j++) {
	    if ( (*i_el_i) == (*i_el_j) ) {
	      i_ccs_other->insert(i_ccs_other->end(),i_ccs->begin(),i_ccs->end());
	      i_in_j = true;
	    break;
	    }
	}
	if (i_in_j)
	  i_ccs->clear();
	}
      }
    }

    for (DINT_ITRTR i_con=connected_to_cc.begin(); i_con < connected_to_cc.end(); i_con++) {
      // count whether *i_witness_cfg can connect to this cc in rdmp	
      // count the number of queries that could be done through this cc
      sort(i_con->begin(),i_con->end());
      INT_ITRTR newEnd;
      newEnd = unique(i_con->begin(),i_con->end());
      int i_con_size = newEnd - i_con->begin();
      possible_queries += (i_con_size)*(i_con_size-1); //remember to divide by 2 at the end
    }

    return pair < unsigned int, unsigned int>(possible_connections, possible_queries/2);
  }

  ///@todo operators () without parameters and taking in pairs of region ids
/*   virtual void operator() () = 0; */
/*   virtual void operator() (int in_RegionID_a, int in_RegionID_b) = 0; */

  virtual bool Compare(int in_region_a, int in_region_b) = 0;

  virtual void PrintOptions(ostream& out_os) {
    out_os << "    " << GetLabel() << ":: ";
  }

  MPProblem * m_pProblem;
  vector< CFG > m_witness_cfgs;
};


template <class CFG, class WEIGHT>
class ConnectableComponentComparer : public MPRegionComparerMethod<CFG,WEIGHT> 
{
public:

  ConnectableComponentComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
    LOG_DEBUG_MSG("ConnectableComponentComparer::ConnectableComponentComparer()");
   
    in_Node.verifyName(string("ConnectableComponentComparer"));
   
    LOG_DEBUG_MSG("ConnectableComponentComparer::ConnectableComponentComparer() end");    
  }


  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("ConnectableComponentComparer::Compare(region_a, region_b)");

    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b
    
    pair< unsigned int, unsigned int > spanning_a = ComponentsASpanningInB(rdmp_a, rdmp_b);
    cout << "ConnectableComponentComparer::Compare: Not small ccs in A " << spanning_a.second << "; Span more than 1 in B: " << spanning_a.first << " = " << 100*spanning_a.first/spanning_a.second << "%" << endl;

    pair< unsigned int, unsigned int > spanning_b = ComponentsASpanningInB(rdmp_b, rdmp_a);
    cout << "ConnectableComponentComparer::Compare: Not small ccs in B " << spanning_b.second << "; Span more than 1 in A: " << spanning_b.first << " = " << 100*spanning_b.first/spanning_b.second << "%" << endl;
  

  }


private:

};


template <class CFG, class WEIGHT>
class RandomConnectComparer : public MPRegionComparerMethod< CFG, WEIGHT > 
{
public:
  RandomConnectComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
    LOG_DEBUG_MSG("RandomConnectComparer::RandomConnectComparer()");
    
    in_Node.verifyName(string("RandomConnectComparer"));
    
    LOG_DEBUG_MSG("RandomConnectComparer::RandomconnectComparer() end");    
  }

  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("RandomConnectComparer::Compare(region_a, region_b)");    

    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b

    unsigned int witness_queries = this->m_witness_cfgs.size()*(this->m_witness_cfgs.size()-1)/2;
    cout << "RandomConnectComparer::Compare: Witness size: " << this->m_witness_cfgs.size() << "; Witness queries: " << witness_queries << endl;

    pair< unsigned int, unsigned int > witness_to_a = ConnectionsWitnessToRoadmap(this->m_witness_cfgs, rdmp_a);
    double witness_to_a_connections_percent = 100*witness_to_a.first/this->m_witness_cfgs.size();
    double witness_to_a_succ_queries_percent = 100*witness_to_a.second/witness_queries;
    cout << "RandomConnectComparer::Compare: Witness nodes connected to non-small ccs in A: " << witness_to_a.first << " = " << witness_to_a_connections_percent << "%" << endl;
    cout << "RandomConnectComparer::Compare: Witness queries succesful in non-small ccs in A: " << witness_to_a.second << " = " << witness_to_a_succ_queries_percent << "%" << endl;

    pair< unsigned int, unsigned int > witness_to_b = ConnectionsWitnessToRoadmap(this->m_witness_cfgs, rdmp_b);
    double witness_to_b_connections_percent = 100*witness_to_b.first/this->m_witness_cfgs.size();
    double witness_to_b_succ_queries_percent = 100*witness_to_b.second/witness_queries;
    cout << "RandomConnectComparer::Compare: Witness nodes connected to non-small ccs in B: " << witness_to_b.first << " = " << witness_to_b_connections_percent << "%" << endl;
    cout << "RandomConnectComparer::Compare: Witness queries succesful in non-small ccs in B: " << witness_to_b.second << " = " << witness_to_b_succ_queries_percent << "%" << endl;
  }

private:

};


template <class CFG, class WEIGHT>
class RegionCoverageComparer : public MPRegionComparerMethod< CFG, WEIGHT > 
{
public:
  RegionCoverageComparer(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
    LOG_DEBUG_MSG("RegionCoverageComparer::IncrementalRegionComparer()");

    in_Node.verifyName(string("RegionCoverageComparer"));
    
    LOG_DEBUG_MSG("~RegionCoverageComparer::RegionCoverageComparer()");    
  }

  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("RegionCoverageComparer::Compare(region_a, region_b)");    

    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b


    int a_small_cc_size =0;
    int b_small_cc_size =0;
    
    vector< pair <int,VID > > a_cc_stats;
    GetCCStats (*(rdmp_a->m_pRoadmap),a_cc_stats);
    vector< pair <int,VID > > b_cc_stats;
    GetCCStats (*(rdmp_b->m_pRoadmap),b_cc_stats);
    a_small_cc_size = int(double(a_cc_stats[0].first) * double(0.01));
    b_small_cc_size = int(double(b_cc_stats[0].first) * double(0.01));
    cout << "RegionCoverageComparer::Compare: a_small_cc_size: " << a_small_cc_size << endl;
    cout << "RegionCoverageComparer::Compare: b_small_cc_size: " << b_small_cc_size << endl;
    
    //Get vector<VID> from a;
    vector<CFG> rdmp_a_cfg, rdmp_b_cfg;  ///\todo implement this better with iterators.
    vector<VID> rdmp_a_vids, rdmp_b_vids;
    int a_revealing_node =0;
    int a_trapped_node =0;
    int b_revealing_node =0;
    int b_trapped_node =0;
    rdmp_a->m_pRoadmap->GetVerticesData(rdmp_a_cfg);
    rdmp_b->m_pRoadmap->GetVerticesData(rdmp_b_cfg);
    
    rdmp_a->m_pRoadmap->GetVerticesVID(rdmp_a_vids);
    rdmp_b->m_pRoadmap->GetVerticesVID(rdmp_b_vids);

    typename vector<VID>::iterator a_vid_itr;
    for(a_vid_itr = rdmp_a_vids.begin(); a_vid_itr!=rdmp_a_vids.end(); ++a_vid_itr) {
      if(!CfgIsVisible(*(rdmp_a->m_pRoadmap->GetReferenceofData(*a_vid_itr)), rdmp_b_cfg)) {
         // found a potential revealing/trapped node
        vector<VID> tmp_cc;
        GetCC(*(rdmp_a->m_pRoadmap),(*a_vid_itr), tmp_cc);
        if( tmp_cc.size() >= a_small_cc_size) { 
          // found a revealing node
          a_revealing_node++;
        } else { a_trapped_node++;} //found a trapped node.
      }
    }

    typename vector<VID>::iterator b_vid_itr;
    for(b_vid_itr = rdmp_b_vids.begin(); b_vid_itr!=rdmp_b_vids.end(); ++b_vid_itr) {
     if(!CfgIsVisible(*(rdmp_b->m_pRoadmap->GetReferenceofData(*b_vid_itr)), rdmp_a_cfg)) {
        // found a potential revealing/trapped node
        vector<VID> tmp_cc;
        GetCC(*(rdmp_b->m_pRoadmap),(*b_vid_itr), tmp_cc);
        if( tmp_cc.size() >= b_small_cc_size) { 
          // found a revealing node
          b_revealing_node++;
        } else { b_trapped_node++;} //found a trapped node.
      }
    }

    cout << "RegionCoverageComparer::Compare: a_revealing_node: " << a_revealing_node << endl;
    cout << "RegionCoverageComparer::Compare: a_trapped_node: " << a_trapped_node << endl;
    cout << "RegionCoverageComparer::Compare: b_revealing_node: " << b_revealing_node << endl;
    cout << "RegionCoverageComparer::Compare: b_trapped_node: " << b_trapped_node << endl;

   LOG_DEBUG_MSG("~RegionCoverageComparer::Compare(region_a, region_b)");    
  }

private:

};


template <class CFG, class WEIGHT>
class RegionSimilarity : public MPRegionComparerMethod< CFG, WEIGHT > 
{
public:
  RegionSimilarity(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
    LOG_DEBUG_MSG("RegionSimilarity::IncrementalRegionComparer()");
    
    in_Node.verifyName(string("RegionSimilarity"));
    
    LOG_DEBUG_MSG("~RegionSimilarity::RegionSimilarity()");    
  }

  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("RegionSimilarity::Compare(region_a, region_b)");    

    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b


    int a_small_cc_size =0;
    int b_small_cc_size =0;
    
    vector< pair <int,VID > > a_cc_stats;
    GetCCStats (*(rdmp_a->m_pRoadmap),a_cc_stats);
    vector< pair <int,VID > > b_cc_stats;
    GetCCStats (*(rdmp_b->m_pRoadmap),b_cc_stats);
    a_small_cc_size = int(double(a_cc_stats[0].first) * double(0.01));
    b_small_cc_size = int(double(b_cc_stats[0].first) * double(0.01));
    cout << "RegionSimilarity::Compare: a_small_cc_size: " << a_small_cc_size << endl;
    cout << "RegionSimilarity::Compare: b_small_cc_size: " << b_small_cc_size << endl;

    //Get vector<VID> from a;
    vector<CFG> usable_rdmp_a_cfg, usable_rdmp_b_cfg;  ///\todo implement this better with iterators.
    vector<VID> all_rdmp_a_vids, all_rdmp_b_vids;
    vector<VID> usable_rdmp_a_vids, usable_rdmp_b_vids;
    int a_revealing_node =0;
    int a_trapped_node =0;
    int b_revealing_node =0;
    int b_trapped_node =0;

    rdmp_a->m_pRoadmap->GetVerticesVID(all_rdmp_a_vids);
    rdmp_b->m_pRoadmap->GetVerticesVID(all_rdmp_b_vids);
    

    //Get usable nodes in a
    typename vector<VID>::iterator all_a_vid_itr;
    for(all_a_vid_itr = all_rdmp_a_vids.begin(); all_a_vid_itr!=all_rdmp_a_vids.end(); ++all_a_vid_itr) {
      vector<VID> tmp_cc;
      GetCC(*(rdmp_a->m_pRoadmap),(*all_a_vid_itr), tmp_cc);
      if( tmp_cc.size() >= a_small_cc_size) {
         usable_rdmp_a_vids.push_back((*all_a_vid_itr));
         usable_rdmp_a_cfg.push_back(rdmp_a->m_pRoadmap->GetData((*all_a_vid_itr)));
      } //else not usable!!
    }

    typename vector<VID>::iterator all_b_vid_itr;
    for(all_b_vid_itr = all_rdmp_b_vids.begin(); all_b_vid_itr!=all_rdmp_b_vids.end(); ++all_b_vid_itr) {
      vector<VID> tmp_cc;
      GetCC(*(rdmp_b->m_pRoadmap),(*all_b_vid_itr), tmp_cc);
      if( tmp_cc.size() >= b_small_cc_size) {
         usable_rdmp_b_vids.push_back((*all_b_vid_itr));
         usable_rdmp_b_cfg.push_back(rdmp_b->m_pRoadmap->GetData((*all_b_vid_itr)));
      } //else not usable!!
    }

    //For all usable nodes in A, check if visible to usable node in B
    //Print total number of visible + non-visible in A/B
    int node_a_visible_in_b = 0;
    int node_a_not_visible_in_b = 0;
    int node_b_visible_in_a = 0;
    int node_b_not_visible_in_a = 0;
    
    typename vector<VID>::iterator usable_a_vid_itr;
    for(usable_a_vid_itr = usable_rdmp_a_vids.begin(); 
              usable_a_vid_itr!=usable_rdmp_a_vids.end(); ++usable_a_vid_itr) {
        vector<VID> tmp_cc;
        GetCC(*(rdmp_a->m_pRoadmap),(*usable_a_vid_itr), tmp_cc);
        if( tmp_cc.size() >= a_small_cc_size) {  // Make sure its a "usable" node
          if(CfgIsVisible(*(rdmp_a->m_pRoadmap->GetReferenceofData(*usable_a_vid_itr)), usable_rdmp_b_cfg)) {
            ++node_a_visible_in_b;
          } else { ++node_a_not_visible_in_b; }
      }
    }

    typename vector<VID>::iterator usable_b_vid_itr;
    for(usable_b_vid_itr = usable_rdmp_b_vids.begin(); 
              usable_b_vid_itr!=usable_rdmp_b_vids.end(); ++usable_b_vid_itr) {
        vector<VID> tmp_cc;
        GetCC(*(rdmp_b->m_pRoadmap),(*usable_b_vid_itr), tmp_cc);
        if( tmp_cc.size() >= b_small_cc_size) {  // Make sure its a "usable" node
          if(CfgIsVisible(*(rdmp_b->m_pRoadmap->GetReferenceofData(*usable_b_vid_itr)), usable_rdmp_a_cfg)) {
            ++node_b_visible_in_a;
          } else { ++node_b_not_visible_in_a; }
      }
    }


    
    //For all edges in A, if in usable component, check if visible to same component in B
    int edge_a_visible_in_b = 0;
    int edge_a_not_visible_in_b = 0;
    int edge_b_visible_in_a = 0;
    int edge_b_not_visible_in_a = 0;
    vector< pair<VID,VID> > all_edges_a, all_edges_b;
    rdmp_a->m_pRoadmap->GetEdges(all_edges_a);
    rdmp_b->m_pRoadmap->GetEdges(all_edges_b);

    typename vector< pair<VID,VID> >::iterator all_edges_a_itr;
    for(all_edges_a_itr = all_edges_a.begin(); all_edges_a_itr != all_edges_a.end(); 
           ++all_edges_a_itr) {
      vector<VID> tmp_cc;
      GetCC(*(rdmp_a->m_pRoadmap),(*all_edges_a_itr).first, tmp_cc);
      if( tmp_cc.size() < a_small_cc_size) { continue; } // not a edge in "usable" component
      //for all connected components in rdmp_b....
      vector< pair<int,VID> > all_cc_b;
      GetCCStats(*(rdmp_b->m_pRoadmap), all_cc_b);
      bool found_edge_a_visible_in_b=false;
      typename vector< pair<int,VID> >::iterator all_cc_b_itr;
      for(all_cc_b_itr = all_cc_b.begin(); all_cc_b_itr != all_cc_b.end(); ++all_cc_b_itr) {
        if((*all_cc_b_itr).first < b_small_cc_size) { continue; } //not usable cc
        vector<CFG> usable_cc_in_b;
        GetCC(*(rdmp_b->m_pRoadmap), 
              *(rdmp_b->m_pRoadmap->GetReferenceofData((*all_cc_b_itr).second)), 
              usable_cc_in_b);
        if(CfgIsVisible(*(rdmp_a->m_pRoadmap->GetReferenceofData((*all_edges_a_itr).first)), 
                        usable_cc_in_b)) {
          if(CfgIsVisible(*(rdmp_a->m_pRoadmap->GetReferenceofData((*all_edges_a_itr).second)), 
                        usable_cc_in_b)) {
            ++edge_a_visible_in_b;
            found_edge_a_visible_in_b = true;
            break;
          }
        }
    }
    if(!found_edge_a_visible_in_b) {++edge_a_not_visible_in_b;}
  }

  typename vector< pair<VID,VID> >::iterator all_edges_b_itr;
  for(all_edges_b_itr = all_edges_b.begin(); all_edges_b_itr != all_edges_b.end(); 
         ++all_edges_b_itr) {
    vector<VID> tmp_cc;
    GetCC(*(rdmp_b->m_pRoadmap),(*all_edges_b_itr).first, tmp_cc);
    if( tmp_cc.size() < b_small_cc_size) { continue; } // not a edge in "usable" component
    //for all connected components in rdmp_a....
    vector< pair<int,VID> > all_cc_a;
    GetCCStats(*(rdmp_a->m_pRoadmap), all_cc_a);
    typename vector< pair<int,VID> >::iterator all_cc_a_itr;
    bool found_edge_b_visible_in_a = false;
    for(all_cc_a_itr = all_cc_a.begin(); all_cc_a_itr != all_cc_a.end(); ++all_cc_a_itr) {
      if((*all_cc_a_itr).first < a_small_cc_size) { continue; } //not usable cc
      vector<CFG> usable_cc_in_a;
      GetCC(*(rdmp_a->m_pRoadmap), 
            *(rdmp_a->m_pRoadmap->GetReferenceofData((*all_cc_a_itr).second)), 
            usable_cc_in_a);
      if(CfgIsVisible(*(rdmp_b->m_pRoadmap->GetReferenceofData((*all_edges_b_itr).first)), 
                      usable_cc_in_a)) {
        if(CfgIsVisible(*(rdmp_b->m_pRoadmap->GetReferenceofData((*all_edges_b_itr).second)), 
                      usable_cc_in_a)) {
          ++edge_b_visible_in_a;
          found_edge_b_visible_in_a = true;
          break;
        }
      }
    }
    if(!found_edge_b_visible_in_a) {++edge_b_not_visible_in_a;}
  }
  
  cout << "node_a_visible_in_b = " << node_a_visible_in_b << endl;
  cout << "node_a_not_visible_in_b = " << node_a_not_visible_in_b << endl;

  cout << "node_b_visible_in_a = " << node_b_visible_in_a << endl;
  cout << "node_b_not_visible_in_a = " << node_b_not_visible_in_a << endl;

  cout << "edge_a_visible_in_b = " << edge_a_visible_in_b << endl;
  cout << "edge_a_not_visible_in_b = " << edge_a_not_visible_in_b << endl;

  cout << "edge_b_visible_in_a = " << edge_b_visible_in_a << endl;
  cout << "edge_b_not_visible_in_a = " << edge_b_not_visible_in_a << endl;
  
  LOG_DEBUG_MSG("~RegionSimilarity::Compare(region_a, region_b)");    
  }

private:

};

#endif /* _MPREGIONCOMPARERMETHOD_H_ */
