#ifndef _MPREGIONCOMPARERMETHOD_H_
#define _MPREGIONCOMPARERMETHOD_H_

#include "MPUtils.h"
class MPProblem;
#include "Roadmap.h"
#include "MetricUtils.h"
#include "LocalPlanners.h"
#include "MPStrategy.h"
#include "DistanceMetrics.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class compare_distance : public binary_function<const CFG, const CFG, bool>
{
 public:
  compare_distance(CFG& start_cfg, shared_ptr<DistanceMetricMethod> _dm, Environment* _env)
   : m_cfg(start_cfg), m_dm(_dm), m_env(_env)
  {}

  bool operator()(const CFG& _cc1, const CFG& _cc2)
  {
    double dcc1 = m_dm->Distance(m_env, m_cfg, _cc1);
    double dcc2 = m_dm->Distance(m_env, m_cfg, _cc2);
    return dcc1 < dcc2;
  }

 private:
  compare_distance() {}
  string m_lp;
  CFG& m_cfg;
  shared_ptr<DistanceMetricMethod> m_dm;
  Environment* m_env;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class MPRegionComparerMethod: public MPBaseObject {
 private:
  string m_lp;

 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  MPRegionComparerMethod(MPProblem * _m_pProblem, vector<CFG> _m_witness_cfgs, shared_ptr<DistanceMetricMethod> _dm) :
    m_pProblem(_m_pProblem), m_witness_cfgs(_m_witness_cfgs), dm(_dm) {}
    MPRegionComparerMethod(XMLNode& in_Node, MPProblem* in_pProblem) :
    MPBaseObject(in_Node, in_pProblem) {
    m_pProblem = in_pProblem;

    string filename = in_Node.Read("witness_file",false,"","Witness Filename");
    string dm_label = in_Node.Read("dm_method", false, "default", "Distance Metric Method");
    string m_lp     = in_Node.Read("lp_method", true, "", "Local Planner Method");
    dm = in_pProblem->GetDistanceMetric()->GetMethod(dm_label);

    if (filename.length() > 0) {
      Roadmap< CFG, WEIGHT > tmp_roadmap;
      tmp_roadmap.ReadRoadmapGRAPHONLY(filename.c_str());
      tmp_roadmap.m_pRoadmap->GetVerticesData(m_witness_cfgs);
    }

  }

  virtual ~MPRegionComparerMethod() {}

  bool CfgIsVisible(CFG& in_cfg, vector< CFG >& in_vec_cfg) {
    //TODO - revisit
    #ifndef _PARALLEL
    LocalPlanners < CFG, WEIGHT > * lp = m_pProblem->GetMPStrategy()->GetLocalPlanners();
    LPOutput< CFG, WEIGHT > lp_output;
    Environment * env = m_pProblem->GetEnvironment();
//    CollisionDetection * cd = m_pProblem->GetCollisionDetection();
    double pos_res = m_pProblem->GetEnvironment()->GetPositionRes();
    double ori_res = m_pProblem->GetEnvironment()->GetOrientationRes();
    StatClass Stats;

    //compare_distance<CFG> mydistcompare(in_cfg, dm, env);


    typedef typename vector< CFG >::iterator CFG_ITRTR;
    for(CFG_ITRTR i_vec_cfg = in_vec_cfg.begin(); i_vec_cfg < in_vec_cfg.end(); i_vec_cfg++) {
      if (in_cfg == (*i_vec_cfg)) {
	return true; //   in_cfg is actually inside in_vec_cfg
      }
    }

   sort(in_vec_cfg.begin(), in_vec_cfg.end(), compare_distance<CFG>(in_cfg, dm, env));
   for(CFG_ITRTR i_vec_cfg = in_vec_cfg.begin(); i_vec_cfg < in_vec_cfg.end(); i_vec_cfg++) {
		 if (lp->GetMethod(m_lp)->
           IsConnected(env, Stats, dm, in_cfg, (*i_vec_cfg),
                       &lp_output, pos_res, ori_res, true)) {
	return true; // 	stop as soon in_cfg can connect to one cfg in in_vec_cfg
      }
   }
   #endif
    return false;  //in_cfg is not visible to anything in_vec_cfg
  }

  bool CanConnectComponents(vector < CFG > & cc_a, vector < CFG > & cc_b) {
    // variables needed for the local planner call in loop
    LocalPlanners < CFG, WEIGHT > * lp = m_pProblem->GetMPStrategy()->GetLocalPlanners();
    LPOutput< CFG, WEIGHT > lp_output;
    Environment * env = m_pProblem->GetEnvironment();
//    CollisionDetection * cd = m_pProblem->GetCollisionDetection();
    double pos_res = m_pProblem->GetEnvironment()->GetPositionRes();
    double ori_res = m_pProblem->GetEnvironment()->GetOrientationRes();
    StatClass Stats;

    typedef typename vector< CFG >::iterator CFG_ITRTR;
    for(CFG_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
      for (CFG_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
				if (lp->GetMethod(m_lp)->
              IsConnected(env, Stats, dm, (*i_cc_a), (*i_cc_b),
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

    vector < pair< size_t, VID > > cc_a;
    stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
    get_cc_stats(*(rdmp_a->m_pRoadmap),cmap, cc_a);
    a_small_cc_size = int(double(cc_a[0].first) * double(0.01));
    cout << " Components in Roadmap A " << cc_a.size() << " small_cc_size " << a_small_cc_size << endl;
    vector < pair< size_t, VID > > cc_b;
    cmap.reset();
    get_cc_stats(*(rdmp_b->m_pRoadmap),cmap, cc_b);
    b_small_cc_size = int(double(cc_b[0].first) * double(0.01));
    cout << " Components in Roadmap B " << cc_b.size() << " small_cc_size " << b_small_cc_size << endl;

    typedef typename vector < pair< size_t, VID > >::iterator CC_ITRTR;

    int spanning_cc_a = 0; // components in A that can connect to B
    int not_small_cc_as = 0;
    // for each connected component cc_a in A
    for (CC_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
      vector < CFG > cc_a_cfgs;
      cc_a_cfgs.clear();
      vector <VID> cc_a_cfgs_aux;
      cc_a_cfgs_aux.clear();
      cmap.reset();
      get_cc(*(rdmp_a->m_pRoadmap), cmap, ((*(rdmp_a->m_pRoadmap->find_vertex(i_cc_a->second))).descriptor()), cc_a_cfgs_aux);

     for(typename vector<VID>::iterator itr=cc_a_cfgs_aux.begin(); itr!=cc_a_cfgs_aux.end(); ++itr)
                cc_a_cfgs.push_back((*(rdmp_a->m_pRoadmap->find_vertex(*itr))).property());

      if ((int)cc_a_cfgs.size() >= a_small_cc_size) {
	not_small_cc_as++;
	int connectable_cc_a_to_b = 0;
	// for each connected component cc_b in B
	for (CC_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
	  vector <CFG > cc_b_cfgs;
	  vector <VID> cc_b_cfgs_aux;
	  cmap.reset();
	  get_cc(*(rdmp_b->m_pRoadmap), cmap, (*(rdmp_b->m_pRoadmap->find_vertex(i_cc_b->second))).descriptor(), cc_b_cfgs_aux);

        for(typename vector<VID>::iterator itr=cc_b_cfgs_aux.begin(); itr!=cc_b_cfgs_aux.end(); ++itr)
                cc_b_cfgs.push_back((*(rdmp_b->m_pRoadmap->find_vertex(*itr))).property());

	  if ((int)cc_b_cfgs.size() >= b_small_cc_size) {
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
    vector < pair< size_t, VID > > cc;
    stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
    get_cc_stats(*(rdmp->m_pRoadmap), cmap, cc);
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
      //unsigned int j=0;
      bool i_witness_can_connect = false;
      for (unsigned int j=0; j < cc.size(); j++) {
	vector < CFG > cc_cfgs;
	vector <VID> cc_cfgs_aux;
	cmap.reset();
	//get_cc(*(rdmp->m_pRoadmap), cmap, *(new CFG(rdmp->m_pRoadmap->find_vertex(cc[j].second).descriptor())), cc_cfgs_aux);
	//fix_lantao is following right?
        //GetCC(*(rdmp->m_pRoadmap), *(new CFG(rdmp->m_pRoadmap->GetData(cc[j].second))), cc_cfgs);
	get_cc(*(rdmp->m_pRoadmap), cmap, (*(rdmp->m_pRoadmap->find_vertex(cc[j].second))).descriptor(), cc_cfgs_aux);
	for(typename vector<VID>::iterator itr=cc_cfgs_aux.begin(); itr!=cc_cfgs_aux.end(); ++itr)
                cc_cfgs.push_back((*(rdmp->m_pRoadmap->find_vertex(*itr))).property());

	if ((int)cc_cfgs.size() >= small_cc_size) {
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

  virtual void Compare(int in_region_a, int in_region_b) = 0;

  virtual void Print(ostream& out_os) const {
    out_os << "    " << GetLabel() << ":: ";
  }

  MPProblem * m_pProblem;
  vector< CFG > m_witness_cfgs;
  shared_ptr< DistanceMetricMethod> dm;
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class ConnectableComponentComparer : public MPRegionComparerMethod<CFG,WEIGHT>
{
public:

  ConnectableComponentComparer(XMLNode& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
  }


  virtual void Compare(int in_region_a, int in_region_b) {

    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b

    pair< unsigned int, unsigned int > spanning_a = this->ComponentsASpanningInB(rdmp_a, rdmp_b);
    cout << "ConnectableComponentComparer::Compare: Not small ccs in A " << spanning_a.second << "; Span more than 1 in B: " << spanning_a.first << " = " << 100*spanning_a.first/spanning_a.second << "%" << endl;

    pair< unsigned int, unsigned int > spanning_b = this->ComponentsASpanningInB(rdmp_b, rdmp_a);
    cout << "ConnectableComponentComparer::Compare: Not small ccs in B " << spanning_b.second << "; Span more than 1 in A: " << spanning_b.first << " = " << 100*spanning_b.first/spanning_b.second << "%" << endl;


  }


private:
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class RandomConnectComparer : public MPRegionComparerMethod< CFG, WEIGHT >
{
public:
  RandomConnectComparer(XMLNode& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
  }

  virtual void Compare(int in_region_a, int in_region_b) {

    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b

    unsigned int witness_queries = this->m_witness_cfgs.size()*(this->m_witness_cfgs.size()-1)/2;
    cout << "RandomConnectComparer::Compare: Witness size: " << this->m_witness_cfgs.size() << "; Witness queries: " << witness_queries << endl;

    pair< unsigned int, unsigned int > witness_to_a = this->ConnectionsWitnessToRoadmap(this->m_witness_cfgs, rdmp_a);
    double witness_to_a_connections_percent = 100*witness_to_a.first/this->m_witness_cfgs.size();
    double witness_to_a_succ_queries_percent = 100*witness_to_a.second/witness_queries;
    cout << "RandomConnectComparer::Compare: Witness nodes connected to non-small ccs in A: " << witness_to_a.first << " = " << witness_to_a_connections_percent << "%" << endl;
    cout << "RandomConnectComparer::Compare: Witness queries succesful in non-small ccs in A: " << witness_to_a.second << " = " << witness_to_a_succ_queries_percent << "%" << endl;

    pair< unsigned int, unsigned int > witness_to_b = this->ConnectionsWitnessToRoadmap(this->m_witness_cfgs, rdmp_b);
    double witness_to_b_connections_percent = 100*witness_to_b.first/this->m_witness_cfgs.size();
    double witness_to_b_succ_queries_percent = 100*witness_to_b.second/witness_queries;
    cout << "RandomConnectComparer::Compare: Witness nodes connected to non-small ccs in B: " << witness_to_b.first << " = " << witness_to_b_connections_percent << "%" << endl;
    cout << "RandomConnectComparer::Compare: Witness queries succesful in non-small ccs in B: " << witness_to_b.second << " = " << witness_to_b_succ_queries_percent << "%" << endl;
  }

private:

};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class RegionCoverageComparer : public MPRegionComparerMethod< CFG, WEIGHT >
{
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  RegionCoverageComparer(XMLNode& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
  }

  virtual void Compare(int in_region_a, int in_region_b) {
   ///TODO - temporarily disable - revisit if needed
   #ifndef _PARALLEL

    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b


    int a_small_cc_size =0;
    int b_small_cc_size =0;

    stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
    vector< pair <size_t,VID > > a_cc_stats;
    get_cc_stats (*(rdmp_a->m_pRoadmap),cmap, a_cc_stats);
    vector< pair <size_t,VID > > b_cc_stats;
    cmap.reset();
    get_cc_stats(*(rdmp_b->m_pRoadmap),cmap, b_cc_stats);
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
      if(!this->CfgIsVisible(((*(rdmp_a->m_pRoadmap->find_vertex(*a_vid_itr))).property()), rdmp_b_cfg)) {
         // found a potential revealing/trapped node
        vector<VID> tmp_cc;
	cmap.reset();
        get_cc(*(rdmp_a->m_pRoadmap),cmap,(*a_vid_itr), tmp_cc);
        if( (int)tmp_cc.size() >= a_small_cc_size) {
          // found a revealing node
          a_revealing_node++;
        } else { a_trapped_node++;} //found a trapped node.
      }
    }

    typename vector<VID>::iterator b_vid_itr;
    for(b_vid_itr = rdmp_b_vids.begin(); b_vid_itr!=rdmp_b_vids.end(); ++b_vid_itr) {
     if(!this->CfgIsVisible(((*(rdmp_b->m_pRoadmap->find_vertex(*b_vid_itr))).property()), rdmp_a_cfg)) {
        // found a potential revealing/trapped node
        vector<VID> tmp_cc;
	cmap.reset();
        get_cc(*(rdmp_b->m_pRoadmap),cmap,(*b_vid_itr), tmp_cc);
        if( (int)tmp_cc.size() >= b_small_cc_size) {
          // found a revealing node
          b_revealing_node++;
        } else { b_trapped_node++;} //found a trapped node.
      }
    }

    cout << "RegionCoverageComparer::Compare: a_revealing_node: " << a_revealing_node << endl;
    cout << "RegionCoverageComparer::Compare: a_trapped_node: " << a_trapped_node << endl;
    cout << "RegionCoverageComparer::Compare: b_revealing_node: " << b_revealing_node << endl;
    cout << "RegionCoverageComparer::Compare: b_trapped_node: " << b_trapped_node << endl;
    #endif

  }

private:

};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class RegionSimilarity : public MPRegionComparerMethod< CFG, WEIGHT >
{
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  RegionSimilarity(XMLNode& in_Node, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_Node, in_pProblem) {
  }

  virtual void Compare(int in_region_a, int in_region_b) {
    ///TODO - not compiling in parallel, may have to do with constness problem in pgraph
    /// Revisit after we've decided on what to do with this class- seems not being used anywhere
    #ifndef _PARALLEL
    Roadmap< CFG, WEIGHT >* rdmp_a = this->m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = this->m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b


    int a_small_cc_size =0;
    int b_small_cc_size =0;

    vector< pair <size_t,VID > > a_cc_stats;
    stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
    get_cc_stats(*(rdmp_a->m_pRoadmap), cmap, a_cc_stats);
    vector< pair <size_t,VID > > b_cc_stats;
    cmap.reset();
    get_cc_stats(*(rdmp_b->m_pRoadmap), cmap, b_cc_stats);
    a_small_cc_size = int(double(a_cc_stats[0].first) * double(0.01));
    b_small_cc_size = int(double(b_cc_stats[0].first) * double(0.01));
    cout << "RegionSimilarity::Compare: a_small_cc_size: " << a_small_cc_size << endl;
    cout << "RegionSimilarity::Compare: b_small_cc_size: " << b_small_cc_size << endl;

    //Get vector<VID> from a;
    vector<CFG> usable_rdmp_a_cfg, usable_rdmp_b_cfg;  ///\todo implement this better with iterators.
    vector<VID> all_rdmp_a_vids, all_rdmp_b_vids;
    vector<VID> usable_rdmp_a_vids, usable_rdmp_b_vids;

    rdmp_a->m_pRoadmap->GetVerticesVID(all_rdmp_a_vids);
    rdmp_b->m_pRoadmap->GetVerticesVID(all_rdmp_b_vids);


    //Get usable nodes in a
    typename vector<VID>::iterator all_a_vid_itr;
    for(all_a_vid_itr = all_rdmp_a_vids.begin(); all_a_vid_itr!=all_rdmp_a_vids.end(); ++all_a_vid_itr) {
      vector<VID> tmp_cc;
      cmap.reset();
      get_cc(*(rdmp_a->m_pRoadmap), cmap, (*all_a_vid_itr), tmp_cc);
      if( (int)tmp_cc.size() >= a_small_cc_size) {
         usable_rdmp_a_vids.push_back((*all_a_vid_itr));
         usable_rdmp_a_cfg.push_back((*(rdmp_a->m_pRoadmap->find_vertex((*all_a_vid_itr)))).property());
      } //else not usable!!
    }

    typename vector<VID>::iterator all_b_vid_itr;
    for(all_b_vid_itr = all_rdmp_b_vids.begin(); all_b_vid_itr!=all_rdmp_b_vids.end(); ++all_b_vid_itr) {
      vector<VID> tmp_cc;
      cmap.reset();
      get_cc(*(rdmp_b->m_pRoadmap), cmap, (*all_b_vid_itr), tmp_cc);
      if( (int)tmp_cc.size() >= b_small_cc_size) {
         usable_rdmp_b_vids.push_back((*all_b_vid_itr));
         usable_rdmp_b_cfg.push_back((*(rdmp_b->m_pRoadmap->find_vertex((*all_b_vid_itr)))).property());
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
        cmap.reset();
        get_cc(*(rdmp_a->m_pRoadmap),cmap, (*usable_a_vid_itr), tmp_cc);
        if( (int)tmp_cc.size() >= a_small_cc_size) {  // Make sure its a "usable" node
          if(this->CfgIsVisible(((*(rdmp_a->m_pRoadmap->find_vertex(*usable_a_vid_itr))).property()), usable_rdmp_b_cfg)) {
            ++node_a_visible_in_b;
          } else { ++node_a_not_visible_in_b; }
      }
    }

    typename vector<VID>::iterator usable_b_vid_itr;
    for(usable_b_vid_itr = usable_rdmp_b_vids.begin();
              usable_b_vid_itr!=usable_rdmp_b_vids.end(); ++usable_b_vid_itr) {
        vector<VID> tmp_cc;
  	cmap.reset();
        get_cc(*(rdmp_b->m_pRoadmap), cmap, (*usable_b_vid_itr), tmp_cc);
        if( (int)tmp_cc.size() >= b_small_cc_size) {  // Make sure its a "usable" node
          if(this->CfgIsVisible(((*(rdmp_b->m_pRoadmap->find_vertex(*usable_b_vid_itr))).property()), usable_rdmp_a_cfg)) {
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
    //rdmp_a->m_pRoadmap->GetEdges(all_edges_a);
    //rdmp_b->m_pRoadmap->GetEdges(all_edges_b);
//    vector<GRAPH::edge_descriptor> v_ed;
    RoadmapGraph<CFG, WEIGHT> ga = *(rdmp_a->m_pRoadmap);
    RoadmapGraph<CFG, WEIGHT> gb = *(rdmp_b->m_pRoadmap);

/*
for(typename RoadmapGraph<CFG, WEIGHT>::edge_iterator ei_a = ga.edges_begin(); ei_a != ga.edges_end(); ++ei_a){
  //all_edges_a.push_back(ei_a.property());
  all_edges_a.push_back(pair<VID, VID>(ei_a.source(), ei_a.target()));

}
*/

    //typename vector< pair<VID,VID> >::iterator all_edges_a_itr;
    //for(all_edges_a_itr = all_edges_a.begin(); all_edges_a_itr != all_edges_a.end();
    //       ++all_edges_a_itr) {
    typename RoadmapGraph<CFG, WEIGHT>::edge_iterator all_edges_a_itr;
    for(all_edges_a_itr = ga.edges_begin(); all_edges_a_itr != ga.edges_end(); ++all_edges_a_itr) {
      vector<VID> tmp_cc;
      cmap.reset();
      get_cc(*(rdmp_a->m_pRoadmap),cmap,(*(all_edges_a_itr)).source(), tmp_cc);
      if( (int)tmp_cc.size() < a_small_cc_size) { continue; } // not a edge in "usable" component
      //for all connected components in rdmp_b....
      vector< pair<size_t,VID> > all_cc_b;
      cmap.reset();
      get_cc_stats(*(rdmp_b->m_pRoadmap), cmap, all_cc_b);
      bool found_edge_a_visible_in_b=false;
      typename vector< pair<size_t,VID> >::iterator all_cc_b_itr;
      for(all_cc_b_itr = all_cc_b.begin(); all_cc_b_itr != all_cc_b.end(); ++all_cc_b_itr) {
        if((int)(*all_cc_b_itr).first < b_small_cc_size) { continue; } //not usable cc
        vector<CFG> usable_cc_in_b;
	usable_cc_in_b.clear();
 	vector<VID> usable_cc_in_b_aux;
	usable_cc_in_b_aux.clear();
	cmap.reset();
        get_cc(*(rdmp_b->m_pRoadmap), cmap,
              ((*(rdmp_b->m_pRoadmap->find_vertex((*all_cc_b_itr).second))).descriptor()),
              usable_cc_in_b_aux);
	for(typename vector<VID>::iterator itr=usable_cc_in_b_aux.begin(); itr!=usable_cc_in_b_aux.end(); ++itr)
		usable_cc_in_b.push_back((*(rdmp_b->m_pRoadmap->find_vertex(*itr))).property());

        //if(CfgIsVisible((rdmp_a->m_pRoadmap->find_vertex((*all_edges_a_itr).first).property()),
        if(this->CfgIsVisible(((*(rdmp_a->m_pRoadmap->find_vertex((*(all_edges_a_itr)).source()))).property()),
                        usable_cc_in_b)) {
          if(this->CfgIsVisible(((*(rdmp_a->m_pRoadmap->find_vertex((*(all_edges_a_itr)).target()))).property()),
                        usable_cc_in_b)) {
            ++edge_a_visible_in_b;
            found_edge_a_visible_in_b = true;
            break;
          }
        }
    }
    if(!found_edge_a_visible_in_b) {++edge_a_not_visible_in_b;}
  }

  //typename vector< pair<VID,VID> >::iterator all_edges_b_itr;
  //for(all_edges_b_itr = all_edges_b.begin(); all_edges_b_itr != all_edges_b.end();
  //       ++all_edges_b_itr) {
  typename RoadmapGraph<CFG, WEIGHT>::edge_iterator all_edges_b_itr;
  for(all_edges_b_itr = gb.edges_begin(); all_edges_b_itr != gb.edges_end(); ++all_edges_b_itr) {
    vector<VID> tmp_cc;
    cmap.reset();
    //get_cc(*(rdmp_b->m_pRoadmap), cmap, (*all_edges_b_itr).first, tmp_cc);
    get_cc(*(rdmp_b->m_pRoadmap), cmap, (*(all_edges_b_itr)).source(), tmp_cc);
    if( (int)tmp_cc.size() < b_small_cc_size) { continue; } // not a edge in "usable" component
    //for all connected components in rdmp_a....
    vector< pair<size_t,VID> > all_cc_a;
    cmap.reset();
    get_cc_stats(*(rdmp_a->m_pRoadmap),cmap, all_cc_a);
    typename vector< pair<size_t,VID> >::iterator all_cc_a_itr;
    bool found_edge_b_visible_in_a = false;
    for(all_cc_a_itr = all_cc_a.begin(); all_cc_a_itr != all_cc_a.end(); ++all_cc_a_itr) {
      if((int)(*all_cc_a_itr).first < a_small_cc_size) { continue; } //not usable cc
      vector<CFG> usable_cc_in_a;
      vector<VID> usable_cc_in_a_aux;
      cmap.reset();
      get_cc(*(rdmp_a->m_pRoadmap),cmap,
            (*(rdmp_a->m_pRoadmap->find_vertex((*all_cc_a_itr).second))).descriptor(),
            usable_cc_in_a_aux);
      for(typename vector<VID>::iterator itr=usable_cc_in_a_aux.begin(); itr!=usable_cc_in_a_aux.end(); ++itr)
                usable_cc_in_a.push_back((*(rdmp_a->m_pRoadmap->find_vertex(*itr))).property());

      if(this->CfgIsVisible((*(rdmp_b->m_pRoadmap->find_vertex((*(all_edges_b_itr)).source()))).property(),
                      usable_cc_in_a)) {
        if(this->CfgIsVisible((*(rdmp_b->m_pRoadmap->find_vertex((*(all_edges_b_itr)).target()))).property(),
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
  #endif

  }

private:
};

#endif /* _MPREGIONCOMPARERMETHOD_H_ */
