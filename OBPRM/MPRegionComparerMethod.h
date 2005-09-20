#ifndef _MPREGIONCOMPARERMETHOD_H_
#define _MPREGIONCOMPARERMETHOD_H_

#include "MPRegion.h"

#include "Roadmap.h"

template <class CFG, class WEIGHT>
class MPRegionComparerMethod: public MPBaseObject {
 public:
  MPRegionComparerMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPBaseObject(in_pNode, in_pProblem) { 
    m_pProblem = in_pProblem;
    
    const char * witness_file = in_pNode->ToElement()->Attribute("witness_file");
    if (witness_file) {
      Roadmap< CFG, WEIGHT > tmp_roadmap;
      tmp_roadmap.ReadRoadmapGRAPHONLY(witness_file);
      tmp_roadmap.m_pRoadmap->GetVerticesData(m_witness_cfgs);
    LOG_DEBUG_MSG("MPRegionComparerMethod:: input cfgs: " << m_witness_cfgs.size());

    }

  }

  bool CanConnectComponents(vector < CFG > & cc_a, vector < CFG > & cc_b) {
    // variables needed for the local planner call in loop
    LocalPlanners < CFG, WEIGHT > * lp = m_pProblem->GetMPStrategy()->GetLocalPlanners();
    LPOutput< CFG, WEIGHT > lp_output; 
    Environment * env = m_pProblem->GetEnvironment();
    CollisionDetection * cd = m_pProblem->GetCollisionDetection();
    DistanceMetric * dm = m_pProblem->GetDistanceMetric();
    double pos_res = m_pProblem->GetEnvironment()->GetPositionRes();
    double ori_res = m_pProblem->GetEnvironment()->GetOrientationRes();
    Stat_Class Stats;
 
    typedef typename vector< CFG >::iterator CFG_ITRTR;
    for(CFG_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
      for (CFG_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
	if (lp->IsConnected(env, Stats, cd, dm, (*i_cc_a), (*i_cc_b), 
			    &lp_output, pos_res, ori_res, true))
	  return true; // stop as soon as one cc in a can connect to a node in b
      }
    }
    return false;
  }
 
  pair < unsigned int, unsigned int > ConnectionsWitnessToRoadmap(vector < CFG > & witness_cfgs, Roadmap< CFG, WEIGHT > *rdmp) {
    vector < pair< int, VID > > cc;
    GetCCStats(*(rdmp->m_pRoadmap), cc);
    vector < unsigned int > connected_to_cc;
    for(unsigned int i=0; i< cc.size(); i++)
      connected_to_cc.push_back(0);
    
    unsigned int possible_connections = 0;
    vector< CFG > witness_test_cfg;
    typedef typename vector< CFG >::iterator CFG_ITRTR;
    for (CFG_ITRTR i_witness_cfgs=witness_cfgs.begin(); i_witness_cfgs < witness_cfgs.end(); i_witness_cfgs++) {
      witness_test_cfg.clear();
      witness_test_cfg.push_back(*i_witness_cfgs);

      typedef typename vector < pair< int, VID > >::iterator CC_ITRTR;
      unsigned int i=0;
      for (CC_ITRTR i_cc=cc.begin(); i_cc < cc.end(); i_cc++,i++) {
	vector < CFG > cc_cfgs;
	GetCC(*(rdmp->m_pRoadmap), *(new CFG(rdmp->m_pRoadmap->GetData(i_cc->second))), cc_cfgs);
	if ( CanConnectComponents(witness_test_cfg, cc_cfgs) ) {
	  possible_connections++; 
	  connected_to_cc[i]++;
	}
      }
    }

    unsigned int possible_queries = 0;
    typedef typename vector< unsigned int >::iterator INT_ITRTR;
    for (INT_ITRTR i_con=connected_to_cc.begin(); i_con < connected_to_cc.end(); i_con++)
      if ((*i_con) != 0) {
	// count whether *i_witness_cfg can connect to this cc in rdmp
	// count the number of queries that could be done through this cc
	possible_queries += (*i_con)*((*i_con)-1); //remember to divide by 2 at the end
	cout << "here => " << possible_connections << " queries" << possible_queries << endl;
      }

    cout << "possible connections " << possible_connections << "; possible queries " << possible_queries << endl;
    return pair < unsigned int, unsigned int>(possible_connections, possible_queries/2);
  }

  //@todo operators () without parameters and taking in pairs of region ids
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

  ConnectableComponentComparer(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_pNode, in_pProblem) {
    LOG_DEBUG_MSG("ConnectableComponentComparer::ConnectableComponentComparer()");
    if (!in_pNode) {
      LOG_ERROR_MSG("ConnectableComponentComparer::ConnectableComponentComparer() error xml input");
      exit(-1);
    }
    if (string(in_pNode->Value()) != "ConnectableComponentComparer") {
      LOG_ERROR_MSG("ConnectableComponentComparer::ConnectableComponentComparer() error xml input");
      exit(-1);
    }
    LOG_DEBUG_MSG("ConnectableComponentComparer::ConnectableComponentComparer() end");    
  }


  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("ConnectableComponentComparer::Compare(region_a, region_b)");

    Roadmap< CFG, WEIGHT >* rdmp_a = m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b

 
    vector < pair< int, VID > > cc_a;
    GetCCStats(*(rdmp_a->m_pRoadmap), cc_a);
    cout << " Components in Roadmap A " << cc_a.size() << endl;
    vector < pair< int, VID > > cc_b;
    GetCCStats(*(rdmp_b->m_pRoadmap), cc_b);
    cout << " Components in Roadmap B " << cc_b.size() << endl;

    typedef typename vector < pair< int, VID > >::iterator CC_ITRTR;

    int connectable_cc_a_cntr = 0; // components in A that can connect to B
    // for each connected component cc_a in A
    for (CC_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
      vector < CFG > cc_a_cfgs;
      GetCC(*(rdmp_a->m_pRoadmap), *(new CFG(rdmp_a->m_pRoadmap->GetData(i_cc_a->second))), cc_a_cfgs);

      // for each connected component cc_b in B
      for (CC_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
	vector <CFG > cc_b_cfgs;
	GetCC(*(rdmp_b->m_pRoadmap), *(new CFG(rdmp_b->m_pRoadmap->GetData(i_cc_b->second))), cc_b_cfgs);

	// try to connect component A to component B
	if (CanConnectComponents(cc_a_cfgs, cc_b_cfgs)) {
	  connectable_cc_a_cntr++;
	  break;
	}
      }
    }

    cout << "ConnectableComponentComparer::Compare: Components in A->B Counter: " << connectable_cc_a_cntr << " = " << 100*connectable_cc_a_cntr/cc_a.size() << "%" << endl;

    int connectable_cc_b_cntr = 0; // components in B that can connect to A
    // for each connected component cc_b in B
    for (CC_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
      vector < CFG > cc_b_cfgs;
      GetCC(*(rdmp_b->m_pRoadmap), *(new CFG(rdmp_b->m_pRoadmap->GetData(i_cc_b->second))), cc_b_cfgs);

      // for each connected component cc_a in B
      for (CC_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
	vector <CFG > cc_a_cfgs;
	GetCC(*(rdmp_a->m_pRoadmap), *(new CFG(rdmp_a->m_pRoadmap->GetData(i_cc_a->second))), cc_a_cfgs);

	// try to connect component A to component B
	if (CanConnectComponents(cc_b_cfgs, cc_a_cfgs)) {
	  connectable_cc_b_cntr++;
	  break;
	}
      }
    }

    cout << "ConnectableComponentComparer::Compare: Components in B->A Counter: " << connectable_cc_b_cntr << " = " << 100*connectable_cc_b_cntr/cc_b.size() << "%" << endl;

  }


private:

};


template <class CFG, class WEIGHT>
class RandomConnectComparer : public MPRegionComparerMethod< CFG, WEIGHT > 
{
public:
  RandomConnectComparer(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_pNode, in_pProblem) {
    LOG_DEBUG_MSG("RandomConnectComparer::RandomConnectComparer()");
    if (!in_pNode) {
      LOG_ERROR_MSG("RandomConnectComparer::RandomConnectComparer() error xml input");
      exit(-1);
    }
    if (string(in_pNode->Value()) != "RandomConnectComparer") {
      LOG_ERROR_MSG("RandomConnectComparer::RandomConnectComparer() error xml input");
      exit(-1);
    }
    
    LOG_DEBUG_MSG("RandomConnectComparer::RandomconnectComparer() end");    
  }

  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("RandomConnectComparer::Compare(region_a, region_b)");    

    Roadmap< CFG, WEIGHT >* rdmp_a = m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b

    unsigned int witness_queries = m_witness_cfgs.size()*(m_witness_cfgs.size()-1)/2;
    cout << "RandomConnectComparer::Compare: Witness size: " << m_witness_cfgs.size() << "Witness queries: " << witness_queries << endl;

    pair< unsigned int, unsigned int > witness_to_a = ConnectionsWitnessToRoadmap(m_witness_cfgs, rdmp_a);
    double witness_to_a_connections_percent = 100*witness_to_a.first/m_witness_cfgs.size();
    double witness_to_a_succ_queries_percent = 100*witness_to_a.second/witness_queries;
    cout << "RandomConnectComparer::Compare: Witness nodes connected to A: " << witness_to_a.first << " = " << witness_to_a_connections_percent << "%" << endl;
    cout << "RandomConnectComparer::Compare: Witness queries succesful in A: " << witness_to_a.second << " = " << witness_to_a_succ_queries_percent << "%" << endl;

    pair< unsigned int, unsigned int > witness_to_b = ConnectionsWitnessToRoadmap(m_witness_cfgs, rdmp_b);
    double witness_to_b_connections_percent = 100*witness_to_b.first/m_witness_cfgs.size();
    double witness_to_b_succ_queries_percent = 100*witness_to_b.second/witness_queries;
    cout << "RandomConnectComparer::Compare: Witness nodes connected to B: " << witness_to_b.first << " = " << witness_to_b_connections_percent << "%" << endl;
    cout << "RandomConnectComparer::Compare: Witness queries succesful in B: " << witness_to_b.second << " = " << witness_to_b_succ_queries_percent << "%" << endl;
  }

private:

};



#endif /* _MPREGIONCOMPARERMETHOD_H_ */
