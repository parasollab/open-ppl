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
  }

  //@todo operators () without parameters and taking in pairs of region ids
/*   virtual void operator() () = 0; */
/*   virtual void operator() (int in_RegionID_a, int in_RegionID_b) = 0; */

  virtual bool Compare(int in_region_a, int in_region_b) = 0;

  virtual void PrintOptions(ostream& out_os) {
    out_os << "    " << GetLabel() << ":: ";
  }

  MPProblem * m_pProblem;
};


template <class CFG, class WEIGHT>
class ConnectableNodeComparer : public MPRegionComparerMethod<CFG,WEIGHT> 
{
public:

  ConnectableNodeComparer(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
    MPRegionComparerMethod<CFG,WEIGHT> (in_pNode, in_pProblem) {
    LOG_DEBUG_MSG("ConnectableNodeComparer::ConnectableNodeComparer()");
    if (!in_pNode) {
      LOG_ERROR_MSG("ConnectableNodeComparer::ConnectableNodeComparer() error xml input");
      exit(-1);
    }
    if (string(in_pNode->Value()) != "ConnectableNodeComparer") {
      LOG_ERROR_MSG("ConnectableNodeComparer::ConnectableNodeComparer() error xml input");
      exit(-1);
    }
    LOG_DEBUG_MSG("ConnectableNodeComparer::ConnectableNodeComparer() end");    
  }


  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("ConnectableNodeComparer::Compare(region_a, region_b)");

    Roadmap< CFG, WEIGHT >* rdmp_a = m_pProblem->GetMPRegion(in_region_a)->GetRoadmap(); // get rdmp_a from region_a
    Roadmap< CFG, WEIGHT >* rdmp_b = m_pProblem->GetMPRegion(in_region_b)->GetRoadmap(); // get rdmp_b from region_b

    int connectable_cc_cntr = 0; // components in A that can connect to B
    vector < pair< int, VID > > cc_a;
    GetCCStats(*(rdmp_a->m_pRoadmap), cc_a);
    cout << " Components in Roadmap A " << cc_a.size() << endl;
    vector < pair< int, VID > > cc_b;
    GetCCStats(*(rdmp_b->m_pRoadmap), cc_b);
    cout << " Components in Roadmap B " << cc_b.size() << endl;

    typedef typename vector < pair< int, VID > >::iterator CC_ITRTR;

    // for each connected component cc_a in A
    for (CC_ITRTR i_cc_a = cc_a.begin(); i_cc_a < cc_a.end(); i_cc_a++) {
      vector < CFG > cc_a_cfgs;
      CFG tmp_cfg_a = rdmp_a->m_pRoadmap->GetData(i_cc_a->second);
      GetCC(*(rdmp_a->m_pRoadmap), *(new CFG(rdmp_a->m_pRoadmap->GetData(i_cc_a->second))), cc_a_cfgs);

      // for each connected component cc_b in B
      for (CC_ITRTR i_cc_b = cc_b.begin(); i_cc_b < cc_b.end(); i_cc_b++) {
	vector <CFG > cc_b_cfgs;
	GetCC(*(rdmp_b->m_pRoadmap), *(new CFG(rdmp_b->m_pRoadmap->GetData(i_cc_b->second))), cc_b_cfgs);

	// try to connect component A to component B
	if (CanConnectComponents(cc_a_cfgs, cc_b_cfgs)) {
	  connectable_cc_cntr++;
	  break;
	}
      }
    }

    cout << "ConnectableNodeComparer::Compare: Components in A->B Counter: " << connectable_cc_cntr << endl;


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

private:

};


template <class CFG, class WEIGHT>
class RandomConnectComparer : public MPRegionComparerMethod<CFG,WEIGHT> 
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
    
    in_pNode->ToElement()->QueryIntAttribute("number_of_configurations", &m_number_of_configurations);

    LOG_DEBUG_MSG("RandomConnectComparer::RandomconnectComparer() end");    
  }

  virtual bool Compare(int in_region_a, int in_region_b) {
    LOG_DEBUG_MSG("RandomConnectComparer::Compare(region_a, region_b)");    


  }

private:

  int m_number_of_configurations;
};



#endif /* _MPREGIONCOMPARERMETHOD_H_ */
