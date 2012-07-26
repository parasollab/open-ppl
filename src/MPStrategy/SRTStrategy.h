/**
 * SRTStrategy.h
 * 
 * Description: RRT Strategy header file
 *
 * Author: Kasra Manavi
 * Last Edited: 04/04/2011
 */

#ifndef BasicSRTStrategy_h
#define BasicSRTStrategy_h

#include "Roadmap.h"
#include "MPProblem/RoadmapGraph.h" //for VID typedef
#include "MetricUtils.h"
//#include "Clock_Class.h"
//#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ValidityChecker.hpp"
#include "Connector.h"
#include "LocalPlanners.h"
#include "MPStrategy/MPStrategyMethod.h"
//#include "Connector/RRTConnect.h"
#include "RRTcomponents.h"
#include "MPProblem/MPProblem.h"
#include "MPCharacterizer.h"
#include "MPRegion.h"
#include "Sampler.h"

class SRTStrategy : public MPStrategyMethod {
 
 public:

   SRTStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool isInherited=false);
   virtual ~SRTStrategy();
   virtual void ParseXML(XMLNodeReader& in_Node);
   virtual void Initialize(int in_RegionID);
   virtual void Run(int in_RegionID);
   virtual void Run(int in_RegionID, //RoadmapGraph<CfgType, WeightType>* candGraph,
		    vector<pair<CfgType,vector<VID> > >* trees);
   virtual void Finalize(int in_RegionID);
   virtual void PrintOptions(ostream& out_os);
   virtual void RRTStrategy(int in_RegionID, vector<CfgType> RRTQueue, 
			    //RoadmapGraph<CfgType, WeightType>* candGraph,
			    vector<pair<CfgType,vector<VID> > >* trees);

 protected:
   
   // Helper functions
   void ConnectComponents(MPRegion<CfgType, WeightType>* region);
   bool EvaluateMap(int in_RegionID);

   // Data
   vector<pair<string, int> > m_NodeGenerationLabels;
   vector<string> m_NodeConnectionLabels;
   vector<string> m_ComponentConnectionLabels;
   vector<string> m_EvaluatorLabels;
   string m_LPMethod, dm_label, nf_label, strVcmethod;
   double delta, minDist, obsDist, roots, growthFocus;
   int m_CurrentIteration;
   bool attempts;

 private:

   ClockClass MapGenClock;
   
};

#include "MPStrategy.h"
#include "MapEvaluator.h"

#endif
