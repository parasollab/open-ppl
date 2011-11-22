/**
 * BasicRRTStrategy.h
 * 
 * Description: RRT Strategy header file
 *
 * Author: Kasra Manavi
 * Last Edited: 04/04/2011
 */

#ifndef BasicRRTStrategy_h
#define BasicRRTStrategy_h

#include "Roadmap.h"
#include "MPProblem/RoadmapGraph.h" //for VID typedef
#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ValidityChecker.hpp"
#include "ConnectMap.h"
#include "LocalPlanners.h"
#include "MPStrategy/MPStrategyMethod.h"
#include "MPProblem/MPProblem.h"
#include "MPCharacterizer.h"
#include "MPRegion.h"
#include "Sampler.h"
//#include "Connector/RRTConnect.h"

class BasicRRTStrategy : public MPStrategyMethod {
 
 public:

   BasicRRTStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool isInherited=false);
   virtual ~BasicRRTStrategy();
   virtual void ParseXML(XMLNodeReader& in_Node);
   virtual void Initialize(int in_RegionID);
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID);
   virtual void PrintOptions(ostream& out_os);
   virtual void RRT(int in_RegionID, vector<CfgType>);

 protected:
   
   // Helper functions
   void ConnectComponents(MPRegion<CfgType, WeightType>* region);
   bool EvaluateMap(int in_RegionID);

   // Data
   vector<pair<string, int> > m_NodeGenerationLabels;
   vector<string> m_NodeConnectionLabels;
   vector<string> m_ComponentConnectionLabels;
   vector<string> m_EvaluatorLabels;
   string m_LPMethod;
   string dm_label;
   string nf_label;
   string strVcmethod;
   double delta, minDist, obsDist, roots, growthFocus;
   int m_CurrentIteration;

 private:

   Clock_Class MapGenClock;
   
};

#include "MPStrategy.h"
#include "MapEvaluator.h"

#endif
