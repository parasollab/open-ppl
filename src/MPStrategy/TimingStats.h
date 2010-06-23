



#ifndef TimingStats
#define TimingStats




#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "GraphAlgo.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"
#include "Query.h"
#include "GeneratePartitions.h"
#include <limits>

//#include "ExplicitInstantiation.h"
// util.h defines PMPL_EXIT used in initializing the environment
#include "util.h"
#include "MPProblem.h"
#include "MPCharacterizer.h"
#include "MapEvaluator.h"
#include "MPStrategy/MPStrategyMethod.h"
#include <math.h>


class RoadmapTimingStats : public MPStrategyMethod {
  private:
    vector<string> files;
    vector<string> method_names;
    string outfile;
    int interval;
    int queries;
    int k;
    int getOnlyAt;

  public:
    typedef RoadmapGraph<CfgType, WeightType>::VID VID;
    
    RoadmapTimingStats(XMLNodeReader& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
      ParseXML(in_Node);
    }

    void ParseXML(XMLNodeReader& in_Node) {
      cout << "TimingStats::ParseXML()" << endl;
      LOG_DEBUG_MSG("TimingStats::ParseXML()");
      interval = 100;
      queries=2;
      outfile="outfile.stats";
      k=10;
      outfile = in_Node.stringXMLParameter(string("outfile"), true, string(""),string("Outfile"));  
      cout<<"outfile="<<outfile<<endl;

      interval = in_Node.numberXMLParameter(string("interval"), true, 100, 1, 1000000, string("Printout Interval"));
      cout<<"interval="<<interval<<endl;

      queries = in_Node.numberXMLParameter(string("queries"), true, 100, 1, 1000000, string("queries"));
      cout<<"queries="<<queries<<endl;

      k = in_Node.numberXMLParameter(string("k"), true, 100, 1, 1000000, string("l"));
      cout<<"k="<<k<<endl;

      getOnlyAt = in_Node.numberXMLParameter(string("getOnlyAt"), true, 100, -1, 1000000, string("getOnlyAt"));
      cout<<"getOnlyAt="<<getOnlyAt<<endl;

      XMLNodeReader::childiterator citr;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr){
        if(citr->getName() == "file"){
          string filename = citr->stringXMLParameter(string("filename"), true, string(""),string("File Name"));
          cout<<"filename="<<filename<<endl;
          files.push_back(filename);
        }else if(citr->getName() == "nnMethod"){
          string method_name = citr->stringXMLParameter(string("method_name"), true, string(""),string("Method Name"));
          cout<<"method_name="<<method_name<<endl;
          method_names.push_back(method_name);
        } else{
          citr->warnUnknownNode();
        }
      }
    }

  


    void print_rmp_timing_stats(Roadmap<CfgType,WeightType> &rmp, int _interval){
      //vector<VID> vertices(rmp.m_pRoadmap->begin(),rmp.m_pRoadmap->end());
      vector<VID> vertices;
      vector<int> intervals;
      vector<vector<double> > times;
      while(vertices.size()!=queries && vertices.size()>0){
        //cout<<"n verticies ="<<vertices.size()<<endl;;
        if(((vertices.size()-queries) % _interval == 0 && getOnlyAt==-1)||(vertices.size()-queries)==getOnlyAt){
	  //cout<<"computing times at interval"<<vertices.size()-queries<<endl;
	  intervals.push_back(vertices.size()-queries);
	  //compute timing stuff here
	  vector<double> *times_at_interval = new vector<double>;
	  for(vector<string>::iterator iter = method_names.begin(); iter!=method_names.end(); iter++){
	    //cout<<"getting time for nm method "<<*iter<<endl;
	    NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
	    NeighborhoodFinder::NeighborhoodFinderPointer nfmp;
	    nfmp = nf->GetNFMethod(*iter);
	    //double time_bf=nfmp->GetTotalTime();
            RoadmapGraph<CfgType,WeightType>* pMap = rmp.m_pRoadmap;
            RoadmapGraph<CfgType,WeightType>::VI vitr;
	    double time_el_total=0;
            for(vitr = pMap->begin(); vitr != pMap->end(); ++vitr) {
	      vector<VID> neighbors(k)
		;	    double time_bf=nfmp->GetTotalTime();
	      nf->KClosest(nfmp, &rmp, vertices.begin(), vertices.end(), (*vitr).descriptor(), k, neighbors.begin());
	      double time_el=nfmp->GetTotalTime()-time_bf;
	      time_el_total+=time_el;
	      //nf->KClosest(nfmp, &rmp, vitr.descriptor(), k, neighbors.begin());
	    }
	    //double time_el=nfmp->GetTotalTime()-time_bf;
	    cout<<"time_el_total ="<<time_el_total<<endl;
	    times_at_interval->push_back(time_el_total);
	  }
	  times.push_back(*times_at_interval);
	  //nfmp.get()->reset();

	}
	//rmp.m_pRoadmap->delete_vertex(vertices.back());
	if((vertices.size()-queries)==getOnlyAt){
	  break;
        }
	vertices.pop_back();
      }
      cout<<"___________________________"<<endl;
      for(vector<string>::iterator iter = method_names.begin(); iter!=method_names.end(); iter++){ cout<<*iter<<" ";}
      cout<<endl;
      while(!intervals.empty()){
        cout<<intervals.back();
        for(vector<double>::iterator iter = times.back().begin(); iter!=times.back().end(); iter++){ cout<<" "<<*iter;}
	cout<<endl;
	intervals.pop_back();
	times.pop_back();
      }
    }

  void operator()(){
      cout<<"*************in operator()***********************"<<endl;
      if(files.size()==0){
        cout<<"no files"<<endl;
      }

      for(vector<string>::iterator iter = files.begin(); iter!=files.end(); iter++){
        cout<<"loading _"<<iter->c_str()<<"_"<<endl;
        Roadmap<CfgType,WeightType> rmp;
	rmp.SetEnvironment(GetMPProblem()->GetEnvironment());
        rmp.ReadRoadmapGRAPHONLY(iter->c_str());
        cout<<"timing stats for "<<iter->c_str()<<endl;
	print_rmp_timing_stats(rmp,interval);
        
      }
      cout<<"returning"<<endl;
  }


  virtual void PrintOptions(ostream& out_os) { }
  virtual void operator()(int in_RegionID) { }
};

 


#endif
