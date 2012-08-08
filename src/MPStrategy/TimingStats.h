#ifndef TimingStats
#define TimingStats

#include "MPStrategyMethod.h"
#include "Roadmap.h"
#include "MPProblem.h"

class RTSContainer : public MPSMContainer {
public:
  RTSContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
    vector<string> files;
    vector<string> method_names;
    string outfile;
    int interval;
    int queries;
    int k;
    int getOnlyAt;
    MPSMContainer parent;

};

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
    RoadmapTimingStats(RTSContainer cont) : MPStrategyMethod(cont.parent) {
      files = cont.files;
      method_names = cont.method_names;
      outfile = cont.outfile;
      interval = cont.interval;
      queries = cont.queries;
      k = cont.k;
      getOnlyAt = cont.getOnlyAt;
}
    RoadmapTimingStats(XMLNodeReader& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
      ParseXML(in_Node);
    }

    void ParseXML(XMLNodeReader& in_Node) {
      cout << "TimingStats::ParseXML()" << endl;
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
      while((int)vertices.size()!=queries && (int)vertices.size()>0){
        //cout<<"n verticies ="<<vertices.size()<<endl;;
        if((((int)vertices.size()-queries) % _interval == 0 && getOnlyAt==-1)||((int)vertices.size()-queries)==getOnlyAt){
	  //cout<<"computing times at interval"<<vertices.size()-queries<<endl;
	  intervals.push_back(vertices.size()-queries);
	  //compute timing stuff here
	  vector<double> *times_at_interval = new vector<double>;
	  for(vector<string>::iterator iter = method_names.begin(); iter!=method_names.end(); iter++){
	    //cout<<"getting time for nm method "<<*iter<<endl;
	    NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
	    NeighborhoodFinder::NeighborhoodFinderPointer nfmp;
	    nfmp = nf->GetMethod(*iter);
	    //double time_bf=nfmp->GetTotalTime();
            RoadmapGraph<CfgType,WeightType>* pMap = rmp.m_pRoadmap;
	    double time_el_total=0;
            for(RoadmapGraph<CfgType,WeightType>::VDI vitr = pMap->descriptor_begin(); vitr != pMap->descriptor_end(); ++vitr) {
	      vector<VID> neighbors(k)
		;	    double time_bf=nfmp->GetTotalTime();
	      nfmp->KClosest(&rmp, vertices.begin(), vertices.end(), *vitr, k, neighbors.begin());
	      double time_el=nfmp->GetTotalTime()-time_bf;
	      time_el_total+=time_el;
	      //nf->KClosest(nfmp, &rmp, *vitr, k, neighbors.begin());
	    }
	    //double time_el=nfmp->GetTotalTime()-time_bf;
	    cout<<"time_el_total ="<<time_el_total<<endl;
	    times_at_interval->push_back(time_el_total);
	  }
	  times.push_back(*times_at_interval);
	  //nfmp.get()->reset();

	}
	//rmp.m_pRoadmap->delete_vertex(vertices.back());
	if(((int)vertices.size()-queries)==getOnlyAt){
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

  virtual void PrintOptions(ostream& out_os) { }
   virtual void Initialize(int in_RegionID){}
   virtual void Run(int in_RegionID){
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
   virtual void Finalize(int in_RegionID){}

};

 


#endif
