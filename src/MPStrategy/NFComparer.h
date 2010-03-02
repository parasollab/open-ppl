



#ifndef NFComparer
#define NFComparer




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

class OnlineStats {
public:
 OnlineStats () {
   Clear();
 }
 
 void AddData(const double& _d) {
   m_min = min(m_min,_d);
   m_max = max(m_max,_d);
   ++m_count;
   double delta = _d - m_mean;
   m_mean = m_mean + delta/m_count;
   m_M2 = m_M2 + delta*(_d-m_mean);
 }
 
 void Clear() {
   m_min = std::numeric_limits<double>::max();
   m_max = std::numeric_limits<double>::min();
   m_count = 0;
   m_mean = double(0);
   m_M2 = double(0);
 }
 
 double GetVariance() const { if(m_count ==0) return 0;  
        return m_M2/(m_count - double(1)); }
  double GetStandardDeviation() const { return sqrt(GetVariance()); }
 double GetMean() const { if(m_count ==0) return 0;  
  if(abs(m_mean) > 1e-3) return m_mean; else return double(0); }
 double GetMax() const { if(m_count ==0) return 0;  
  if(abs(m_mean) > 1e-3) return m_max;else return double(0); }
 double GetMin() const { if(m_count ==0) return 0;  return m_min;}
 unsigned long long int GetCount() const { return m_count;}

private:
 double m_min;
 double m_max;
 unsigned long long int m_count;
 double m_mean;
 double m_M2;
};


struct unionStats{
  int n;
  int sameCCPair;
  int differenceInSameCC;
  double ratioToUnion;
  int sameCCSizes[5];
};

//NFUnionRoadmap
class NFUnionRoadmap : public MPStrategyMethod {
 private:
  vector<string> files;
  string outfile;
  int interval;
  int queries;
 public:
  typedef RoadmapGraph<CfgType, WeightType>::VID VID;
  
  NFUnionRoadmap(XMLNodeReader& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
    ParseXML(in_Node);   
  }

  void ParseXML(XMLNodeReader& in_Node) {
    cout << "NFUnionRoadmap::ParseXML()" << endl;
    LOG_DEBUG_MSG("NFUnionRoadmap::ParseXML()");
    interval = 100;
    queries=2;
    outfile="outfile.unionstats";
    //OBPRM_srand(getSeed());
    //_____change error messages
    outfile = in_Node.stringXMLParameter(string("outfile"), true, string(""),string("Node Union Method"));
    interval = in_Node.numberXMLParameter(string("interval"), true, 100, 1, 1000000, string("Same CC Printout Interval"));
    cout<<"interval="<<interval<<endl;

    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr){
      if(citr->getName() == "file"){
        string filename = citr->stringXMLParameter(string("filename"), true, string(""),string("Union File Name"));
        cout<<"filename="<<filename<<endl;
        //cout<<"interval="<<interval<<endl;
        files.push_back(filename);
        //interval = citr->numberXMLParameter(string("interval"), true, 100, 1, 1000000, string("Same CC Printout Interval"));
      } else {
          citr->warnUnknownNode();
      }
    }
  }
  
  int compareAllPairs(Roadmap<CfgType,WeightType>& rmp){
    int sameCCPairs=0;
    //int sameCCPairsUnion=0;
    //cout<<"entering compare all"<<endl;
    vector<VID> vertices;
    rmp.m_pRoadmap->GetVerticesVID(vertices);

    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    
    for(vector<VID>::iterator iter1 = vertices.begin(); iter1!= vertices.end(); ++iter1) {
      //cout<<"in loop outer"<<endl;
      for(vector<VID>::iterator iter2 = iter1; iter2!= vertices.end(); ++iter2) {
        //cout<<"in loop inner"<<endl;
        if(*iter1!=*iter2){
          if(is_same_cc(*(rmp.m_pRoadmap), cmap, *iter1, *iter2)){
            sameCCPairs++;
          }
        }
      }
    }
    return sameCCPairs;
  }

  int fastCompareAllPairs(const Roadmap<CfgType,WeightType>& rmp){
    int sameCCPairs=0;
    vector< pair<size_t, VID> > ccstats;
    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats);
    for(vector< pair<size_t, VID> >::iterator iter = ccstats.begin(); iter < ccstats.end(); iter++){
      sameCCPairs+=iter->first*(iter->first-1)/2;      
    }
    return sameCCPairs;
  }

  //returns same cc pairs of nodes in roadmap rmp (thresholdVID is inclusive)
  int fastCompareAllPairs(const Roadmap<CfgType,WeightType>& rmp, int thresholdVID){
    //cout<<"in fastCompareAllPairs"<<endl;
    int sameCCPairs=0;
    vector< pair<size_t, VID> > ccstats;
    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats);
    //TODO:get cc for each stat
    //remove ones with VID > thresholdVID
    //add choose 2 of that
    for(vector< pair<size_t,VID> >::iterator iter = ccstats.begin(); iter < ccstats.end(); iter++){
      vector<VID> cci_vid;  

      get_cc(*(rmp.m_pRoadmap), cmap, iter->second, cci_vid);
      int count = 0;
      for(vector<VID>::iterator iter2 = cci_vid.begin(); iter2 < cci_vid.end(); iter2++){
        if(*iter2<=thresholdVID)
          count++;
      }
      sameCCPairs+=count*(count-1)/2;
    }
    //cout<<"out fastCompareAllPairs"<<endl;
    return sameCCPairs;
  }

  vector<int> getSameCCStatsInUnion(const Roadmap<CfgType,WeightType>& unionrmp, int _interval){
    //cout<<"in getSameCCStatsInUnion"<<endl;
    vector<int> sameCCStats;
    for(int i = 0; i < unionrmp.m_pRoadmap->get_num_vertices(); i++){
      if(i % _interval == 0){
        int ccStats=fastCompareAllPairs(unionrmp, unionrmp.m_pRoadmap->get_num_vertices()-i);
        sameCCStats.push_back(ccStats);
      }
    }
    //cout<<"out getSameCCStatsInUnion"<<endl;
    return sameCCStats;
  }

  // TODO: remove printSameCC
  void storeStats(unionStats& stats, int sameCCPair, int sameCCPairUnion, int n){
    //______________
    //string statsFileName=name+outfile+".unionStats";
    //ofstream  myofstream(statsFileName.c_str());
    //int sameCCPair = fastCompareAllPairs(rmp);
    /* 
    vector<int>::iterator unionIter;
    if(is_union){
      sameCCPairUnion=fastCompareAllPairs(union_rmp);
      unionSameCCs.push_back(sameCCPairUnion);
    }else{
      unionIter=unionSameCCs.begin();
      sameCCPairUnion=*unionIter;
    }
    */
    
    //myofstream<<name.c_str();
    stats.n=n;
    //cout<<", "<<rmp.m_pRoadmap->GetNumVerts();
    stats.sameCCPair=sameCCPair;
    stats.differenceInSameCC=sameCCPairUnion-sameCCPair;
    stats.ratioToUnion=(double)sameCCPair/(double)sameCCPairUnion;
    /*
    if(printSameCC){
      vector< pair<int,VID> > ccstats;
      get_cc_stats(*(rmp.m_pRoadmap),ccstats);
      vector< pair<int,VID> >::iterator iter = ccstats.begin();
      for(int i=0; i<5; i++){
        if(iter!= ccstats.end()){
          myofstream<<", "<<iter->first;
   iter++;
        }else{
    myofstream<<", 0";
        }
      }
    }
    */
    //myofstream<<endl;
    //myofstream.close();
    
  }

  
  void storeSameCCInfo(unionStats& stats, Roadmap<CfgType,WeightType>& rmp){
    vector< pair<size_t, VID> > ccstats;
    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats);
    vector< pair<size_t, VID> >::iterator iter = ccstats.begin();
    //stats.sameCCSizes.reserve(5);
    for(int i=0; i<5; i++){
      if(iter != ccstats.end()){
 stats.sameCCSizes[i]=iter->first;
 cout<<"cc size = "<<stats.sameCCSizes[i]<<endl;
   //stats.sameCCSizes.push_back(iter->first);
        iter++;
      }else{
 stats.sameCCSizes[i]=0;
 cout<<"cc size = 0 = "<<stats.sameCCSizes[i]<<endl;
        //stats.sameCCSizes.push_back(0);
      }
      cout<<endl;
    }
  }  
  

  
  void printAllPairsAtInterval(string name, Roadmap<CfgType,WeightType>& rmp, const vector<int>& union_pairs, int _interval){
    
    //int sameCCPairUnion = fastCompareAllPairs(union_rmp);
    //cout<<"in print at interval"<<endl;    
    string statsFileName = "";
    if (name == "")
      statsFileName = outfile+".unionStats";
    else
      statsFileName = name+".unionStats";
      
    ofstream  myofstream(statsFileName.c_str());
    myofstream<<"#number of nodes, sameCCPairs, difference to union sameCCPairs, ratio to union";
    myofstream<<", size_largest_CC1,size_largest_CC2, size_largest_CC3,  size_largest_CC4, size_largest_CC5";
    myofstream<<endl;
    
    int sameCCPair=fastCompareAllPairs(rmp);
    /*if(is_union){
      //int sameCCPairUnion=fastCompareAllPairs(rmp);
      union_pairs.clear();
      union_pairs.push_back(sameCCPair);
      }*/
    unionStats stats;
    storeStats(stats, sameCCPair, union_pairs[0],rmp.m_pRoadmap->get_num_vertices()-queries);
    storeSameCCInfo(stats, rmp);
    vector<unionStats> reverseStats;
    //todo: move cc stats to its own function
    /*    vector< pair<int,VID> > ccstats;
    get_cc_stats(*(rmp.m_pRoadmap),ccstats);
    vector< pair<int,VID> >::iterator iter = ccstats.begin();
    for(int i=0; i<5; i++){
      if(iter!= ccstats.end()){
        myofstream<<", "<<iter->first;
        iter++;
      }else{
        myofstream<<", 0";
      }
    }
    */
    reverseStats.push_back(stats);
    
    int removed = 0;
    int number_intervals_printed=1;
    //cout<<"before loop"<<endl;
    
    //for(vector<VID>::iterator iter = vertices.end(); iter!= vertices.begin(); iter--) {
    GRAPH::vertex_iterator vi;
    for (vi = rmp.m_pRoadmap->end(); vi != rmp.m_pRoadmap->begin(); --vi) {    
      //cout<<"k="<<k<<endl;
      if (vi != rmp.m_pRoadmap->end()) {
        //cout<<"deleting vid="<<*iter<<endl;
        rmp.m_pRoadmap->delete_vertex((*vi).descriptor());
        //union_rmp.m_pRoadmap->DeleteVertex(*iter);
        removed ++;
        if(removed % _interval == 0){
          sameCCPair=fastCompareAllPairs(rmp);
          //if(is_union){
          // //int sameCCPairUnion=fastCompareAllPairs(rmp);
          // union_pairs.push_back(sameCCPair);
          //}
          unionStats union_stats;
          storeStats(union_stats, sameCCPair, union_pairs[number_intervals_printed], rmp.m_pRoadmap->get_num_vertices()-queries);
          storeSameCCInfo(union_stats, rmp);
          reverseStats.push_back(union_stats);
          
          //todo:move print cc to its own function
          //vector< pair<int,VID> > ccstats;
          //      get_cc_stats(*(rmp.m_pRoadmap),ccstats);
          //      vector< pair<int,VID> >::iterator iter = ccstats.begin();
          //      for(int i=0; i<5; i++){
          //        if(iter!= ccstats.end()){
          //          myofstream<<", "<<iter->first;
          //   iter++;
          // }else{
          //   myofstream<<", 0";
          // }
          //}
          //myofstream<<endl;
          
          number_intervals_printed++;
        }
      }
    }
    vector<unionStats>::iterator stats_iter = reverseStats.end();
    while(stats_iter>reverseStats.begin()){
      stats_iter--;
      myofstream<<stats_iter->n<<", ";
      myofstream<<stats_iter->sameCCPair<<", ";
      myofstream<<stats_iter->differenceInSameCC<<", ";
      myofstream<<stats_iter->ratioToUnion<<", ";
      myofstream<<stats_iter->sameCCSizes[0]<<", ";
      myofstream<<stats_iter->sameCCSizes[1]<<", ";
      myofstream<<stats_iter->sameCCSizes[2]<<", ";
      myofstream<<stats_iter->sameCCSizes[3]<<", ";
      myofstream<<stats_iter->sameCCSizes[4]<<", ";


      myofstream<<endl;
    }
    
    myofstream.close();
    
  }
  
  //removes nodes untile rmp has specified size
  void trim(Roadmap<CfgType,WeightType>& rmp, int size){
    
    GRAPH::vertex_iterator vi;
    vector<GRAPH::vertex_descriptor> v_vd;
    for (vi = rmp.m_pRoadmap->begin(); vi != rmp.m_pRoadmap->end(); ++vi) {
      v_vd.push_back((*vi).descriptor());
    }
    
    for(int i=size; i < v_vd.size(); i++){
      //cout<<"size="<<size<<"rmpsize = "<<vertices.size()<<"i="<<i<<endl;
      rmp.m_pRoadmap->delete_vertex(v_vd[i]);
    }
  }


  void printAllPairs(Roadmap<CfgType,WeightType>& union_rmp,  vector<string> _files){
    //cout<<"in print all pairs"<<endl;
    int sizeUnion;
    //cout<<"before get size"<<endl;
    sizeUnion=union_rmp.m_pRoadmap->get_num_vertices();
    //cout<<"after get size"<<endl;
    //printAllPairsAtInterval("union_rmp", union_rmp, union_rmp, interval, true);
    vector<int> union_pairs;
    cout<<"calling func"<<endl;
    union_pairs=getSameCCStatsInUnion(union_rmp, interval);
    printAllPairsAtInterval("union_rmp", union_rmp, union_pairs, interval);
    //int sameCCPairUnion = fastCompareAllPairs(union_rmp);
    //cout<<"*****************************"<<endl;
    //cout<<"map name, same cc pairs, difference between union"<<endl;
    //cout<<"union map, "<<sameCCPairUnion<<", 0"<<endl;
    //cout<<"all pairsconnectivity of union map = "<<sameCCPairUnion<<endl;
    //todo, place previously loaded roadmaps in a vector to avoid having to reload them
    int j=0;
    for(vector<string>::iterator iter = _files.begin(); iter!=_files.end(); iter++){
      //cout<<"j-"<<j<<endl;
      //char* filename = iter->c_str();
      //cout<<"printing for file"<<iter->c_str()<<endl;
      Roadmap<CfgType,WeightType> rmp;
      rmp.ReadRoadmapGRAPHONLY(iter->c_str());
      trim(rmp,sizeUnion);
      printAllPairsAtInterval(*iter, rmp, union_pairs, interval);
      //int sameCCPair = fastCompareAllPairs(rmp);
      //cout<<"*****************************"<<endl;
      //cout<<iter->c_str();
      //cout<<", "<<sameCCPair;
      //cout<<"ratio to union pair = "<<(double)sameCCPair/sameCCPairUnion<<endl;
      //cout<<", "<<sameCCPairUnion-sameCCPair<<endl;
      //cout<<"ratio of difference to connections in union pair = "<<(double)(sameCCPairUnion-sameCCPair)/sameCCPairUnion<<endl;
    }
    //cout<<"after loop 1"<<endl;
  }
   
  void mergeRoadmaps(Roadmap<CfgType,WeightType>& union_rmp, Roadmap<CfgType,WeightType>& rmp){
    //get roadmaps and compare
   
    //cout<<"entering merge"<<endl;
    vector<CfgType> verticesData1;
    union_rmp.m_pRoadmap->GetVerticesData(verticesData1);
    
    vector<CfgType> verticesData2;
    rmp.m_pRoadmap->GetVerticesData(verticesData2);
    //sort(verticesData1.begin(), verticesData1.end());
    //sort(verticesData2.begin(), verticesData2.end());
    //cout<<"sizes = "<<verticesData1.size()<<" and "<<verticesData2.size()<<endl;
    if(verticesData1!=verticesData2){
      cout<<"roadmaps do not contain the same set of verticies";
      cout<<"size set 1 ="<<verticesData1.size();
      cout<<"size set 2 ="<<verticesData2.size();
      exit(-1);
    }
    /*
    vector<CfgType>::iterator iter1 = verticesData1.begin();

    for(vector<CfgType>::iterator iter2 = verticesData2.begin(); iter2!= verticesData2.end(); ++iter2) {

      if(iter1==verticesData1.end() || *iter1!=*iter2){
        cout<<"roadmaps do not contain the same set of verticies";
        exit(-1);
        break;
      }
      iter1++;

    }
    if(iter1!=verticesData1.end()){
      cout<<"roadmaps do not contain the same set of verticies";
      exit(-1);
    }
    */
    //vector< pair< pair<VID,VID>, WeightType > > edges;
    
    //rmp.m_pRoadmap->GetEdges(edges);

    for(GRAPH::edge_iterator ei = rmp.m_pRoadmap->edges_begin(); ei != rmp.m_pRoadmap->edges_end(); ++ei){
      union_rmp.m_pRoadmap->add_edge((*ei).source(), (*ei).target(), (*ei).property());
    }
    //cout<<"exiting merge"<<endl;
  }
  /*
  //removes nodes untile rmp has specified size 
  void trim(Roadmap<CfgType,WeightType>& rmp, int size){
    vector<VID> vertices;
    rmp.m_pRoadmap->GetVerticesVID(vertices);
    for(int i=vertices.size()-1; i>size;i++){
      rmp.m_pRoadmap->DeleteVertex(vertices[i]);
    }
  }
  */

  void operator()(){
    cout<<"entering ()() union function"<<endl;
    if(files.size()==0){
      cout<<"no files"<<endl;
    }
    Roadmap<CfgType,WeightType> union_rmp;
    //vector<Roadmap<CfgType,WeightType> > maps;
    for(vector<string>::iterator iter = files.begin(); iter!=files.end(); iter++){
      //char* filename = iter->c_str();
      //cout<<"loading file name"<<iter->c_str()<<endl;
      if(iter == files.begin()){
        //cout<<"loading file"<<endl;
        //Roadmap<CfgType,WeightType> rmp;
        union_rmp.ReadRoadmapGRAPHONLY(iter->c_str());
        //union_rmp=rmp;
        //cout<<"file loaded"<<endl;
 //maps.push_back(rmp);
 //union_rmp.m_pRoadmap->GetNumVerts(sizeUnion);
      }else{
        //cout<<"loading file name"<<*iter<<endl;
    

        Roadmap<CfgType,WeightType> rmp;
        rmp.ReadRoadmapGRAPHONLY(iter->c_str());
        trim(rmp, union_rmp.m_pRoadmap->get_num_vertices());
        //int size_rmp;
        //rmp.m_pRoadmap->GetNumVerts(size_rmp);
        trim(union_rmp, rmp.m_pRoadmap->get_num_vertices());
        //merge the 2
        //cout<<"merging file"<<endl;
        mergeRoadmaps(union_rmp, rmp);
        //maps.push_back(rmp);
      }
    }
    MPRegion<CfgType,WeightType> region(0, GetMPProblem());
    region.roadmap=union_rmp;
    //string outputFilename = GetMPProblem()->GetOutputRoadmap() + "RegionId" + str_index + ".map";
    ofstream  myofstream(outfile.c_str());

    if (!myofstream) {
      LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
      exit(-1);
    }
    region.WriteRoadmapForVizmo(myofstream);
    myofstream.close();
    //cout<<"printing all pairs"<<endl;
    printAllPairs(union_rmp, files);
    /*
    int sameCCPairUnion = compareAllPairs(union_rmp);
    cout<<"*****************************"<<endl;
    cout<<"all pairsconnectivity of union map = "<<sameCCPairUnion<<endl;
    //todo, place previously loaded roadmaps in a vector to avoid having to reload them
    
    for(vector<string>::iterator iter = files.begin(); iter!=files.end(); iter++){
      //char* filename = iter->c_str();
      cout<<"loading file name"<<iter->c_str()<<endl;
      Roadmap<CfgType,WeightType> rmp;
      rmp.ReadRoadmapGRAPHONLY(iter->c_str());
      int sameCCPair = compareAllPairs(rmp);
      cout<<"*****************************"<<endl;
      cout<<"map name = "<<iter->c_str();
      cout<<"all pairsconnectivity of map = "<<sameCCPair<<endl;
      cout<<"ratio to union pair = "<<sameCCPair/sameCCPairUnion<<endl;
      cout<<"difference compared to union map = "<<sameCCPairUnion-sameCCPair<<endl;
      cout<<"ratio of difference to connections in union pair = "<<(sameCCPairUnion-sameCCPair)/sameCCPairUnion<<endl;
    }
    */
    //for(vector<Roadmap<CfgType,WeightType> >::iterator iter = maps.begin(); iter!=maps.end(); iter++){
    //}
    exit(-1);

  }
    

  virtual void PrintOptions(ostream& out_os) { }
  virtual void operator()(int in_RegionID) { }

};


class NFRoadmapCompare : public MPStrategyMethod {
  private:
    vector<string> files;
    string outfile;
    int interval;
    int queries;
  public:
    typedef RoadmapGraph<CfgType, WeightType>::VID VID;
    
    NFRoadmapCompare(XMLNodeReader& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
      ParseXML(in_Node);
    }

    void ParseXML(XMLNodeReader& in_Node) {
      cout << "NFRoadmapCompare::ParseXML()" << endl;
      LOG_DEBUG_MSG("NFRoadmapCompare::ParseXML()");
      interval = 100;
      queries=2;
      outfile="outfile.stats";
      outfile = in_Node.stringXMLParameter(string("outfile"), true, string(""),string("Node Compare Method"));
     
      interval = in_Node.numberXMLParameter(string("interval"), true, 100, 1, 1000000, string("Same CC Printout Interval"));
      cout<<"interval="<<interval<<endl;

      XMLNodeReader::childiterator citr;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr){
        if(citr->getName() == "file"){
          string filename = citr->stringXMLParameter(string("filename"), true, string(""),string("Node Compare Method"));
          cout<<"filename="<<filename<<endl;
          files.push_back(filename);
 } else {
          citr->warnUnknownNode();
 }
      }
    }

    void trim(Roadmap<CfgType,WeightType>& rmp, int size){
      vector<VID> vertices;
      rmp.m_pRoadmap->GetVerticesVID(vertices);
      for(int i=size; i<vertices.size(); i++){
        rmp.m_pRoadmap->delete_vertex(vertices[i]);
      }
    }


    //gets mean distance, max epsilon, avg episilon
    vector<int> getCCSizes(Roadmap<CfgType,WeightType>& rmp){
      vector<int> ccSizes;
      vector< pair<size_t, VID> > ccstats;
      stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
      
      get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats);
      cout<<" "<<ccstats.size();
      for(vector< pair<size_t, VID> >::iterator iter = ccstats.begin(); iter < ccstats.end(); iter++){
        int size=iter->first;
        cout<<" "<<size;
        ccSizes.push_back(size);
      }
      
      return ccSizes;
    }

    double distance(Roadmap<CfgType,WeightType>& rmp,VID v1,VID v2){
      RoadmapGraph<CfgType,WeightType>* pMap = rmp.m_pRoadmap;
      //cout<<"getting cfg"<<endl;
      CfgType cfg1 = (*(pMap->find_vertex(v1))).property();
      CfgType cfg2 = (*(pMap->find_vertex(v2))).property();
      //cout<<"getting DMM"<<endl;
      //DistanceMetric* dmm=GetMPProblem()->GetDistanceMetric();
      //cout<<"getting Distance"<<endl;
      double dist=GetMPProblem()->GetDistanceMetric()->Distance(GetMPProblem()->GetEnvironment(),cfg1,cfg2);
      //cout<<"got Distance"<<endl;     
      return dist;
      //return dmm->Distance(rmp.GetEnvironment(), cfg1, cfg2);
    }

    double getMaxDist(Roadmap<CfgType,WeightType>& rmp, VID vertex, vector<VID> successors){
      double maxDis=0;
      for(vector<VID>::iterator iter = successors.begin(); iter < successors.end(); iter++){
 maxDis=max(maxDis,distance(rmp,vertex,*iter));
      }
      return maxDis;
    }

    double getTotalDist(Roadmap<CfgType,WeightType>& rmp, VID vertex, vector<VID> successors){
      double totalDis=0;
      for(vector<VID>::iterator iter = successors.begin(); iter < successors.end(); iter++){
 //cout<<"before dist"<<endl;
        totalDis+=distance(rmp,vertex,*iter);
        //cout<<"after dist"<<endl;
      }
      return totalDis;
    }

    //gets mean distance, max epsilon, avg episilon, rfd
    void printStats(Roadmap<CfgType,WeightType>& rmp, Roadmap<CfgType,WeightType>& base_rmp){
      //cout<<"printing stats"<<endl;
      vector<VID> vertices;
      rmp.m_pRoadmap->GetVerticesVID(vertices);
      vector<VID> vertices_base;
      base_rmp.m_pRoadmap->GetVerticesVID(vertices_base);
      //cout<<"got verticies"<<endl;
      double maxEpsilon=0;
      double avgEpsilon=0;
      double totalDist=0;
      double totalDist_base=0;
      int countTotal=0;
      int countTotal_base=0;
      int countEps=0;
      int totalFalseDismiss=0;
      int baseFalseDis=0;
      vector<VID>::iterator iter_base = vertices_base.begin();
      for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
 //cout<<"in loop"<<endl;
 //iter_base++;
        vector<VID> successors;
        rmp.m_pRoadmap->get_successors(*iter, successors); 
        vector<VID> successors_base;
        base_rmp.m_pRoadmap->get_successors(*iter_base, successors_base);
 //cout<<"before whiles"<<endl;
 int size_suc=successors.size();
        for(int i=0; i<size_suc; i++){
          VID vid=successors.back();
   successors.pop_back();
   if(vid>*iter)
     successors.insert(successors.begin(),vid);
   if(successors.size()==0)
     break;
 }
 size_suc=successors_base.size();
        for(int i=0; i<size_suc; i++){
   VID vid=successors_base.back();
          successors_base.pop_back();
          if(vid>*iter_base)
            successors_base.insert(successors_base.begin(),vid);
   if(successors_base.size()==0)
            break;
        }
        //cout<<"after whiles"<<endl;
        totalDist+=getTotalDist(rmp,*iter,successors);
        //cout<<"after get total dist"<<endl;
        totalDist_base+=getTotalDist(base_rmp,*iter_base,successors_base);
        //cout<<"after get total dist2"<<endl;
        countTotal+=successors.size();
        countTotal_base+=successors_base.size();
 //cout<<"after countTotal"<<endl;
        double maxDis=getMaxDist(rmp,*iter,successors);
        double maxDist_base=getMaxDist(base_rmp,*iter_base,successors_base);
        //Question: how to handle 0 values
        ///cout<<"before if"<<endl;
   //if(maxDis!=maxDist_base)
 //  cout<<"max dist"<<maxDis<<" "<<maxDist_base<<endl;
        if(maxDist_base!=0 && maxDis!=0){
          double epsilon=maxDis/maxDist_base;
          avgEpsilon+=epsilon;
          maxEpsilon=max(maxEpsilon,epsilon);
   countEps++;
 }
        //cout<<"before for"<<endl;
 //int totalFalseDismiss=0;
 for(vector<VID>::iterator succ_base_iter = successors_base.begin(); succ_base_iter!= successors_base.end(); succ_base_iter++) {
   //cout<<"in for"<<endl;
   if(*succ_base_iter<*iter_base){
     cout<<"***************error: not selecting edges correctly 1 ***************"<<endl;
   }
   int bFalseDismiss=1;
   for(vector<VID>::iterator succ_iter = successors.begin(); succ_iter!= successors.end(); succ_iter++) {
     if(*succ_iter < *iter){
       cout<<"***************error: not selecting edges correctly 2 ***************"<<endl;
     }
     if(*succ_base_iter == *succ_iter){
       bFalseDismiss=0;
     }
   }
   baseFalseDis++;
   totalFalseDismiss+=bFalseDismiss;
 }
 iter_base++;
      }
      //cout<<"after for"<<endl;
      //cout<<"here"<<totalDist<<" "<<countTotal<<" "<<totalDist_base<<" "<<countTotal_base<<endl;
      double ratioMeanDist=(totalDist/countTotal)/(totalDist_base/countTotal_base);
      avgEpsilon/=countEps;
      double rfd=(double)totalFalseDismiss/(double)baseFalseDis;
      cout<<ratioMeanDist<<" "<<avgEpsilon<<" "<<maxEpsilon<<" "<<rfd<<" ";
    }

    void operator()(){
      cout<<"*************in operator()***********************"<<endl;
      if(files.size()==0){
        cout<<"no files"<<endl;
      }
      Roadmap<CfgType,WeightType> base_rmp;
      for(vector<string>::iterator iter = files.begin(); iter!=files.end(); iter++){
 cout<<"loading "<<iter->c_str()<<endl;
        if(iter == files.begin()){
          base_rmp.ReadRoadmapGRAPHONLY(iter->c_str());
        }else{
     Roadmap<CfgType,WeightType> rmp;
          rmp.ReadRoadmapGRAPHONLY(iter->c_str());
          trim(rmp, base_rmp.m_pRoadmap->get_num_vertices());
          //trim(base_rmp, rmp.m_pRoadmap->GetNumVerts());
          //compareRoadmaps(base_rmp, rmp);
   //cout<<"getting stats"<<endl;
   //getCCSizes(rmp);
   cout<<endl;
   printStats(rmp, base_rmp);
   getCCSizes(rmp);
   cout<<endl;
        }
 cout<<endl;
      }
      
    }
    

    virtual void PrintOptions(ostream& out_os) { }
    virtual void operator()(int in_RegionID) { }
};
  

 
class NFIncrementalRoadmap : public MPStrategyMethod {
  public:
   typedef RoadmapGraph<CfgType, WeightType>::VID VID;   
    
  NFIncrementalRoadmap(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    LOG_DEBUG_MSG("NFIncrementalRoadmap::NFIncrementalRoadmap()");
    ParseXML(in_Node);    
    LOG_DEBUG_MSG("~NFIncrementalRoadmap::NFIncrementalRoadmap()");
    };
  virtual ~NFIncrementalRoadmap() {}
    
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(XMLNodeReader& in_Node) {
    cout << "NFIncrementalRoadmap::ParseXML()" << endl;
    LOG_DEBUG_MSG("NFIncrementalRoadmap::ParseXML()");
    //OBPRM_srand(getSeed());
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(citr->getName() == "node_generation_method") {
        string node_gen_method = citr->stringXMLParameter(string("Method"),true,
                                   string(""),string("Node Generation Method"));
        m_vecStrNodeGenLabels.push_back(node_gen_method);
        citr->warnUnrequestedAttributes();
      } else if(citr->getName() == "node_connection_method") {
        string connect_method = citr->stringXMLParameter(string("Method"),true,
                                   string(""),string("Node Connection Method"));
        m_vecStrNodeConnectionLabels.push_back(connect_method);
        citr->warnUnrequestedAttributes();
      } else if(citr->getName() == "component_connection_method") {
        string connect_method = citr->stringXMLParameter(string("Method"),true,
                                   string(""),string("CC Connection Method"));
        m_vecStrComponentConnectionLabels.push_back(connect_method);
        citr->warnUnrequestedAttributes();
      } else if(citr->getName() == "lp_method") {
        m_strLocalPlannerLabel = citr->stringXMLParameter(string("Method"),true,
                                   string(""),string("Local Planner Method"));
        citr->warnUnrequestedAttributes();
      } else if (citr->getName() == "step_size") {
        m_stepSize = citr->numberXMLParameter(string("step_size"), true,
                                              int(100),int(0),int(MAX_INT), 
                                              string("Iteration step size")); 
      } else if(citr->getName() == "NeighborhoodFinder") {
        string nf_method = citr->stringXMLParameter(string("Method"),true,
                                   string(""),string("NeighborhoodFinder Method"));
        m_NF = GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(nf_method);
        citr->warnUnrequestedAttributes();
      } else {
        citr->warnUnknownNode();
      }
    }
    
    m_queryFilename = in_Node.stringXMLParameter("query_filename", true, "", "Query Filename");
    m_nfStats = in_Node.stringXMLParameter("nf_stat", true, "", "NF for stat output");
    
    //--------------------------
    //Reading in witness queries
    //--------------------------
    CfgType tempCfg;
    ifstream  myifstream(m_queryFilename.c_str());
    if (!myifstream) {
      cout << endl << "In NFIncrementalRoadmap: can't open witness file: " << m_queryFilename;
      exit(-1);
    }
    while (1) {
      tempCfg.Read(myifstream);
      if(!myifstream) break;
      m_vecWitnessNodes.push_back(tempCfg);
    }
    myifstream.close();
    
    
    LOG_DEBUG_MSG("~NFIncrementalRoadmap::ParseXML()");
    cout << "leaving NFIncrementalRoadmap" << endl;
  };
   
  virtual void operator()(int in_RegionID) {
    cout << "NFIncrementalRoadmap::()" << endl;
    LOG_DEBUG_MSG("NFIncrementalRoadmap::()");

    OBPRM_srand(getSeed()); 
    MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
    Stat_Class * pStatClass = region->GetStatClass();
    
    ///\todo why this is here?
    //if (m_reset_stats)
      pStatClass->ClearStats();
  
    Clock_Class Allstuff;
    //string base_filename = "itr_test_";
    
    //open output file for stats
    //filename env_ng_con_nf_seed.stats
    stringstream basefname;
    basefname << getBaseFilename();// << "-" << getSeed();
    ofstream stat_out((basefname.str() + ".stats").c_str());
    
    ofstream map_out((basefname.str() + ".map").c_str());
    //write output file format
    stat_out << "#num_nodes \t num_edges \t lp_attempts \t lp_succ \t lp_cd  \t ng_time \t ng_cd \t con_time"
             << "\t nf_qry_time \t nf_const_time"
             << "\t num_ccs \t max_cc_size \t min_cc_size \t solve_qry \t min_edge_len \t max_edge_len \t ave_edge_len \t std_edge_len"
             << "\t min_degree \t max_degree \t ave_degree \t std_degree \t approx_dia"
             << endl;


    Allstuff.StartClock("Everything");
    
    double elappsed_ng(0.0), ellapsed_con(0.0);
    bool querySucceeded = false;
    bool queryFirstSucceeded = false;
    int iteration = 0;
    int nodes_added = 0;
    
    //for (int step = 0; step < m_iterations; step++)
    region->GetRoadmap()->m_pRoadmap->AddVertex(m_vecWitnessNodes[0]);
    region->GetRoadmap()->m_pRoadmap->AddVertex(m_vecWitnessNodes[1]);
    while(iteration < m_iterations) {
      cout << "Iteration #" << iteration << " of " << m_iterations << endl;
      Clock_Class        NodeGenClock;
      Clock_Class        ConnectionClock;
      //---------------------------
      // Generate roadmap nodes
      //---------------------------
      cout << "GENERATE ROADMAP NODES" << endl;
      
      NodeGenClock.StartClock("Node Generation");
      vector<VID> newVids;
      typedef vector<string>::iterator I;
      for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
      {
        vector< CfgType > vectorCfgs;
        Sampler<CfgType>::SamplerPointer pNodeGen;
        pNodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(*itr);
        pNodeGen->GetSampler()->Sample(pNodeGen, GetMPProblem()->GetEnvironment(), *pStatClass, num_nodes, 2*num_nodes, back_inserter(vectorCfgs));  
        
        cout << "Finished ... I did this many : " << vectorCfgs.size();
        vector<VID> vids =  region->AddToRoadmap(vectorCfgs);
        nodes_added += vids.size();
        cout << " - total VIDS: " << nodes_added << endl;
        for(int i=0; i<vids.size(); ++i) {
          newVids.push_back(vids[i]);
        }
      }
        
      NodeGenClock.StopClock();
      elappsed_ng += NodeGenClock.GetClock_SEC();
      
      //---------------------------
      // Connect roadmap nodes
      //---------------------------
      cout << "CONNECT ROADMAP NODES" << endl;
      
      ConnectionClock.StartClock("Node Connection");
      ConnectMap<CfgType, WeightType>* connectmap = GetMPProblem()->GetMPStrategy()->GetConnectMap();
      typedef vector<string>::iterator J;
      for(J itr = m_vecStrNodeConnectionLabels.begin(); 
          itr != m_vecStrNodeConnectionLabels.end(); ++itr)
      {
        LOG_DEBUG_MSG("NFIncrementalRoadmap:: " << *itr);
        
        ConnectMap<CfgType,WeightType>::NodeConnectionPointer pConnection;
        pConnection = connectmap->GetNodeMethod(*itr);
        cout << "Calling connection method:: " << pConnection->GetLabel() << endl;
        connectmap->ConnectNodes(pConnection, region->GetRoadmap(), *pStatClass, 
                             GetMPProblem()->GetDistanceMetric(), 
                             GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                             GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                             GetMPProblem()->GetMPStrategy()->addAllEdges,
                             newVids.begin(), newVids.end());
      }
        
      ConnectionClock.StopClock();
      ellapsed_con += ConnectionClock.GetClock_SEC();
   
      /*string outputFilename = getBaseFilename() + ".map"; 
      ofstream  myofstream(outputFilename.c_str());
    
      if (!myofstream) {
        LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
        exit(-1);
      }
      region->WriteRoadmapForVizmo(myofstream);
      myofstream.close();
      */
      
      
      
      //pStatClass->PrintAllStats(region->GetRoadmap());
      //NodeGenClock.PrintClock();
      ConnectionClock.PrintClock();
      //Allstuff.StopPrintClock();
      
      
      //////////
      // query
      //////////
      /*Query<CfgType, WeightType> query;
     
      int oriVertID = region->GetRoadmap()->m_pRoadmap->getVertIDs(); 
      VID vid_start = region->GetRoadmap()->m_pRoadmap->AddVertex(m_vecWitnessNodes[0]);
      VID vid_goal = region->GetRoadmap()->m_pRoadmap->AddVertex(m_vecWitnessNodes[1]);
      
      vector<CfgType> result_path;
      Stat_Class query_stats;
      string bfnf("queryconnect");
      bool querySucceeded = query.PerformQuery(m_vecWitnessNodes[0], m_vecWitnessNodes[1],
              region->GetRoadmap(),
              query_stats,
              GetMPProblem()->GetCollisionDetection(),
              connectmap, 
              connectmap->GetNodeMethod(bfnf),
              GetMPProblem()->GetMPStrategy()->GetLocalPlanners(), 
              GetMPProblem()->GetDistanceMetric(), 
              &result_path);
      
      if (querySucceeded)
        cout << "The query succeeded!" << endl;
      else
        cout << "The query failed..." << endl;
      
      region->GetRoadmap()->m_pRoadmap->DeleteVertex(vid_start);
      region->GetRoadmap()->m_pRoadmap->DeleteVertex(vid_goal);
      region->GetRoadmap()->m_pRoadmap->setVertIDs(oriVertID);
      */
      stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
      querySucceeded = is_same_cc(*region->GetRoadmap()->m_pRoadmap, cmap, 0, 1);
      //CanSolveQuery(*region->GetRoadmap()->m_pRoadmap, m_vecWitnessNodes[0], m_vecWitnessNodes[1]);

      ///////////////////
      //Output stat info
      NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
      OnlineStats degree = calcDegreeStats(*region->GetRoadmap()->m_pRoadmap);
      OnlineStats edges = calcEdgeStats(*region->GetRoadmap()->m_pRoadmap);
      vector<pair<size_t, VID> > CCStats;
      get_cc_stats (*region->GetRoadmap()->m_pRoadmap, cmap, CCStats);
      // run dia twice, start from largest component
      VID far_vid(-1), far_vid2(-1); 
      ComponentDiameter(*region->GetRoadmap()->m_pRoadmap,CCStats[0].second, &far_vid);
      double diameter = ComponentDiameter(*region->GetRoadmap()->m_pRoadmap, far_vid,&far_vid2);
      stat_out << region->GetRoadmap()->m_pRoadmap->get_num_vertices() - 2
        << "\t" << region->GetRoadmap()->m_pRoadmap->get_num_edges() / 2
        << "\t" << pStatClass->Connections_Attempted << "\t" << double(pStatClass->Connections_Made) / double(pStatClass->Connections_Attempted)
        << "\t" << pStatClass->IsCollByName["straightline-straightline::IsConnectedSLBinary"]
        << "\t" << elappsed_ng << "\t" <<  pStatClass->IsCollTotal -
                                pStatClass->IsCollByName["straightline-straightline::IsConnectedSLBinary"]
        << "\t" << ellapsed_con
        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetQueryTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()- 2)
        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetConstructionTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
        << "\t" << CCStats.size() << "\t" << double(CCStats[0].first) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
        << "\t" << double(CCStats[CCStats.size()-1].first) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2) 
        << "\t" << querySucceeded 
        << "\t" << edges.GetMin() << "\t" << edges.GetMax() << "\t" << edges.GetMean() 
        << "\t" << edges.GetStandardDeviation() << "\t" << degree.GetMin() << "\t" << degree.GetMax() 
        << "\t" << degree.GetMean() << "\t" << degree.GetStandardDeviation() 
        << "\t" << diameter
        << endl;


       
      ///////////////////
      // Output Total info
      if(querySucceeded && !queryFirstSucceeded) {
        ofstream total_out((basefname.str() + ".total").c_str());
        total_out << region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2 
        << "\t" << region->GetRoadmap()->m_pRoadmap->get_num_edges() / 2
        << "\t" << pStatClass->Connections_Attempted << "\t" << double(pStatClass->Connections_Made) / double(pStatClass->Connections_Attempted)
        << "\t" << pStatClass->IsCollByName["straightline-straightline::IsConnectedSLBinary"]
        << "\t" << elappsed_ng << "\t" <<  pStatClass->IsCollTotal -
                                pStatClass->IsCollByName["straightline-straightline::IsConnectedSLBinary"]
        << "\t" << ellapsed_con
        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetQueryTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetConstructionTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
        << "\t" << CCStats.size() << "\t" << double(CCStats[0].first) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
        << "\t" << double(CCStats[CCStats.size()-1].first) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2) 
        << "\t" << querySucceeded 
        << "\t" << edges.GetMin() << "\t" << edges.GetMax() << "\t" << edges.GetMean() 
        << "\t" << edges.GetStandardDeviation() << "\t" << degree.GetMin() << "\t" << degree.GetMax() 
        << "\t" << degree.GetMean() << "\t" << degree.GetStandardDeviation() 
        << "\t" << diameter
        << endl;

        if (!map_out) {
          LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
          exit(-1);
        }

        queryFirstSucceeded = true;
        cout << "Solved query! " << endl;
        //return;
        
      }
      
      iteration++;
    }
    
    cout << "Finished map " << endl;
    LOG_DEBUG_MSG("~NFIncrementalRoadmap::()");
    
    region->WriteRoadmapForVizmo(map_out);
    map_out.close();
  }
  
  virtual void operator()() {
    int newRegionId = GetMPProblem()->CreateMPRegion();
    (*this)(newRegionId);      
  };

private:

  OnlineStats calcDegreeStats(RoadmapGraph<CfgType,WeightType>& _graph) {
    OnlineStats to_return;
    RoadmapGraph<CfgType,WeightType>::VI vitr;
    for(vitr =_graph.begin(); vitr != _graph.end(); ++vitr) {
      to_return.AddData(_graph.get_out_degree((*vitr).descriptor()));
    }
    return to_return;
  };


  OnlineStats calcEdgeStats(RoadmapGraph<CfgType,WeightType>& _graph) {
    OnlineStats to_return;
    RoadmapGraph<CfgType,WeightType>::VI vitr;
    RoadmapGraph<CfgType,WeightType>::EI eitr;
    
    
    for(vitr = _graph.begin(); vitr != _graph.end(); ++vitr) {
      for (eitr = (*vitr).begin(); eitr != (*vitr).end(); ++eitr) {
        to_return.AddData((*(eitr)).property().GetWeight());
      }
    }
    return to_return;
  };


  bool CanConnectToComponent(RoadmapGraph<CfgType,WeightType>& _graph, VID _cc, CfgType _test) {
    vector<VID> vec_cc;
    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    get_cc(_graph, cmap, _cc, vec_cc);

    vector<pair<double,VID> > vec_dist_vid;
    vec_dist_vid.reserve(vec_cc.size());
    
    for(int i=0; i<vec_cc.size(); ++i) {
      double dist = GetMPProblem()->GetDistanceMetric()->Distance(GetMPProblem()->GetEnvironment(),
                              _test, (*(_graph.find_vertex(vec_cc[i]))).property());
      vec_dist_vid.push_back(make_pair(dist, vec_cc[i]));
    }
  
    sort(vec_dist_vid.begin(), vec_dist_vid.end());
    Stat_Class _mystat;
    LPOutput<CfgType,WeightType> out_lp_output;
    for(int i=0; i<vec_dist_vid.size(); ++i) {
      if(GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->
              IsConnected(GetMPProblem()->GetEnvironment(), _mystat, 
              GetMPProblem()->GetDistanceMetric(), 
              _test, (*(_graph.find_vertex(vec_dist_vid[i].second))).property(),  &out_lp_output, 
              GetMPProblem()->GetEnvironment()->GetPositionRes(), 
              GetMPProblem()->GetEnvironment()->GetOrientationRes(),
              true, false, false)) {
        return true;
      }
    }
    return false;
  }

  bool CanSolveQuery(RoadmapGraph<CfgType,WeightType>& _graph, CfgType _start, CfgType _goal) {
    vector<pair<size_t, VID> > CCStats;
    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    get_cc_stats(_graph, cmap, CCStats);
    for(int i=0; i<CCStats.size(); ++i) {
      if(CanConnectToComponent(_graph, CCStats[i].second, _start) 
          && CanConnectToComponent(_graph, CCStats[i].second, _goal)) {
        return true;
      }
    }
    return false;
  }

  vector<string> m_vecStrNodeGenLabels;
  vector<string> m_vecStrNodeConnectionLabels;
  vector<string> m_vecStrComponentConnectionLabels;
  vector<string> m_vecNodeCharacterizerLabels;
  NeighborhoodFinder::NeighborhoodFinderPointer m_NF;
  string m_strLocalPlannerLabel;
  string m_nfStats;
  vector<CfgType> m_vecWitnessNodes;
  string m_queryFilename;
  int m_stepSize;
   
};


class NFTester : public MPStrategyMethod {
  public:
  typedef RoadmapGraph<CfgType, WeightType>::VID VID;
  
  NFTester( XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPStrategyMethod(in_Node,in_pProblem) {
    LOG_DEBUG_MSG("NFTester::NFTester()");
    ParseXML(in_Node);    
    LOG_DEBUG_MSG("~NFTester::NFTester()");
    };
  virtual ~NFTester() {}
    
  virtual void PrintOptions(ostream& out_os) { };
  
  virtual void ParseXML(XMLNodeReader& in_Node) {
    LOG_DEBUG_MSG("NFTester::ParseXML()");
    //OBPRM_srand(getSeed());
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(citr->getName() == "node_generation_method") {
        string node_gen_method = citr->stringXMLParameter(string("Method"),true,
                                   string(""),string("Node Generation Method"));
        m_vecStrNodeGenLabels.push_back(node_gen_method);
        citr->warnUnrequestedAttributes();
      } else if(citr->getName() == "NeighborhoodFinder") {
        string nf_method = citr->stringXMLParameter(string("Method"),true,
                                   string(""),string("NeighborhoodFinder Method"));
        m_vecNF.push_back(GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(nf_method));
        citr->warnUnrequestedAttributes();
      } else {
        citr->warnUnknownNode();
      }
    }
    
   //get baseline nf method
   string nf_method = in_Node.stringXMLParameter(string("BaselineNFMethod"),true,
                                   string(""),string("BaselineNeighborhoodFinder Method"));
    m_BaselineNF = GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(nf_method);
    m_kclosest = in_Node.numberXMLParameter("kclosest", true, int(10), int(0), int(MAX_INT), 
                                            "Number of K-Closest");
    LOG_DEBUG_MSG("~NFTester::ParseXML()");
  };
   
    virtual void operator()(int in_RegionID) {
      LOG_DEBUG_MSG("NFTester::()");
      OBPRM_srand(getSeed()); 
      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
     NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
      Stat_Class * pStatClass = region->GetStatClass();
      pStatClass->ClearStats(); 
     Clock_Class NodeGenClock;
      //---------------------------
      // Generate roadmap nodes
      //---------------------------
      NodeGenClock.StartClock("Node Generation");
      
      typedef vector<string>::iterator I;
      for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
      {
        vector< CfgType > vectorCfgs;
        Sampler<CfgType>::SamplerPointer pNodeGen;
        pNodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(*itr);
        pNodeGen->GetSampler()->Sample(pNodeGen, GetMPProblem()->GetEnvironment(), *pStatClass, num_nodes, 2*num_nodes, back_inserter(vectorCfgs));  
        
        cout << "Finished ... I did this many : " << vectorCfgs.size() << endl;
        //for (int i = 0; i < vectorCfgs.size(); i++) {
        //  
        //}
        region->AddToRoadmap(vectorCfgs);
      }

      NodeGenClock.StopClock();
     vector< VID > roadmap_vids;
     region->GetRoadmap()->m_pRoadmap->GetVerticesVID(roadmap_vids);
     cout << "Finished ... I did this many : " << roadmap_vids.size() << endl; 
     for (int i = 0; i < roadmap_vids.size(); i++) {
        cout << "\t" << roadmap_vids[i] << endl;
      }
      //int kclosest = 10;
      VID query_point;
     vector<VID> baseline_closest(m_kclosest);

      for(vector<VID>::iterator iter = roadmap_vids.begin(); iter != roadmap_vids.end(); ++iter){
      query_point = *iter;
      cout << "Query point = VID: " << query_point << endl;
     cout << "CALLING:: Baseline NF-Method = " << m_BaselineNF->GetObjectLabel() << endl;

      nf->KClosest(m_BaselineNF, region->GetRoadmap(), 
                     //roadmap_vids.begin(), roadmap_vids.end(), 
                     query_point,
                     m_kclosest, baseline_closest.begin());

  //    cout << "TIME ELAPSED FOR cur_total_time " << m_BaselineNF->GetLabel() << " : " << m_BaselineNF->GetTotalTime() << " CALCULATED BY NEW TIMER " << endl;
  //    cout << "TIME ELAPSED FOR cur_query_time " << m_BaselineNF->GetLabel() << " : " << m_BaselineNF->GetQueryTime() << " CALCULATED BY NEW TIMER " << endl;
  //    cout << "The Number of Queries are " << m_BaselineNF->GetNumQueries() << endl;

      vector<NeighborhoodFinder::NeighborhoodFinderPointer>::iterator nfitr;
     //output k-closest VIDs for baseline
  //   for(int i=0; i<baseline_closest.size(); ++i)
  //     { cout << baseline_closest[i] << " "; }
  //   cout << endl;


     for(nfitr = m_vecNF.begin(); nfitr != m_vecNF.end(); ++nfitr) {
       vector<VID> nf_vids(m_kclosest);
  //     cout << "CALLING:: NF-method = " << (*nfitr)->GetLabel() << endl;
        //start

       nf->KClosest(*nfitr, region->GetRoadmap(), 
                     //roadmap_vids.begin(), roadmap_vids.end(), 
                     query_point,
                     m_kclosest, nf_vids.begin());

  //      cout << "TIME ELAPSED FOR cur_total_time " << (*nfitr)->GetLabel() << " : " << (*nfitr)->GetTotalTime() << " CALCULATED BY NEW TIMER " << endl;
  //      cout << "TIME ELAPSED FOR cur_query_time " << (*nfitr)->GetLabel() << " : " << (*nfitr)->GetQueryTime() << " CALCULATED BY NEW TIMER " << endl;
  //      cout << "The Number of Queries are " << (*nfitr)->GetNumQueries() << endl;

        //end

       //output k-closest VIDs for this nf method
  //!      for(int i=0; i<nf_vids.size(); ++i)
  //!      { cout << nf_vids[i] << " "; }
  //!      cout << endl;

       //add code to compare the VIDs in baseline_closest & nf_vids
       //"Name" of the method is (*nfitr)->GetLabel()
       // to use the NeighborhoodFinder use pointer "nf" nf->RFD(....)
        // query point = *(roadmap_vids.begin())

        double ede = nf->EDE(region->GetRoadmap(),
                                            query_point,baseline_closest.begin(), baseline_closest.end(),
                                            nf_vids.begin(), nf_vids.end(), 
                                            GetMPProblem()->GetDistanceMetric()->GetDefault()[0]);
  //      cout << "EDE between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << ede << endl;



        double rde = nf->RDE(region->GetRoadmap(),
                                            query_point, baseline_closest.begin(), baseline_closest.end(),
                                            nf_vids.begin(), nf_vids.end(), 
                                            GetMPProblem()->GetDistanceMetric()->GetDefault()[0]);
  //      cout << "RDE between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << rde << endl;


        double rfd = nf->RFD(region->GetRoadmap(),
                                            query_point, baseline_closest.begin(), baseline_closest.end(),
                                            nf_vids.begin(), nf_vids.end(), 
                                            GetMPProblem()->GetDistanceMetric()->GetDefault()[0], 0.001);
  //      cout << "RFD between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << rfd << endl;

        double oeps = nf->OEPS(region->GetRoadmap(),
                                            query_point, baseline_closest.begin(), baseline_closest.end(),
                                            nf_vids.begin(), nf_vids.end(),
                                            GetMPProblem()->GetDistanceMetric()->GetDefault()[0]);      
  //      cout << "OEPS between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << oeps << endl;

        m_map_nfp_RDE[(*nfitr)].AddData(rde);
        m_map_nfp_RFD[(*nfitr)].AddData(rfd);
        m_map_nfp_EDE[(*nfitr)].AddData(ede);  
        m_map_nfp_OEPS[(*nfitr)].AddData(oeps);
     }

    }// loop query point 

    /*cout << "The Number of Queries are " << m_BaselineNF->GetNumQueries() << endl;

    cout << "Total Time Elapsed for " << m_BaselineNF->GetLabel() << " is: " << m_BaselineNF->GetTotalTime() << endl;
    cout << "Total Query Time Elapsed for " << m_BaselineNF->GetLabel() << " is: " << m_BaselineNF->GetQueryTime() << endl;

    cout << "Average Time Elapsed for " << m_BaselineNF->GetLabel() << " is: " << (m_BaselineNF->GetTotalTime())/roadmap_vids.size() << endl;
    cout << "Average Query Time Elapsed for " << m_BaselineNF->GetLabel() << " is: " << (m_BaselineNF->GetQueryTime())/roadmap_vids.size() << endl;
  */
    vector<NeighborhoodFinder::NeighborhoodFinderPointer>::iterator nfitr;
    for(nfitr = m_vecNF.begin(); nfitr != m_vecNF.end(); ++nfitr) {

    /*cout << "Total Time Elapsed for " << (*nfitr)->GetLabel() << " is: " << (*nfitr)->GetTotalTime() << endl;
    cout << "Total Query Time Elapsed for " << (*nfitr)->GetLabel() << " is: " << (*nfitr)->GetQueryTime() << endl;   

    cout << "Average Time Elapsed for " << (*nfitr)->GetLabel() << " is: " << ((*nfitr)->GetTotalTime())/roadmap_vids.size() << endl;
    cout << "Average Query Time Elapsed for " << (*nfitr)->GetLabel() << " is: " << ((*nfitr)->GetQueryTime())/roadmap_vids.size() << endl;   

    //cout << "Total EDE between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << m_map_nfp_EDE[(*nfitr)] << endl;
    //cout << "Total RDE between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << m_map_nfp_RDE[(*nfitr)] << endl;
    //cout << "Total RFD between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << m_map_nfp_RFD[(*nfitr)] << endl;
    */
    /*cout << "Average EDE between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << m_map_nfp_EDE[(*nfitr)]/roadmap_vids.size() << endl;
    cout << "Average RDE between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << m_map_nfp_RDE[(*nfitr)]/roadmap_vids.size() << endl;
    cout << "Average RFD between " << (*nfitr)->GetLabel() << " & " << m_BaselineNF->GetLabel() << " = " << m_map_nfp_RFD[(*nfitr)]/roadmap_vids.size() << endl;   
    */
    cout << "Statistics for Neighborhood Finder Test" << endl;
    cout << "Number of Nodes: " << roadmap_vids.size() << endl;
    cout << "Number of Queries: " << m_BaselineNF->GetNumQueries() << endl;
    cout << "KClosest: " << m_kclosest << endl;
    cout << "DistnaceMetric: "; GetMPProblem()->GetDistanceMetric()->GetDefault()[0]->PrintOptions(cout);
    cout << "Baseline NF: "; m_BaselineNF->PrintOptions(cout);
    cout << "Test NF: "; (*nfitr)->PrintOptions(cout);
  /*  cout << "#AveBaseTotalTime, AveBaseQryTime, AveNFTotalTime, AveNFQryTime, MinEDE, MaxEDE, AveEDE, StdEDE, MinRDE, MaxRDE, "
         << "AveRDE, StdRDE, MinRFD, MaxRFD, AveRFD, StdRFD" << endl
         << (m_BaselineNF->GetTotalTime())/m_BaselineNF->GetNumQueries() << ", "
         << (m_BaselineNF->GetQueryTime())/m_BaselineNF->GetNumQueries() << ", "
         << ((*nfitr)->GetTotalTime())/m_BaselineNF->GetNumQueries() << ", "
         << ((*nfitr)->GetQueryTime())/m_BaselineNF->GetNumQueries() << ", "
  */       
    cout << "#AveBaseTotalTime, AveBaseQryTime, AveBaseConTime, AveNFTotalTime, AveNFQryTime, AveNFConTime, BaseTotalTime, BaseQryTime, BaseConTime, NFTotalTime, NFQryTime, NFConTime, MinEDE, MaxEDE, AveEDE, StdEDE, MinRDE, MaxRDE, "
         << "AveRDE, StdRDE, MinRFD, MaxRFD, AveRFD, StdRFD, MinOEPS, MaxOEPS, AveOEPS, StdOEPS" << endl
         << (m_BaselineNF->GetTotalTime())/(m_BaselineNF->GetNumQueries() * m_kclosest )<< ", "
         << (m_BaselineNF->GetQueryTime())/(m_BaselineNF->GetNumQueries() *m_kclosest )<< ", "
         << (m_BaselineNF->GetConstructionTime())/(m_BaselineNF->GetNumQueries() *m_kclosest )<< ", "
         << ((*nfitr)->GetTotalTime())/(m_BaselineNF->GetNumQueries() * m_kclosest )<< ", "
         << ((*nfitr)->GetQueryTime())/(m_BaselineNF->GetNumQueries() * m_kclosest )<< ", "       
         << ((*nfitr)->GetConstructionTime())/(m_BaselineNF->GetNumQueries() * m_kclosest )<< ", "       
         << (m_BaselineNF->GetTotalTime()) << ", "
         << (m_BaselineNF->GetQueryTime()) << ", "
         << (m_BaselineNF->GetConstructionTime()) << ", "
         << ((*nfitr)->GetTotalTime()) << ", "
         << ((*nfitr)->GetQueryTime()) << ", "
         << ((*nfitr)->GetConstructionTime()) << ", "

         << m_map_nfp_EDE[(*nfitr)].GetMin() << ", "
         << m_map_nfp_EDE[(*nfitr)].GetMax() << ", "
         << m_map_nfp_EDE[(*nfitr)].GetMean() << ", "
         << m_map_nfp_EDE[(*nfitr)].GetStandardDeviation() << ", "

         << m_map_nfp_RDE[(*nfitr)].GetMin() << ", "
         << m_map_nfp_RDE[(*nfitr)].GetMax() << ", "
         << m_map_nfp_RDE[(*nfitr)].GetMean() << ", "
         << m_map_nfp_RDE[(*nfitr)].GetStandardDeviation() << ", "

         << m_map_nfp_RFD[(*nfitr)].GetMin() << ", "
         << m_map_nfp_RFD[(*nfitr)].GetMax() << ", "
         << m_map_nfp_RFD[(*nfitr)].GetMean() << ", "
         << m_map_nfp_RFD[(*nfitr)].GetStandardDeviation() << ", " 

         << m_map_nfp_OEPS[(*nfitr)].GetMin() << ", "
         << m_map_nfp_OEPS[(*nfitr)].GetMax() << ", "
         << m_map_nfp_OEPS[(*nfitr)].GetMean() << ", "
         << m_map_nfp_OEPS[(*nfitr)].GetStandardDeviation() << ", " 
         << endl;

    }

    cout << "Finished!!" << endl;
     LOG_DEBUG_MSG("~NFTester::()");
    };
  
  virtual void operator()() {
    int newRegionId = GetMPProblem()->CreateMPRegion();
    (*this)(newRegionId);      
  };

private:
  int m_kclosest;
  vector<string> m_vecStrNodeGenLabels;
  vector<NeighborhoodFinder::NeighborhoodFinderPointer> m_vecNF;
  NeighborhoodFinder::NeighborhoodFinderPointer m_BaselineNF;
  map<NeighborhoodFinder::NeighborhoodFinderPointer,OnlineStats> m_map_nfp_RDE, m_map_nfp_RFD, m_map_nfp_EDE, m_map_nfp_OEPS;
};


#endif
