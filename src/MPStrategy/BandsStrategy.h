#ifndef BandsStrategy
#define BandsStrategy

//#include<sys/time.h>

#include "MPStrategyMethod.h"
#include "ExpanderStats.h"
#include "MPProblem.h"
#include "Roadmap.h"
#include "MetricUtils.h"
#include "ConnectMap.h"
#include "DistanceMetricMethod.h"
#include "LocalPlanners.h"


//extern vector< vector< double > > g_Min, g_Max;
/*
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
 */

/*
   struct unionStats{
   int n;
   int sameCCPair;
   int differenceInSameCC;
   double ratioToUnion;
   int sameCCSizes[5];
   };
 */

template <class T>
struct __CCVID_Compare : public std::binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.first > y.first; }
};

class BIRContainer : public MPSMContainer {
  public:
    BIRContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
    vector<string> m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_vecStrComponentConnectionLabels;
    vector<string> m_vecNodeCharacterizerLabels;
    NeighborhoodFinder::NeighborhoodFinderPointer m_NF;
    string m_strLocalPlannerLabel;
    //  string m_nfStats;
    vector<CfgType> m_vecWitnessNodes;
    string m_queryFilename;
    int m_stepSize;
    double m_posRes;
    int m_RegionNdx;
    bool resize_bbox;
    shared_ptr<DistanceMetricMethod> dm;
    MPSMContainer parent;
    int m_numNodes;
    int m_iterations;
};


class BandsIncrementalRoadmap : public MPStrategyMethod {
  public:
    typedef RoadmapGraph<CfgType, WeightType>::VID VID;   
    BandsIncrementalRoadmap(BIRContainer cont) : MPStrategyMethod(cont.parent) {
      m_vecStrNodeGenLabels = cont.m_vecStrNodeGenLabels;
      m_vecStrNodeConnectionLabels = cont.m_vecStrNodeConnectionLabels;
      m_vecStrComponentConnectionLabels = cont.m_vecStrComponentConnectionLabels;
      m_vecNodeCharacterizerLabels = cont.m_vecNodeCharacterizerLabels;
      m_NF = cont.m_NF;
      m_strLocalPlannerLabel = cont.m_strLocalPlannerLabel;
      //  string m_nfStats;
      m_vecWitnessNodes = cont.m_vecWitnessNodes;
      m_queryFilename = cont.m_queryFilename;
      m_stepSize = cont.m_stepSize;
      m_posRes = cont.m_posRes;
      m_RegionNdx = cont.m_RegionNdx;
      resize_bbox = cont.resize_bbox;
      dm = cont.dm;

    }  
    BandsIncrementalRoadmap(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
      MPStrategyMethod(in_Node,in_pProblem) {
        ParseXML(in_Node);    
      };
    virtual ~BandsIncrementalRoadmap() {}

    virtual void PrintOptions(ostream& out_os) { };

    virtual void ParseXML(XMLNodeReader& in_Node) {
      cout << "BandsIncrementalRoadmap::ParseXML()" << endl;
      //SRand(getSeed());
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
      m_posRes = in_Node.numberXMLParameter(string("pos_res"), false, 0.0, 0.0, 10.0, 
          string("positional resolution"));
      //    m_nfStats = in_Node.stringXMLParameter("nf_stat", true, "", "NF for stat output");
      resize_bbox = in_Node.boolXMLParameter(string("resize_bbox"), false, false,
          string("if true, bounding box size will be doubled"));

      m_numNodes = in_Node.numberXMLParameter(string("num_samples"), true, 1,0,MAX_INT, "Number of Samples");
      m_iterations = in_Node.numberXMLParameter("iterations", true, 1,0,MAX_INT, "Number of Iterations");

      //--------------------------
      //Reading in witness queries
      //--------------------------
      CfgType tempCfg;
      ifstream  myifstream(m_queryFilename.c_str());
      if (!myifstream) {
        cout << endl << "In BandsIncrementalRoadmap: can't open witness file: " << m_queryFilename;
        exit(-1);
      }
      while (1) {
        tempCfg.Read(myifstream);
        if(!myifstream) break;
        m_vecWitnessNodes.push_back(tempCfg);
      }
      myifstream.close();


      cout << "leaving BandsIncrementalRoadmap" << endl;
    };

    virtual void Initialize(int in_RegionID){}
    virtual void Run(int in_RegionID){

      MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
      StatClass * pStatClass = region->GetStatClass();

      pStatClass->ClearStats();

      ClockClass Allstuff;
      //string base_filename = "itr_test_";

      //open output file for stats
      //filename env_ng_con_nf_seed.stats
      stringstream basefname;
      basefname << GetBaseFilename() << "." << GetBaseSeed();
      ofstream stat_out((basefname.str() + ".stats").c_str());
      ofstream map_out((basefname.str() + ".map").c_str());

      //write output file format
      stat_out << " #num_nodes \t num_edges \t lp_attempts \t lp_succ \t lp_cd  \t ng_time \t ng_cd \t con_time"
        //             << "\t nf_qry_time \t nf_const_time"
        << "\t num_ccs "
        << "\t max_cc_size \t min_cc_size"
        << "\t solve_qry \t min_edge_len \t max_edge_len \t ave_edge_len \t std_edge_len"
        << "\t min_degree \t max_degree \t ave_degree \t std_degree "
        << "\t approx_dia"
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
        cout << "--------------" << endl << "iter " << iteration << " of " << m_iterations << endl;
        ClockClass        NodeGenClock;
        ClockClass        ConnectionClock;
        ClockClass        IterationClock;
        ClockClass        QueryClock;
        ClockClass        StatClock;
        cout << "defined clocks" << endl;
        IterationClock.StartClock("Iteration");
        cout << "started clock" << endl;
        //---------------------------
        // Generate roadmap nodes
        //---------------------------
        cout << "GENERATE ROADMAP NODES - ";

        NodeGenClock.StartClock("Node Generation");
        vector<VID> newVids;
        typedef vector<string>::iterator I;
        for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
        {
          vector< CfgType > vectorCfgs;
          Sampler<CfgType>::SamplerPointer pNodeGen;
          cout << "reading sampler with label: " << *itr << endl;
          pNodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(*itr);
          pNodeGen->Sample(GetMPProblem()->GetEnvironment(), *pStatClass, m_numNodes, 100*m_numNodes,
              back_inserter(vectorCfgs));  

          cout << "Finished : " << vectorCfgs.size();
          vector<VID> vids =  region->AddToRoadmap(vectorCfgs);
          nodes_added += vids.size();
          cout << " - total VIDS: " << nodes_added << endl;
          for(size_t i=0; i<vids.size(); ++i) {
            newVids.push_back(vids[i]);
          }
        }
        // Now double the bounding box for connection
        if(resize_bbox){
          ResizeBbox(std::cout,2.0);
        }

        NodeGenClock.StopClock();
        elappsed_ng += NodeGenClock.GetSeconds();

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

          ConnectMap<CfgType,WeightType>::NodeConnectionPointer pConnection;
          pConnection = connectmap->GetNodeMethod(*itr);
          //cout << "Calling connection method:: " << pConnection->GetLabel() << endl;
          connectmap->ConnectNodes(pConnection, region->GetRoadmap(), *pStatClass, 
              GetMPProblem()->GetMPStrategy()->addPartialEdge, 
              GetMPProblem()->GetMPStrategy()->addAllEdges,
              newVids.begin(), newVids.end());
        }

        //Now Restore bounding box

        if(resize_bbox){
          ResizeBbox(std::cout,0.5);
        }
        ConnectionClock.StopClock();
        ellapsed_con += ConnectionClock.GetSeconds();




        /*string outputFilename = getBaseFilename() + ".map"; 
          ofstream  myofstream(outputFilename.c_str());

          if (!myofstream) {
          LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
          exit(-1);
          }
          region->WriteRoadmapForVizmo(myofstream);
          myofstream.close();
         */

        IterationClock.StopClock();

        //pStatClass->PrintAllStats(region->GetRoadmap());

        QueryClock.StartClock("Query");
        cout << "BEGIN isSameCCC" << endl <<flush;  
        stapl::vector_property_map< GRAPH,size_t > cmap;
        querySucceeded = is_same_cc(*region->GetRoadmap()->m_pRoadmap, cmap, 0, 1);

        QueryClock.StopClock();


        StatClock.StartClock("Stats Output");

        ///////////////////
        //Output stat info
        //NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
        OnlineStats degree = calcDegreeStats(*region->GetRoadmap()->m_pRoadmap);
        OnlineStats edges = calcEdgeStats(*region->GetRoadmap()->m_pRoadmap);
        vector<pair<size_t, VID> > CCStats;
        cmap.reset();
        get_cc_stats (*region->GetRoadmap()->m_pRoadmap, cmap, CCStats);
        // diameter computed in BandStats
        double diameter = 0.0;
        // std::sort (CCStats.begin(),  CCStats.end(), __CCVID_Compare<std::pair<int,VID> >() );
        // cout << "Begin run dia twice, start from largest component" << endl;
        //cout << "CCStats.size" << CCStats.size()<< endl ;
        //test
        /*int ccnum = 0;
          cout << "\nThere are " << CCStats.size() << " connected components:";
          for (vector< pair<size_t,VID> >::iterator vi = CCStats.begin(); vi != CCStats.end(); vi++) {
          cout << "\nCC[" << ccnum << "]: " << vi->first ;
          cout << " (vid=" << size_t(vi->second) << ")";
          ccnum++;
          }*/
        // VID far_vid(-1), far_vid2(-1); 

        //ComponentDiameter(*region->GetRoadmap()->m_pRoadmap,CCStats[0].second, &far_vid);
        //double diameter = ComponentDiameter(*region->GetRoadmap()->m_pRoadmap, far_vid,&far_vid2);
        // stat_out << basefname.str()
        stat_out << region->GetRoadmap()->m_pRoadmap->get_num_vertices() - 2
          << "\t" << region->GetRoadmap()->m_pRoadmap->get_num_edges() / 2
          << "\t" << pStatClass->m_connectionsAttempted << "\t" << double(pStatClass->m_connectionsMade) / double(pStatClass->m_connectionsAttempted)
          << "\t" << pStatClass->m_isCollByName["straightline-straightline::IsConnectedSLBinary"]
          << "\t" << elappsed_ng << "\t" <<  pStatClass->m_isCollTotal -
          pStatClass->m_isCollByName["straightline-straightline::IsConnectedSLBinary"]
          << "\t" << ellapsed_con
          //        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetQueryTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()- 2)
          //        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetConstructionTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
          << "\t" << CCStats.size()
          << "\t" << double(CCStats[0].first) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
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
          //total_out << basefname.str()
          total_out << region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2 
            << "\t" << region->GetRoadmap()->m_pRoadmap->get_num_edges() / 2
            << "\t" << pStatClass->m_connectionsAttempted << "\t" << double(pStatClass->m_connectionsMade) / double(pStatClass->m_connectionsAttempted)
            << "\t" << pStatClass->m_isCollByName["straightline-straightline::IsConnectedSLBinary"]
            << "\t" << elappsed_ng << "\t" <<  pStatClass->m_isCollTotal -
            pStatClass->m_isCollByName["straightline-straightline::IsConnectedSLBinary"]
            << "\t" << ellapsed_con
            //        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetQueryTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
            //        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetConstructionTime()) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
            << "\t" << CCStats.size()
            << "\t" << double(CCStats[0].first) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
            << "\t" << double(CCStats[CCStats.size()-1].first) / double(region->GetRoadmap()->m_pRoadmap->get_num_vertices()-2) 
            << "\t" << querySucceeded
            << "\t" << edges.GetMin() << "\t" << edges.GetMax() << "\t" << edges.GetMean() 
            << "\t" << edges.GetStandardDeviation() << "\t" << degree.GetMin() << "\t" << degree.GetMax() 
            << "\t" << degree.GetMean() << "\t" << degree.GetStandardDeviation() 
            << "\t" << diameter
            << endl;

          if (!map_out) {
            cerr << "MPRegion::WriteRoadmapForVizmo: can't open outfile: " << endl;
            exit(-1);
          }

          queryFirstSucceeded = true;
          cout << "Solved query! " << endl;
          //return;

        }

        StatClock.StopClock();

        NodeGenClock.PrintClock();
        ConnectionClock.PrintClock();
        QueryClock.PrintClock();
        StatClock.PrintClock();
        IterationClock.PrintClock();
        Allstuff.StopPrintClock();


        iteration++;
        cout << endl;
      }

      cout << "Finished map " << endl;
      cout << "~BandsIncrementalRoadmap::()" << endl;

      region->WriteRoadmapForVizmo(map_out);
      map_out.close();
    }

    virtual void Finalize(int in_RegionID){}

  private:

    OnlineStats calcDegreeStats(RoadmapGraph<CfgType,WeightType>& _graph) {
      OnlineStats to_return;
      for(RoadmapGraph<CfgType,WeightType>::VDI vitr =_graph.descriptor_begin(); vitr != _graph.descriptor_end(); ++vitr) {
        to_return.AddData(_graph.get_out_degree(*vitr));
      }
      return to_return;
    };


    OnlineStats calcEdgeStats(RoadmapGraph<CfgType,WeightType>& _graph) {
      OnlineStats to_return;
      RoadmapGraph<CfgType,WeightType>::VI vitr;
      RoadmapGraph<CfgType,WeightType>::EI eitr;


      for(vitr = _graph.begin(); vitr != _graph.end(); ++vitr) {
        for (eitr = (*vitr).begin(); eitr != (*vitr).end(); ++eitr) {
          to_return.AddData((*eitr).property().GetWeight());
        }
      }
      return to_return;
    };


    bool CanConnectToComponent(RoadmapGraph<CfgType,WeightType>& _graph, VID _cc, CfgType _test) {
      vector<VID> vec_cc;
      stapl::vector_property_map< GRAPH,size_t > cmap;
      get_cc(_graph, cmap, _cc, vec_cc);

      vector<pair<double,VID> > vec_dist_vid;
      vec_dist_vid.reserve(vec_cc.size());

      for(size_t i=0; i<vec_cc.size(); ++i) {
        double dist = dm->Distance(GetMPProblem()->GetEnvironment(),
            _test, (*(_graph.find_vertex(vec_cc[i]))).property());
        vec_dist_vid.push_back(make_pair(dist, vec_cc[i]));
      }

      sort(vec_dist_vid.begin(), vec_dist_vid.end());
      StatClass _mystat;
      LPOutput<CfgType,WeightType> out_lp_output;
      for ( size_t i=0; i<vec_dist_vid.size(); ++i) {
        if ( GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetLocalPlannerMethod(m_strLocalPlannerLabel)->
            IsConnected ( GetMPProblem()->GetEnvironment(), _mystat, dm, _test, 
              (*(_graph.find_vertex(vec_dist_vid[i].second))).property(), &out_lp_output, 
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
      stapl::vector_property_map< GRAPH,size_t > cmap;
      get_cc_stats(_graph, cmap, CCStats);
      for(size_t i=0; i<CCStats.size(); ++i) {
        if(CanConnectToComponent(_graph, CCStats[i].second, _start) 
            && CanConnectToComponent(_graph, CCStats[i].second, _goal)) {
          return true;
        }
      }
      return false;
    }



    void ResizeBbox(ostream& out_os, double scale_factor){
      cout<<"BandsStrategy::Resize Bounding box"<<endl<<flush;
      shared_ptr<BoundingBox> pBoundBox = (GetMPProblem()->GetMPRegion(0))->GetBoundingBox();
      pBoundBox->Print(out_os);
      pBoundBox->TranslationalScale(scale_factor);
      cout<<"BandsStrategy::After Resize Bounding Box"<<endl<<flush;
      pBoundBox->Print(out_os);
    }



    vector<string> m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    vector<string> m_vecStrComponentConnectionLabels;
    vector<string> m_vecNodeCharacterizerLabels;
    NeighborhoodFinder::NeighborhoodFinderPointer m_NF;
    string m_strLocalPlannerLabel;
    //  string m_nfStats;
    vector<CfgType> m_vecWitnessNodes;
    string m_queryFilename;
    int m_stepSize;
    double m_posRes;
    int m_RegionNdx;
    bool resize_bbox;
    shared_ptr<DistanceMetricMethod> dm;
    int m_numNodes;
    int m_iterations;

};







// this MPStrategy reads in a single roadmap, k, and d and outputs two things:
// 1) a file containing a list of the distance to the k-th closest neighbor in the roadmap, sorted ascending
//   e.g.   "1 7\n2 4\n3 5" ... "5000 8";
// 2) for several values of d, a file containing a list of the # neighbors within d distance, sorted decreasing    
//
//  it also computes the connectivity stats and others (scale-free, expansion)
//@note we are no longer computing 1 and 2 above by default , the compute_dist_neighbor need to be set to do this

class BSContainer : public MPSMContainer {
  public:
    BSContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
    string input_map_filename;
    string ideal_map_filename;
    string out_filename;
    string out_filename_dist;
    string out_filename_num_neighbors;
    bool compute_dist_neighbor;
    int interval;
    int k;
    double dist;
    EdgeExpanderStats *ExpanderStatsClass;
    MPSMContainer parent;

};

class BandsStats : public MPStrategyMethod {
  private:
    string input_map_filename;
    string ideal_map_filename;
    string out_filename;
    string out_filename_dist;
    string out_filename_num_neighbors;
    bool compute_dist_neighbor;
    int interval;
    int k;
    double dist;
    EdgeExpanderStats *ExpanderStatsClass;
  public:
    typedef RoadmapGraph<CfgType, WeightType>::VID VID;
    BandsStats(BSContainer cont) : MPStrategyMethod(cont.parent) {
      input_map_filename = cont.input_map_filename;
      ideal_map_filename = cont.ideal_map_filename;
      out_filename = cont.out_filename;
      out_filename_dist = cont.out_filename_dist;
      out_filename_num_neighbors = cont.out_filename_num_neighbors;
      compute_dist_neighbor = cont.compute_dist_neighbor;
      interval = cont.interval;
      k = cont.k;
      dist = cont.dist;
      ExpanderStatsClass = cont.ExpanderStatsClass;


    }
    BandsStats(XMLNodeReader& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
      ParseXML(in_Node);   
      ExpanderStatsClass = new EdgeExpanderStats(in_Node, problem);
    }

    void ParseXML(XMLNodeReader& in_Node) {
      cout << "BandsStats::ParseXML()" << endl;

      input_map_filename = in_Node.stringXMLParameter(string("input_map_filename"), true, string(""),string("input .map filename"));
      ideal_map_filename = in_Node.stringXMLParameter(string("ideal_map_filename"), true, string(""),string("all-pairs .map filename"));
      interval = in_Node.numberXMLParameter(string("interval"), false, 100, 1, 1000000, string("interval for stats"));		
      k = in_Node.numberXMLParameter(string("k"), true, 0, 0, 10000, string("k-value for calculation"));
      dist = in_Node.numberXMLParameter(string("dist"), true, 0.0, 0.0, 1000.0, string("dist for calculation"));
      out_filename_dist = in_Node.stringXMLParameter(string("out_filename_dist"), true, string(""),string("output filename - dist of k-th closest node"));
      out_filename_num_neighbors = in_Node.stringXMLParameter(string("out_filename_num_neighbors"), true, string(""),string("output filename - num neighbors inside dist"));
      compute_dist_neighbor = in_Node.boolXMLParameter(string("compute_dist_neighbor"), false, false,
          string("if true, the function to compute distance and num of neighbors will be called"));
    }

    long scaleFree(Roadmap<CfgType,WeightType> &rmp) {
      RoadmapGraph<CfgType,WeightType>* pMap = rmp.m_pRoadmap;

      int scaleFreeMetric = 0;

      for(RoadmapGraph<CfgType,WeightType>::VDI vitr = pMap->descriptor_begin(); vitr != pMap->descriptor_end(); ++vitr) {
        vector<VID> adj_verts;
        pMap->get_successors(*vitr, adj_verts);
        for(vector<VID>::iterator iter2 = adj_verts.begin(); iter2 != adj_verts.end(); iter2++) {    
          vector<VID> adj_verts2;
          pMap->get_successors(*vitr, adj_verts2);
          scaleFreeMetric += adj_verts.size()*adj_verts2.size();
        }
      }
      return scaleFreeMetric;
    }

    int compareAllPairs(Roadmap<CfgType,WeightType>& rmp){
      int sameCCPairs=0;
      //int sameCCPairsUnion=0;
      //cout<<"entering compare all"<<endl;
      vector<VID> vertices;
      rmp.m_pRoadmap->GetVerticesVID(vertices);

      stapl::vector_property_map< GRAPH,size_t > cmap;

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
      stapl::vector_property_map< GRAPH,size_t > cmap;
      get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats);
      for(vector< pair<size_t, VID> >::iterator iter = ccstats.begin(); iter < ccstats.end(); iter++){
        sameCCPairs+=iter->first*(iter->first-1)/2;      
      }
      return sameCCPairs;
    }

    //returns same cc pairs of nodes in roadmap rmp (thresholdVID is inclusive)
    int fastCompareAllPairs(const Roadmap<CfgType,WeightType>& rmp, size_t thresholdVID){
      //cout<<"in fastCompareAllPairs"<<endl;
      int sameCCPairs=0;
      vector< pair<size_t, VID> > ccstats;
      stapl::vector_property_map< GRAPH,size_t > cmap;
      vector< vector<VID> > ccverts;

      get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats, ccverts);

      //cout << "------------" << endl;
      //for (int i = 0; i < ccstats.size(); i++) {
      //cout << ccstats[i].first << ", " << ccstats[i].second << endl;
      //}

      //for (int i = 0; i < ccverts.size(); i++) {
      //cout << ccverts[i].size() << ":";
      //for (int j = 0; j < ccverts[i].size(); j++) {
      //cout << " | " << ccverts[i].at(j);
      //}
      //cout << endl;
      //}

      //TODO:get cc for each stat
      //remove ones with VID > thresholdVID
      //add choose 2 of that
      for(vector< vector<VID> >::iterator iter = ccverts.begin(); iter < ccverts.end(); iter++){
        vector<VID> cci_vid = *iter;

        //cout << "cc vids (" << cci_vid.size() << ")";
        //for(vector<VID>::iterator iter2 = cci_vid.begin(); iter2 != cci_vid.end(); iter2++){cout << " | " << *iter2;}cout << endl;

        int count = 0;
        for(vector<VID>::iterator iter2 = cci_vid.begin(); iter2 != cci_vid.end(); iter2++){
          //cout << *iter2 << " <= " << thresholdVID;
          if(*iter2<=thresholdVID) {
            count++;
            //cout << " | yes";
          }
        }
        //cout << endl;
        //cout << "count = " << count << endl;
        sameCCPairs+=count*(count-1)/2;
      }
      //cout<<"out fastCompareAllPairs"<<endl;
      return sameCCPairs;
    }

    vector<int> getSameCCStatsInUnion(const Roadmap<CfgType,WeightType>& unionrmp, int _interval){
      //cout<<"in getSameCCStatsInUnion... numVertices = " << unionrmp.m_pRoadmap->get_num_vertices() << endl;
      vector<int> sameCCStats;
      for(size_t i = 0; i < unionrmp.m_pRoadmap->get_num_vertices(); i++){
        if(i % _interval == 0){
          int ccStats=fastCompareAllPairs(unionrmp, unionrmp.m_pRoadmap->get_num_vertices()-i);
          sameCCStats.push_back(ccStats);
          //cout << "ccStats in union at i = " << i << ": " << ccStats << endl;
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


    double computeDiameter(Roadmap<CfgType,WeightType>& rmp){

      vector<pair<size_t, VID> > CCStats;
      stapl::vector_property_map< GRAPH,size_t > cmap;
      get_cc_stats(*(rmp.m_pRoadmap), cmap, CCStats);
      std::sort (CCStats.begin(),  CCStats.end(), __CCVID_Compare<std::pair<int,VID> >() );
      // test 
      /* int ccnum = 0;
         cout << "\nThere are " << CCStats.size() << " connected components:";
         for (vector< pair<size_t,VID> >::iterator vi = CCStats.begin(); vi != CCStats.end(); vi++) {
         cout << "\nCC[" << ccnum << "]: " << vi->first ;
         cout << " (vid=" << size_t(vi->second) << ")";
         ccnum++;
         }*/
      VID far_vid(-1), far_vid2(-1); 
      ComponentDiameter(*(rmp.m_pRoadmap),CCStats[0].second, &far_vid);
      double diameter = ComponentDiameter(*(rmp.m_pRoadmap), far_vid,&far_vid2);
      return diameter;
    }


    void storeSameCCInfo(unionStats& stats, Roadmap<CfgType,WeightType>& rmp){
      vector< pair<size_t, VID> > ccstats;
      stapl::vector_property_map< GRAPH,size_t > cmap;
      get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats);
      std::sort (ccstats.begin(),  ccstats.end(), __CCVID_Compare<std::pair<int,VID> >() );
      vector< pair<size_t, VID> >::iterator iter = ccstats.begin();
      //stats.sameCCSizes.reserve(5);
      for(int i=0; i<5; i++){
        if(iter != ccstats.end()){
          stats.sameCCSizes[i]=iter->first;
          //cout<<"cc size = "<<stats.sameCCSizes[i]<<endl;
          //stats.sameCCSizes.push_back(iter->first);
          iter++;
        }else{
          stats.sameCCSizes[i]=0;
          //cout<<"cc size = 0 = "<<stats.sameCCSizes[i]<<endl;
          //stats.sameCCSizes.push_back(0);
        } 
        //cout<<endl;
      }
    }  



    void printAllPairsAtInterval(Roadmap<CfgType,WeightType>& rmp, const vector<int>& union_pairs, int _interval){

      //todo: change compute edge length are each interval
      double avg_edge_length, max_edge_length, average_max_edge_length;
      computeEdgeLengthFromdm(rmp,avg_edge_length, max_edge_length, average_max_edge_length);

      //int sameCCPairUnion = fastCompareAllPairs(union_rmp);
      //cout<<"in print at interval"<<endl;    
      string statsFileName = input_map_filename + ".ccstats";

      ofstream  myofstream(statsFileName.c_str());
      myofstream<<"#number of nodes, diameter, scale-free, sameCCPairs, difference to union sameCCPairs, ratio to union";
      myofstream<<", size_largest_CC1,size_largest_CC2, size_largest_CC3,  size_largest_CC4, size_largest_CC5, avg_edge_length_dm, max_edge_length_dm, average_max_edge_length_dm";
      /* myofstream<<", max_path_length";
         for(int i=0; i<ExpanderStatsClass->mpl;i++){
         myofstream<<"hop_graph_"<<i<<", ";
         }
         myofstream<<", triangle participation length, triangle participation";*/
      myofstream<<endl;

      int sameCCPair=fastCompareAllPairs(rmp);
      long scale_free = scaleFree(rmp);
      double diameter = computeDiameter(rmp);

      vector<int> hop_graph_stats;
      //hop_graph.resize(ExpanderStatsClass->mpl); 
      hop_graph_stats= ExpanderStatsClass->hop_graph(rmp);
      vector<int> triangle_participation = ExpanderStatsClass->triangleParticipation(rmp);
      /*
         cout<<"pringing hg"<<endl;
         for(int i=0; i<ExpanderStatsClass->mpl-1;i++){
         cout<<"pringing hg loop"<<endl;
         cout<< hop_graph_stats[i]<<", ";
         }
         cout<<"pringing tp"<<endl;
         cout<<triangle_participation.size()<<", ";
         for(int i=0; i<triangle_participation.size();i++){
         cout<<"pringing tp loop"<<endl;
         cout<< triangle_participation[i]<<", ";
         }
       */


      /*if(is_union){
      //int sameCCPairUnion=fastCompareAllPairs(rmp);
      union_pairs.clear();
      union_pairs.push_back(sameCCPair);
      }*/
      unionStats stats;
      storeStats(stats, sameCCPair, union_pairs[0],rmp.m_pRoadmap->get_num_vertices()-2);
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
      for (RoadmapGraph<CfgType,WeightType>::VDI vi = rmp.m_pRoadmap->descriptor_end(); vi != rmp.m_pRoadmap->descriptor_begin(); --vi) {    
        //cout<<"k="<<k<<endl;
        if (vi != rmp.m_pRoadmap->descriptor_end()) {
          //cout<<"deleting vid="<<*iter<<endl;
          rmp.m_pRoadmap->delete_vertex(*vi);
          //union_rmp.m_pRoadmap->DeleteVertex(*iter);
          removed++;
          if(removed % _interval == 0){
            sameCCPair=fastCompareAllPairs(rmp);
            //if(is_union){
            // //int sameCCPairUnion=fastCompareAllPairs(rmp);
            // union_pairs.push_back(sameCCPair);
            //}
            unionStats union_stats;
            storeStats(union_stats, sameCCPair, union_pairs[number_intervals_printed], rmp.m_pRoadmap->get_num_vertices()-2);
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
        myofstream<<diameter<<", ";
        myofstream<<scale_free<<", ";
        myofstream<<stats_iter->sameCCPair<<", ";
        myofstream<<stats_iter->differenceInSameCC<<", ";
        myofstream<<stats_iter->ratioToUnion<<", ";
        myofstream<<stats_iter->sameCCSizes[0]<<", ";
        myofstream<<stats_iter->sameCCSizes[1]<<", ";
        myofstream<<stats_iter->sameCCSizes[2]<<", ";
        myofstream<<stats_iter->sameCCSizes[3]<<", ";
        myofstream<<stats_iter->sameCCSizes[4]<<", ";
        myofstream<<avg_edge_length<<", ";
        myofstream<<max_edge_length<<", ";
        myofstream<<average_max_edge_length<<", ";

        /*myofstream<<ExpanderStatsClass->mpl<<", ";


          for(int i=0; i<ExpanderStatsClass->mpl;i++){				 
          myofstream<< hop_graph_stats[i]<<", ";
          }*/


        /*myofstream<<triangle_participation.size()<<", ";
          for(int i=0; i<triangle_participation.size();i++){
          myofstream<< triangle_participation[i]<<", ";
          }*/

        myofstream<<endl;
      }

      myofstream.close();

    }

    //removes nodes untile rmp has specified size
    void trim(Roadmap<CfgType,WeightType>& rmp, int size){

      vector<GRAPH::vertex_descriptor> v_vd;
      for (RoadmapGraph<CfgType,WeightType>::VDI vi = rmp.m_pRoadmap->descriptor_begin(); vi != rmp.m_pRoadmap->descriptor_end(); ++vi) {
        v_vd.push_back(*vi);
      }

      for(size_t i=size; i < v_vd.size(); i++){
        //cout<<"size="<<size<<"rmpsize = "<<vertices.size()<<"i="<<i<<endl;
        rmp.m_pRoadmap->delete_vertex(v_vd[i]);
      }
    }


    void printAllPairs(Roadmap<CfgType,WeightType>& ideal_rmp, Roadmap<CfgType, WeightType>& input_rmp){
      vector<int> ideal_pairs;
      ideal_pairs=getSameCCStatsInUnion(ideal_rmp, interval);
      //printAllPairsAtInterval(ideal_rmp, ideal_pairs, interval);
      printAllPairsAtInterval(input_rmp, ideal_pairs, interval);
    }

    void distCalc() {
      Roadmap<CfgType,WeightType> rmp1;

      rmp1.ReadRoadmapGRAPHONLY(input_map_filename.c_str());

      int pos = out_filename_num_neighbors.find("num_neighbors");

      vector< Roadmap<CfgType,WeightType>* > rmps;

      std::stringstream ss;
      ss << "dist." << dist << "_";
      string dist_str = ss.str();

      rmps.push_back(&rmp1);
      out_filename_num_neighbors.insert(pos, dist_str);

      // note: BFNF neighborhood finder must be specified in XML file
      NeighborhoodFinder::NeighborhoodFinderPointer nfptr;
      nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod("BFNF");
      shared_ptr<DistanceMetricMethod> dmm = nfptr->GetDMMethod();

      Environment* _env = this->GetMPProblem()->GetEnvironment();

      for (size_t rmp_index = 0; rmp_index < rmps.size(); rmp_index++) {
        Roadmap<CfgType,WeightType>* rmp = rmps[rmp_index];

        rmp->SetEnvironment(_env);

        vector<double> dist_list;
        vector<int> valid_neighbor_count;

        // iterate through all roadmap vertices, find:
        //   1) distance to the k-th closest neighbor.
        //   2) # neighbors within d
        RoadmapGraph<CfgType,WeightType>* pMap = rmp->m_pRoadmap;
        RoadmapGraph<CfgType,WeightType>::VI vitr;
        RoadmapGraph<CfgType,WeightType>::VI vitr2;
        int ind = 0;
        for(vitr = pMap->begin(); vitr != pMap->end(); ++vitr) {
          if (ind % 100 == 0) cout << ind << endl;
          vector<VID> neighbors(k);

          this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, rmp, (*vitr).descriptor(), k, neighbors.begin());
          double k_dist = dmm->Distance(_env, (*vitr).property(), (*(pMap->find_vertex(neighbors.back()))).property());
          dist_list.push_back(k_dist);

          int vnc = 0;
          for (vitr2 = pMap->begin(); vitr2 != pMap->end(); ++vitr2) {
            double curr_dist = dmm->Distance(_env, (*vitr).property(), (*vitr2).property());
            if (curr_dist < dist) {
              //cout << curr_dist << " < " << dist << endl;
              vnc++;
            }
          }
          valid_neighbor_count.push_back(vnc);

          ind++;
        }

        sort(dist_list.begin(), dist_list.end());
        sort(valid_neighbor_count.rbegin(), valid_neighbor_count.rend());

        // write to file
        ofstream myofstream(out_filename_dist.c_str());
        ofstream myofstream2(out_filename_num_neighbors.c_str());

        if (!myofstream) {
          cerr << "can't open out_filename_dist" << endl;
          exit(-1);
        }
        if (!myofstream2) {
          cerr << "can't open out_filename_num_neighbors" << endl;
          exit(-1);
        }

        int count = 1;  
        for (vector<double>::iterator itr = dist_list.begin(); itr != dist_list.end(); itr++) {
          myofstream << count << "\t" << *itr << endl;
          //cout << count << "\t" << *itr << endl;
          count++;
        }

        count = 1;  
        for (vector<int>::iterator itr = valid_neighbor_count.begin(); itr != valid_neighbor_count.end(); itr++) {
          myofstream2 << count << "\t" << *itr << endl;
          //cout << count << "\t" << *itr << endl;
          count++;
        }

        myofstream.close();
        myofstream2.close();
      }
    }


    void operator()(){
      cout << "BandsStats::BandsStats()" << endl;

      // do dist/num_neighbors work if flag is set from xml
      if(compute_dist_neighbor){
        distCalc();
      }

      // do the other stats
      Roadmap<CfgType, WeightType> ideal_rmp;
      Roadmap<CfgType, WeightType> input_rmp;

      input_rmp.ReadRoadmapGRAPHONLY(input_map_filename.c_str());
      ideal_rmp.ReadRoadmapGRAPHONLY(ideal_map_filename.c_str());

      MPRegion<CfgType,WeightType> region(0, GetMPProblem());
      region.roadmap = ideal_rmp;
      ideal_rmp.SetEnvironment(GetMPProblem()->GetEnvironment());
      input_rmp.SetEnvironment(GetMPProblem()->GetEnvironment());
      //cout<<"printing all pairs"<<endl;
      printAllPairs(ideal_rmp, input_rmp);

      exit(-1);	
      //(*ExpanderStatsClass)();
    }


    virtual void PrintOptions(ostream& out_os) { }
    virtual void operator()(int in_RegionID) { }
    virtual void Initialize(int in_RegionID){}
    virtual void Run(int in_RegionID){}
    virtual void Finalize(int in_RegionID){}


    void computeEdgeLengthFromdm(Roadmap<CfgType, WeightType> rmp,double &avg_dist, double &max_dist, double &avg_max_dist){

      max_dist=0;
      avg_dist=0;
      avg_max_dist=0;
      int count=0;
      //todo: change to use count from rmp graph
      int nodes=0;
      for(RoadmapGraph<CfgType,WeightType>::VDI iter=rmp.m_pRoadmap->descriptor_begin(); iter<rmp.m_pRoadmap->descriptor_end(); iter++){
        nodes++;
        vector<VID> succ;
        rmp.m_pRoadmap->get_successors(*iter, succ); 
        double max_for_node=0;
        for(vector<VID>::iterator iter2=succ.begin(); iter2<succ.end(); iter2++){
          RoadmapGraph<CfgType,WeightType>* pMap = rmp.m_pRoadmap;
          CfgType cfg1 = (pMap->find_vertex(*iter))->property();
          CfgType cfg2 = (pMap->find_vertex(*iter2))->property();
          shared_ptr<DistanceMetricMethod>dmm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod("BFNF")->GetDMMethod();
          double dist=dmm->Distance(rmp.GetEnvironment(), cfg1, cfg2);
          count++;
          max_dist=max(max_dist,dist);
          max_for_node=max(dist,max_for_node);
          avg_dist+=dist;
        }
        avg_max_dist+=max_for_node;

      }
      avg_max_dist/=nodes;
      avg_dist/=count;
    }


    };






#endif
