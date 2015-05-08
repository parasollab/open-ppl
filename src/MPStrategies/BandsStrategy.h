#ifndef BandsStrategy
#define BandsStrategy

//#include<sys/time.h>
//#include <math.h>

#include "MPStrategyMethod.h"
#include "ExpanderStats.h"
#include "MPProblem.h"
#include "Roadmap.h"
#include "MetricUtils.h"
#include "Connector.h"
#include "DistanceMetricMethod.h"
#include "LocalPlanners.h"
#include <graph/algorithms/diameter.h>

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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class T>
struct __CCVID_Compare : public std::binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.first > y.first; }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
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
    bool resize_bbox;
    shared_ptr<DistanceMetricMethod> dm;
    MPSMContainer parent;
    int m_numNodes;
    int m_iterations;
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
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
      resize_bbox = cont.resize_bbox;
      dm = cont.dm;

    }
    BandsIncrementalRoadmap(XMLNode& in_Node, MPProblem* in_pProblem) :
      MPStrategyMethod(in_Node,in_pProblem) {
        ParseXML(in_Node);
      };
    virtual ~BandsIncrementalRoadmap() {}

    virtual void Print(ostream& out_os) const {};

    virtual void ParseXML(XMLNode& in_Node) {
      cout << "BandsIncrementalRoadmap::ParseXML()" << endl;
      //SRand(getSeed());
      XMLNode::childiterator citr;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
        if(citr->getName() == "node_generation_method") {
          string node_gen_method = citr->Read(string("Method"),true,
              string(""),string("Node Generation Method"));
          m_vecStrNodeGenLabels.push_back(node_gen_method);
          citr->warnUnrequestedAttributes();
        } else if(citr->getName() == "node_connection_method") {
          string connect_method = citr->Read(string("Method"),true,
              string(""),string("Node Connection Method"));
          m_vecStrNodeConnectionLabels.push_back(connect_method);
          citr->warnUnrequestedAttributes();
        } else if(citr->getName() == "component_connection_method") {
          string connect_method = citr->Read(string("Method"),true,
              string(""),string("CC Connection Method"));
          m_vecStrComponentConnectionLabels.push_back(connect_method);
          citr->warnUnrequestedAttributes();
        } else if(citr->getName() == "lp_method") {
          m_strLocalPlannerLabel = citr->Read(string("Method"),true,
              string(""),string("Local Planner Method"));
          citr->warnUnrequestedAttributes();
        } else if (citr->getName() == "step_size") {
          m_stepSize = citr->Read(string("step_size"), true,
              int(100),int(0),int(MAX_INT),
              string("Iteration step size"));
        } else if(citr->getName() == "NeighborhoodFinder") {
          string nf_method = citr->Read(string("Method"),true,
              string(""),string("NeighborhoodFinder Method"));
          m_NF = GetMPProblem()->GetNeighborhoodFinder()->GetMethod(nf_method);
          citr->warnUnrequestedAttributes();
        } else {
          citr->warnUnknownNode();
        }
      }

      m_queryFilename = in_Node.Read("query_filename", true, "", "Query Filename");
      m_posRes = in_Node.Read(string("pos_res"), false, 0.0, 0.0, 10.0,
          string("positional resolution"));
      //    m_nfStats = in_Node.Read("nf_stat", true, "", "NF for stat output");
      resize_bbox = in_Node.Read(string("resize_bbox"), false, false,
          string("if true, bounding box size will be doubled"));

      m_numNodes = in_Node.Read(string("num_samples"), true, 1,0,MAX_INT, "Number of Samples");
      m_iterations = in_Node.Read("iterations", true, 1,0,MAX_INT, "Number of Iterations");

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

    virtual void Initialize(){}
    virtual void Run(){

      StatClass * pStatClass = GetMPProblem()->GetStatClass();

      pStatClass->ClearStats();

      //string base_filename = "itr_test_";

      //open output file for stats
      //filename env_ng_con_nf_seed.stats
      stringstream basefname;
      basefname << GetBaseFilename() << "." << GetBaseSeed();
      ofstream stat_out((basefname.str() + ".stats").c_str());
      ofstream map_out((basefname.str() + ".map").c_str());

      //write output file format
      stat_out << " #num_nodes \t num_edges \t lp_attempts \t lp_succ \t lp_cd  \t ng_time \t ng_cd \t con_time"
        << "\t num_ccs "
        << "\t max_cc_size \t min_cc_size"
        << "\t solve_qry \t min_edge_len \t max_edge_len \t ave_edge_len \t std_edge_len"
        << "\t min_degree \t max_degree \t ave_degree \t std_degree "
        << "\t hop_diam \t weight_diam"
        << endl;


      pStatClass->StartClock("Everything");

      double elapsed_ng(0.0), elapsed_con(0.0);
      bool querySucceeded = false;
      bool queryFirstSucceeded = false;
      int iteration = 0;
      int nodes_added = 0;

      //for (int step = 0; step < m_iterations; step++)
      for(size_t i=0; i<m_vecWitnessNodes.size();i++){
        GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(m_vecWitnessNodes[i]);
      }
      while(iteration < m_iterations) {
        cout << "--------------" << endl << "iter " << iteration << " of " << m_iterations << endl;
        cout << "defined clocks" << endl;
        pStatClass->StartClock("Iteration");
        cout << "started clock" << endl;
        //---------------------------
        // Generate roadmap nodes
        //---------------------------
        cout << "GENERATE ROADMAP NODES - ";

        pStatClass->StartClock("Node Generation");
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
          vector<VID> vids =  GetMPProblem()->AddToRoadmap(vectorCfgs);
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

        pStatClass->StopClock("Node Generation");
        elapsed_ng += pStatClass->GetSeconds("Node Generation");

        //---------------------------
        // Connect roadmap nodes
        //----------
        //
        //-----------------
        cout << "CONNECT ROADMAP NODES" << endl;

        pStatClass->StartClock("Node Connection");
        Connector<CfgType, WeightType>* connector = GetMPProblem()->GetMPStrategy()->GetConnector();
        stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
        cmap.reset();
        typedef vector<string>::iterator J;
        for(J itr = m_vecStrNodeConnectionLabels.begin();
            itr != m_vecStrNodeConnectionLabels.end(); ++itr)
        {

          Connector<CfgType,WeightType>::ConnectionPointer pConnection;
          pConnection = connector->GetMethod(*itr);
          //cout << "Calling connection method:: " << pConnection->GetLabel() << endl;
          pConnection->Connect(GetMPProblem()->GetRoadmap(), *pStatClass, cmap, newVids.begin(), newVids.end(), newVids.begin(), newVids.end());
        }

        //Now Restore bounding box

        if(resize_bbox){
          ResizeBbox(std::cout,0.5);
        }
        pStatClass->StopClock("Node Connection");
        elapsed_con += pStatClass->GetSeconds("Node Generation");




        /*string outputFilename = getBaseFilename() + ".map";
          ofstream  myofstream(outputFilename.c_str());

          if (!myofstream) {
          LOG_ERROR_MSG("Run: can't open outfile: ");
          exit(-1);
          }
          GetMPProblem()->WriteRoadmapForVizmo(myofstream);
          myofstream.close();
          */

        pStatClass->StopClock("Iteration");

        //pStatClass->PrintAllStats(GetMPProblem()->GetRoadmap());

        pStatClass->StartClock("Query");
        cout << "BEGIN isSameCCC" << endl <<flush;
        cmap.reset();
        querySucceeded = is_same_cc(*GetMPProblem()->GetRoadmap()->m_pRoadmap, cmap, 0, 1);

        pStatClass->StopClock("Query");


        pStatClass->StartClock("Stats Output");

        ///////////////////
        //Output stat info
        //NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
        OnlineStats degree = calcDegreeStats(*GetMPProblem()->GetRoadmap()->m_pRoadmap);
        OnlineStats edges = calcEdgeStats(*GetMPProblem()->GetRoadmap()->m_pRoadmap);
        vector<pair<size_t, VID> > CCStats;
        cmap.reset();
        get_cc_stats (*GetMPProblem()->GetRoadmap()->m_pRoadmap, cmap, CCStats);
        // diameter computed in BandStats
        double hop_diameter = 0.0;
        double weight_diameter = 0.0;
        //TODO: Note- diameter as computed by STAPL now is hop_diameer, discussion is on going as to whether STAPL will support both diamter compuation on user will provide it as part of edge property
        hop_diameter = stapl::sequential::diameter(*(GetMPProblem()->GetRoadmap()->m_pRoadmap), CCStats[0].second);
        weight_diameter = stapl::sequential::diameter(*(GetMPProblem()->GetRoadmap()->m_pRoadmap), CCStats[0].second);
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

        //ComponentDiameter(*GetMPProblem()->GetRoadmap()->m_pRoadmap,CCStats[0].second, &far_vid);
        //double diameter = ComponentDiameter(*GetMPProblem()->GetRoadmap()->m_pRoadmap, far_vid,&far_vid2);
        // stat_out << basefname.str()

        // NOTE: this assumes we use the first local planner defined
        int lp_attempts = pStatClass->m_lpInfo.begin()->second.get<0>();
        int lp_connections = pStatClass->m_lpInfo.begin()->second.get<1>();
        int lp_cdcalls = pStatClass->m_lpInfo.begin()->second.get<2>();
        double lp_succ = double(lp_connections) / double(lp_attempts);

        stat_out << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices() - 2
          << "\t" << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_edges() / 2
          << "\t" << lp_attempts << "\t" << lp_succ << "\t" << lp_cdcalls
          << "\t" << elapsed_ng << "\t" <<  "-1" << "\t" << elapsed_con
          << "\t" << CCStats.size()
          << "\t" << double(CCStats[0].first) / double(GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
          << "\t" << double(CCStats[CCStats.size()-1].first) / double(GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
          << "\t" << querySucceeded
          << "\t" << edges.GetMin() << "\t" << edges.GetMax() << "\t" << edges.GetMean()
          << "\t" << edges.GetStandardDeviation() << "\t" << degree.GetMin() << "\t" << degree.GetMax()
          << "\t" << degree.GetMean() << "\t" << degree.GetStandardDeviation()
          << "\t" << hop_diameter << "\t" << weight_diameter
          << endl;



        ///////////////////
        // Output Total info
        if(querySucceeded && !queryFirstSucceeded) {
          /*
             ofstream total_out((basefname.str() + ".total").c_str());
          //total_out << basefname.str()
          total_out << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()-2
          << "\t" << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_edges() / 2
          //<< "\t" << pStatClass->m_connectionsAttempted << "\t" << double(pStatClass->m_connectionsMade) / double(pStatClass->m_connectionsAttempted)
          << "\t" << pStatClass->m_isCollByName["straightline-straightline::IsConnectedSLBinary"]
          << "\t" << elapsed_ng << "\t" <<  pStatClass->m_isCollTotal -
          pStatClass->m_isCollByName["straightline-straightline::IsConnectedSLBinary"]
          << "\t" << elapsed_con
          //        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetQueryTime()) / double(GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
          //        << "\t" << double(nf->GetNFMethod(m_nfStats)->GetConstructionTime()) / double(GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
          << "\t" << CCStats.size()
          << "\t" << double(CCStats[0].first) / double(GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
          << "\t" << double(CCStats[CCStats.size()-1].first) / double(GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()-2)
          << "\t" << querySucceeded
          << "\t" << edges.GetMin() << "\t" << edges.GetMax() << "\t" << edges.GetMean()
          << "\t" << edges.GetStandardDeviation() << "\t" << degree.GetMin() << "\t" << degree.GetMax()
          << "\t" << degree.GetMean() << "\t" << degree.GetStandardDeviation()
          << "\t" << diameter
          << endl;

          if (!map_out) {
          cerr << "Run: can't open outfile: " << endl;
          exit(-1);
          }
          */

          queryFirstSucceeded = true;
          cout << "Solved query! " << endl;
          //return;

        }

        pStatClass->StopClock("Stats Output");

        pStatClass->PrintClock("Node Generation", cout);
        pStatClass->PrintClock("Node Connection", cout);
        pStatClass->PrintClock("Query Time", cout);
        pStatClass->PrintClock("Stats Output", cout);
        pStatClass->PrintClock("Iteration", cout);
        pStatClass->StopPrintClock("Everything", cout);


        iteration++;
        cout << endl;
      }

      cout << "Finished map " << endl;
      cout << "~BandsIncrementalRoadmap::()" << endl;

      GetMPProblem()->WriteRoadmapForVizmo(map_out);
      map_out.close();
    }

    virtual void Finalize(){}

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
      stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
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
        if ( GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_strLocalPlannerLabel)->
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
      stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
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
      shared_ptr<Boundary> pBoundBox = GetMPProblem()->GetEnvironment()->GetBoundary();
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
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
    BandsStats(XMLNode& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
      ParseXML(in_Node);
      ExpanderStatsClass = new EdgeExpanderStats(in_Node, problem);
    }

    void ParseXML(XMLNode& in_Node) {
      cout << "BandsStats::ParseXML()" << endl;

      input_map_filename = in_Node.Read(string("input_map_filename"), true, string(""),string("input .map filename"));
      ideal_map_filename = in_Node.Read(string("ideal_map_filename"), true, string(""),string("all-pairs .map filename"));
      interval = in_Node.Read(string("interval"), false, 100, 1, 1000000, string("interval for stats"));
      k = in_Node.Read(string("k"), true, 0, 0, 10000, string("k-value for calculation"));
      dist = in_Node.Read(string("dist"), true, 0.0, 0.0, 1000.0, string("dist for calculation"));
      out_filename_dist = in_Node.Read(string("out_filename_dist"), true, string(""),string("output filename - dist of k-th closest node"));
      out_filename_num_neighbors = in_Node.Read(string("out_filename_num_neighbors"), true, string(""),string("output filename - num neighbors inside dist"));
      compute_dist_neighbor = in_Node.Read(string("compute_dist_neighbor"), false, false,
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

      stapl::sequential::vector_property_map< GRAPH,size_t > cmap;

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
      stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
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
      stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
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
      stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
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
      /* with the useEdgeWeight flag set to true, this version of  diameter
         computation from STAPL graph algo returns diameter computed from actual
         edge weight. If the flag is set to false, we use the number of edges
         along the path to determine the diameter*/
      double diameter = stapl::sequential::diameter(*(rmp.m_pRoadmap), CCStats[0].second);
      return diameter;
    }


    void storeSameCCInfo(unionStats& stats, Roadmap<CfgType,WeightType>& rmp){
      vector< pair<size_t, VID> > ccstats;
      stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
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
      nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod("BFNF");
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

          nfptr->KClosest(rmp, (*vitr).descriptor(), k, neighbors.begin());
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

    virtual void Print(ostream& out_os) const {}
    virtual void operator()() { }
    virtual void Initialize(){}
    virtual void Run() {
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

      GetMPProblem()->SetRoadmap(new Roadmap<CfgType,WeightType>(ideal_rmp));
      ideal_rmp.SetEnvironment(GetMPProblem()->GetEnvironment());
      input_rmp.SetEnvironment(GetMPProblem()->GetEnvironment());
      //cout<<"printing all pairs"<<endl;
      printAllPairs(ideal_rmp, input_rmp);

      exit(-1);
      //(*ExpanderStatsClass)();
    }
    virtual void Finalize(){}


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
          shared_ptr<DistanceMetricMethod> dmm = this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod("")->GetDMMethod();
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

    ////////////////////////////////////////////////////////////////////////////////
    /// @ingroup MotionPlanningStrategyUtils
    /// @ingroup DeadCode
    /// @brief TODO Dead Code
    ///
    /// TODO
    /// @todo Dead code. Figure out what to do with this.
    ////////////////////////////////////////////////////////////////////////////////
    class EESContainer : public MPSMContainer {
      public:
        EESContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
        vector<string> files;
        string outfile;
        int interval;
        int queries;
        double resolution;
        double epsilon;
        double mu;
        MPSMContainer parent;

    };



    ////////////////////////////////////////////////////////////////////////////////
    /// @ingroup MotionPlanningStrategies
    /// @ingroup DeadCode
    /// @brief TODO Dead Code
    ///
    /// TODO
    /// @todo Dead code. Figure out what to do with this.
    ////////////////////////////////////////////////////////////////////////////////
    class EdgeExpanderStats : public MPStrategyMethod {
      private:
        vector<string> files;
        string outfile;
        int interval;
        int queries;
        double resolution;
        double epsilon;
        double mu;
        typedef pair<int,vector<float> > expanderStats;

      public:
        int mpl;
        typedef RoadmapGraph<CfgType, WeightType>::VID VID;
        EdgeExpanderStats(EESContainer cont) : MPStrategyMethod(cont) {
          files = cont.files;;
          outfile = cont.outfile;
          interval = cont.interval;
          queries = cont.queries;
          resolution = cont.resolution;
          epsilon = cont.epsilon;
          mu = cont.mu;

        }
        EdgeExpanderStats(XMLNode& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
          ParseXML(in_Node);
        }

        void ParseXML(XMLNode& in_Node) {
          cout << "edgeExpanderStats::ParseXML()" << endl;
          interval = 100;
          queries=2;
          outfile="outfile.stats";
          outfile = in_Node.Read(string("filename"), true, string(""),string("Filename"));

          interval = in_Node.Read(string("interval"), true, 100, 1, 1000000, string("Printout Interval"));
          cout<<"interval="<<interval<<endl;
          mu=.2;

          mu = in_Node.Read("mu", false, double(0.0),double(0.0), double(0.5),"Mu paremeter for expander metrics");
          cout<<"Mu ="<<mu<<endl;
          epsilon = .9;
          epsilon = in_Node.Read("epsilon", false, double(0.0),double(0.0), double(5),"Epsilon paremeter for expander metrics");
          cout<<"Epsilon ="<<epsilon<<endl;
          resolution = .1;
          resolution = in_Node.Read("resolution", false, double(0.0),double(0.0), double(5),"Resolution");
          cout<<"resolution ="<<resolution<<endl;
          mpl=10;
          mpl = in_Node.Read("max_path_length", false, int(0),int(0), int(1000),"max path length");
          cout<<"max_path_length ="<<mpl<<endl;
          XMLNode::childiterator citr;
          for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr){
            if(citr->getName() == "file"){
              string filename = citr->Read(string("filename"), true, string(""),string("Node Compare Method"));
              cout<<"filename="<<filename<<endl;
              files.push_back(filename);
            } else{
              citr->warnUnknownNode();
            }
          }
        }

        long scaleFree(Roadmap<CfgType,WeightType> &rmp, vector<VID> &vertices){
          int scaleFreeMetric=0;
          for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
            vector<VID> adj_verts;

            rmp.m_pRoadmap->get_successors(*iter, adj_verts);
            for(vector<VID>::iterator iter2 = adj_verts.begin(); iter2!= adj_verts.end(); iter2++) {
              vector<VID> adj_verts2;
              rmp.m_pRoadmap->get_successors(*iter, adj_verts2);
              scaleFreeMetric+=adj_verts.size()*adj_verts2.size();
            }

          }
          return scaleFreeMetric;
        }


        //t,m,M,N = paremeters from randomized algorithm
        //rmp=roadmap
        //vertices = vertices to be included in calculation
        bool ExpanderTester(double t, double m, double M, double N, Roadmap<CfgType,WeightType> &rmp, vector<VID> &vertices){
          //int X;
          //cout<<"in funct"<<endl;
          for(int i=0; i<N; i++){                              //choose N verts
            //cout<<"in outter for i="<<i<<endl;
            int ranIndex = rand()%vertices.size();
            VID v = vertices[ranIndex];
            int X=0;
            vector<VID> visited;
            //VID walk_v = v;
            for(int j=0; j<m; j++){                            //preform m random walks
              //cout<<"in midd for j="<<j<<endl;
              VID walk_v = v;
              for(int k=0; k<t; k++){                          //random walk
                //cout<<"in inner for k="<<k<<endl;
                vector<VID> succ;
                rmp.m_pRoadmap->get_successors(walk_v, succ);
                //cout<<"getting rand succ "<<succ.size()<<endl;
                if(succ.size()>0){
                  int ranSuccIndex = rand()%succ.size();
                  //cout<<"setting walk"<<endl;
                  walk_v=succ[ranSuccIndex];
                  //cout<<"walk_v="<<walk_v<<endl;
                }
              }
              //cout<<"after loop"<<endl;

              //cout<<"before if"<<endl;
              if(find(visited.begin(), visited.end(), walk_v)==visited.end()){
                visited.push_back(walk_v);
              }else{
                X++;
              }
            }
            cout<<"x ="<<X<<" m= "<<M<<endl;
            if(X>M)
              return false;
          }
          return true;
        }

        double vertex_expansion(Roadmap<CfgType,WeightType> &rmp, vector<VID> &vertices, int d){
          double alpha_top=1;
          double alpha_bottom=0;
          /*
             int d=0;
             for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
             vector<VID> succ;
             rmp.m_pRoadmap->get_successors(*iter, succ);
             d=max(d,succ.size());
             }
             */
          cout<<"d="<<d<<endl;
          while(alpha_top-alpha_bottom>resolution){
            cout<<"in while"<<endl;
            double alpha=(alpha_top+alpha_bottom)/2;
            cout<<"alpha"<<alpha<<endl;
            double t = 16*pow((double)d,(int)2)*log(vertices.size())/(double)pow((double)alpha,(int)2);
            cout<<"t="<<t<<endl;
            double m = pow(vertices.size(),.5+mu);
            cout<<"m="<<m<<endl;
            double M = pow(vertices.size(),2*mu)/2+pow(vertices.size(),7*mu/4)/128;
            cout<<"M="<<M<<endl;
            double N =300/epsilon;
            cout<<"N="<<N<<endl;
            cout<<"before"<<endl;
            if(ExpanderTester(t, m, M, N, rmp, vertices)){
              alpha_bottom=alpha;
            }else{
              alpha_top=alpha;
            }
            cout<<"alpha="<<alpha_top<<" "<<alpha_bottom<<endl;
          }
          cout<<(alpha_top+alpha_bottom)/2;
          //vector<VID> vertices;
          //rmp.m_pRoadmap->GetVerticesVID(vertices);
          //for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
          //compute stuff here
          //}
          return (alpha_top+alpha_bottom)/2;
        }

        double edge_expansion(Roadmap<CfgType,WeightType> rmp, int d){

          return 0;
        }


        vector<int> hop_graph_vertex(Roadmap<CfgType,WeightType> &rmp, VID v){
          vector<int> hop_graph;
          hop_graph.push_back(0);
          //hop_graph.resize(mpl,0);
          vector<VID> visited;
          visited.push_back(v);
          for(int i=1; i<mpl; i++){
            vector<VID> tmp = visited;
            for(vector<VID>::iterator iter=tmp.begin(); iter<tmp.end(); iter++){
              vector<VID> succ;
              rmp.m_pRoadmap->get_successors(*iter, succ);
              for(vector<VID>::iterator iter2=succ.begin(); iter2<succ.end(); iter2++){
                if(find(visited.begin(),visited.end(),*iter2)==visited.end()){
                  visited.push_back(*iter2);
                }
              }
            }
            hop_graph.push_back(visited.size()-1);  //not counting self
          }
          return hop_graph;
        }

        vector<int> hop_graph(Roadmap<CfgType,WeightType> &rmp,vector<VID> &vertices){
          vector<int> sum;
          sum.resize(mpl,0);
          for(vector<VID>::iterator iter=vertices.begin(); iter<vertices.end(); iter++){
            vector<int> hop_graph_v=hop_graph_vertex(rmp,*iter);
            for(int i=0;i<mpl;i++){
              sum[i] += hop_graph_v[i];
            }
          }
          for(int i=0;i<mpl;i++){
            sum[i]/=2;
          }
          return sum;
        }


        vector<int> triangleParticipation(Roadmap<CfgType,WeightType> &rmp,vector<VID> &vertices, int d){
          vector<int> participation;
          participation.resize(d*(d-1)/2,0);
          for(vector<VID>::iterator iter=vertices.begin(); iter<vertices.end(); iter++){
            vector<VID> succ;
            succ.reserve(d);
            rmp.m_pRoadmap->get_successors(*iter, succ);
            int sum=0;
            for(vector<VID>::iterator iter2=succ.begin(); iter2<succ.end(); iter2++){
              vector<VID> succ_neighbor;
              succ_neighbor.reserve(d);
              rmp.m_pRoadmap->get_successors(*iter2, succ_neighbor);
              for(vector<VID>::iterator iter3=succ_neighbor.begin(); iter3<succ_neighbor.end(); iter3++){
                if(find(succ.begin(),succ.end(),*iter3)!=succ.end() && *iter2<*iter3) {sum++;}
              }
            }
            participation[sum]++;
          }
          int return_size=0;
          for(size_t i=0; i<participation.size(); i++){
            if(participation[i]>0){
              return_size=i+1;
            }
          }
          participation.resize(return_size);
          return participation;
        }

        vector<int> hop_graph(Roadmap<CfgType,WeightType> &rmp){
          // vector<VID> vertices(rmp.m_pRoadmap->begin(),rmp.m_pRoadmap->end());
          vector<VID> vertices;
          // vector<VID> vertices(rmp.m_pRoadmap.size());
          return hop_graph(rmp, vertices);
        }


        vector<int> triangleParticipation(Roadmap<CfgType,WeightType> &rmp){
          // vector<VID> vertices(rmp.m_pRoadmap->begin(),rmp.m_pRoadmap->end());
          vector<VID> vertices;
          int d=0;
          for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
            vector<VID> succ;
            rmp.m_pRoadmap->get_successors(*iter, succ);
            d=max(d,(int)succ.size());
          }
          return triangleParticipation(rmp,vertices,d);
        }




        void print_expansion_properties(Roadmap<CfgType,WeightType> &rmp, int _interval,ofstream *myofstream){
          // vector<VID> vertices(rmp.m_pRoadmap->begin(),rmp.m_pRoadmap->end());
          vector<VID> vertices;
          int d=0;
          for(vector<VID>::iterator iter = vertices.begin(); iter!= vertices.end(); iter++) {
            vector<VID> succ;
            rmp.m_pRoadmap->get_successors(*iter, succ);
            d=max(d,(int)succ.size());
          }
          //rmp.m_pRoadmap->GetVerticesVID(vertices);
          //vector<expanderStats> stats;
          vector<int> intervals;
          vector<long> scaleFreeStats;
          vector<double> vertexExpander;
          vector<vector<int> > hop_graph_stats;
          vector<vector<int> > triangleParticipation_stats;
          while((int)vertices.size()!=queries && (int)vertices.size()>0){
            if(((int)vertices.size()-queries) % _interval == 0){
              intervals.push_back((int)vertices.size()-queries);
              long sf= scaleFree(rmp,vertices);
              scaleFreeStats.push_back(sf);

              //vertex expansion
              vector< pair<size_t, VID> > ccstats;
              stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
              vector <vector <VID> > ccs;
              get_cc_stats(*(rmp.m_pRoadmap), cmap, ccstats, ccs);
              vector<VID> cc1;
              //get_cc(*(rmp.m_pRoadmap), cmap, ccstats.front().second, cc1);
              //vertexExpander.push_back(vertex_expansion(rmp, ccs[0]),d);  //change to do for largest
              hop_graph_stats.push_back(hop_graph(rmp,vertices));
              triangleParticipation_stats.push_back(triangleParticipation(rmp,vertices,d));
            }
            rmp.m_pRoadmap->delete_vertex(vertices.back());
            vertices.pop_back();
          }
          *myofstream<<"n scale free ";
          for(int i=1; i<mpl; i++){
            *myofstream<<"hop_graph_"<<i<<" ";
          }
          //cout<<"triangle_participation "<<endl;
          vector<int> intervalsTriangle=intervals;
          while(!intervals.empty()){
            *myofstream<<intervals.back()<<" "<<scaleFreeStats.back()<<" ";
            for(int i=1; i<mpl; i++){
              *myofstream<<hop_graph_stats.back()[i]<<" ";
            }
            *myofstream<<endl;
            intervals.pop_back();
            scaleFreeStats.pop_back();
            hop_graph_stats.pop_back();
          }
          *myofstream<<"interval triangle_participation "<<endl;
          while(!intervalsTriangle.empty()){
            *myofstream<<intervalsTriangle.back();
            for(size_t i=1; i<triangleParticipation_stats.back().size(); i++){
              *myofstream<<" "<<triangleParticipation_stats.back()[i];
            }
            *myofstream<<endl;
            intervalsTriangle.pop_back();
            triangleParticipation_stats.pop_back();
          }
        }

        virtual void Print(ostream& out_os) const {}

        virtual void Initialize(){}
        virtual void Run(){
          cout<<"*************in operator()***********************"<<endl;
          if(files.size()==0){
            cout<<"no files"<<endl;
          }
          ofstream myofstream(outfile.c_str());

          for(vector<string>::iterator iter = files.begin(); iter!=files.end(); iter++){
            cout<<"loading "<<iter->c_str()<<endl;
            Roadmap<CfgType,WeightType> rmp;
            rmp.ReadRoadmapGRAPHONLY(iter->c_str());
            rmp.SetEnvironment(GetMPProblem()->GetEnvironment());
            cout<<"computing stats for "<<iter->c_str()<<endl;
            myofstream<<iter->c_str()<<endl;
            print_expansion_properties(rmp,interval,&myofstream);

          }
          cout<<"returning"<<endl;
          myofstream.close();

        }
        virtual void Finalize(){}

    };

    ////////////////////////////////////////////////////////////////////////////////
    /// @ingroup MotionPlanningStrategyUtils
    /// @ingroup DeadCode
    /// @brief TODO Dead Code
    ///
    /// TODO
    /// @todo Dead code. Figure out what to do with this.
    ////////////////////////////////////////////////////////////////////////////////
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

    ////////////////////////////////////////////////////////////////////////////////
    /// @ingroup MotionPlanningStrategies
    /// @ingroup DeadCode
    /// @brief TODO Dead Code
    ///
    /// TODO
    /// @todo Dead code. Figure out what to do with this.
    ////////////////////////////////////////////////////////////////////////////////
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
        RoadmapTimingStats(XMLNode& in_Node, MPProblem* problem) : MPStrategyMethod(in_Node, problem){
          ParseXML(in_Node);
        }

        void ParseXML(XMLNode& in_Node) {
          cout << "TimingStats::ParseXML()" << endl;
          interval = 100;
          queries=2;
          outfile="outfile.stats";
          k=10;
          outfile = in_Node.Read(string("outfile"), true, string(""),string("Outfile"));
          cout<<"outfile="<<outfile<<endl;

          interval = in_Node.Read(string("interval"), true, 100, 1, 1000000, string("Printout Interval"));
          cout<<"interval="<<interval<<endl;

          queries = in_Node.Read(string("queries"), true, 100, 1, 1000000, string("queries"));
          cout<<"queries="<<queries<<endl;

          k = in_Node.Read(string("k"), true, 100, 1, 1000000, string("l"));
          cout<<"k="<<k<<endl;

          getOnlyAt = in_Node.Read(string("getOnlyAt"), true, 100, -1, 1000000, string("getOnlyAt"));
          cout<<"getOnlyAt="<<getOnlyAt<<endl;

          XMLNode::childiterator citr;
          for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr){
            if(citr->getName() == "file"){
              string filename = citr->Read(string("filename"), true, string(""),string("File Name"));
              cout<<"filename="<<filename<<endl;
              files.push_back(filename);
            }else if(citr->getName() == "nnMethod"){
              string method_name = citr->Read(string("method_name"), true, string(""),string("Method Name"));
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

        virtual void Print(ostream& out_os) const {}
        virtual void Initialize(){}
        virtual void Run(){
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
        virtual void Finalize(){}

    };

#endif
