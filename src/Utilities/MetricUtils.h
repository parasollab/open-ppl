#ifndef METRICUTILS_H_
#define METRICUTILS_H_

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <iomanip>
#include <tuple>
using namespace std;

#include "Environment/Environment.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MetricUtils
/// @brief Timing utility.
///
/// This class is used to measure the running time between @c StartClock and
/// @c StopClock. Client side could provide clock name. Output functions are
/// provided.
////////////////////////////////////////////////////////////////////////////////
class ClockClass {
  public:

    ClockClass();
    ~ClockClass();

    void SetName(string _name);

    ///Set every thing to zero
    void ClearClock();

    ///Start the clock and the name is identity of this clock.
    void StartClock();

    ///Stop the clock and calculate the total running time.
    void StopClock();

    ///Call StopClock and PrintClock.
    void StopPrintClock(ostream& _os);

    /**Output the clock name given in StartClock and running time accosited with this name
     *to the standard output.
     */
    void PrintClock(ostream& _os);

    ///Get how many seconds of running time are.
    double GetSeconds();

    ///Get how many 1e-6 seconds of running time are.
    int GetUSeconds();

  private:
    int m_sTime{0}, m_uTime{0};
    int m_suTime{0}, m_uuTime{0};
    string m_clockName;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MetricUtils
/// @brief Statistics tracker.
///
/// The StatClass is a storage hub of all statistics to be tracked in PMPL,
/// including but not limited to timing, success/fail attempts,
/// collision detection calls.
////////////////////////////////////////////////////////////////////////////////
class StatClass {

  public:
    StatClass();
    ~StatClass();

    void ClearStats();

    int IncNumCollDetCalls(string _cdName, const string& _callName);
    unsigned long int GetIsCollTotal() { return m_isCollTotal; }
    void IncCfgIsColl(const string& _callName);

    int IncLPConnections(string _lpName , int _incr=1);
    int IncLPAttempts(string _lpName, int _incr=1 );
    int IncLPCollDetCalls(string _lpName, int _incr=1);

    static const int ALL;
    template<class RoadmapType>
      void PrintAllStats(ostream& _os, RoadmapType* _rmap);
    template<class RoadmapType>
      void PrintAllStats(ostream& _os, RoadmapType* _rmap, int _numCCs);

    template<class RoadmapType>
      void PrintDataLine(ostream&, RoadmapType*, int _showColumnHeaders=0);

    template<class RoadmapType, class DistanceMetricPointer>
      void ComputeIntraCCFeatures(RoadmapType* _rdmp, Environment* _env, DistanceMetricPointer _dm);

    template<class MPProblemType, class RoadmapType>
      void ComputeInterCCFeatures(MPProblemType* _problem, RoadmapType* _rdmp, string _nfMethod, string _dmMethod);

    void PrintFeatures(ostream& _os);
    void IncNodesGenerated(string _samplerName, size_t _incr=1);
    void IncNodesAttempted(string _samplerName, size_t _incr=1);

    //Clock Accessors
    void ClearClock(string _name);
    void StartClock(string _name);
    void StopClock(string _name);
    void StopPrintClock(string _name, ostream& _os);
    void PrintClock(string _name, ostream& _os);
    double GetSeconds(string _name);
    int GetUSeconds(string _name);

    // Graph Operation Statistics Accessors/Modifiers
    int GetGOStat(string _s) {return m_goStats[_s];}
    void SetGOStat(string _s, int _v) {m_goStats[_s]=_v;}
    void IncGOStat(string _s, int _v = 1) {m_goStats[_s]+=_v;}

    // RRT Statistics Accessors/Modifiers
    int GetRRTStat(string _s) {return m_rrtStats[_s];}
    void SetRRTStat(string _s, int _v) {m_rrtStats[_s]=_v;}
    void IncRRTStat(string _s, int _v = 1) {m_rrtStats[_s]+=_v;}

    //Local Planner Statistics Accessors/Modifiers
    double GetLPStat(string _s){return m_lpStats[_s];}
    void SetLPStat(string _s, double _v) {m_lpStats[_s]=_v;}
    void IncLPStat(string _s, double _v = 1.0) {m_lpStats[_s]+=_v;}

    //Neighborhood Finder Statistics Accessors/Modifiers
    double GetNFStat(string _s){return m_nfStats[_s];}
    void SetNFStat(string _s, double _v) {m_nfStats[_s]=_v;}
    void IncNFStat(string _s, double _v = 1.0) {m_nfStats[_s]+=_v;}

    //histories of numbers
    vector<double>& GetHistory(string _s){return m_histories[_s];}
    void AddToHistory(string _s, double _v){m_histories[_s].push_back(_v);}
    void SetAuxDest(string _s) {m_auxFileDest = _s;}

    //help
    template<class GraphType>
      void DisplayCCStats(ostream& _os, GraphType&);

    // m_lpInfo represents information about the Local Planners, referenced by
    // name
    // m_lpInfo.first is the name of the Local Planner
    // m_lpInfo.second.get<0>() is the # of LP attempts
    // m_lpInfo.second.get<1>() is the # of LP connections (successes)
    // m_lpInfo.second.get<2>() is the # of LP collision detection calls
    map<string, tuple<unsigned long int, unsigned long int, unsigned long int> > m_lpInfo;
    map<string, unsigned long int> m_collDetCountByName;

    map<string, ClockClass> m_clockMap;

    ///IsColl simply counts the number of times a Cfg is tested for Collision.
    ///\see Cfg::isCollision
    std::map<string, unsigned long int> m_isCollByName;
    unsigned long int m_isCollTotal;

    //features

    //m_samplerInfo represents sampler nodes attempted and generated
    //  map<string, pair<int, int> > represents a mapping between the sampler name
    //  and first the # of attempted samples, then the number of generated samples
    //  that is, pair.first = attempts, pair.second = generated (successful attempts)
    map<string, pair<unsigned long int, unsigned long int> > m_samplerInfo;

    int m_ccNumber;

    //Intra-CC features:
    double m_avgMinIntraCCDist;
    double m_avgMaxIntraCCDist;
    double m_avgMeanIntraCCDist;
    double m_avgSigmaIntraCCDist;

    double m_avgMinIntraCCEdge;
    double m_avgMaxIntraCCEdge;
    double m_avgMeanIntraCCEdge;
    double m_avgSigmaIntraCCEdge;

    double m_avgMaxIntraCCDistToCm;
    double m_avgMinIntraCCDistToCm;
    double m_avgMeanIntraCCDistToCm;
    double m_avgSigmaIntraCCDistToCm;

    //Inter-CC features:
    double m_maxInterCCDist;
    double m_avgInterCCDist;
    double m_sigmaInterCCDist;
    double m_minInterCCDist;

    double m_maxCCSize;
    double m_minCCSize;
    double m_avgCCSize;
    double m_sigmaCCSize;

  protected:
    map<string, unsigned long int> m_numCollDetCalls;

  private:
    //LP Statistics
    map<string, int> m_goStats, m_rrtStats;
    map<string, double> m_lpStats, m_nfStats;
    map<string, vector<double> > m_histories;
    string m_auxFileDest;

};

//definitions of templated functions
template<class RoadmapType>
void
StatClass::PrintAllStats(ostream& _os, RoadmapType* _rmap) {
  PrintAllStats(_os, _rmap, ALL);
}

template<class RoadmapType>
void
StatClass::PrintAllStats(ostream& _os, RoadmapType* _rmap, int _numCCs) {
#ifndef _PARALLEL

  //output Sampler Statistics
  //only output if any nodes were attempted to be generated
  if (m_samplerInfo.size()>0) {
    int totalAttempts=0, totalGenerated=0;
    _os << "\n\nSamplers Statistics:\n";
    _os << setw(40) << "Name" << setw(20) << "Attempts" << setw(20) << "Successes\n\n";
    map<string, pair<unsigned long int, unsigned long int> >::iterator nodeIter;
    for(nodeIter=m_samplerInfo.begin();nodeIter!=m_samplerInfo.end();nodeIter++) {
      _os << setw(40) << nodeIter->first << setw(20) << nodeIter->second.first
        << setw(20) << nodeIter->second.second << endl;
      totalAttempts += nodeIter->second.first;
      totalGenerated += nodeIter->second.second;
    }//end for loop

    if(totalAttempts > 0)
      _os << "  Total Sampler Attempts: " << totalAttempts << "\n  Total Succeeded: "
        << totalGenerated << "\n  Success %: "
        << ((double)totalGenerated)/totalAttempts * 100.0 << endl;
  }//end check on attempted nodes generated

  size_t i;
  std::map<string, unsigned long int>::const_iterator iter;

  _os << endl << endl << "Local Planners:" << endl;
  _os << setw(20) << "Name"
    <<setw(15) << "Attempts"
    <<setw(15) << "Connections"
    <<setw(15) << "Coll Det Calls" << endl;

  std::map<string, tuple<unsigned long int, unsigned long int, unsigned long int> >::const_iterator lpIter;
  for(lpIter = m_lpInfo.begin(); lpIter != m_lpInfo.end(); ++lpIter) {
    _os << setw(20) << lpIter->first;
    _os << setw(15) << get<0>(lpIter->second);
    _os << setw(15) << get<1>(lpIter->second);
    _os << setw(15) << get<2>(lpIter->second) << endl;
  }

  //output for graph operation statistics.
  if(m_goStats.size()>0){
    _os<<"\n\n Graph Operation Statistics:\n\n";
    _os<< setw(40) << "Statistic"
      << setw(40) << "Value" << endl << endl;
    typedef map<string, int>::iterator GOSIT;
    for(GOSIT gosit=m_goStats.begin(); gosit!=m_goStats.end(); gosit++){
      _os << setw(40) << gosit->first
        << setw(40) << gosit->second << endl;
    }
  }

  // output for RRT statistics.
  if(m_rrtStats.size()>0){
    _os<<"\n\n RRT Statistics:\n\n";
    _os<< setw(40) << "Statistic"
      << setw(40) << "Value" << endl << endl;
    typedef map<string, int>::iterator RRTSIT;
    for(RRTSIT rrtsit=m_rrtStats.begin(); rrtsit!=m_rrtStats.end(); rrtsit++){
      _os << setw(40) << rrtsit->first
        << setw(40) << rrtsit->second << endl;
    }
  }

  //output for local planner statistics. Only output if map is populated
  if(m_lpStats.size()>0){
    _os<<"\n\n Local Planner Statistics:\n\n";
    _os<< setw(40) << "Statistic"
      << setw(40) << "Value" << endl << endl;
    typedef map<string, double>::iterator LPSIT;
    for(LPSIT lpsit=m_lpStats.begin(); lpsit!=m_lpStats.end(); lpsit++){
      _os << setw(40) << lpsit->first
        << setw(40) << lpsit->second << endl;
    }
  }

  //output for neighborhood finder statistics. Only output if map is populated
  if(m_nfStats.size()>0){
    _os<<"\n\n Neighborhood Finder Statistics:\n\n";
    _os<< setw(40) << "Statistic"
      << setw(40) << "Value" << endl << endl;
    typedef map<string, double>::iterator NFSIT;
    for(NFSIT nfsit=m_nfStats.begin(); nfsit!=m_nfStats.end(); nfsit++){
      _os << setw(40) << nfsit->first
        << setw(40) << nfsit->second << endl;
    }
  }

  //output history statistics
  typedef map<string, vector<double> >::iterator HIT;
  for(HIT hit = m_histories.begin(); hit!=m_histories.end(); hit++){
    ofstream ofs((m_auxFileDest+"."+hit->first+".hist").c_str());
    typedef vector<double>::iterator DIT;
    for(DIT dit = hit->second.begin(); dit!=hit->second.end(); dit++){
      ofs << *dit << endl;
    }
    ofs.close();
  }


  _os << endl << endl;
  _os << "Number of Nodes: " << _rmap->GetGraph()->get_num_vertices() << endl;
  _os << "Number of Edges: " << _rmap->GetGraph()->get_num_edges() << endl;
  /*  Removed by Roger for reasons given below
      _os << "Number of Collision Detection Calls: " << endl;
      for(i=0;i<MaxCD;i++)
      if (strcmp(CDNameList[i],"empty")!=0)
      _os << setw(20) << CDNameList[i]
      << setw(15) << NumCollDetCalls[i] << endl;
   */

  _os << endl;

  DisplayCCStats(_os, *_rmap->GetGraph());

  ///Below removed b/c it counts Coll Detection too fine grained.  We have decided
  ///to only keep Total times Cfg::isCollision is called.  This makes the 'price' for a
  ///free node the same as a collision node.  Will be added back after collision detection
  ///counting is properly fixed     --Roger 9/17/2005
  /*
     _os << endl << endl << "Collision Detection Exact Counts:" << endl;
     for (i=0, iter=CollDetCountByName.begin(); iter != CollDetCountByName.end(); iter++, i++)
     {
     total+=iter->second;
     _os << i << ") " << iter->second << " ";
     _os << flush;
     printf("%s\n", iter->first.data()); //K2 does not have the << operator defined for string
     fflush(stdout);
     }

     _os << "total " << total << endl;
   */

  _os << endl << endl << "Cfg::isCollision() Exact Counts:" << endl;
  for (i=0, iter=m_isCollByName.begin(); iter != m_isCollByName.end(); iter++, i++) {
    _os << i << ") " << iter->second << " " << iter->first << endl;;
  }
  _os << "Total Cfg::isCollision() = " << m_isCollTotal << endl << endl;

  //Output Clock Statistics.
  _os << "Clocks " << endl << endl;
  _os << setw(40) << "Name" << setw(40) << "Time (Seconds)" << endl << endl;
  typedef map<string, ClockClass>::iterator CIT;
  for(CIT cit =m_clockMap.begin(); cit!= m_clockMap.end(); cit++) {
    _os << setw(40) << cit->first << setw(40) << cit->second.GetSeconds() << endl;
  }

#endif

}

template<class RoadmapType>
void StatClass::
PrintDataLine(ostream& _myostream, RoadmapType* _rmap, int _showColumnHeaders) {
#ifndef _PARALLEL
  // Default is to NOT print out column headers
  if (_showColumnHeaders) {
    _myostream <<"\nV  E #CC 1 2 3 4 5 6 7 8 9 10 #iso CD  LPattSum LPcdSum\n";
  }//endif

  _myostream << _rmap->GetGraph()->get_num_vertices() << " ";
  _myostream << _rmap->GetGraph()->get_num_edges()   << " ";

  typedef typename RoadmapType::GraphType GraphType;
  typedef typename GraphType::vertex_descriptor VID;
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  vector< pair<size_t,VID> > ccStats;
  get_cc_stats(*(_rmap->GetGraph()), cMap, ccStats);
  _myostream << ccStats.size() << "  ";

  for (size_t i=0;i<10;++i)
    if (ccStats.size() > i)
      _myostream << ccStats[i].first << " ";
    else
      _myostream << 0                << " ";

  int sumIsolatedNodes=0;
  for (size_t i=0;i<ccStats.size();++i)
    if (ccStats[i].first == 1) ++sumIsolatedNodes;
  _myostream << sumIsolatedNodes << " ";

  int sumCDcalls=0;
  for(map<string, unsigned long int>::const_iterator M = m_numCollDetCalls.begin(); M != m_numCollDetCalls.end(); ++M)
    sumCDcalls += M->second;
  _myostream << sumCDcalls << " ";

  int sumAtt=0;
  int sumCD =0;

  map<string, tuple<unsigned long int, unsigned long int, unsigned long int> >::const_iterator iter1;
  for(iter1 = m_lpInfo.begin(); iter1 != m_lpInfo.end(); ++iter1) {
    sumAtt += get<0>(iter1->second);
    sumCD += get<2>(iter1->second);
  }

  _myostream << sumAtt << " ";
  _myostream << sumCD  << " ";
  ccStats.clear();
#endif
}


// Compute intra-connected-component statistics
template<class RoadmapType, class DistanceMetricPointer>
void
StatClass::ComputeIntraCCFeatures(RoadmapType* _rdmp, Environment* _env, DistanceMetricPointer _dm) {
#ifndef _PARALLEL
  m_avgMinIntraCCDist = 0;
  m_avgMaxIntraCCDist = 0;
  m_avgMeanIntraCCDist = 0;
  m_avgSigmaIntraCCDist = 0;

  m_avgMinIntraCCEdge = 0;
  m_avgMaxIntraCCEdge = 0;
  m_avgMeanIntraCCEdge = 0;
  m_avgSigmaIntraCCEdge = 0;

  m_avgMeanIntraCCDistToCm = 0;

  typedef typename RoadmapType::CfgType CfgType;
  typedef typename RoadmapType::GraphType GraphType;
  typedef typename GraphType::vertex_descriptor VID;
  vector< pair<size_t,VID> > ccs; //vector of connected components in the roadmap
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  stapl::sequential::get_cc_stats(*(_rdmp->GetGraph()),cMap, ccs);//fill ccs from the roadmap
  cout << "in intra ccs portion" << endl;

  typename vector< pair<size_t, VID> >::iterator cci; // cci is CC[i] hereafter
  for (cci = ccs.begin(); cci < ccs.end(); cci++) {
    vector<VID> cciCfgsAux; //use this auxiliary vec to convert between VID and CfgType
    vector<CfgType> cciCfgs; //configurations in cci
    VID cci_tmp = (*(_rdmp->GetGraph()->find_vertex(cci->second))).descriptor();//cci->second: vertex ID of first node in cci
    cMap.reset();
    get_cc(*(_rdmp->GetGraph()), cMap, cci_tmp, cciCfgsAux); //fill cci_cfgs
    for(typename vector<VID>::iterator itr = cciCfgsAux.begin(); itr!=cciCfgsAux.end(); itr++)
      cciCfgs.push_back((*(_rdmp->GetGraph()->find_vertex(*itr))).property());

    if (cciCfgs.size() > 1) {
      //compute shortest, longest, mean, and std-dev (sigma) distances
      //between nodes for cci

      double cciMinIntraCCDist = 0;
      double cciMaxIntraCCDist = 0;
      double cciMeanIntraCCDist = 0;
      double nPairs = 0;
      typename vector<CfgType>::iterator cci_i;
      typename vector<CfgType>::iterator cci_j;
      for (cci_i=cciCfgs.begin(); cci_i < cciCfgs.end(); cci_i++) {
        for (cci_j = cci_i+1; cci_j < cciCfgs.end(); cci_j++) {
          double c_dist = _dm->Distance(_env, *cci_i, *cci_j);
          if (c_dist > cciMaxIntraCCDist)
            cciMaxIntraCCDist = c_dist;
          if (cciMinIntraCCDist == 0 || c_dist < cciMinIntraCCDist)
            cciMinIntraCCDist = c_dist;
          cciMeanIntraCCDist += c_dist;
          nPairs++;
        }
      }
      if (nPairs > 0)
        cciMeanIntraCCDist /= nPairs;

      double cciSigmaIntraCCDist = 0;
      for (cci_i=cciCfgs.begin(); cci_i < cciCfgs.end(); cci_i++) {
        for (cci_j = cci_i+1; cci_j < cciCfgs.end(); cci_j++) {
          cciSigmaIntraCCDist += pow(_dm->Distance(_env, *cci_i, *cci_j)-cciMeanIntraCCDist, 2);
        }
      }
      if (nPairs > 1)
        cciSigmaIntraCCDist /= (nPairs-1);
      cciSigmaIntraCCDist = sqrt(cciSigmaIntraCCDist);

      //compute shortest, longest, mean and std-dev (sigma) edge sizes
      //in the component
      vector<pair<VID,VID> > cciEdges;
      cMap.reset();
      get_cc_edges(*(_rdmp->GetGraph()),cMap, cciEdges, cci->second);
      double cciMaxIntraCCEdge = 0;
      double cciMinIntraCCEdge = 0;
      double cciMeanIntraCCEdge = 0;
      double cciSigmaIntraCCEdge = 0;
      for (typename vector< pair<VID,VID> >::iterator eIter = cciEdges.begin(); eIter < cciEdges.end(); eIter++) {
        CfgType e_v1 = (*(_rdmp->GetGraph()->find_vertex(eIter->first))).property();
        CfgType e_v2 = (*(_rdmp->GetGraph()->find_vertex(eIter->second))).property();
        double eDist = _dm->Distance(_env, e_v1, e_v2);
        if (eDist > cciMaxIntraCCEdge)
          cciMaxIntraCCEdge = eDist;
        if (cciMinIntraCCEdge == 0 || eDist < cciMinIntraCCEdge)
          cciMinIntraCCEdge = eDist;
        cciMeanIntraCCEdge += eDist;
      }
      if (cciEdges.size() > 0)
        cciMeanIntraCCEdge /= cciEdges.size();
      for (typename vector< pair<VID,VID> >::iterator eIter = cciEdges.begin(); eIter < cciEdges.end(); eIter++) {
        CfgType e_v1 = (*(_rdmp->GetGraph()->find_vertex(eIter->first))).property();
        CfgType e_v2 = (*(_rdmp->GetGraph()->find_vertex(eIter->second))).property();
        double eDist = _dm->Distance(_env, e_v1, e_v2);
        cciSigmaIntraCCEdge += pow(eDist-cciMeanIntraCCEdge, 2);
      }
      if (cciEdges.size() > 1)
        cciSigmaIntraCCEdge /= cciEdges.size() - 1;
      cciSigmaIntraCCEdge = sqrt(cciSigmaIntraCCEdge);

      //get center of mass of all Cfgs in current CC
      CfgType center_of_mass = cciCfgs[0];
      for (size_t j = 1; j < cciCfgs.size(); ++j)
        center_of_mass.add(center_of_mass, cciCfgs[j]);
      center_of_mass.divide(center_of_mass, cciCfgs.size());

      //compute average distance of the nodes to the center of mass
      double cciMinIntraCCDistToCm = 0;
      double cciMaxIntraCCDistToCm = 0;
      double cciMeanIntraCCDistToCm = 0;
      double cciSigmaIntraCCDistToCm = 0;
      for (size_t j = 0; j < cciCfgs.size(); ++j) {
        CfgType tmp = cciCfgs[j];
        double eDist = _dm->Distance(_env, center_of_mass, tmp);
        if (eDist > cciMaxIntraCCDistToCm)
          cciMaxIntraCCDistToCm = eDist;
        if (cciMinIntraCCDistToCm == 0 || eDist < cciMinIntraCCDistToCm)
          cciMinIntraCCDistToCm = eDist;
        cciMeanIntraCCDistToCm += eDist;
      }
      if (cciCfgs.size() > 0 )
        cciMeanIntraCCDistToCm /= cciCfgs.size();
      for (size_t j = 0; j < cciCfgs.size(); ++j) {
        CfgType tmp = cciCfgs[j];
        double eDist = _dm->Distance(_env, center_of_mass, tmp);
        cciSigmaIntraCCDistToCm += pow(eDist-cciMeanIntraCCDistToCm, 2);
      }
      if (cciCfgs.size() > 1)
        cciSigmaIntraCCDistToCm /= cciCfgs.size() - 1;
      cciSigmaIntraCCDistToCm = sqrt(cciSigmaIntraCCDistToCm);

      m_avgMinIntraCCDist += cciMinIntraCCDist;
      m_avgMaxIntraCCDist += cciMaxIntraCCDist;
      m_avgMeanIntraCCDist += cciMeanIntraCCDist;
      m_avgSigmaIntraCCDist += cciSigmaIntraCCDist;

      m_avgMinIntraCCEdge += cciMinIntraCCEdge;
      m_avgMaxIntraCCEdge += cciMaxIntraCCEdge;
      m_avgMeanIntraCCEdge += cciMeanIntraCCEdge;
      m_avgSigmaIntraCCEdge += cciSigmaIntraCCEdge;

      m_avgMaxIntraCCDistToCm += cciMaxIntraCCDistToCm;
      m_avgMinIntraCCDistToCm += cciMinIntraCCDistToCm;
      m_avgMeanIntraCCDistToCm += cciMeanIntraCCDistToCm;
      m_avgSigmaIntraCCDistToCm += cciSigmaIntraCCDistToCm;
    }
  }
  if (ccs.size() > 0) {
    m_avgMinIntraCCDist /= ccs.size();
    m_avgMaxIntraCCDist /= ccs.size();
    m_avgMeanIntraCCDist /= ccs.size();
    m_avgSigmaIntraCCDist /= ccs.size();

    m_avgMinIntraCCEdge /= ccs.size();
    m_avgMaxIntraCCEdge /= ccs.size();
    m_avgMeanIntraCCEdge /= ccs.size();
    m_avgSigmaIntraCCEdge /= ccs.size();

    m_avgMaxIntraCCDistToCm /= ccs.size();
    m_avgMinIntraCCDistToCm /= ccs.size();
    m_avgMeanIntraCCDistToCm /= ccs.size();
    m_avgSigmaIntraCCDistToCm /= ccs.size();
  }
#endif
}

// Compute inter-connected-component statistics
// Find Closest nodes between two ccs and find distance
// between the closest pairs of ccs
template<class MPProblemType, class RoadmapType>
void
StatClass::ComputeInterCCFeatures(MPProblemType* _problem, RoadmapType* _rdmp, string _nfMethod, string _dmMethod) {
#ifndef _PARALLEL
  typename MPProblemType::DistanceMetricPointer dm = _problem->GetDistanceMetric(_dmMethod);
  typename MPProblemType::NeighborhoodFinderPointer nf = _problem->GetNeighborhoodFinder(_nfMethod);
  typedef typename MPProblemType::CfgType CfgType;
  typedef typename RoadmapType::GraphType GraphType;
  typedef typename GraphType::vertex_descriptor VID;
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  vector< pair<size_t,VID> > ccs; //connected components in the roadmap
  cout << "in inter ccs portion" << endl;
  get_cc_stats(*(_rdmp->m_pRoadmap), cMap, ccs);//fill ccs

  m_maxInterCCDist = 0.0;
  m_avgInterCCDist = 0.0;
  m_sigmaInterCCDist = 0.0;
  m_minInterCCDist = 0.0;

  int pairsChecked = 0;
  vector<double> minCCDistanceBetweenClosestPairs;
  vector<pair<VID,VID> > ccEdges;
  vector<double> ccSizes;

  m_maxCCSize = 0;
  m_minCCSize = 0;
  m_avgCCSize = 0;
  m_sigmaCCSize = 0;

  m_ccNumber = ccs.size();

  double totalComponentsDist = 0;
  for(typename vector< pair<size_t,VID> >::iterator cce=ccs.begin(); cce < ccs.end(); cce++){
    cMap.reset();
    get_cc_edges(*(_rdmp->GetGraph()),cMap,ccEdges,cce->second);
    double totalSize = 0.0;
    cout << "size of edge list for cc:" << cce->second << " "<< ccEdges.size() << endl;
    for(typename vector< pair<VID,VID> >::iterator ccIter=ccEdges.begin(); ccIter<ccEdges.end();ccIter++) {
      CfgType ccIterA = (*(_rdmp->GetGraph()->find_vertex(ccIter->first))).property();
      CfgType ccIterB = (*(_rdmp->GetGraph()->find_vertex(ccIter->second))).property();
      double dist = dm->Distance(_problem->GetEnvironment(), ccIterA, ccIterB);
      totalSize += dist;
    }
    ccEdges.clear();
    totalSize /= 2;
    ccSizes.push_back(totalSize);
    if(totalSize > m_maxCCSize)
      m_maxCCSize=totalSize;
    if(m_minCCSize == 0 || totalSize < m_minCCSize)
      m_minCCSize=totalSize;
    m_avgCCSize+=totalSize;
    totalComponentsDist+=totalSize;
  }
  m_avgCCSize /= ccs.size();
  m_sigmaCCSize = 0;
  for (size_t j = 0; j < ccSizes.size(); j++)
    m_sigmaCCSize  += pow(ccSizes[j]-m_avgCCSize, 2);
  if (ccSizes.size() > 1)
    m_sigmaCCSize /= (ccSizes.size()-1);
  m_sigmaCCSize = sqrt(m_sigmaCCSize);
  if (totalComponentsDist>0) {
    m_minCCSize/=totalComponentsDist;
    m_maxCCSize/=totalComponentsDist;
    m_avgCCSize/=totalComponentsDist;
    m_sigmaCCSize/=totalComponentsDist;
  }

  typename vector< pair<size_t,VID> >::iterator cci; // cci is CC[i] hereafter
  if(ccs.size()>1)
    for (cci = ccs.begin(); cci < ccs.end(); cci++) {
      typename vector< pair<size_t,VID> >::iterator ccj;//ccj will be the rest of the ccs that cci has not
      //checked against
      ccj = cci;
      //ccj++;
      for(ccj++;ccj<ccs.end();ccj++) {
        vector<VID> cciCfgs; //configurations in cci
        vector<VID> ccjCfgs; //configurations in ccj

        cMap.reset();
        get_cc(*(_rdmp->GetGraph()),cMap, cci->second, cciCfgs); //fill cciCfgs
        cMap.reset();
        get_cc(*(_rdmp->GetGraph()),cMap, ccj->second, ccjCfgs); //fill ccjCfgs

        vector< pair<VID,VID> > pairs;
        nf->KClosestPairs(_rdmp,
            cciCfgs.begin(), cciCfgs.end(), ccjCfgs.begin(), ccjCfgs.end(), 1, back_inserter(pairs));
        double tmpDist = dm->Distance(_problem->GetEnvironment(), (*(_rdmp->GetGraph()->find_vertex(pairs[0].first))).property(),
            (*(_rdmp->GetGraph()->find_vertex(pairs[0].second))).property() );
        if(tmpDist > m_maxInterCCDist)
          m_maxInterCCDist = tmpDist;
        if(m_minInterCCDist == 0.0 || tmpDist < m_minInterCCDist)
          m_minInterCCDist = tmpDist;
        m_avgInterCCDist += tmpDist;
        minCCDistanceBetweenClosestPairs.push_back( tmpDist );
        pairsChecked++;
      }
    }

  if (pairsChecked > 0)
    m_avgInterCCDist /= pairsChecked;
  for (size_t j = 0; j < minCCDistanceBetweenClosestPairs.size(); j++)
    m_sigmaInterCCDist += pow(minCCDistanceBetweenClosestPairs[j]-m_avgInterCCDist , 2);
  if (minCCDistanceBetweenClosestPairs.size() > 1)
    m_sigmaInterCCDist /= minCCDistanceBetweenClosestPairs.size() - 1;
  m_sigmaInterCCDist = sqrt(m_sigmaInterCCDist);
#endif
}

/**Output Connected Component information in graph _G.
 *This method will print out _maxCCprint number of Connected Component,
 *,and size and start vertex of each Connected Component.
 *_g the graph
 *_maxCCprint the number of connected components to print.
 */
template<class GraphType>
void
StatClass::DisplayCCStats(ostream& _os, GraphType& _g)  {
  #ifndef _PARALLEL
  typedef typename GraphType::vertex_descriptor VID;
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;

  vector<pair<size_t, VID> > ccs;
  stapl::sequential::get_cc_stats(_g, cMap, ccs);

  size_t ccnum = 0;
  _os << "\nThere are " << ccs.size() << " connected components:";
  for(typename vector<pair<size_t,VID> >::iterator vi = ccs.begin(); vi != ccs.end(); ++vi) {
    _os << "\nCC[" << ccnum++ << "]: " << vi->first ;
    _os << " (vid=" << size_t(vi->second) << ")";
  }
  #else
    _os << "WARNING: CCs not computed, called sequential implementation of CC stats \n" ;
  #endif
}

#endif
