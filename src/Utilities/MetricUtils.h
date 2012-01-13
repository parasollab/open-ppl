#ifndef METRICUTILS_H_
#define METRICUTILS_H_

#include "Roadmap.h"
#include "DistanceMetricMethod.h"
#include "NeighborhoodFinder.h"
#include <string>

class StatClass {
public:
  StatClass();
  ~StatClass();

  void ClearStats();

  int IncNumCollDetCalls( string _cdName , string *_pCallName = NULL);
  unsigned long int GetIsCollTotal() { return m_isCollTotal; }

  void IncCfgIsColl( std::string *_pCallName = NULL);

  int IncLPConnections( string _lpName , int _incr=1);
  int SetLPConnections( string _lpName, int _connections );
  int IncLPAttempts( string _lpName, int _incr=1 );
  int SetLPAttempts( string _lpName, int _attempts );
  int IncLPCollDetCalls( string _lpName, int _incr=1);

  static const int ALL;
  template <class CFG, class WEIGHT>
  void PrintAllStats(ostream& _os, Roadmap<CFG, WEIGHT> *_rmap);
  template <class CFG, class WEIGHT>
  void PrintAllStats(ostream& _os, Roadmap<CFG, WEIGHT> *_rmap, int _numCCs);

  template <class CFG, class WEIGHT>
  void PrintDataLine(ostream&, Roadmap<CFG, WEIGHT>* , int _showColumnHeaders=0);

  template <class CFG, class WEIGHT>
  void ComputeIntraCCFeatures(Roadmap<CFG,WEIGHT> *_rdmp, shared_ptr<DistanceMetricMethod> _dm);

  template <class CFG, class WEIGHT>
  void ComputeInterCCFeatures(Roadmap<CFG,WEIGHT> *_rdmp, NeighborhoodFinder* _nf, string _nfMethod);
  void PrintFeatures(ostream& _os);
  void IncNodesGenerated();
  void IncNodesAttempted();

  double GetLPStat(string _s){return m_lpStats[_s];}
  void SetLPStat(string _s, double _v) {m_lpStats[_s]=_v;}
  void IncLPStat(string _s, double _v) {m_lpStats[_s]+=_v;}

  vector<double>& GetHistory(string _s){return m_histories[_s];}
  void AddToHistory(string _s, double _v){m_histories[_s].push_back(_v);}
  void SetAuxDest(string _s) {m_auxFileDest = _s;}

  //help
  template <class CFG, class WEIGHT>
  void DisplayCCStats(ostream& _os, RoadmapGraph<CFG, WEIGHT>&, int);

  map<string, unsigned long int> m_lpConnections;
  map<string, unsigned long int> m_lpAttempts;
  map<string, unsigned long int> m_lpCollDetCalls;
  map<string, unsigned long int> m_collDetCountByName;

  ///IsColl simply counts the number of times a Cfg is tested for Collision.
  ///\see Cfg::isCollision
  std::map<string, unsigned long int> m_isCollByName;
  unsigned long int m_isCollTotal;

  //features
  int m_connectionsAttempted;
  int m_connectionsMade;
  int m_nodesAttempted;
  int m_nodesGenerated;
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
  map<string, double> m_lpStats;
  map<string, vector<double> > m_histories;
  string m_auxFileDest;

};

//definitions of templated functions
template <class CFG, class WEIGHT>
void
StatClass::
PrintAllStats( ostream& _os, Roadmap<CFG, WEIGHT>* _rmap) {
  PrintAllStats(_os, _rmap, ALL);
}

template <class CFG, class WEIGHT>
void
StatClass::
PrintAllStats( ostream& _os, Roadmap<CFG, WEIGHT>* _rmap, int _numCCs) {
        #ifndef _PARALLEL
  size_t i;
  std::map<string, unsigned long int>::const_iterator iter;
  //int total=0;

  _os << endl << endl << "Local Planners:" << endl;
  _os << setw(20) << "Name"
  <<setw(15) << "Connections"
  <<setw(15) << "Attempts"
  <<setw(15) << "Coll Det Calls" << endl;

  std::map<string, unsigned long int>::const_iterator iter1, iter2, iter3;
  for(iter1 = m_lpConnections.begin(), iter2 = m_lpAttempts.begin(), iter3 = m_lpCollDetCalls.begin(); 
      iter1 != m_lpConnections.end() || iter2 !=m_lpAttempts.end() || iter3 != m_lpCollDetCalls.end(); 
      ++iter1, ++iter2, ++iter3) {
    _os << setw(20) << iter1->first;
    _os << setw(15) << iter1->second;
    _os << setw(15) << iter2->second;
    _os << setw(15) << iter3->second << endl;
  }

  //output for local planner statistics. Only output if map is populated
  if(m_lpStats.size()>0){
    _os<<"\n\n Local Planner Statistics:\n\n";
    _os<< setw(40) << "Statistic"
      << setw(40) << "Value" << endl << endl;;
    typedef map<string, double>::iterator LPSIT;
    for(LPSIT lpsit=m_lpStats.begin(); lpsit!=m_lpStats.end(); lpsit++){
      _os << setw(40) << lpsit->first
        << setw(40) << lpsit->second << endl;
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
  _os << "Number of Nodes: " << _rmap->m_pRoadmap->get_num_vertices() << endl;
  _os << "Number of Edges: " << _rmap->m_pRoadmap->get_num_edges() << endl;
/*  Removed by Roger for reasons given below
  _os << "Number of Collision Detection Calls: " << endl;
  for(i=0;i<MaxCD;i++)
    if (strcmp(CDNameList[i],"empty")!=0)
      _os << setw(20) << CDNameList[i] 
        << setw(15) << NumCollDetCalls[i] << endl;
*/

  _os << endl;

  if (_numCCs==ALL)    {DisplayCCStats(_os, *(_rmap->m_pRoadmap));      }
  else if (_numCCs==0) {DisplayCCStats(_os, *(_rmap->m_pRoadmap),0);     }
  else                {DisplayCCStats(_os, *(_rmap->m_pRoadmap),_numCCs);}


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
  #endif

}

template <class CFG, class WEIGHT>
void
StatClass::
PrintDataLine(ostream& _myostream, Roadmap<CFG, WEIGHT> *_rmap, int _showColumnHeaders) {
  #ifndef _PARALLEL
  // Default is to NOT print out column headers
  if (_showColumnHeaders) {
    _myostream <<"\nV  E #CC 1 2 3 4 5 6 7 8 9 10 #iso CD  LPattSum LPcdSum\n";
  }//endif
  
  _myostream << _rmap->m_pRoadmap->get_num_vertices() << " ";
  _myostream << _rmap->m_pRoadmap->get_num_edges()   << " ";

  typedef typename RoadmapGraph<CFG,WEIGHT>::vertex_descriptor VID;
  stapl::vector_property_map<RoadmapGraph<CFG,WEIGHT>,size_t > cMap;
  vector< pair<size_t,VID> > ccStats;
  get_cc_stats(*(_rmap->m_pRoadmap), cMap, ccStats);
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

  std::map<string, unsigned long int>::const_iterator iter1, iter2;
  for(iter1 = m_lpAttempts.begin(), iter2 = m_lpCollDetCalls.begin(); iter1 != m_lpAttempts.end() || iter2 != m_lpCollDetCalls.end(); ++iter1, ++iter2) {
    sumAtt += iter1->second;
    sumCD += iter2->second;
  }

  _myostream << sumAtt << " ";
  _myostream << sumCD  << " ";
  ccStats.clear();
   #endif
}

// Compute intra-connected-component statistics
template <class CFG, class WEIGHT>
void
StatClass::
ComputeIntraCCFeatures(Roadmap<CFG,WEIGHT> * _rdmp, shared_ptr<DistanceMetricMethod> _dm) {
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

  typedef typename RoadmapGraph<CFG,WEIGHT>::vertex_descriptor VID;
  vector< pair<size_t,VID> > ccs; //vector of connected components in the roadmap
  stapl::vector_property_map<RoadmapGraph<CFG,WEIGHT>,size_t > cMap;
  stapl::get_cc_stats(*(_rdmp->m_pRoadmap),cMap, ccs);//fill ccs from the roadmap
  cout << "in intra ccs portion" << endl;

  typename vector< pair<size_t, VID> >::iterator cci; // cci is CC[i] hereafter
  for (cci = ccs.begin(); cci < ccs.end(); cci++) {
    vector<VID> cciCfgsAux; //use this auxiliary vec to convert between VID and CFG
    vector<CFG> cciCfgs; //configurations in cci
    VID cci_tmp = (*(_rdmp->m_pRoadmap->find_vertex(cci->second))).descriptor();//cci->second: vertex ID of first node in cci
    cMap.reset();
    get_cc(*(_rdmp->m_pRoadmap), cMap, cci_tmp, cciCfgsAux); //fill cci_cfgs
    for(typename vector<VID>::iterator itr = cciCfgsAux.begin(); itr!=cciCfgsAux.end(); itr++)
      cciCfgs.push_back((*(_rdmp->m_pRoadmap->find_vertex(*itr))).property());

    if (cciCfgs.size() > 1) {
     //compute shortest, longest, mean, and std-dev (sigma) distances 
     //between nodes for cci

      double cciMinIntraCCDist = 0;
      double cciMaxIntraCCDist = 0;
      double cciMeanIntraCCDist = 0;
      double nPairs = 0;
      typename vector<CFG>::iterator cci_i;
      typename vector<CFG>::iterator cci_j;
      for (cci_i=cciCfgs.begin(); cci_i < cciCfgs.end(); cci_i++) {
        for (cci_j = cci_i+1; cci_j < cciCfgs.end(); cci_j++) {
          double c_dist = _dm->Distance(_rdmp->GetEnvironment(), *cci_i, *cci_j);
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
          cciSigmaIntraCCDist += pow(_dm->Distance(_rdmp->GetEnvironment(), *cci_i, *cci_j)-cciMeanIntraCCDist, 2);
        }
      }
      if (nPairs > 1)
        cciSigmaIntraCCDist /= (nPairs-1);
      cciSigmaIntraCCDist = sqrt(cciSigmaIntraCCDist);

      //compute shortest, longest, mean and std-dev (sigma) edge sizes 
      //in the component
      vector<pair<VID,VID> > cciEdges;
      cMap.reset();
      get_cc_edges(*(_rdmp->m_pRoadmap),cMap, cciEdges, cci->second);
      double cciMaxIntraCCEdge = 0;
      double cciMinIntraCCEdge = 0;
      double cciMeanIntraCCEdge = 0;
      double cciSigmaIntraCCEdge = 0;
      for (typename vector< pair<VID,VID> >::iterator eIter = cciEdges.begin(); eIter < cciEdges.end(); eIter++) {
        CFG e_v1 = (*(_rdmp->m_pRoadmap->find_vertex(eIter->first))).property();
        CFG e_v2 = (*(_rdmp->m_pRoadmap->find_vertex(eIter->second))).property();
        double eDist = _dm->Distance(_rdmp->GetEnvironment(), e_v1, e_v2);
        if (eDist > cciMaxIntraCCEdge)
          cciMaxIntraCCEdge = eDist;
        if (cciMinIntraCCEdge == 0 || eDist < cciMinIntraCCEdge)
          cciMinIntraCCEdge = eDist;
        cciMeanIntraCCEdge += eDist;
      }
      if (cciEdges.size() > 0)
        cciMeanIntraCCEdge /= cciEdges.size();
      for (typename vector< pair<VID,VID> >::iterator eIter = cciEdges.begin(); eIter < cciEdges.end(); eIter++) {
        CFG e_v1 = (*(_rdmp->m_pRoadmap->find_vertex(eIter->first))).property();
        CFG e_v2 = (*(_rdmp->m_pRoadmap->find_vertex(eIter->second))).property();
        double eDist = _dm->Distance(_rdmp->GetEnvironment(), e_v1, e_v2);
        cciSigmaIntraCCEdge += pow(eDist-cciMeanIntraCCEdge, 2);
      }
      if (cciEdges.size() > 1)
        cciSigmaIntraCCEdge /= cciEdges.size() - 1;
      cciSigmaIntraCCEdge = sqrt(cciSigmaIntraCCEdge);

      //get center of mass of all Cfgs in current CC
      CFG center_of_mass = cciCfgs[0];
      for (size_t j = 1; j < cciCfgs.size(); ++j)
        center_of_mass.add(center_of_mass, cciCfgs[j]);
      center_of_mass.divide(center_of_mass, cciCfgs.size());

      //compute average distance of the nodes to the center of mass
      double cciMinIntraCCDistToCm = 0;
      double cciMaxIntraCCDistToCm = 0;
      double cciMeanIntraCCDistToCm = 0;
      double cciSigmaIntraCCDistToCm = 0;
      for (size_t j = 0; j < cciCfgs.size(); ++j) {
        CFG tmp = cciCfgs[j];
        double eDist = _dm->Distance(_rdmp->GetEnvironment(), center_of_mass, tmp);
        if (eDist > cciMaxIntraCCDistToCm)
          cciMaxIntraCCDistToCm = eDist;
        if (cciMinIntraCCDistToCm == 0 || eDist < cciMinIntraCCDistToCm)
          cciMinIntraCCDistToCm = eDist;
        cciMeanIntraCCDistToCm += eDist;
      }
      if (cciCfgs.size() > 0 )
        cciMeanIntraCCDistToCm /= cciCfgs.size();
      for (size_t j = 0; j < cciCfgs.size(); ++j) {
        CFG tmp = cciCfgs[j];
        double eDist = _dm->Distance(_rdmp->GetEnvironment(), center_of_mass, tmp);
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

  CFG tcfg;
  double norm = _rdmp->GetEnvironment()->Getminmax_BodyAxisRange()*tcfg.DOF();
  cout << "norm value = " << norm << endl;

  #endif
}

// Compute inter-connected-component statistics
// Find Closest nodes between two ccs and find distance
// between the closest pairs of ccs
template <class CFG, class WEIGHT>
void
StatClass::
ComputeInterCCFeatures(Roadmap<CFG,WEIGHT> * _rdmp, NeighborhoodFinder* _nf, string _nfMethod) {
  #ifndef _PARALLEL
  shared_ptr<DistanceMetricMethod> dm = _nf->GetNFMethod(_nfMethod)->GetDMMethod();
  typedef typename RoadmapGraph<CFG,WEIGHT>::vertex_descriptor VID;
  stapl::vector_property_map<RoadmapGraph<CFG,WEIGHT>,size_t > cMap;
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
    get_cc_edges(*(_rdmp->m_pRoadmap),cMap,ccEdges,cce->second);
    double totalSize = 0.0;
    cout << "size of edge list for cc:" << cce->second << " "<< ccEdges.size() << endl;
    for(typename vector< pair<VID,VID> >::iterator ccIter=ccEdges.begin(); ccIter<ccEdges.end();ccIter++) {
      CFG ccIterA = (*(_rdmp->m_pRoadmap->find_vertex(ccIter->first))).property();
      CFG ccIterB = (*(_rdmp->m_pRoadmap->find_vertex(ccIter->second))).property();
      double dist = dm->Distance(_rdmp->GetEnvironment(), ccIterA, ccIterB);
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
        get_cc(*(_rdmp->m_pRoadmap),cMap, cci->second, cciCfgs); //fill cciCfgs
        cMap.reset();
        get_cc(*(_rdmp->m_pRoadmap),cMap, ccj->second, ccjCfgs); //fill ccjCfgs

        vector< pair<VID,VID> > pairs;
        _nf->KClosestPairs(_nf->GetNFMethod(_nfMethod), _rdmp,
        cciCfgs.begin(), cciCfgs.end(), ccjCfgs.begin(), ccjCfgs.end(), 1, back_inserter(pairs));
        double tmpDist = dm->Distance(_rdmp->GetEnvironment(), (*(_rdmp->m_pRoadmap->find_vertex(pairs[0].first))).property(),
                         (*(_rdmp->m_pRoadmap->find_vertex(pairs[0].second))).property() );
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

  CFG tcfg;
  double norm = _rdmp->GetEnvironment()->Getminmax_BodyAxisRange()
  *_rdmp->GetEnvironment()->GetBoundingBox()->GetDOFs();
  cout << "norm value = " << norm << endl;
  #endif
}

/**Output Connected Component information in graph _G.
 *This method will print out _maxCCprint number of Connected Component,
 *,and size and start vertex of each Connected Component.
 *@param _g the graph
 *@param _maxCCprint the number of connected components to print.
 */
template <class CFG, class WEIGHT>
void
StatClass::
DisplayCCStats(ostream& _os, RoadmapGraph<CFG, WEIGHT>& _g, int _maxCCPrint=-1)  {

  ///Modified for VC
  //temporary ifdef because of color map and get_cc_stats, we need a pDisplayCCStats
  #ifndef _PARALLEL

  typedef typename RoadmapGraph<CFG,WEIGHT>::vertex_descriptor VID;
  stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cMap;

  vector< pair<size_t,VID> > ccStats;
  stapl::get_cc_stats(_g, cMap, ccStats);
  if (_maxCCPrint == -1) {
    _maxCCPrint = ccStats.size();
  }

  int ccNum = 1;
  _os << "\nThere are " << ccStats.size() << " connected components:";
  for (typename vector< pair<size_t,VID> >::iterator vi = ccStats.begin(); vi != ccStats.end(); vi++) {
    _os << "\nCC[" << ccNum << "]: " << vi->first ;
    _os << " (vid=" << size_t(vi->second) << ")";
    ccNum++;
    if (ccNum > _maxCCPrint) return;
  }
  #endif
}

/**Provide timing information.
  *This class is used to measure the running time between StartClock and 
  *StopClock. Client side could provide clock name, when StopClock is called 
  *the name will be print out, and running time as well.
  */
class ClockClass {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  ///////////////////////////////////////////////////////////////////////////////////////////

  //-----------------------------------------------------------
  /**@name Constructors and Destructor.*/
  //-----------------------------------------------------------
  //@{
  ///Default constructor. Set every thing to zero
  ClockClass();

  ///Destructor. Do nothing
  ~ClockClass();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Clock methods
  //
  //
  ///////////////////////////////////////////////////////////////////////////////////////////

  //----------------------------------------------------------
  /**@name Clock Methods.*/
  //-----------------------------------------------------------
  //@{
  ///Set every thing to zero
  void ClearClock();

  ///Start the clock and the name is identity of this clock.
  void StartClock(string _name);

  ///Stop the clock and calculate the total running time.
  void StopClock();

  ///Call StopClock and PrintClock.
  void StopPrintClock();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //-----------------------------------------------------------
  /**@name Input and Output Methods.*/
  //-----------------------------------------------------------
  //@{
  /**Output the clock name given in StartClock and running time accosited with this name
    *to the standard output.
    */
  void PrintClock();

  ///Output the clock name given in StartClock to the standard output
  void PrintName();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  ///////////////////////////////////////////////////////////////////////////////////////////

  //-----------------------------------------------------------
  /**@name Access Methods.*/
  //-----------------------------------------------------------
  //@{
  ///Get how many seconds of running time are. (integer, without fraction part!!)
  int GetClock();

  ///Get how many seconds of running time are.
  double GetSeconds();

  ///Get how many 1e-6 seconds of running time are.
  int GetUSeconds();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data Members and MEmber Methods Methods (Undocumented)
  //
  //
  ///////////////////////////////////////////////////////////////////////////////////////////
private:
  int m_sTime, m_uTime;
  int m_suTime, m_uuTime;
  string m_clockName;
};


#ifndef _H_UTILITY
///Return minimun between a and b.
inline double min(double _a, double _b) {
  return _a < _b ? _a : _b;
}

///Return maximun between a and b.
inline double max(double _a, double _b) {
  return _a > _b ? _a : _b;
}

/// Return the square of a.
inline double sqr(double _a) {
  return _a*_a;
}
#endif

      
#endif
