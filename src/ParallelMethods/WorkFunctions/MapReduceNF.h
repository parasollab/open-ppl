/////////////////////////////////
//HEADER MapReduceNF.h
/////////////////////////////////

#ifndef MapReduceNF_H_
#define MapReduceNF_H_


#include "MPProblem/MPTraits.h"
#include "ParallelMethods/ParallelSBMPHeader.h"
#include <stapl/containers/graph/views/graph_view.hpp>
// needed for proxy of pair specialization in reduce wf
#include <stapl/../tools/libstdc++/proxy/pair.h>

using namespace psbmp;
using namespace stapl;

/*   
     typedef RoadmapGraph<CfgType,WeightType> rGraph;
     typedef rGraph::vertex_iterator rGraphIterator;
     typedef graph_view<rGraph>  gviewType;
     typedef pair<VID, double> NFType;
     */

template<class MPTraits>
class NFMapFunc {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef pair<VID, double> NFType;
    typedef pair<pair<VID,CfgType>, double> NFType2;
    typedef vector<NFType2> NFResultType; 
    typedef CfgType RegionType;
    MPProblemType* m_problem;
    CfgType m_cfg;
    size_t  m_k;
    bool m_radial;
    string m_dmLabel;

  public:
  typedef NFResultType result_type;
    NFMapFunc(MPProblemType* _problem=NULL, CfgType _cfg = CfgType(), size_t _k=0, bool _radial=false, string _dmLabel=""): 
      m_problem(_problem),m_cfg(_cfg), m_k(_k),m_radial(_radial), m_dmLabel(_dmLabel){
      }

    void define_type(stapl::typer& _t) {
      _t.member(m_problem);
      _t.member(m_cfg); 
      _t.member(m_k);
      _t.member(m_radial);
    }

    template <typename View>
      vector<NFType> operator() (View _v) {
        ///replace with call to NF
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric(m_dmLabel);
        Environment* env = m_problem->GetEnvironment();
        // PrintValue("NFMapFunc view size: ", _v.size());
        typename View::iterator it1 = _v.begin();
        typename View::iterator it2 = _v.end();
        vector<NFType> result = FindNeighbors(env, dmm, it1, it2, m_cfg, m_k, m_radial);

        return result;

      }

    //Compute Neighbor Function
    template<typename InputIterator>
      vector<NFType>
      FindNeighbors(Environment* _env, DistanceMetricPointer _dmm,
          InputIterator _input_first, InputIterator _input_last, CfgType  _cfg, 
          int k, bool radial=false) {
        int max_index = 0;
        double max_value = MAX_DIST;
	cout << "FINDNEIGHBOR " << endl;
        vector< pair< VID, double > > closest(k, make_pair(-999, max_value));
        // now go through all kp and find closest k
        int count = 0;
        for(InputIterator V1 = _input_first; V1 != _input_last; ++V1) {
          count++;
          CfgType  v1 = (*V1).property().GetCandidate();

          if(v1 == _cfg || v1 == CfgType() )
            continue; //don't connect same or invalid cfg

          double dist = _dmm->Distance(_env, _cfg, v1);

          if(dist < closest[max_index].second) { 
            VID tmp = (*V1).descriptor();
            closest[max_index] = make_pair(tmp, dist);
            max_value = dist;
            //search for new max_index (faster O(k) than sort O(k log k) )
            for (size_t p = 0; p < closest.size(); ++p) {
              if (max_value < closest[p].second) {
                max_value = closest[p].second;
                max_index = p;
              }
            }
          }
        }
        ////why sorting?
        // sort(closest.begin(), closest.end(), compare_second<VID, double>());
        return closest;

      }

      //Compute Neighbor Function
    template<typename InputIterator>
      vector<NFType2>
      FindNeighbor(Environment* _env, DistanceMetricPointer _dmm,
          InputIterator _input_first, InputIterator _input_last, CfgType  _cfg, 
          int k) {
        int max_index = 0;
        double max_value = MAX_DIST;
	//cout << "k " << k << endl;
        //vector< pair< VID, double > > closest(k, make_pair(-999, max_value));
	vector<NFType2> closest(k, make_pair(make_pair(-999,CfgType()), max_value));
        // now go through all kp and find closest k
        int count = 0;
        for(InputIterator V1 = _input_first; V1 != _input_last; ++V1) {
          count++;
          CfgType  v1 = (*V1).property();
	  //cout << "cfg : " << v1 << endl;

          //if(v1 == _cfg || v1 == CfgType() )
	  if(v1 == _cfg)
            continue; //don't connect same or invalid cfg

          double dist = _dmm->Distance(_env, _cfg, v1);
          //cout << "dist : " << dist << endl;
          if(dist < closest[max_index].second) { 
            VID tmp = (*V1).descriptor();
            //closest[max_index] = make_pair(tmp, dist);
	    //cout << "id : " << tmp << endl;
	    //cout << "cfg : " << v1 << endl;
	    closest[max_index] = make_pair(make_pair(tmp,v1), dist);
            max_value = dist;
            //search for new max_index (faster O(k) than sort O(k log k) )
            for (size_t p = 0; p < closest.size(); ++p) {
              if (max_value < closest[p].second) {
                max_value = closest[p].second;
                max_index = p;
              }
            }
          }
        }
        ////why sorting?
        // sort(closest.begin(), closest.end(), compare_second<VID, double>());
        return closest;

      }
};

template<class MPTraits>
struct NFReduceFunc {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPTraits::CfgType CfgType;
  //typedef pair<VID, double> NFType;
  typedef pair<pair<VID,CfgType>, double> NFType;
  typedef vector<NFType> result_type;

  template <typename View1, typename View2>
    result_type operator() (View1 vec1, View2 vec2) {
      size_t k = vec1.size();
      vector<NFType> closest;
      closest.clear();
      for(size_t i=0; i<k;++i){
        if(vec1[i].second <= vec2[i].second){
          closest.push_back(vec1[i]);
        }else{
          closest.push_back(vec2[i]);
        }
      }
      return closest;
    }
};

#endif 
