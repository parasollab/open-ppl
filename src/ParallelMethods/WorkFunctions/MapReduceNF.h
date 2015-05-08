/////////////////////////////////
//HEADER MapReduceNF.h
/////////////////////////////////

#ifndef MapReduceNF_H_
#define MapReduceNF_H_


#include "ParallelMethods/ParallelSBMPHeader.h"
#include <stapl/containers/graph/views/graph_view.hpp>
// needed for proxy of pair specialization in reduce wf
#include <stapl/../tools/libstdc++/proxy/pair.h>

using namespace psbmp;
using namespace stapl;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NFMapFunc {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef pair<pair<VID, CfgType>, double> NFType;
    typedef vector<NFType> NFResultType;

    MPProblemType* m_problem;
    CfgType m_cfg;
    size_t  m_k;
    string m_dmLabel;

  public:
    NFMapFunc(MPProblemType* _problem, CfgType _cfg, size_t _k, string _dmLabel):
      m_problem(_problem),m_cfg(_cfg), m_k(_k), m_dmLabel(_dmLabel){
      }

    void define_type(stapl::typer& _t) {
      _t.member(m_problem);
      _t.member(m_cfg);
      _t.member(m_k);
      _t.member(m_dmLabel);
    }

    typedef NFResultType result_type;

    template <typename View>
      result_type operator()(View _v) {
        cout << "MapReduceNF called." << endl;
        ///replace with call to NF
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric(m_dmLabel);
        return FindNeighbors(dmm, _v.begin(), _v.end(), m_cfg, m_k);
      }

    //Compute Neighbor Function
    template<typename InputIterator>
      NFResultType
      FindNeighbors(DistanceMetricPointer _dmm,
          InputIterator _first, InputIterator _last, CfgType  _cfg,
          int k) {
        // Keep sorted list of k best so far
        priority_queue<NFType, NFResultType, CompareSecond<pair<VID, CfgType>, double>> pq;
        for(InputIterator it = _first; it != _last; it++) {

          CfgType node = (*it).property();

          if(node == _cfg) // Don't connect to self
            continue;

          double dist = _dmm->Distance(_cfg, node);

          if(pq.size() < this->m_k){
            VID vid = (*it).descriptor();
            pq.push(make_pair(make_pair(vid, node), dist));
          }
          // If better than the worst so far, replace worst so far
          else if(dist < pq.top().second) {
            pq.pop();
            VID vid = (*it).descriptor();
            pq.push(make_pair(make_pair(vid, node), dist));
          }
        }

        // Transfer k closest to vector, sorted greatest to least dist
        NFResultType closest;
        while(!pq.empty()){
          closest.push_back(pq.top());
          pq.pop();
        }
        reverse(closest.begin(), closest.end());
        return closest;
      }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
struct NFReduceFunc {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPTraits::CfgType CfgType;
  typedef pair<pair<VID,CfgType>, double> NFType;
  typedef vector<NFType> result_type;

  NFReduceFunc(size_t _k): m_k(_k) {}

  void define_type(stapl::typer& _t) {
    _t.member(m_k);
  }

  template <typename View1, typename View2>
    result_type operator() (View1 vec1, View2 vec2) {
      vector<NFType> closest;
      size_t indx1 = 0, indx2 = 0;
      while(closest.size() < m_k && indx1 < vec1.size() && indx2 < vec2.size()) {
        if(vec1[indx1].first.first == vec2[indx2].first.first) {
          closest.push_back(vec1[indx1]);
          indx1++; indx2++;
        }
        else {
          closest.push_back(
              vec1[indx1].second <= vec2[indx2].second ?
              vec1[indx1++] : vec2[indx2++]
              );
        }
      }
      while(closest.size() < m_k && indx1 < vec1.size())
        closest.push_back(vec1[indx1++]);
      while(closest.size() < m_k && indx2 < vec2.size())
        closest.push_back(vec2[indx1++]);
      return closest;
    }

  size_t m_k;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NFMapFuncRRRT {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef pair<pair<VID, CfgType>, double> NFType;
    typedef vector<NFType> NFResultType;

    MPProblemType* m_problem;
    CfgType m_cfg;
    size_t  m_k;
    string m_dmLabel;

  public:
    NFMapFuncRRRT(MPProblemType* _problem, CfgType _cfg, size_t _k, string _dmLabel):
      m_problem(_problem),m_cfg(_cfg), m_k(_k), m_dmLabel(_dmLabel){
      }

    void define_type(stapl::typer& _t) {
      _t.member(m_problem);
      _t.member(m_cfg);
      _t.member(m_k);
      _t.member(m_dmLabel);
    }

    typedef NFResultType result_type;

    template <typename View>
      result_type operator()(View _v) {
        cout << "MapReduceNF called." << endl;
        ///replace with call to NF
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric(m_dmLabel);
        return FindNeighbors(dmm, _v.begin(), _v.end(), m_cfg, m_k);
      }

    //Compute Neighbor Function
    template<typename InputIterator>
      NFResultType
      FindNeighbors(DistanceMetricPointer _dmm,
          InputIterator _first, InputIterator _last, CfgType  _cfg,
          int k) {
        // Keep sorted list of k best so far
        priority_queue<NFType, NFResultType, CompareSecond<pair<VID, CfgType>, double>> pq;
        for(InputIterator it = _first; it != _last; it++) {

          CfgType node = (*it).property().GetCandidate();

          if(node == _cfg) // Don't connect to self
            continue;

          double dist = _dmm->Distance(_cfg, node);

          if(pq.size() < this->m_k){
            VID vid = (*it).descriptor();
            pq.push(make_pair(make_pair(vid, node), dist));
          }
          // If better than the worst so far, replace worst so far
          else if(dist < pq.top().second) {
            pq.pop();
            VID vid = (*it).descriptor();
            pq.push(make_pair(make_pair(vid, node), dist));
          }
        }

        // Transfer k closest to vector, sorted greatest to least dist
        NFResultType closest;
        while(!pq.empty()){
          closest.push_back(pq.top());
          pq.pop();
        }
        reverse(closest.begin(), closest.end());
        return closest;
      }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
struct NFReduceFuncRRRT {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPTraits::CfgType CfgType;
  typedef pair<pair<VID,CfgType>, double> NFType;
  typedef vector<NFType> result_type;

  NFReduceFuncRRRT(size_t _k): m_k(_k) {}

  void define_type(stapl::typer& _t) {
    _t.member(m_k);
  }

  template <typename View1, typename View2>
    result_type operator() (View1 vec1, View2 vec2) {
      vector<NFType> closest;
      size_t indx1 = 0, indx2 = 0;
      while(closest.size() < m_k && indx1 < vec1.size() && indx2 < vec2.size()) {
        if(vec1[indx1].first.first == vec2[indx2].first.first) {
          closest.push_back(vec1[indx1]);
          indx1++; indx2++;
        }
        else {
          closest.push_back(
              vec1[indx1].second <= vec2[indx2].second ?
              vec1[indx1++] : vec2[indx2++]
              );
        }
      }
      while(closest.size() < m_k && indx1 < vec1.size())
        closest.push_back(vec1[indx1++]);
      while(closest.size() < m_k && indx2 < vec2.size())
        closest.push_back(vec2[indx1++]);
      return closest;
    }

  size_t m_k;
};

#endif
