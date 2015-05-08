#ifndef DPES_H
#define DPES_H

const double MIN_DIST =  -1e10;

using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
class DPES{
public:

  //Default Constructor
  DPES() { };

  DPES(const CMPPROXY& _compare_proxy, const CMPPIVOT& _compare_proxy_pivot) :
    m_distance_proxy(_compare_proxy), m_distance_pivot(_compare_proxy_pivot){

  }
  //Destructor
  //~DPES() { };

  int AddNode(SPROXY v);
	void CreatPivots(int m);
  void UpdateProjected();
  vector<double> UpdateProjected_s(SPROXY s);
  vector<double> UpdateProjected_s_PIVOT(PIVOT s);
  const size_t GetVS_Size() const { return VS.size(); }

  void Clear() {
   VS.clear();
   P.clear();
   S.clear();
  }

  template <typename OutputIterator>
  OutputIterator
  KClosestSPROXY(SPROXY _cfg,
//           InputIterator _input_first, InputIterator _input_last,
           OutputIterator _out, int k, int l);


  template <typename OutputIterator>
  OutputIterator
  KClosestPIVOT(PIVOT _cfg,
//           InputIterator _input_first, InputIterator _input_last,
           OutputIterator _out, int k, int l);

  template <typename OutputIterator>
  OutputIterator
  KClosestBF_Euclidean(vector<double> _cfg,
//             InputIterator _input_first, InputIterator _input_last,
             OutputIterator _out, int k);

  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosestBF_RMSD(SPROXY _cfg,
             InputIterator _input_first, InputIterator _input_last,
             OutputIterator _out, int k);

  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosestBF_RMSD_PIVOT(PIVOT _cfg,
             InputIterator _input_first, InputIterator _input_last,
             OutputIterator _out, int k);
private:
  double Euclidean( const vector<double>& v1, const vector<double>& v2);
  CMPPROXY m_distance_proxy;
  CMPPIVOT m_distance_pivot;
  vector<SPROXY> S;
  vector<vector<double> > VS;
  vector<PIVOT> P;

};


template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
int
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
AddNode(SPROXY v){
	S.push_back(v);
	return(S.size() - 1);
}

template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
void
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
CreatPivots(int m){

  int iRand;
  srand ( time(NULL) );
  iRand = rand() % S.size();
  P.push_back(S[iRand]);
  unsigned index;
//  int min_index_current;
  double min_value = MAX_DIST;
  double min_value_current;
  double max_value = MIN_DIST;

   for(int p = 1; p < m; ++p){

     max_value = MIN_DIST;
     for(size_t ix = 0; ix != S.size(); ++ix){
       min_value = MAX_DIST;
        bool flag = false;
        for(size_t ixp = 0; ixp != P.size(); ++ixp)
          if( P[ixp] == S[ix])
            flag = true;
        if(!flag){
          for(size_t ixp = 0; ixp != P.size(); ++ixp){
             min_value_current = m_distance_pivot(S[ix], P[ixp]);
          if(min_value_current < min_value){
              min_value = min_value_current;
            }
          }
          if(min_value > max_value){
            max_value = min_value;
            index = ix;
          }
        }
     }
     P.push_back(S[index]);
   }
}

template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
void
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
UpdateProjected(){
  double dist = 0.0;
  for(size_t ix = 0; ix != S.size(); ++ix){
    vector<double> v;
    for(size_t ixp = 0; ixp != P.size(); ++ixp){
    dist = m_distance_pivot(S[ix], P[ixp]);
    v.push_back(dist);
    }
    VS.push_back(v);
  }
}

template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
vector<double>
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
UpdateProjected_s(SPROXY s){
    double dist = 0.0;
    vector<double> v;
    for(size_t ixp = 0; ixp != P.size(); ++ixp){
    dist = m_distance_pivot(s, P[ixp]);
    v.push_back(dist);
    }
    return v;
}

template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
vector<double>
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
UpdateProjected_s_PIVOT(PIVOT s){
    double dist = 0.0;
    vector<double> v;
    for(size_t ixp = 0; ixp != P.size(); ++ixp){
    dist = m_distance_pivot(s, P[ixp]);
    v.push_back(dist);
    }
    return v;
}

template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
template <typename OutputIterator>
OutputIterator
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
KClosestSPROXY(SPROXY _cfg,
//         InputIterator _input_first, InputIterator _input_last,
         OutputIterator _out, int k, int l){
  vector<double> v = UpdateProjected_s(_cfg);
  vector< pair<int, double> > lclosest(l);
  vector<int> ll;

  KClosestBF_Euclidean(v, lclosest.begin(), l);

  for(size_t i = 0; i < lclosest.size(); ++i)
    ll.push_back(lclosest[i].first);

  return KClosestBF_RMSD(_cfg, ll.begin(), ll.end(), _out, k);
}

template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
template <typename OutputIterator>
OutputIterator
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
KClosestPIVOT(PIVOT _cfg,
//         InputIterator _input_first, InputIterator _input_last,
         OutputIterator _out, int k, int l){
  vector<double> v = UpdateProjected_s_PIVOT(_cfg);
  vector< pair<int, double> > lclosest(l);
  vector<int> ll;

  KClosestBF_Euclidean(v, lclosest.begin(), l);

  for(size_t i = 0; i < lclosest.size(); ++i)
    ll.push_back(lclosest[i].first);

  return KClosestBF_RMSD_PIVOT(_cfg, ll.begin(), ll.end(), _out, k);
}


template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
template <typename OutputIterator>
OutputIterator
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
KClosestBF_Euclidean(vector<double> _cfg,
//           InputIterator _input_first, InputIterator _input_last,
           OutputIterator _out, int k){

  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< int, double > > closest(k, make_pair(-999, max_value));

  // now go through all kp and find closest k
//  InputIterator V1;
//  int count = 0;
//  for(V1 = _input_first; V1 != _input_last; ++V1) {
  for(size_t V1 = 0; V1 != VS.size(); ++V1){
  if(VS[V1] == _cfg)
      continue; //don't connect same

    double dist = Euclidean(_cfg, VS[V1]);
    if(dist < closest[max_index].second) {
      closest[max_index] = make_pair(V1, dist);
      max_value = dist;
      for (size_t p = 0; p < closest.size(); ++p) {
        if (max_value < closest[p].second) {
          max_value = closest[p].second;
          max_index = p;
        }
      }
    }
  }
  sort(closest.begin(), closest.end(), compare_second<int, double>());

  for (size_t p = 0; p < closest.size(); ++p)
      if (closest[p].first != -999){
//        *_out = closest[p].first;
        *_out = make_pair(closest[p].first, closest[p].second);
//        cout << "\t\t\t l closest  " << (*_out).first << "\t" << closest[p].second << endl;
        ++_out;
      }
 return _out;
}


template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
template <typename InputIterator, typename OutputIterator>
OutputIterator
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
KClosestBF_RMSD(SPROXY _cfg,
           InputIterator _input_first, InputIterator _input_last,
           OutputIterator _out, int k){

  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< int, double > > closest(k, make_pair(-999, max_value));

  // now go through all kp and find closest k
  InputIterator V1;
  int count = 0;
  for(V1 = _input_first; V1 != _input_last; ++V1) {
    ++count;

    if(S[*V1] == _cfg)
      continue; //don't connect same

    double dist = m_distance_proxy(_cfg, S[*V1]);

    if(dist < closest[max_index].second) {
      closest[max_index] = make_pair(*V1, dist);
      max_value = dist;

      for (int p = 0; p < closest.size(); ++p) {
        if (max_value < closest[p].second) {
          max_value = closest[p].second;
          max_index = p;
        }
      }
    }
  }

  sort(closest.begin(), closest.end(), compare_second<int, double>());

  for (int p = 0; p < closest.size(); ++p)
      if (closest[p].first != -999)
      {
//        *_out = closest[p].first;
*_out = make_pair(closest[p].first, closest[p].second);
//        cout << "\t\t\t" << *_out << "\t" << closest[p].second << endl;
        ++_out;
      }
 return _out;
}



template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
template <typename InputIterator, typename OutputIterator>
OutputIterator
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
KClosestBF_RMSD_PIVOT(PIVOT _cfg,
           InputIterator _input_first, InputIterator _input_last,
           OutputIterator _out, int k){

  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< int, double > > closest(k, make_pair(-999, max_value));

  InputIterator V1;
  int count = 0;
  for(V1 = _input_first; V1 != _input_last; ++V1) {
    ++count;
    if(_cfg == S[*V1])
      continue; //don't connect same

    double dist = m_distance_proxy(_cfg, S[*V1]);

    if(dist < closest[max_index].second) {
      closest[max_index] = make_pair(*V1, dist);
      max_value = dist;

      for (size_t p = 0; p < closest.size(); ++p) {
        if (max_value < closest[p].second) {
          max_value = closest[p].second;
          max_index = p;
        }
      }
    }
  }

  sort(closest.begin(), closest.end(), compare_second<int, double>());

    for (size_t p = 0; p < closest.size(); ++p)
      if (closest[p].first != -999)
      {
//        *_out = closest[p].first;
*_out = make_pair(closest[p].first, closest[p].second);
//        cout << "\t\t\t" << *_out << "\t" << closest[p].second << endl;
        ++_out;
      }
 return _out;
}

template<typename SPROXY, typename PIVOT, typename CMPPROXY, typename CMPPIVOT>
double
DPES<SPROXY,PIVOT,CMPPROXY,CMPPIVOT>::
Euclidean( const vector<double>& v1, const vector<double>& v2){
  double diff, dist = 0.0;
  int d;

  if(v1.size() != v2.size() ){
      cout << "Dimensons of points are not consistent when calculating Euclidean Distance"
           << endl;
      return -1;
  }
  else {
    d = v1.size();
    for(int i = 0; i < d; ++i){
      diff = v1[i] - v2[i];
      dist += diff * diff;
    }
    return sqrt(dist);
  }
}

#endif // DPES_H
