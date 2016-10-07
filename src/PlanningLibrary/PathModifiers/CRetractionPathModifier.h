#ifndef C_RETRACTION_PATH_MODIFIER_H_
#define C_RETRACTION_PATH_MODIFIER_H_

#include "PathModifierMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// @brief Retract path to higher clearance.
/// @tparam MPTraits Motion planning universe
///
/// This method represents algorithm 7, C-Retraction from Geraerts and Overmars,
/// "Creating High-quality Paths for Motion Planning," IJRR 2007. Essentially it
/// optimizes clearance based upon picking nearby neighbors to path
/// configurations, and modifying the path based on if the random configurations
/// have better clearance.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CRetractionPathModifier : public PathModifierMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    CRetractionPathModifier(size_t _iter = 10,
        const ClearanceUtility<MPTraits>& _cu = ClearanceUtility<MPTraits>());
    CRetractionPathModifier(MPProblemType* _problem, XMLNode& _node);

    void Print(ostream& _os) const;

    bool ModifyImpl(vector<CfgType>& _path, vector<CfgType>& _newPath);

  private:
    void ValidatePath(vector<CfgType>& _path,
        vector<CfgType>& _path1, vector<CfgType>& _path2);

    size_t m_iter, m_maxIter;
    ClearanceUtility<MPTraits> m_clearanceUtility;
};

template<class MPTraits>
CRetractionPathModifier<MPTraits>::
CRetractionPathModifier(size_t _iter,
        const ClearanceUtility<MPTraits>& _cu) :
  PathModifierMethod<MPTraits>(), m_iter(_iter), m_clearanceUtility(_cu) {
    this->SetName("CRetractionPathModifier");
  }

template<class MPTraits>
CRetractionPathModifier<MPTraits>::
CRetractionPathModifier(MPProblemType* _problem, XMLNode& _node) :
  PathModifierMethod<MPTraits>(_problem, _node),
  m_clearanceUtility(_problem, _node) {
    this->SetName("CRetractionPathModifier");
    m_iter = _node.Read("iter", true, 10, 0, MAX_INT, "Loop iterations");
    m_maxIter = _node.Read("maxIter", false, 1000, 0, MAX_INT,
        "Max loop iterations");
  }

template<class MPTraits>
void
CRetractionPathModifier<MPTraits>::
Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\titerations: " << m_iter << endl;
  _os << "\tclearance utility: " << endl;
  m_clearanceUtility.Print(_os);
}


template<class MPTraits>
bool
CRetractionPathModifier<MPTraits>::
ModifyImpl(vector<CfgType>& _path, vector<CfgType>& _newPath) {
  //C-Retraction
  //Input: _path
  //Output: _newPath
  //
  //path <- _path
  //loop
  //  path' <- path
  //  dir <- RANDDir(step)
  //  for each q in path' do
  //    q' <- q + dir
  //    if(CLEARANCE(q') > CLEARANCE(q)) then
  //      q <- q'
  //  path <- ValidatePath(path, path');
  //_newPath <- path
  //return _newPath

  //TMP
  vector<CfgType> p;
  this->RemoveBranches(m_clearanceUtility.GetDistanceMetricLabel(), _path, p);

  Environment* env = this->GetEnvironment();
  shared_ptr<Boundary> boundary = env->GetBoundary();
  DistanceMetricPointer dm =
    this->GetDistanceMetric(m_clearanceUtility.GetDistanceMetricLabel());

  double step = min(env->GetPositionRes(), env->GetOrientationRes());

  vector<CfgType> p1 = p;

  deque<double> avgs;
  size_t i = 0;
  while(i < m_maxIter &&
      (avgs.size() < m_iter || fabs(avgs.front() - avgs.back()) > step/10)) {
    if(this->m_debug)
      cout << "CRetraction: Iteration: " << i++ << endl;
    vector<CfgType> p2 = p1;

    CfgType dir;
    dir.GetRandomRay(step, dm);

    int j = 0;
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit = p2.begin()+1; cit+1 != p2.end(); ++cit) {

      CfgType q = *cit + dir;

      CfgType clr1, clr2;
      CDInfo cdInfo1, cdInfo2;

      bool ci1 = m_clearanceUtility.CollisionInfo(q, clr1, boundary, cdInfo1);
      if(ci1) {
        bool ci2 = m_clearanceUtility.CollisionInfo(
            *cit, clr2, boundary, cdInfo2);
        if(ci2 && cdInfo1.m_minDist > cdInfo2.m_minDist) {
          *cit = q;
          j++;
        }
      }
    }
    if(this->m_debug)
      cout << j << " cfgs altered" << endl;

    vector<CfgType> p;
    ValidatePath(p1, p2, p);
    p1 = p;

    //calculate average clearance
    double avgClr = 0;
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit = p1.begin(); cit != p1.end(); ++cit) {
      CfgType clr;
      CDInfo cdInfo;
      bool ci = m_clearanceUtility.CollisionInfo(*cit, clr, boundary, cdInfo);
      if(ci)
        avgClr += cdInfo.m_minDist;
    }
    avgClr /= p1.size();

    avgs.push_back(avgClr);
    if(avgs.size() > m_iter)
      avgs.pop_front();
  }

  _newPath = p1;

  return true;
}

template<typename MPTraits>
void
CRetractionPathModifier<MPTraits>::
ValidatePath(vector<CfgType>& _path,
    vector<CfgType>& _path1, vector<CfgType>& _path2) {
  //ValidatePath
  //Input: path, path'
  //Output: path''
  //
  //i <- 0
  //path'' <- empty
  //while(i < |path|-1 do
  //  path'' <- path'' + q_i'
  //  if(d(q_i', q_i+1') > step) then
  //    q_int' <- Interpolate(q_i', q_i+1', 0.5)
  //    if(Clr(q_int') > Clr(q_i+1) then
  //      path'' <- path'' + q_int'
  //    else
  //      path'' <- path'' + q_i+1
  //  i <- i+1
  //path'' <- path'' + q_i'
  //path'' <- RemoveBranches(path'')

  Environment* env = this->GetEnvironment();
  shared_ptr<Boundary> boundary = env->GetBoundary();
  DistanceMetricPointer dm =
    this->GetDistanceMetric(m_clearanceUtility.GetDistanceMetricLabel());

  double step = min(env->GetPositionRes(), env->GetOrientationRes());

  size_t i = 0;
  _path2.clear();

  while(i < _path.size() - 1) {
    _path2.push_back(_path1[i]);
    double dist = dm->Distance(_path1[i], _path1[i+1]);
    if(dist > step) {

      CfgType q = (_path1[i] + _path1[i+1])/2;

      CfgType clr1, clr2, clr3;
      CDInfo cdInfo1, cdInfo2, cdInfo3;

      //Must check both _path[i] and _path[i+1] as algorithm above
      //and theory in 2007 paper is incorrect
      bool ci1 = m_clearanceUtility.CollisionInfo(q, clr1, boundary, cdInfo1);
      bool ci2 = m_clearanceUtility.CollisionInfo(
          _path[i], clr2, boundary, cdInfo2);
      bool ci3 = m_clearanceUtility.CollisionInfo(
          _path[i+1], clr3, boundary, cdInfo3);
      if(ci1 &&
          ((ci2 && cdInfo1.m_minDist > cdInfo2.m_minDist) ||
           (ci3 && cdInfo1.m_minDist > cdInfo3.m_minDist)))
        _path2.push_back(q);
      else {
        _path2.push_back(_path[i]);
        _path2.push_back(_path[i+1]);
      }
    }
    i++;
  }
  _path2.push_back(_path1[i]);

  vector<CfgType> p = _path2;
  this->RemoveBranches(m_clearanceUtility.GetDistanceMetricLabel(), p, _path2);
}

#endif

