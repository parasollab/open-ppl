#ifndef LOCALMANEUVERINGSTRATEGY_H
#define LOCALMANEUVERINGSTRATEGY_H

#ifdef PMPSSSurfaceMult

#include "Utilities/IOUtils.h"
#include "Utilities/MetricUtils.h"
#include "MPStrategyMethod.h"
#include "Cfg/SSSurfaceMult.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// A strategy for a car that needs to exit a parking space this is based on
/// Reed-Shepp curves, which is basically concatenating circles to achieve a
/// stabilized trajectory. This version is modified with some randomization to
/// plan for obstacles in the env.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class LocalManeuveringStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::MPProblemType* MPProblemPtr;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename vector<CfgType>::iterator CfgIter;
    typedef typename MPTraits::CfgType::CompositeCfgType CompositeCfgType;

    LocalManeuveringStrategy(MPProblemPtr _problem, XMLNode& _node, bool _warnXML=true);
    LocalManeuveringStrategy();
    virtual ~LocalManeuveringStrategy();

    typedef vector< pair<Point2d,double> > Path3D;

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

    struct CompareStartTimes {
      bool operator() (const pair<int,int>& _lhs, const pair<int,int>& _rhs) const{
        return _lhs.first < _rhs.first;
      }
    };
    //////////////////////////////////////////
    //Specific functions for this strategy///
    /////////////////////////////////////////

    //AttemptTrajectory, will attempt to plan a trajectory from one configuration to another by concatenating
    //circles of radius between the vehicle's possible minimal turning radius and infinity (a straight line)
    //this behavior is specialized for exiting a parking space, where the maximal number of concatenations is
    //3. For a "forward" exiting maneuever, the car only needs to follow one circle (and then reactive
    //behaviors can take over), and then go straight. A backwards maneuver first backs straight out a bit,
    //turns, and then moves forward a bit toward a goal, at this point, reactive behaviors may take over.
    //returns true if trajectory successfully planned, false otherwise
    bool AttemptPullOutTrajectory( size_t _startTick, size_t _index);
    bool AttemptPullInTrajectory( size_t _startTick, size_t _index);
    bool AttemptDeadlockTrajectory( size_t _startTick, size_t _index);
    bool AttemptDeadlockTrajectory2( size_t _startTick, size_t _index);

    //writes the list of configurations along the trajectory (_path) to a file (_outputFile)
    void WriteGBPathConfigurations(string _outputFile, vector<CfgType>& _path);
    void WriteVizmoPathConfigurations(const string _outputFile, vector<CfgType>& _path, Environment* _env );

    //BuffPath will take the agent at _index, and check to see if it managed to plan a path until the end of
    //m_gbpaths. If not, then it will fill out the vacant spaces to the end with the last generated
    //configuration. This is to avoid the agent "popping" back to its start position
    void BuffPath(size_t _index);

    //EvaluateGoalReach checks to see if an agent is within m_goalReachdistance of its goal If
    //so, returns true. Else returns false
    bool EvaluateGoalReach(size_t _index, Point2d _goal, double _tolerance=1.0);

    //CombinePaths will combine the input vector of cfgs with _newPath
    //that is, for each overlapping config, the one in _originalPath is replaced by the one in _newPath
    //overlap is determined by _plantime which is the start time and stop time that _newPath runs for
    void CombinePaths(vector<CfgType>& _originalPath, vector<CfgType>& _newPath, size_t _index);

    bool TryPlan(size_t& _startTick, size_t _cfgIndex, int _planSteps, vector<CfgType>& _potentialPath, double steeringAngle,
    bool reverse=false);

    //TryPlanTowardGoal will take the guide_path, find the agent's goal on the guide path, and plan directly
    //toward via the strategy of turning at its maximum steering angle, and then moving forward when facing
    //that point within some small tolerance (~10 degrees). If _divisor is set above 1.0, then the agent will
    //turn at that fraction of its maximum steering angle
    bool TryPlanTowardGoal(size_t& _startTick, size_t _cfgIndex, int _planSteps,
        vector<CfgType>& _potentialPath, Point2d _goal, double _orientationTolerance=0.10, double _divisor=1.0);

    //Gets and Sets for all variables
    std::string GetPlanType(){return m_planType;}
    void SetPlanType(string _newPlanType){m_planType = _newPlanType;}

    double GetDelta(){return m_delta;}
    void SetDelta(double _newDelta){m_delta = fabs(_newDelta);}

    size_t GetMaxNumTicks(){return m_maxNumTicks;}
    void SetMaxNumTicks(size_t _newMaxNumTicks){m_maxNumTicks = _newMaxNumTicks;}

    size_t GetMaxNumIters(){return m_maxNumIter;}
    void SetMaxNumIterss(size_t _newMaxNumIters){m_maxNumIter = _newMaxNumIters;}

    size_t GetNumRoots(){return m_numRoots;}
    void SetNumRoots(size_t _newNumRoots){m_numRoots = _newNumRoots;}

    string GetValidityChecker(){return m_vc;}
    void SetValidityChecker(string _newVC){m_vc = _newVC;}

    string GetQueryFile(){return m_query;}
    void SetQueryFile(string _newQueryFile){m_query = _newQueryFile;}

    string GetVizmoPathFile(){return m_path;}
    void SetVizmoPathFile(string _newVizmoPathFile){m_path = _newVizmoPathFile;}

    string GetGBPathFile(){return m_gbPath;}
    void SetGBPathFile(string _newGBPathFile){m_gbPath = _newGBPathFile;}

    double GetGoalReachDistance(){return m_goalReachDistance;}
    void SetGoalReachDistance(double _newGoalReach){m_goalReachDistance = fabs(_newGoalReach);}

    double GetGoalReachOrientation(){return m_goalReachOrientation;}
    void SetGoalReachOrientation(double _newGoalReach){m_goalReachOrientation = fabs(_newGoalReach);}

    size_t GetGuidePathSubGoal(){return m_guidePathSubGoal;}
    void SetGuidePathSubGoal(size_t _newSubGoal){m_guidePathSubGoal = _newSubGoal;}

    string GetGuidePathFile(){return m_guidePathFile;}
    void SetGuidePathFile(string _newGuidePath){m_guidePathFile = _newGuidePath;}

    size_t GetMaxStartTime(){return m_maxStartTime;}
    void SetMaxStartTime(size_t _newMaxStart){m_maxStartTime = _newMaxStart;}

    double GetMinForwardPullOutDistMin(){return m_forwardPullOutDistMin;}
    void SetMinForwardPullOutDistMin(double _newDist){m_forwardPullOutDistMin = fabs(_newDist);}

    double GetMinForwardPullOutDistMax(){return m_forwardPullOutDistMax;}
    void SetMinForwardPullOutDistMax(double _newDist){m_forwardPullOutDistMax = fabs(_newDist);}

    double GetForwardAngleTolerance(){return m_forwardReqAngle;}
    void SetForwardAngleTolerance(double _newAngle){m_forwardReqAngle = NormalizeTheta(fabs(_newAngle));}

    double GetForwardPullOutTurnDistMin(){ return m_forwardPullOutTurnDistMin;}
    void SetForwardPullOutTurnDistMin(double _newDist){ m_forwardPullOutTurnDistMin = fabs(_newDist);}

    double GetForwardPullOutTurnDistMax(){ return m_forwardPullOutTurnDistMax;}
    void SetForwardPullOutTurnDistMax(double _newDist){ m_forwardPullOutTurnDistMax = fabs(_newDist);}

    double GetForwardPullOutSteerMin(){return m_forwardPullOutSteerMin;}
    void SetForwardPullOutSteerMin(double _newAngle){m_forwardPullOutSteerMin = NormalizeTheta(fabs(_newAngle));}

    double GetForwardPullOutSteerMax(){return m_forwardPullOutSteerMax;}
    void SetForwardPullOutSteerMax(double _newAngle){m_forwardPullOutSteerMax = NormalizeTheta(fabs(_newAngle));}

    double GetBackPullOutStraightDistMin(){return m_backPullOutDistMin;}
    void GetBackPullOutStraightDistMin(double _newDist){m_backPullOutDistMin = fabs(_newDist);}

    double GetBackPullOutStraightDistMax(){return m_backPullOutDistMax;}
    void GetBackPullOutStraightDistMax(double _newDist){m_backPullOutDistMax = fabs(_newDist);}

    double GetBackPullOutTurnDistMin(){ return m_backPullOutTurnDistMin;}
    void SetBackPullOutTurnDistMin(double _newDist){ m_backPullOutTurnDistMin = fabs(_newDist);}

    double GetBackPullOutTurnDistMax(){ return m_backPullOutTurnDistMax;}
    void SetBackPullOutTurnDistMax(double _newDist){ m_backPullOutTurnDistMax = fabs(_newDist);}

    double GetBackPullOutGoalDistMin(){return m_backTowardGoalDistMin;}
    void SetBackPullOutGoalDistMin(double _newDist){m_backTowardGoalDistMin = fabs(_newDist);}

    double GetBackPullOutGoalDistMax(){return m_backTowardGoalDistMax;}
    void SetBackPullOutGoalDistMax(double _newDist){m_backTowardGoalDistMax = fabs(_newDist);}

    double GetBackPullOutSteerMin(){return m_backPullOutTurnSteerMin;}
    void SetBackPullOutSteerMin(double _newAngle){m_backPullOutTurnSteerMin = NormalizeTheta(fabs(_newAngle));}

    double GetBackPullOutSteerMax(){return m_backPullOutTurnSteerMax;}
    void SetBackPullOutSteerMax(double _newAngle){m_backPullOutTurnSteerMax = NormalizeTheta(fabs(_newAngle));}

    double GetDeadLock1RightTurnFreq(){return m_deadlock1RightTurnFreq;}
    void SetDeadLock1RightTurnFreq(double _newFreq){m_deadlock1RightTurnFreq = max(min(1.0, _newFreq),0.0);}

    double GetDeadlock1TurnDistMin(){return m_deadlock1TurnDistMin;}
    void GetDeadlock1TurnDistMin(double _newDist){m_deadlock1TurnDistMin = fabs(_newDist);}

    double GetDeadlock1TurnDistMax(){return m_deadlock1TurnDistMax;}
    void GetDeadlock1TurnDistMax(double _newDist){m_deadlock1TurnDistMax = fabs(_newDist);}

    double GetDeadlock1TowardGoalDistMin(){return m_deadlock1TowardGoalDistMin;}
    void SetDeadlock1TowardGoalDistMin(double _newDist){m_deadlock1TowardGoalDistMin = fabs(_newDist);}

    double GetDeadlock1TowardGoalDistMax(){return m_deadlock1TowardGoalDistMax;}
    void SetDeadlock1TowardGoalDistMax(double _newDist){m_deadlock1TowardGoalDistMax = fabs(_newDist);}

    double GetDeadlock1TurnSteerMin(){return m_deadlock1TurnSteerMin;}
    void SetDeadlock1TurnSteerMin(double _newAngle){m_deadlock1TurnSteerMin = NormalizeTheta(fabs(_newAngle));}

    double GetDeadlock1TurnSteerMax(){return m_deadlock1TurnSteerMax;}
    void SetDeadlock1TurnSteerMax(double _newAngle){m_deadlock1TurnSteerMax = NormalizeTheta(fabs(_newAngle));}

    double GetDeadlock2DistMin(){return m_deadlock2DistMin;}
    void SetDeadlock2DistMin(double _newDist){m_deadlock2DistMin = fabs(_newDist);}

    double GetDeadlock2DistMax(){return m_deadlock2DistMax;}
    void SetDeadlock2DistMax(double _newDist){m_deadlock2DistMax = fabs(_newDist);}

    double GetDeadlock2Divisor(){return m_deadlock2Divisor;}
    void GetDeadlock2Divisor(double _newDivisor){m_deadlock2Divisor = _newDivisor;}

    double GetForwardPullInProbability(){return m_pullInForwardProbability;}
    void SetForwardPullInProbability(double _newProb){m_pullInForwardProbability = max(min(1.0,_newProb),0.0);}

    double GetBackwardPullInNormalProbability(){return m_pullInNormalBackwardProbability;}
    void SetBackwardPullInNormalProbability(double _newProb){
      m_pullInNormalBackwardProbability = max(min(1.0,_newProb),0.0);}

    double GetPullInMinSteeringAngle(){return m_pullInSteerMin;}
    void SetPullInMinSteeringAngle(double _newAngle){m_pullInSteerMin = NormalizeTheta(fabs(_newAngle));}

    //interface functions for GB direct interface
    void SetGoals(vector<CfgType>& _goals) { m_goals=_goals; }
    void SetRoots(vector<CfgType>& _roots) { m_roots=_roots; }
    void SetGuidePath(Path3D _gp) { m_guidePath=_gp; }
    bool GetSuccessfulPlanFound() { return m_successfulPlanFound; }
    vector<CfgType>& GetGeneratedTrajectories() { return m_gbPaths; }

  protected:
    string m_vc; //validy checker
    string m_query; //path for query file start and goal configurations)
    string m_path; //path file, where we output the solution to the query in terms of nodes
    string m_gbPath; //path to group behaviors file, which will contain the full path including intermediate
    //								configurations between nodes in the map
    string m_planType; //PullOut, Deadlock1, Deadlock2, PullIn
    string m_guidePathFile; //path to file of subgoals (pts along medial axis typically)
    double m_delta; //how much a configuration changes from tick to tick (mangitude of dt)
    double m_goalReachDistance; //threshold on distance to goal
    double m_goalReachOrientation; //threshold in radians on facing goal
    size_t m_numRoots; //number of roots on the tree
    size_t  m_guidePathSubGoal; //index into subgoal path to plan towards
    int m_currentIteration; //current attempt to create a feasible trajectory
    size_t m_maxNumTicks; //maximum number of times to apply a control
    size_t m_maxNumIter; //maximum number of attempted trajectories until considered failure
    VID m_goal; //VID of configuration in our tree at the goal
    vector<CfgType> m_roots; //starting (input) configurations of each agent
    vector<CfgType> m_goals; //guide path sub goals
    bool m_successfulPlanFound;
    vector<size_t> m_goalsNotFound; //goals along the guide path that haven't yet been reached
    vector<CfgType> m_gbPaths; //our pseudo-roadmap containing all the paths for all agents
    vector<pair<int,int> > m_planTimes; //tracks time agent began planning, and how long each agent planned

    Path3D  m_guidePath; //the guide path
    vector< Path3D > m_perAgentPath; //for each agent we're planning for, this is their path

    //variables for trajectories
    size_t m_maxStartTime;
    double m_forwardPullOutDistMin, m_forwardPullOutDistMax, m_forwardPullOutTurnDistMin, m_forwardPullOutTurnDistMax,
           m_forwardReqAngle, m_backPullOutDistMin,m_backPullOutDistMax,m_backPullOutTurnDistMin,m_backPullOutTurnDistMax,
           m_backTowardGoalDistMin,m_backTowardGoalDistMax, m_deadlock1RightTurnFreq,
           m_deadlock1TurnDistMin,m_deadlock1TurnDistMax,m_deadlock1TowardGoalDistMin,m_deadlock1TowardGoalDistMax,
           m_deadlock2DistMin,m_deadlock2DistMax;

    double m_forwardPullOutSteerMin, m_forwardPullOutSteerMax,m_backPullOutTurnSteerMin,m_backPullOutTurnSteerMax,
    m_deadlock1TurnSteerMin,m_deadlock1TurnSteerMax,m_deadlock2Divisor;

    double m_pullInForwardProbability,m_pullInNormalBackwardProbability, m_pullInSteerMin;
    //debug
    Vector2d m_debugGlobalGoalDir;
};

template<class MPTraits>
LocalManeuveringStrategy<MPTraits>::LocalManeuveringStrategy() :
  MPStrategyMethod<MPTraits>(){
    this->SetName("LocalManeuveringStrategy");
    m_delta = 0.05;
    m_maxNumTicks = 1;
    //m_maxNumIter = 1000;
    //m_maxNumIter = 300;
    m_maxNumIter = 300;
    m_numRoots =  1;
    m_vc = "";
    m_query = "";
    m_path = "";
    m_gbPath = "";
    //m_goalReachDistance = 1.0;
    m_goalReachDistance = 5.0;
    m_goalReachOrientation = 0.34;
    //m_guidePathSubGoal =  3;
    m_guidePathSubGoal = 15;
    m_guidePathFile = "";
    m_planType = "";

    //parameterize the behaviors
    //m_maxStartTime = 50;
    m_maxStartTime = 50;
    m_forwardPullOutDistMin = 1.25;
    m_forwardPullOutDistMax  = 6.0;
    m_forwardReqAngle  = 0.52536;
    m_forwardPullOutTurnDistMin = 10.0;
    m_forwardPullOutTurnDistMax  = 22.5;
    //m_forwardPullOutSteerMin = 0.10;
    //m_forwardPullOutSteerMax = 0.15;
    m_forwardPullOutSteerMin = 0.05;
    m_forwardPullOutSteerMax = 0.08;

    m_backPullOutDistMin = 6.75;
    m_backPullOutDistMax = 12.75;
    //m_backPullOutTurnDistMin = 10.0;
    //m_backPullOutTurnDistMax = 15.0;
    m_backPullOutTurnDistMin = 10.0;
    m_backPullOutTurnDistMax = 15.0;
    //m_backTowardGoalDistMin = 0.25;
    //m_backTowardGoalDistMax = 7.75;
    m_backTowardGoalDistMin = 0.25;
    m_backTowardGoalDistMax = 2.75;
    m_backPullOutTurnSteerMin = 0.05;
    m_backPullOutTurnSteerMax = 0.20;
    //m_backPullOutTurnSteerMax = 0.20;

/*
    m_deadlock1RightTurnFreq = 0.75;
    m_deadlock1TurnDistMin = 3.75;
    m_deadlock1TurnDistMax = 7.50;
    m_deadlock1TowardGoalDistMin = 2.5;
    m_deadlock1TowardGoalDistMax = 12.25;
    m_deadlock1TurnSteerMin = 0.05;
    m_deadlock1TurnSteerMax = 0.20;
*/
    m_deadlock1RightTurnFreq = 0.75;
    m_deadlock1TurnDistMin = 3.25;
    m_deadlock1TurnDistMax = 7.50;
    m_deadlock1TowardGoalDistMin = 2.5;
    m_deadlock1TowardGoalDistMax = 10.25;
    m_deadlock1TurnSteerMin = 0.05;
    m_deadlock1TurnSteerMax = 0.15;

    //m_deadlock2DistMin = 12.5;
    //m_deadlock2DistMax = 22.5;
    //m_deadlock2DistMin = 7.5;
    //m_deadlock2DistMax = 16.5;
    m_deadlock2DistMin = 4.5;
    m_deadlock2DistMax = 10.5;
    m_deadlock2Divisor = 0.5;

    m_pullInForwardProbability = 0.75;

    m_pullInNormalBackwardProbability = 0.5;
    m_pullInSteerMin = 0.10;

    m_successfulPlanFound=false;
}

template<class MPTraits>
LocalManeuveringStrategy<MPTraits>::LocalManeuveringStrategy(MPProblemPtr _problem, XMLNode& _node, bool _warnXML) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_currentIteration(0){
    ParseXML(_node);
    //if (_warnXML) _node.warnUnrequestedAttributes();
    if(this->m_debug && _warnXML) Print(cout);
    m_goal = -1;
    this->SetName("LocalManeuveringStrategy");
  }

template<class MPTraits>
LocalManeuveringStrategy<MPTraits>::~LocalManeuveringStrategy() {
  //empty destructor
}

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::ParseXML(XMLNode& _node) {

  m_delta = _node.Read("delta", false, 0.05, 0.0, 1.0, "Delta Distance");
  m_maxNumTicks = _node.Read("maxnumticks", false, 1, 0, MAX_INT, "Max. Number of Ticks");
  m_maxNumIter = _node.Read("maxnumiter", false, 10000, 0, MAX_INT, "Max. Number of Iterations");
  m_numRoots = _node.Read("numRoots", false, 5, 1, MAX_INT, "Number of Roots");
  m_vc = _node.Read("vc", true, "", "Validity Test Method");
  m_query = _node.Read("query", false, "", "Query Filename");
  m_path = _node.Read("path", false, "", "Output Path Filename");
  m_gbPath = _node.Read("gbpath", false, "", "Output GBPath Filename");
  m_goalReachDistance =  _node.Read("goalreachdistance", false, 1.0, 0.0, 1000.0,  "Distance tolerance to goal");
  m_goalReachOrientation = _node.Read("goalreachorientation", false, 0.34, 0.0, 1.51, "Orientation tolerance for goal");
  m_guidePathSubGoal =  _node.Read("guidepathsubgoal", false, 0, 0, MAX_INT, "Index in guide path file considered goal");
  m_guidePathFile = _node.Read("guidepath", false, "", "Guide Path Filename");
  m_planType = _node.Read("plantype",false,"PullOut","Type of parking lot planning to do.");

  //parameterize the behaviors
  m_maxStartTime = _node.Read("maxStartTime", false, 50, 0, 100000, "When randomizing start times, \
      this value is the maximum timestep at which an agent may begin planning.");
  m_forwardPullOutDistMin = _node.Read("forwardPullOutDistMin",false,1.25,0.0,10000.0,"For forward \
      pullout, how many units at minimum to initially move straight forward");
  m_forwardPullOutDistMax  = _node.Read("forwardPullOutDistMax",false,6.0,0.0,10000.0,"For \
      forward pullout, how many units at most to initially move straight forward");
  m_forwardReqAngle  = _node.Read("forwardReqAngle",false,0.52536,0.0,1.5707,"For \
      forward pullout, tolerance in facing the goal to still be considered 'forward'");
  m_forwardPullOutTurnDistMin = _node.Read("forwardPullOutTurnDistMin",false,10.0,0.0,1000.0,"For forward \
      pullout, how far along a turning arc at minimum to turn");
  m_forwardPullOutTurnDistMax  = _node.Read("forwardPullOutTurnDistMax",false,22.5,0.0,1000.0,"For \
      forward pullout, how far along a turning arc at most to turn");
  m_forwardPullOutSteerMin = _node.Read("forwardPullOutSteerMin",false,0.10,0.0,0.22,"For forward \
      pullout, minimum absolute steering angle when turning");
  m_forwardPullOutSteerMax = _node.Read("forwardPullOutSteerMax",false,0.15,0.0,0.22,"For forward \
      pullout, maximum absolute steering angle when turning");

  m_backPullOutDistMin = _node.Read("backPullOutDistMin",false,3.75,0.0,10000.0,"For reverse \
      pullout, how far to initally move straight back at minimum");
  m_backPullOutDistMax = _node.Read("backPullOutDistMax",false,8.75,0.0,10000.0,"For reverse \
      pullout, how far to initially move straight back at most");
  m_backPullOutTurnDistMin = _node.Read("backPullOutTurnDistMin",false,10.0,0.0,10000.0,"For reverse \
      pullout, how far along a turning arc at minimum to travel");
  m_backPullOutTurnDistMax = _node.Read("backPullOutTurnDistMax",false,15.0,0.0,10000.0,"For reverse \
      pullout, how far along a turning arc at most to travel");
  m_backTowardGoalDistMin = _node.Read("backTowardGoalDistMin",false,0.25, 0.0,100000.0,"For reverse \
      pullout, how far to travel forward, at minimum, toward a goal after backing and turning");
  m_backTowardGoalDistMax = _node.Read("backTowardGoalDistMax",false,7.75, 0.0,100000.0,"For reverse \
      pullout, how far to travel forward, at most, toward a goal after backing and turning");
  m_backPullOutTurnSteerMin = _node.Read("backPullOutTurnSteerMin",false,0.05,0.0,0.22,"For reverse pullout, \
      minimum absolute steering angle at for turning while reversing");
  m_backPullOutTurnSteerMax = _node.Read("backPullOutTurnSteerMax",false,0.20,0.0,0.22,"For reverse pullout, \
      maximum absolute steering angle at for turning while reversing");


  m_deadlock1RightTurnFreq = _node.Read("deadlock1RightTurnFreq",false,0.75,0.0,1.0,"For deadlock1 \
      resolution, how often to attempt a right hand turn to get around other agents");
  m_deadlock1TurnDistMin = _node.Read("deadlock1TurnDistMin",false,3.75,0.0,100000.0,"For deadlock1 \
      resolution, how far along an arc to initially turn, at min");
  m_deadlock1TurnDistMax = _node.Read("deadlock1TurnDistMax",false,7.50,0.0,100000.0,"For deadlock1 \
      resolution, how far along an arc to intially turn, at most");
  m_deadlock1TowardGoalDistMin = _node.Read("deadlock1TowardGoalDistMin",false,2.5,0.0,100000.0,"For \
      deadlock1 resolution, how far to travel forward after turning, at min");
  m_deadlock1TowardGoalDistMax = _node.Read("deadlock1TowardGoalDistMax",false,12.25,0.0,100000.0,"For \
      deadlock1 resolution, how far to travel forward after turning, at most");
  m_deadlock1TurnSteerMin = _node.Read("deadlock1TurnSteerMin",false,0.05,0.0,0.22,"For \
      deadlock1 resolution, minimum absolute steering angle for the turn.");
  m_deadlock1TurnSteerMax = _node.Read("deadlock1TurnSteerMax",false,0.20,0.0,0.22,"For \
      deadlock1 resolution, maximum absolute steering angle for the turn.");

  m_deadlock2DistMin = _node.Read("deadlock2DistMin",false,12.5,0.0,100000.0,"For deadlock2 \
      resolution, how far at minimum to plan toward a goal");
  m_deadlock2DistMax = _node.Read("deadlock2DistMax",false,22.5,0.0,100000.0,"For deadlock2 \
      resolution, how far at most to plan toward a goal");
  m_deadlock2Divisor = _node.Read("deadlock2Divisor",false,0.5,0.0,5.0,"For deadlock2 \
      resolution, maximum amount to divide agent's max steering angle by");

  m_pullInForwardProbability = _node.Read("pullInForwardProbability",false,0.75,0.0,1.0,"For \
      pulling into a parking space, probability agent will attempt to pull in forward (not back in).");

  m_pullInNormalBackwardProbability = _node.Read("pullInNormalBackwardProbability",false,0.5,0.0,1.0,"For \
      pulling into a parking space, if agent decides to back in, probability it will go straight forward and \
      then back into the space in one shot versus turning in the opposite direction of the space and then backing in");
  m_pullInSteerMin = _node.Read("pullInSteerMin", false, 0.10, 0.0, 0.22, "For pulling into a parking \
      spot, we may turn at our maximum or randomly choose to turn at a value less than that. This is the minimum.");
}//end ParseXML

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::Print(ostream& _os) const {
  _os << "LocalManeuveringStrategy::Print" << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tnumber of roots:: " << m_numRoots << endl;
  _os << "\tmax number of ticks:: " << m_maxNumTicks << endl;
  _os << "\tmax number of iters:: " << m_maxNumIter << endl;
  _os << "\tquery:: " << m_query << endl;
  _os << "\tpath:: " << m_path << endl;
  _os << "\tgbpath:: " << m_gbPath << endl;
  _os << "\tgoal reach tolerance:: " << m_goalReachDistance << endl;
  _os << "\tgoal reach orientation tolerance:: " << m_goalReachOrientation << endl;
  _os << "\tguide path file:: " << m_guidePathFile << endl;
  _os << "\tGuide path subgoal:: " << m_guidePathSubGoal << endl;
  _os << "\tmaximum random start time:: " << m_maxStartTime << endl;
  _os << "\tforward forward distance minimum:: " << m_forwardPullOutDistMin << endl;
  _os << "\tforward forward distance max:: " << m_forwardPullOutDistMax << endl;
  _os << "\tforward turn dist (arc length to travel) minimum:: " << m_forwardPullOutTurnDistMin << endl;
  _os << "\tforward turn dist (arc length to travel) maximum:: " << m_forwardPullOutTurnDistMax << endl;
  _os << "\tforward steering angle min to use:: " << m_forwardPullOutSteerMin << endl;
  _os << "\tforward steering angle max to use:: " << m_forwardPullOutSteerMax << endl;
  _os << "\tforward angle tolerance:: " << m_forwardReqAngle << endl;
  _os << "\tbackward backward distance minimum:: " << m_backPullOutDistMin << endl;
  _os << "\tbackward backward distance maximum:: " << m_backPullOutDistMax << endl;
  _os << "\tbackward turn distance minimum:: " << m_backPullOutTurnDistMin << endl;
  _os << "\tbackward turn distance maximum:: " << m_backPullOutTurnDistMax << endl;
  _os << "\tbackward steering angle min:: << " << m_backPullOutTurnSteerMin << endl;
  _os << "\tbackward steering angle max:: << " << m_backPullOutTurnSteerMax << endl;
  _os << "\tbackward distance toward goal to move min:: " << m_backTowardGoalDistMin << endl;
  _os << "\tbackward distance toward goal to move max:: " << m_backTowardGoalDistMax << endl;
  _os << "\tdeadlock1 probability of right turn:: " << m_deadlock1RightTurnFreq << endl;
  _os << "\tdeadlock1 turn distance min:: " << m_deadlock1TurnDistMin << endl;
  _os << "\tdeadlock1 turn distance max:: " << m_deadlock1TurnDistMax << endl;
  _os << "\tdeadlock1 forward portion min distance:: " << m_deadlock1TowardGoalDistMin << endl;
  _os << "\tdeadlock1 forward portion max distance:: " << m_deadlock1TowardGoalDistMax << endl;
  _os << "\tdeadlock1 minimum steering angle for turn:: " << m_deadlock1TurnSteerMin << endl;
  _os << "\tdeadlock1 maximum steering angle for turn:: " << m_deadlock1TurnSteerMax << endl;
  _os << "\tdeadlock2 distance to move toward goal min:: " << m_deadlock2DistMin << endl;
  _os << "\tdeadlock2 distance to move toward goal max:: " << m_deadlock2DistMax << endl;
  _os << "\tdeadlock2 maximum amount to divide maxSteeringAngle by:: " << m_deadlock2Divisor << endl;
  _os << "\tpull-in probability of forward pull-in:: " << m_pullInForwardProbability << endl;
  _os << "\tpull-in probability of normal (versus method2) backward pull-in given a backward-pull in:: " <<
    m_pullInNormalBackwardProbability << endl;
  _os << "\tpull-in minimum steering angle:: " << m_pullInSteerMin << endl;

  _os << "\tplanType: " << m_planType << endl;
  _os << " Set roots: " << endl;
  for(int i=0; i<(int)m_roots.size(); i++) {
     _os << " root: " << i << " ["<< m_roots[i]<<"]"<<endl;
  }
  _os << " Set goals: " << endl;
  for(int i=0; i<(int)m_goals.size(); i++) {
     _os << " root: " << i << " ["<< m_goals[i]<<"]"<<endl;
  }
}

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::WriteVizmoPathConfigurations(const string _outputFile, vector<CfgType>& _path, Environment* _env ) {
  ofstream ofs(_outputFile.c_str());

  ofs << "VIZMO_PATH_FILE   Path Version " << 2012 << endl;
  //ofs << "1" <<endl;

  //print out the number of multibodies
  if (_path.empty()){
    ofs.close();
    return;
  }
  ofs << _path.begin()->GetNumCfgs() << endl;
  ofs << _path.size() << endl;

  for(size_t i = 0 ; i < _path.size() ; i++){
    _path[i].Write(ofs);
  }
  ofs.close();
}


template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::WriteGBPathConfigurations(string _outputFile, vector<CfgType>& _path) {
  ofstream ofs(_outputFile.c_str());

  ofs << _path.size() << endl;

  for(size_t i = 0 ; i < _path.size() ; i++){
    ofs << _path[i] << endl;
  }
  ofs.close();

}//end WriteGBPathConfigurations

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::Initialize(){
/*
  if (this->m_debug) cout << "In LocalManeuveringStrategy::Initialize" << endl;
#ifndef PMPSSSurfaceMult
  cerr << "LocalManeuveringStrategy only defined for SSSurfaceMult configuration" << endl;
  exit(1);
#endif

  // Setup MP variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vc);
  string callee = "LocalManeuveringStrategy::Initialize";

  // Setup RRT Variables
  CfgType tmp, tmp2;
  Print(cout);
  if (this->m_debug) {
    cout << "num robots: " << env->GetRobotCount() << endl;
    cout << " validity checker name: " << vc->GetNameAndLabel() << endl;
  }
  //tmp.SetNumCfgs( env->GetRobotCount() );

  //the query file will first contain the starting positions, and then the goal positions
  //first we'll read the start positions
  if(m_goals.size()==0) {
     if(m_query != ""){
	cout << "Query file is " << m_query << endl;
	ifstream ifs(m_query.c_str());
	ifs >> tmp;
	if(!ifs){
	   cerr << "Error, could not read starting configurations from query file: " << m_query.c_str() <<  endl;
	   exit(1);
	}
	m_roots.push_back(tmp);

	//now read goal positions
	CfgType tmp2;
	//tmp2.SetNumCfgs(env->GetRobotCount());
	ifs >> tmp2;

	if(!ifs){
	   cerr << "Error, could not read end configurations from query file." << endl;
	   exit(1);
	}

	m_goals.push_back(tmp2);
     }
     else{
	// Add random start and goal configurations
	// Add root vertex/vertices
	if (this->m_debug)
	   cout << "No query file given." << endl;

	for (size_t i=0; i<m_numRoots; ) {
	   tmp.GetRandomCfg(env);
	   if( vc->IsValid(tmp, callee)){
	      m_roots.push_back(tmp);
	      m_goals.push_back(tmp);
	      ++i;
	   }//end check for valid config
	   else
	      cerr << "config was not valid, cannot plan for it." << endl;
	}//end for loop
     }//end if-else statement
  }

  /////////////read guide path////////////////////////////////////////
  if( m_guidePath.size()==0 ) {
     if(m_guidePathFile != ""){
	ifstream ifs(m_guidePathFile.c_str());
	double x,y,h;
	ifs >> x >> h >> y;
	while(ifs){
	   m_guidePath.push_back( make_pair(Point2d(x,y), h) );
	   ifs >> x >> h >> y;
	}
     }
  }
  //for now assume that each agent is biased by the same path
  for(size_t i=0; i<env->GetRobotCount(); i++)
    m_perAgentPath.push_back( m_guidePath );
  cout << "Number of active bodies: " << env->GetRobotCount() << " number of usable bodies: " << env->GetUsableMultiBodyCount() << endl;
  cout << "Number of roots: " << m_roots.size() << endl;

  for(CfgIter C = m_roots.begin(); C!=m_roots.end(); C++){
    m_gbPaths.push_back(*C);
  }
  cout << "Finished adding vertices to roadmap." << endl;
  //m_planTimes, all agents have initially plans of length 0
  size_t numCfgs = m_gbPaths.front().GetCfgs().size();
  if (numCfgs == 0){
    cerr << "No initial configuration provided" << endl;
    exit(1);
  }
  m_planTimes.assign(numCfgs,make_pair(0,0));

*/
}//end initialize function

template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::EvaluateGoalReach(size_t _cfgIndex, Point2d _goal, double _tolerance){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::EvaluateGoalReach" << endl;

  //compare position of robot to goal position and return true if it's close enough
  CfgType toCheck = m_gbPaths.back();

  Point2d pos = toCheck.GetCfgs().at(_cfgIndex).GetPos();

  double dist = (pos-_goal).norm();
  if (dist <= _tolerance){
    if (this->m_debug)
      cout<<"GOAL REACHED" << endl;
    return true;
  }

  return false;
}//end EvaluateGoalReach method

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::Run(){
  if (this->m_debug)
    cout << "In LocalManeuveringStrategy::Run" << endl;
  if (m_gbPaths.empty()){
    cerr << "Error, there is no starting configuration." << endl;
    exit(1);
  }

  bool trajectorySuccess;
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stats->StartClock("Trajectory Generation");
  std::multiset<pair<size_t,size_t>,CompareStartTimes> startTimes; //map of start times to their cfg indices, sorted by startTime
  size_t numCfgs = m_gbPaths.front().GetCfgs().size();
  bool plantype[4] = {m_planType=="PullOut", m_planType=="Deadlock1", m_planType=="Deadlock2",
    m_planType=="PullIn"};

  if (this->m_debug){
    if (plantype[0])
      cout << "Performing Parking Space exiting" << endl;
    else if (plantype[1])
      cout << "Performing deadlock resolution, case 1, head on deadlock " << endl;
    else if (plantype[2])
      cout << "Performing deadlock resolution, case 2, perpendicular deadlock" << endl;
    else if (plantype[3])
      cout << "Performing parking space entering"  << endl;
    else{
      cerr << "Unrecognized planning type: " << m_planType <<". Values are: PullOut, PullIn, Deadlock1, Deadlock2" <<
        endl;
      exit(1);
    }
  }

  cerr << "Number of agents to determine start times for: " << numCfgs << " plantype: " << m_planType << endl;
  size_t rind = LRand() % numCfgs;
  for (size_t j=0;j<numCfgs;j++){ //determine all start times
    //determine a random start time for this agent between (0, m_maxStartTime) inclusive
    size_t startTick = LRand()%(m_maxStartTime+1);
    //experimental///////////////////////////////
    /*if( numCfgs == 1 ) startTick=0;
    else {
       if(j==rind) startTick = 0; //the first one will start it off
    }*/
    /////////////////////////////////////////////
    startTimes.insert(make_pair(startTick,j));
  }

  for (typename multiset<pair<size_t,size_t>,CompareStartTimes>::iterator setIter = startTimes.begin(); setIter!=startTimes.end();setIter++){
    size_t startTick = setIter->first;
    //only trying to get one trajectory for each agent
    trajectorySuccess = false;
    for (size_t i=0;i<m_maxNumIter && !trajectorySuccess;i++){
      //ensure that startTime is not longer than m_gbPaths, else duplicate m_gbPaths.back()
      if (startTick >= m_gbPaths.size())
      {
        for(int i=(int)startTick - (int)m_gbPaths.size();i>=0;--i){
          m_gbPaths.push_back(m_gbPaths.back());
        }
      }
      m_planTimes.at(setIter->second).first = startTick;
      if (plantype[0])
        trajectorySuccess = AttemptPullOutTrajectory( startTick, setIter->second);
      else if (plantype[1])
        trajectorySuccess = AttemptDeadlockTrajectory( startTick, setIter->second);
      else if (plantype[2])
        trajectorySuccess = AttemptDeadlockTrajectory2( startTick, setIter->second);
      else if (plantype[3])
        trajectorySuccess = AttemptPullInTrajectory( startTick, setIter->second);
    }
  }
  cerr << "Run method finished with code: " << trajectorySuccess << endl;
  m_successfulPlanFound=trajectorySuccess;
  stats->StopClock("Trajectory Generation");
}//end Run method

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::Finalize(){
  cerr << "In finalize" << endl;
  if(this->m_debug)
    cout << "In LocalManeuveringStrategy::Finalize" << endl;

  size_t numCfgs = m_gbPaths.front().GetCfgs().size();
  for (size_t i=0;i<numCfgs;i++){
    if (m_planTimes.at(i).second == 0) //if agent was unable to get a plan to begin with, don't move forward
      continue;
    BuffPath(i);
  }//end iteration through m_planTimes

/*
  if (m_query.compare("") == 0 && m_path.compare("") == 0) {
    cerr << "No query or path provided." << endl;
    return;
  }
*/

  //iterate through plantimes, if everybody has plan length of 0, no success
  size_t planTimesIndex = 0;
  //iterate through planTimes until we encounter one with plan length > 0
  for (;planTimesIndex<numCfgs && m_planTimes.at(planTimesIndex).second == 0;planTimesIndex++);
  if (planTimesIndex == numCfgs)
    m_gbPath.clear(); //all plan times were 0

    /*
  cerr << "Attempting to build path file: " << m_path << " from query " << m_query << endl;
  if (this->m_debug){
    cout << "\nAuto-building path file " << m_path << " from query " << m_query << endl;
    cout << "\nWriting GBPATH: " << m_gbPath << ", which is of size " << m_gbPaths.size() << endl;
    cout << "Writing Vizmo path configurations to " << m_path << endl;
  }
  */

  cout << "planTimesIndex: " << planTimesIndex << endl;
  cout << "numCfgs: " << numCfgs << endl;
  cout << "size of m_gbPaths: " << m_gbPaths.size() << endl;

  /*
  if (planTimesIndex != numCfgs){
    WriteGBPathConfigurations(m_gbPath, m_gbPaths); //GB

    WriteVizmoPathConfigurations(m_path, m_gbPaths, this->GetMPProblem()->GetEnvironment()); //Vizmo
  }
  */

}//end Finalize

//AttemptPullOutTrajectory, attempts to plan a collision-free trajectory for each individual config inside of
//SSSurfaceMult
template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::AttemptPullOutTrajectory( size_t _startTick, size_t _cfgIndex){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::AttemptPullOutTrajectory" << endl;
  if (_startTick ==  m_gbPaths.size()){
    cerr << "Invalid iterator passed to AttemptPullOutTrajectory" << endl;
    return false;
  }

  CfgType begin = m_gbPaths.at(_startTick);
  CompositeCfgType current = begin.GetCfgs().at(_cfgIndex);
  double maxVel = current.GetMaxSpeed();
  double planDist = 0;
  vector<CfgType> potentialPath; //our potential arc that may be added to m_gbPaths
  double steeringAngle = 0.0; //radians

  /////////////////////////////////////////////////////////////////////////////////////
  //choose which goal we're planning towards
  //determine a direction (forward/backward) to travel based on orientation relative to goal
  Point2d goal = m_guidePath.at(m_guidePathSubGoal).first;
  Point2d pos = current.GetPos();
  double angRad = current.GetRotY();

  cout << " goal pos: " << goal << " cur pos: " << pos << " angRad: " << angRad << endl;
  cout << " maxSpeed/maxVel: " << maxVel << endl;

  angRad += PI/2.0;  //if agent imported facing 0,0, actually is facing 0,1
  //angRad += 3*PI/2.0;  //if agent imported facing 0,0, actually is facing 0,1
  cout << " new angRad: " << angRad;
  if (angRad > PI)
    angRad -= (2.0 * PI);

  Vector2d dirN = Vector2d(cos(angRad),sin(angRad));
  dirN   = dirN.normalize();
  if (this->m_debug){
    cout << " agent: " << _cfgIndex << " pos: " << pos  << endl;
    cout << " direction: " << dirN << " angRad: " << angRad << endl;
  }
  Vector2d dir2N   = (goal - pos);
  dir2N  = dir2N.normalize();
  if( this->m_debug ) { cout << " dir to goal: " << dir2N << endl; }
  m_debugGlobalGoalDir=dir2N;
  double theta = atan2(dir2N[1],-dir2N[0]) - atan2(dirN[1],dirN[0]);
  if (theta > PI)
    theta -= (2.0*PI);
  else if (theta < -PI)
    theta += (2.0*PI);
  ///////////////////////////////////////////////////////////////////////////////////

  if( fabs(theta) <= PI/2.0 ) {
    planDist = DRand()*(m_forwardPullOutTurnDistMax-m_forwardPullOutTurnDistMin) + m_forwardPullOutTurnDistMin;

    if (fabs(theta) > m_forwardReqAngle){
      if (this->m_debug)
        cout << "Not facing subgoal, off by angle: " << theta/PI * 180.0  << endl;

      //try a short forward movement
      double planDist2 = DRand()*(m_forwardPullOutDistMax-m_forwardPullOutDistMin) + m_forwardPullOutDistMin;
      if (!TryPlan( _startTick, _cfgIndex, planDist2/(maxVel*m_delta), potentialPath, 0, false))
        return false;

      //cerr << "Successfully planned forward, here isthe size of potentialPath: " << potentialPath.size() << endl;
      //followed by a turn
      steeringAngle = DRand() * (m_forwardPullOutSteerMin - m_forwardPullOutSteerMax) - m_forwardPullOutSteerMin;
      //^^this calculation is right, I assure you

      if (theta > 0)
        steeringAngle *= -1.0;

      if (!TryPlan(_startTick, _cfgIndex, planDist/(maxVel*m_delta), potentialPath, steeringAngle, false))
        return false;

    }
    else{
      if (this->m_debug)
        cout << "Planning toward goal" << endl;
      if (!TryPlanTowardGoal( _startTick, _cfgIndex, planDist/(maxVel*m_delta), potentialPath,m_guidePath.at(m_guidePathSubGoal).first))
        return false;
    }

  }//end forward movment
  else {
    if (this->m_debug){
      cout << "Moving backward" << endl;
      cout << "Agent theta: " << theta/PI * 180.0 << endl;
    }
    double steeringAngle = DRand() * (m_backPullOutTurnSteerMax-m_backPullOutTurnSteerMin) + m_backPullOutTurnSteerMin;
    if (theta > 0)
      steeringAngle *= -1.0;


    //straight back first
    planDist = DRand() * (m_backPullOutDistMax - m_backPullOutDistMin) + m_backPullOutDistMin;
    if (!TryPlan( _startTick,  _cfgIndex, planDist/(maxVel*m_delta), potentialPath, 0, true))
      return false;


    //now start turning
    planDist = DRand() * (m_backPullOutTurnDistMax-m_backPullOutTurnDistMin) + m_backPullOutTurnDistMin;
    if(!TryPlan( _startTick, _cfgIndex, planDist/(maxVel*m_delta), potentialPath, steeringAngle, true))
      return false;

    //now attempt a short  move forward toward goal
    potentialPath.back().GetCfgs().at(_cfgIndex).SetReverse(false);
    planDist = DRand() * (m_backTowardGoalDistMax - m_backTowardGoalDistMin) + m_backTowardGoalDistMin;
    if (!TryPlanTowardGoal( _startTick, _cfgIndex, planDist/(maxVel*m_delta), potentialPath,m_guidePath.at(m_guidePathSubGoal).first))
      return false;

  }//end backward strategy

  m_planTimes.at(_cfgIndex).second = potentialPath.size();

  CombinePaths(m_gbPaths, potentialPath, _cfgIndex);

  return true;
}//end of AttemptPullOutTrajectory function

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::CombinePaths(vector<CfgType>& _originalPath, vector<CfgType>& _newPath, size_t _index){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::CombinePaths" << endl;

  //CombinePaths will assume that _originalPath may have a number of configurations already in it, and
  //_newPath either needs to overwrite those configurations with new ones, or add configurations to the end of
  //_originalPath. Also, the configurations in _newPath are demarcated by a start and stop time, indicated by
  //the _planTime pair

  //to accomplish this, we'll iterate through the configurations in _newPath, and if _originalPath has a
  //corresponding configuration, replace it. Else, we've moved onto a time outside of _originalPath, so we'll
  //add the configuration to it

  //TMPTMPTMPTMP
  for (CfgIter pathIter = _newPath.begin(); pathIter!= _newPath.end(); pathIter++){
     cout << " path cfg: " << (*pathIter).GetCfgs()[_index].GetPos() << endl;
  }//end for loop

  cout << " size of originalPath: " << _originalPath.size() << " newPath.size: " << _newPath.size() << endl;


  //while index < original's size, replace item
  //while index >= original's size, insert new item into original
  size_t lastIndex = m_planTimes.at(_index).first;
  size_t originalSize = _originalPath.size();
  cout << "_index: " << _index << " lastIndex: " << lastIndex << " originalSize: " << originalSize << endl;
  //if( _newPath.size() < _originalPath.size() ) {
     //int lastValid = _newPath.size()-1;
     //for (CfgIter pathIter = _originalPath.begin()+lastValid; pathIter!= _originalPath.end(); pathIter++){
	//(*pathIter).GetCfgs()[_index]=_originalPath[lastValid-1].GetCfgs()[_index];
     //}//end for loop
  //}
  //else {
     for (CfgIter pathIter = _newPath.begin(); pathIter!= _newPath.end(); pathIter++){
	if (lastIndex < originalSize){
	   _originalPath.at(lastIndex) = *pathIter;
	}else{
	   _originalPath.push_back(*pathIter);
	}
	lastIndex++;
     }//end for loop
  //}
  ///*
  lastIndex = m_planTimes.at(_index).first;
  if( (lastIndex+_newPath.size()) < _originalPath.size() ) {
     cout << " adding right values to path. " << endl;
     //need to copy last of _newPath to the rest of originalPath
     for (CfgIter pathIter = _originalPath.begin()+lastIndex+_newPath.size()-1;
          pathIter!= _originalPath.end(); pathIter++) {
	cout << " copying cfg: " << _originalPath[lastIndex+_newPath.size()-2].GetCfgs()[_index].GetPos() << endl;
	(*pathIter).GetCfgs()[_index]=_originalPath[lastIndex+_newPath.size()-2].GetCfgs()[_index];
     }
  }
  //*/
  cout << " now BuffPath. " << endl;

  //buff out the end, to account for variable start times
  BuffPath(_index);

}//end CombinePaths function

template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::TryPlan(size_t& _startTick, size_t _cfgIndex, int _planSteps,
    vector<CfgType>& _potentialPath, double _steeringAngle, bool _backwards){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::TryPlan" << endl;
  if(_planSteps==0) {
     cout << " plansteps == 0, quick exit" << endl;
     return true;
  }

  //Setup...primarily for collision checks that occur later on
  Environment* env = this->GetMPProblem()->GetEnvironment();
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vc);
  string callee("LocalManeuveringStrategy::TryPlan");

  CfgType tick;
  if (!_potentialPath.empty()){ //in case we're chaining TryPlan's together...
    tick = _potentialPath.back();
  }
  else
    tick = m_gbPaths.at(_startTick);

  CompositeCfgType& current = tick.GetCfgs().at(_cfgIndex);
  current.SetSteeringAngle(NormalizeTheta(_steeringAngle));
  current.SetReverse(_backwards);
  CompositeCfgType next = current;

  if (this->m_debug) {
    cout << "Going to plan for " << _planSteps << " timesteps" << endl;
    cout << " Validity checker: " << vc->GetNameAndLabel() << endl;
  }

  ////////////////////////////////////////////////////////////////////////////

  for(int iteration=0;iteration<_planSteps;iteration++){

    //Update agent's state... should these dynamics and update function go into the Cfg class?
    /*ori += PI/2.0;
      if (ori > PI)
      ori -= (2.0 * PI);*/

    next = next.Update(m_delta);

    double angRad = next.GetRotY();
    angRad += PI/2.0;  //if agent imported facing 0,0, actually is facing 0,1
    //angRad += 3*PI/2.0;  //if agent imported facing 0,0, actually is facing 0,1
    Vector2d dirN = Vector2d(cos(angRad),sin(angRad));
    dirN   = dirN.normalize();
    cout << " TryPlan. pos: " << next.GetPos() << " dirN: " << dirN << endl;
    Vector2d dir2N=m_debugGlobalGoalDir;
    double theta = atan2(dir2N[1],-dir2N[0]) - atan2(dirN[1],dirN[0]);
    if (theta > PI)
       theta -= (2.0*PI);
    else if (theta < -PI)
       theta += (2.0*PI);
    cout << iteration << " off by angle: " << theta/PI * 180.0  << endl;

    /*ori -= PI/2.0;
      if (ori < -PI)
      ori += 2.0 * PI;*/

    if (_startTick < m_gbPaths.size()){
      tick = m_gbPaths.at(_startTick);
      ++_startTick;
    }

    tick.GetCfgs().at(_cfgIndex) = next;
    //tick is now combined config

    ///////////////////////////////////////////////
    //check state
    if(!(vc->IsValid(tick, callee))){
      cout << "Failed movement: " << endl; // << tick << endl;
      return false; //break out of here, collision
    }
    else {
      cout << "Successful movement: " << endl; // << tick << endl;
      _potentialPath.push_back( tick ); //no collision (yet)
    }

    ///////////////////////////////////////////////
  }//end for loop

  //At this point, _potentialPath contains a valid trajectory of length _planSteps
  return true;
}//end TryPlan

template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::TryPlanTowardGoal(size_t& _startTick, size_t _cfgIndex, int _planSteps,
    vector<CfgType>& _potentialPath, Point2d _goal, double _orientationTolerance, double _divisor){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::TryPlanTowardGoal" << endl;

  //Setup...primarily for collision checks that occur later on
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vc);
  string callee("LocalManeuveringStrategy::TryPlanTowardGoal");

  CfgType tick;
  if (!_potentialPath.empty()){ //in case we're chaining TryPlan's together...
    tick = _potentialPath.back();
  }
  else
    tick = m_gbPaths.at(_startTick);


  CompositeCfgType current = tick.GetCfgs().at(_cfgIndex);
  Point2d pos = current.GetPos();
  double ori = current.GetRotY();
  double maxSteeringAngle = current.GetMaxSteeringAngle();

  Vector2d dir;
  Vector2d dir2;
  double theta = 0;

  if (this->m_debug){
    cout << "Moving toward goal" << endl;
    cout << "Going to plan for " << _planSteps << " timesteps" << endl;
  }
  double steeringAngle = 0.0;
  //each iteration, we'll assess the angle between orientation and goal
  //we'll turn towards the goal as much as we can

  for (int iteration=0; iteration < _planSteps; iteration++){
    CompositeCfgType next = current;

    ///*//Adding this back in
      ori += PI/2.0;
      if (ori > PI)
      ori -= (2.0 * PI);//*/

    dir  = Vector2d(cos(ori),sin(ori));
    dir  = dir.normalize();
    dir2 = (_goal - pos);

    Vector2d dir2N=m_debugGlobalGoalDir;
    double thetaDBG = atan2(dir2N[1],-dir2N[0]) - atan2(dir[1],dir[0]);
    if (thetaDBG > PI)
       thetaDBG -= (2.0*PI);
    else if (thetaDBG < -PI)
       thetaDBG += (2.0*PI);
    cout << " dir: " << dir << " dir2 (toward goal): " << dir2 << " prevGlobGorDir: " << dir2N << " off by angle: " << thetaDBG/PI * 180.0  << endl;



    //x is inverted in this coordinate system
    theta = atan2(dir2[1],-dir2[0]) - atan2(dir[1],dir[0]);
    theta = NormalizeTheta(theta);

    if(abs(theta) < _orientationTolerance){ //if we're facing less than _orientationTolerance rads (0.10
      //default) from goal, just go straight
      cout << "just going straight. theta: " << theta << " orientationTolerance: " << _orientationTolerance << endl;
      steeringAngle = 0.0;
    }
    else{
      if (theta > 0)
        steeringAngle = maxSteeringAngle / max(_divisor,1.0);
      else
        steeringAngle = maxSteeringAngle / max(_divisor,1.0) * -1.0;
      cout << " theta: " << theta << " steeringAngle: " << steeringAngle << " maxSteeringAngle: " << maxSteeringAngle << " divisor: " << _divisor << endl;
    }
    next.SetSteeringAngle(steeringAngle);
    cout << " setting steer angle: " << steeringAngle << endl;

    ////////////////////////////////////////////////////////////////
    //POTENTIAL BIG CHANGE, I ADDED THIS BACK IN
    ///* //Adding this back in
    ori -= PI/2.0;
      if (ori < -PI)
      ori += 2.0 * PI;//*/

    next = next.Update(m_delta);

    if (_startTick < m_gbPaths.size()){
      tick = m_gbPaths.at(_startTick);
      ++_startTick;
    }

    tick.GetCfgs().at(_cfgIndex) = next;
    //tick is now combined config
    current = next;

    ///////////////////////////////////////////////
    //check state
    if( !(vc->IsValid(tick, callee))) {
       cout << " found invalid cfg at iter: " << iteration << " cfgIndex: " << _cfgIndex << " bad pos: " << next.GetPos() << endl;
      return false; //break out of here, collision
    }
    else  {
       cout << " found valid cfg at iter: " << iteration << " cfgIndex: " << _cfgIndex << " good pos: " << next.GetPos() << endl;
      _potentialPath.push_back( tick ); //no collision (yet)
    }
    ///////////////////////////////////////////////


    if(EvaluateGoalReach(_cfgIndex, _goal)) //goal is reached
      return true;
  }//end for loop

  //At this point, _potentialPath contains a valid trajectory of length _planSteps
  return true;
}

template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::AttemptDeadlockTrajectory(size_t _startTick, size_t _cfgIndex){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::AttemptDeadlockTrajectory" << endl;

  double planDist = DRand() * (m_deadlock1TurnDistMax-m_deadlock1TurnDistMin) + m_deadlock1TurnDistMin;
  double steeringAngle = DRand() * (m_deadlock1TurnSteerMin - m_deadlock1TurnSteerMax) -
    m_deadlock1TurnSteerMin;

  if (DRand() > m_deadlock1RightTurnFreq) //try a left-hand turn with probability (1-rightTurnFrequency)

    steeringAngle *= -1.0;
  vector<CfgType> potentialPath; //our possible arc

  double maxVel = m_gbPaths.at(_startTick).GetCfgs().at(_cfgIndex).GetMaxSpeed();

  //move back a tad
  double planDist0 = DRand()*10;//btw 0 and 10 dist
  if (!TryPlan( _startTick, _cfgIndex, planDist/(maxVel*m_delta),potentialPath, 0, true))
    return false;

  //attempt an initial turn
  if (!TryPlan( _startTick, _cfgIndex, planDist/(maxVel*m_delta),potentialPath, steeringAngle, false))
    return false;

  //followed by a short straight movement
  double planDist2 = DRand() * (m_deadlock1TowardGoalDistMax-m_deadlock1TowardGoalDistMin) +
    m_deadlock1TowardGoalDistMin;

  if (!TryPlan( _startTick, _cfgIndex, planDist2/(maxVel*m_delta), potentialPath, 0, false))
    return false;

  //and finally an equivalent, but opposite, turn to straighten back out
  if (!TryPlan( _startTick, _cfgIndex, planDist/(maxVel*m_delta), potentialPath, steeringAngle*-1.0, false))
    return false;

  //finally another short forward movement
  if (!TryPlan( _startTick, _cfgIndex, planDist2/(maxVel*m_delta), potentialPath, 0, false))
    return false;

  m_planTimes.at(_cfgIndex).second = potentialPath.size();
  if (this->m_debug)
    cout << "last configuration added to potentialPath: " << potentialPath.back().GetCfgs().at(_cfgIndex).GetPos() <<
      ", " << potentialPath.back().GetCfgs().at(_cfgIndex).GetRotY() << endl;

  CombinePaths(m_gbPaths, potentialPath, _cfgIndex);

  return true;
}//end AttemptDeadlockTrajectory method

/*
template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::AttemptDeadlockTrajectory(size_t _startTick, size_t _cfgIndex){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::AttemptDeadlockTrajectory" << endl;

  double planDist = DRand() * (m_deadlock1TurnDistMax-m_deadlock1TurnDistMin) + m_deadlock1TurnDistMin;
  double steeringAngle = DRand() * (m_deadlock1TurnSteerMin - m_deadlock1TurnSteerMax) -
    m_deadlock1TurnSteerMin;

  if (DRand() > m_deadlock1RightTurnFreq) //try a left-hand turn with probability (1-rightTurnFrequency)

    steeringAngle *= -1.0;
  vector<CfgType> potentialPath; //our possible arc

  double maxVel = m_gbPaths.at(_startTick).GetCfgs().at(_cfgIndex).GetMaxSpeed();

  //attempt an initial turn
  if (!TryPlan( _startTick, _cfgIndex, planDist/(maxVel*m_delta),potentialPath, steeringAngle, false))
    return false;

  //followed by a short straight movement
  double planDist2 = DRand() * (m_deadlock1TowardGoalDistMax-m_deadlock1TowardGoalDistMin) +
    m_deadlock1TowardGoalDistMin;

  if (!TryPlan( _startTick, _cfgIndex, planDist2/(maxVel*m_delta), potentialPath, 0, false))
    return false;

  //and finally an equivalent, but opposite, turn to straighten back out
  if (!TryPlan( _startTick, _cfgIndex, planDist/(maxVel*m_delta), potentialPath, steeringAngle*-1.0, false))
    return false;

  //finally another short forward movement
  if (!TryPlan( _startTick, _cfgIndex, planDist2/(maxVel*m_delta), potentialPath, 0, false))
    return false;

  m_planTimes.at(_cfgIndex).second = potentialPath.size();
  if (this->m_debug)
    cout << "last configuration added to potentialPath: " << potentialPath.back().GetCfgs().at(_cfgIndex).GetPos() <<
      ", " << potentialPath.back().GetCfgs().at(_cfgIndex).GetRotY() << endl;

  CombinePaths(m_gbPaths, potentialPath, _cfgIndex);

  return true;
}//end AttemptDeadlockTrajectory method
*/

template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::AttemptDeadlockTrajectory2(size_t _startTick, size_t _cfgIndex){
  if (this->m_debug)
    cout << "LocalManeuveringStrategy::AttemptDeadlockTrajectory2" << endl;

  double planDist = DRand()*(m_deadlock2DistMax-m_deadlock2DistMin) + m_deadlock2DistMin;
  double divisor = DRand()*(1.0-m_deadlock2Divisor) + m_deadlock2Divisor;

  vector<CfgType> potentialPath; //our possible arc

  double maxVel = m_gbPaths.at(_startTick).GetCfgs().at(_cfgIndex).GetMaxSpeed();

/*
  //move back a tad
  double planDist0 = DRand()*6;//btw 0 and 6 dist
  if (!TryPlan( _startTick, _cfgIndex, planDist/(maxVel*m_delta),potentialPath, 0, true))
    return false;
    */

  //simply try to plan toward the goal, varying the steering angle between maximum and 1/5*maximum
  //vary the steering angle to be between maximum and 1/2 maximum
  //agents will try to plan toward the goal position (not configuration) from the query file
  if (!TryPlanTowardGoal( _startTick, _cfgIndex, planDist/(maxVel*m_delta), potentialPath,
        m_goals.front().GetCfgs().at(_cfgIndex).GetPos(), divisor))
    return false;

  m_planTimes.at(_cfgIndex).second = potentialPath.size();

  if (this->m_debug)
    cout << "last configuration added to potentialPath: " << potentialPath.back().GetCfgs().at(_cfgIndex).GetPos() <<
      ", " << potentialPath.back().GetCfgs().at(_cfgIndex).GetRotY() << endl;

  CombinePaths(m_gbPaths, potentialPath, _cfgIndex);

  return true;
}//end AttemptDeadlockTrajectory2 method

template<class MPTraits>
bool
LocalManeuveringStrategy<MPTraits>::AttemptPullInTrajectory(size_t _startTick, size_t _cfgIndex){

  if (this->m_debug)
    cout << "LocalManeuveringStrategy::AttemptPullInTrajectory" << endl;
  if (_startTick == m_gbPaths.size()){
    cerr << "Invalid iterator passed to AttemptPullInTrajectory" << endl;
    return false;
  }

  int planSteps = 0;
  vector<CfgType> potentialPath; //our potential arc that may be added to m_gbPaths
  double steeringAngle = 0.0; //radians

  /////////////////////////////////////////////////////////////////////////////////////
  //the goal position we're planning toward is given in the query file
  CompositeCfgType currentCfg, goalCfg;
  currentCfg = m_gbPaths.at(_startTick).GetCfgs().at(_cfgIndex);
  double maxVel = currentCfg.GetMaxSpeed();
  double wheelbase = currentCfg.GetWheelbase();
  double maxSteeringAngle = currentCfg.GetMaxSteeringAngle();

  double minTurningRadius = wheelbase/sin(maxSteeringAngle);
  goalCfg = m_goals.front().GetCfgs().at(_cfgIndex);
  Point2d goalPos = goalCfg.GetPos();
  double goalAngRad = goalCfg.GetRotY();
  Point2d pos = currentCfg.GetPos();
  double angRad = currentCfg.GetRotY();

  Vector2d self = Vector2d(cos(angRad),sin(angRad));
  self   = self.normalize();
  Vector2d goalVec = Vector2d(cos(goalAngRad),sin(goalAngRad));
  goalVec = goalVec.normalize();

  if (this->m_debug){
    cout << " agent: " << _cfgIndex << " pos: " << pos  << endl;
    cout << " direction: " << self << " angRad: " << angRad << endl;
    cout << "goalPos: " << goalPos << " goalAngRad: " << goalAngRad << endl;
  }

  double theta = atan2(goalVec[1],goalVec[0]) - atan2(self[1],self[0]);
  if (theta > PI)
    theta -= (2.0*PI);
  else if (theta < -PI)
    theta += (2.0*PI);

  ///////////////////////////////////////////////////////////////////////////////////
  //if theta > 0
  //rotate goal vector right 90 degrees
  //else rotate right 90 degrees
  // turn until facing this vector
  // assuming R = minimum turning radius, V = uniform velocity
  // then V/R = angular velocity = rads/sec
  // if abs(theta_current - theta_desired) = theta_diff
  // then the number of timesteps I need to turn at my maximum velocity, minimum turning radius until I face
  // my desired orientation is theta_diff/rads. Ideally, this would be an integer value, but it won't be in
  // practice, so perhaps we should think about altering R until we can find an integer value?
  //
  // after we're on track, we'll form a triangle with my vector, and a perpendicular vector emanating from the
  // goal. We need to find X=straight-head distance to travel until we intersect the goal vector
  // perpendicularly.
  // Using the some trig, we can determine X using our distance from the goal (hypotenuse)
  // as well as the angle between my current orientation and the vector pointing towards the goal.
  // after X is found, subtract R= turning radius from X. Travel X-R units, and then turn at minimum
  // turning radius until facing goal, at which point, move straight ahead until we're at goal. Stop.
  // with some probability, agent will attempt to back in at the end instead of pull forward
  bool backIn = false;
  bool turnThenReverse = false;
  if (DRand() > m_pullInForwardProbability){
    backIn = true;
    if (DRand() > m_pullInNormalBackwardProbability)
      turnThenReverse = true;
  }

  double desiredTheta = goalAngRad;
  if (theta > 0)
    desiredTheta -= PI/2.0;
  else
    desiredTheta += PI/2.0;

  if (desiredTheta > PI)
    desiredTheta -= 2.0*PI;
  else if (desiredTheta < -PI)
    desiredTheta += 2.0*PI;

  //find angle between angRad and desiredTheta
  Vector2d desiredVec = Vector2d(cos(desiredTheta),sin(desiredTheta));
  desiredVec = desiredVec.normalize();
  double thetaDiff = atan2(desiredVec[1],desiredVec[0]) - atan2(self[1],self[0]);
  if (thetaDiff > PI)
    thetaDiff -= 2.0*PI;
  else if (thetaDiff < -PI)
    thetaDiff += 2.0*PI;

  double actualPlanSteps = thetaDiff / (maxVel*m_delta / minTurningRadius);
  planSteps = int(abs(actualPlanSteps));
  steeringAngle = maxSteeringAngle;
  if (actualPlanSteps < 0)
    steeringAngle *= -1.0;

  cout << " Zero TryPlan for steps: " << planSteps << endl;
  cout << " _cfgIndex: " << _cfgIndex << " thetaDiff: " << thetaDiff << " startTick: " << _startTick << endl;
  if (!TryPlan(_startTick,  _cfgIndex, planSteps, potentialPath, steeringAngle, false))
    return false;

  cout << " just returned from TryPlan. potentialPath size: " << potentialPath.size() << endl;
  if( potentialPath.size()==0 ) { //unsuccessful/unneeded first TryPlan
     CfgType tick;
     //tick=m_gbPaths.at(_startTick);
     tick=m_roots[0];
     potentialPath.push_back( tick );
  }

  //now move X-R distance, so first determine X:
  //to determine X, we need our distance to the goal point, plus the angle between our heading and the vector
  //facing the goal
  pos = potentialPath.back().GetCfgs().at(_cfgIndex).GetPos();

  angRad = potentialPath.back().GetCfgs().at(_cfgIndex).GetRotY();
  angRad += PI/2.0;  //if agent imported facing 0,0, actually is facing 0,1
  //angRad += 3*PI/2.0;  //if agent imported facing 0,0, actually is facing 0,1
  if (angRad > PI)
    angRad -= (2.0 * PI);
  Vector2d dir2N   = (goalPos - pos);
  dir2N  = dir2N.normalize();
  self = Vector2d(cos(angRad),sin(angRad));
  self   = self.normalize();

  theta = atan2(dir2N[1],-dir2N[0]) - atan2(self[1],self[0]);
  if (theta > PI)
    theta -= (2.0*PI);
  else if (theta < -PI)
    theta += (2.0*PI);
  double xDist = fabs((Vector2d(goalPos-pos)).norm() * cos(theta));
  double yDist = fabs((Vector2d(goalPos-pos)).norm() * sin(theta));

  cout << " xDist: " << xDist << " yDist: " << yDist << endl;

  //vary the steering angle so that car may turn sooner in other random attempts
  //but don't let it be too small
  double steeringMin = max(m_pullInSteerMin,fabs(asin(wheelbase/min(xDist/2.0,yDist/2.0))));
  steeringAngle = DRand()*(maxSteeringAngle-steeringMin) + steeringMin;
  double actualTurningRadius = wheelbase/sin(steeringAngle);
  if (backIn && !turnThenReverse)
    xDist += actualTurningRadius;
  else
    xDist = max(0.0,xDist-actualTurningRadius);

  actualPlanSteps = xDist/(maxVel*m_delta);
  planSteps = (int)fabs(actualPlanSteps);

  cout << " First TryPlan for steps: " << planSteps << "steeringAngle: " << steeringAngle << endl;
  if (!TryPlan(_startTick,  _cfgIndex, planSteps, potentialPath, 0.0, false))
    return false;

  actualPlanSteps = (PI/2.0) / (maxVel*m_delta/actualTurningRadius);
  planSteps = (int)fabs(actualPlanSteps);
  if (theta < 0)
    steeringAngle *= -1.0;

  if (backIn && turnThenReverse)
    steeringAngle *= -1.0;

  cout << " Second TryPlan for steps: " << planSteps << endl;
  if (!TryPlan(_startTick,  _cfgIndex, planSteps, potentialPath, steeringAngle, backIn && !turnThenReverse))
    return false;

  pos = potentialPath.back().GetCfgs().at(_cfgIndex).GetPos();
  yDist = (Vector2d(goalPos-pos)).norm();
  actualPlanSteps = (yDist/(maxVel*m_delta));
  planSteps = (int)fabs(actualPlanSteps);
  if (!TryPlan(_startTick,  _cfgIndex, planSteps, potentialPath, 0.0, backIn))
    return false;

  potentialPath.back().GetCfgs().at(_cfgIndex).SetVelocity(Vector2d(0.0,0.0));

  m_planTimes.at(_cfgIndex).second = potentialPath.size();
  CombinePaths(m_gbPaths, potentialPath, _cfgIndex);

  return true;
}//end of AttemptPullInTrajectory function

template<class MPTraits>
void
LocalManeuveringStrategy<MPTraits>::BuffPath(size_t _index){
  int lastCfgIndex = m_planTimes.at(_index).first+m_planTimes.at(_index).second - 1;
  size_t numCfgsToAdd = m_gbPaths.size() - lastCfgIndex-1;
  CfgType lastCfg = m_gbPaths.at(lastCfgIndex);
  vector<CompositeCfgType>& cfgs = lastCfg.GetCfgs();
  Point2d pos = cfgs.at(_index).GetPos();
  double angRad = cfgs.at(_index).GetRotY();
  for (size_t j=1;j<=numCfgsToAdd;j++){
    cfgs = m_gbPaths.at(lastCfgIndex+j).GetCfgs();
    cfgs.at(_index).SetPos(pos);
    cfgs.at(_index).SetRotY(angRad);
  }

  //update the m_planTimes to avoid repeated work in next BuffPath call
  m_planTimes.at(_index).second+=numCfgsToAdd;
}//end BuffPath method

#endif
#endif
