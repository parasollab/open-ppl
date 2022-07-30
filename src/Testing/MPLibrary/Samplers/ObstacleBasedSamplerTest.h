#ifndef PPL_OBSTACLE_BASED_SAMPLER_H_
#define PPL_OBSTACLE_BASED_SAMPLER_H_

#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MetricUtils.h"


template <class MPTraits>
class ObstacleBasedSamplerTest : virtual public ObstacleBasedSampler<MPTraits>, 
                                 public SamplerMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{
    typedef typename MPTraits::CfgType        CfgType;
    typedef TestBaseObject::TestResult        TestResult;
    typedef typename MPTraits::MPLibrary      MPLibrary;

    ///@}
    ///@name Construction
    ///@{

    ObstacleBasedSamplerTest();

    ObstacleBasedSamplerTest(XMLNode& _node);

    ~ObstacleBasedSamplerTest();

    ///@}

  private: 

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestIndividualCfgSample() override;

    virtual TestResult TestIndividualCfgSampleWithEEConstraint() override;

    virtual TestResult TestIndividualFilter() override;

    virtual TestResult TestGroupCfgSampleSingleBoundary() override;

    virtual TestResult TestGroupCfgSampleIndividualBoundaries() override;

    virtual TestResult TestGroupFilter() override;

    MedialAxisUtility<MPTraits> m_medialAxisUtility;

};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest() : ObstacleBasedSampler<MPTraits>() {
  m_medialAxisUtility = MedialAxisUtility<MPTraits>("pqp_solid", "euclidean",
                                false, false, 500, 500, true, true, true, 0.1, 5);
}

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
ObstacleBasedSamplerTest(XMLNode& _node) : SamplerMethod<MPTraits>(_node),
                                           ObstacleBasedSampler<MPTraits>(_node) {
  m_medialAxisUtility = MedialAxisUtility<MPTraits>("pqp_solid", "euclidean",
                                false, false, 500, 500, true, true, true, 0.1, 5);
}

template <typename MPTraits>
ObstacleBasedSamplerTest<MPTraits>::
~ObstacleBasedSamplerTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestIndividualCfgSample() {
  auto mpl = this->GetMPLibrary();

  bool passed = true;
  std::string message = "";

  Boundary* boundary = nullptr;
  std::vector<Cfg> valids;
  std::vector<Cfg> invalids;

  this->IndividualCfgSample(boundary, valids, invalids);
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker("rapid");

  std::cout << "====1 " << mpl << "=====1" << std::endl;

  if(!m_medialAxisUtility.IsInitialized()) {
    std::cout << "=== Initialize MedialAxisUtilitiy (Single Robot) ===" << std::endl;
    // Set MPLibrary pointer
    m_medialAxisUtility.SetMPLibrary(mpl);
    // Initialize
    m_medialAxisUtility.Initialize();
  }

  std::vector<CDInfo> cdInfo_vec;
  std::vector<double> minDistInfo_vec;
  // Iterate through valid cfgs
  for (auto cfg : valids){
    if(!vc->IsValid(cfg, callee)) {   
      passed = false;
      message = message + "\n\tA configuration was incorrectly labeled "
                "valid for the given boundary. n";
      break;
    }

    CDInfo _cdInfo;
    CfgType _cfgClr;
    bool valid = false;

    // Find a nearest obstacle cfg
    valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

    // Cannot find a valid obstacle cfg    
    if (!valid) {
      passed = false;
      message = "Cannot find the closest obstacle from given cfg\n";
      break;
    }

    // Store cdInfo and minimum distance
    cdInfo_vec.push_back(_cdInfo);
    minDistInfo_vec.push_back(_cdInfo.m_minDist);

    // Average out the distances
    double avgDist = std::accumulate(minDistInfo_vec.begin(), minDistInfo_vec.end(), 0.0) / minDistInfo_vec.size();
    // Set a criteria for pass
    double c = 15*min(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    std::cout << "(average distance)" << avgDist << " | (threshold)" << c << std::endl;

    // Compare them 
    if (avgDist > c) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large: ";
      break;
    }
  }

  if(passed) {
    message = "IndividualCfgSample::PASSED!\n";
  }
  else {
    message = "IndividualCfgSample::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestIndividualCfgSampleWithEEConstraint() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->IndividualCfgSampleWithEEConstraint();

  if(passed) {
    message = "IndividualCfgSampleWithEEConstraint::PASSED!\n";
  }
  else {
    message = "IndividualCfgSampleWithEEConstraint::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestIndividualFilter() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->IndividualFilter();

  if(passed) {
    message = "IndividualFilter::PASSED!\n";
  }
  else {
    message = "IndividualFilter::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestGroupCfgSampleSingleBoundary() {
  bool passed = true;
  std::string message = "";

  Boundary* boundary = nullptr;
  std::vector<GroupCfg> valids;
  std::vector<GroupCfg> invalids;

  this->GroupCfgSampleSingleBoundary(boundary, valids, invalids);
  string callee = this->GetNameAndLabel() + "::Sampler()";
  auto vc = this->GetValidityChecker("rapid");

  

  auto mpl = this->GetMPLibrary();
  // if(!m_medialAxisUtility.IsInitialized()) {
  std::cout << "=== Initialize MedialAxisUtilitiy (Group) ===" << std::endl;
  // Set MPLibrary pointer
  m_medialAxisUtility.SetMPLibrary(mpl);
  // Initialize
  m_medialAxisUtility.Initialize();
  // }

  std::vector<CDInfo> cdInfo_vec;
  std::vector<double> minDistInfo_vec;

  // auto groupTask = this->GetGroupTask();
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  auto groupTask = this->GetMPProblem()->GetTasks(group)[0].get();

  m_medialAxisUtility.GetMPLibrary()->SetGroupTask(groupTask);
  m_medialAxisUtility.GetMPLibrary()->SetTask(nullptr);

  // Iterate through valid cfgs
  for (auto groupCfg : valids) {
    
    if(!vc->IsValid(groupCfg, callee)) {
      passed = false;
      message = message + "\n\tA configuration was incorrectly labeled "
                "valid for the given boundary. n";
      break;
    }

    int i = 0;

    for (auto& task : *groupTask) {
      // if (i == 0) {
      //   continue;
      // }
      auto robot = task.GetRobot();
      std::cout << "======= " << i << "task: " << task.GetRobot() << " | robot: " << robot << std::endl;

      // auto task2 = m_medialAxisUtility.GetMPProblem()->GetTasks(robot)[0].get();
      // m_medialAxisUtility.GetMPLibrary()->SetTask(&task);
      // throw invalid_argument("MyFunc argument too large.");
      // if (!m_medialAxisUtility.GetTask()->GetRobot()) {
      //   m_medialAxisUtility.SetRobot(robot);
      // }

      MPTask* tmpTask = &task;
      m_medialAxisUtility.GetMPLibrary()->SetTask(tmpTask);

      CfgType cfg = groupCfg.GetRobotCfg(robot);

      CDInfo _cdInfo;
      CfgType _cfgClr;
      bool valid = false;

      // Find a nearest obstacle cfg
      valid = m_medialAxisUtility.ApproxCollisionInfo(cfg, _cfgClr, boundary, _cdInfo);

      // Cannot find a valid obstacle cfg    
      if (!valid) {
        passed = false;
        message = "Cannot find the closest obstacle from given cfg\n";
        std::cout << message << std::endl;
        break;
      }

      // Store cdInfo and minimum distance
      cdInfo_vec.push_back(_cdInfo);
      minDistInfo_vec.push_back(_cdInfo.m_minDist);
      // throw std::invalid_argument("AddPositiveIntegers a");

      i += 1;
    }



    if (!passed)
      break;
    
    for (unsigned int i=0; i < minDistInfo_vec.size(); i++) {
      std::cout << minDistInfo_vec.at(i) << ' ';
    }

    // Average out the distances
    double avgDist = std::accumulate(minDistInfo_vec.begin(), minDistInfo_vec.end(), 0.0) / minDistInfo_vec.size();
    
    // Set a criteria for pass
    double c = 1000*min(this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
    std::cout << "(average distance)" << avgDist << " | (threshold)" << c << std::endl;

    // Compare them 
    if (avgDist > c) {
      passed = false;
      message = "Average minimum distance from cfgs to obstacles are too large \n";
      break;
    }
  }


  
  if(passed) {
    message = "IndividualCfgSample::PASSED!\n";
  }
  else {
    message = "IndividualCfgSample::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestGroupCfgSampleIndividualBoundaries() {

  bool passed = true;
  std::string message = "";
  
  //TODO::Setup test of this function.
  //this->GroupCfgSampleIndividualBoundaries();

  if(passed) {
    message = "GroupCfgSampleIndividualBoundaries::PASSED!\n";
  }
  else {
    message = "GroupCfgSampleIndividualBoundaries::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename ObstacleBasedSamplerTest<MPTraits>::TestResult
ObstacleBasedSamplerTest<MPTraits>::
TestGroupFilter() {

  bool passed = true;
  std::string message = "";

  //TODO::Setup test of this function.
  //this->GroupFilter();

  if(passed) {
    message = "GroupFilter::PASSED!\n";
  }
  else {
    message = "GroupFilter::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}



















// template<typename MPTraits>
// bool
// ObstacleBasedSamplerTest<MPTraits>::
// MakeRays(const CfgType& _sampledCfg, const std::size_t& _numRays,
//          std::vector<Ray<CfgType> >& _rays) {
//   auto dm  = this->GetDistanceMetric(m_dmLabel);
//   string callee = this->GetNameAndLabel() + "::MakeRays()";

//   // if(m_stepSize <= 0.0)
//   //   m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());

//   //For 2d:
//   //initial ray starts at rand angle, then all others are uniformly distributed:
//   double angleRad = 2.*PI*DRand();//Note 2d case only currently

//   for(size_t i = 0; i < _numRays; ++i) {
//     CfgType tmpDirection = _sampledCfg;

//     ///@TODO expand to 3D, and then to N dimensions for uniform distribution:
//     if(_sampledCfg.DOF() == 2) {
//       // This evenly divides up the rest of the angles all the way around,
//       // starting from the random initial angle.
//       angleRad += 2. * PI * (1. / _numRays);//note this happens numRays times
//       std::vector<double> dofData = {cos(angleRad), sin(angleRad)};
//       tmpDirection.SetData(dofData); // There's likely a better way to do this
//     }
//     else {
//       //The non-uniform way to get each ray:
//       tmpDirection.GetRandomRay(m_rayTickResolution, dm, false);
//     }

//     // if(this->m_debug)
//     //   cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;

//     if(m_positionalDofsOnly) { // Use only positional dofs
//       if(tmpDirection.PosDOF() == 0)
//         throw RunTimeException(WHERE, "Attempting to only use positional DOFs "
//                            "for a fixed-base robot, this is invalid behavior!");
//       double factor = 0.0;
//       for(size_t j = 0; j < tmpDirection.DOF(); ++j) {
//         if(j < tmpDirection.PosDOF())
//           factor += tmpDirection[j] * tmpDirection[j];
//         else
//           tmpDirection[j] = 0.0;
//       }
//       tmpDirection *= m_rayTickResolution / sqrt(factor);
//     }
//     // if(this->m_debug) {
//     //   cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;
//     //   cout << "DEBUG:: \tdistance(_sampledCfg, _sampledCfg+tmpDirection) = "
//     //     << dm->Distance(_sampledCfg, _sampledCfg + tmpDirection) << endl;
//     // }

//     _rays.push_back(Ray<CfgType>(tmpDirection, _sampledCfg));
//   }// end for (_numRays)

//   // if(this->m_debug) {
//   //   cout << "DEBUG:: rays initialized\n";
//   //   cout << "DEBUG:: \ttick are:\n\t\t";
//   //   for(auto&  ray : _rays)
//   //     cout << ray.m_tick << "\n\t\t";
//   //   cout << endl;
//   //   cout << "DEBUG:: \tincr are:\n\t\t";
//   //   for(auto&  ray : _rays)
//   //     cout << ray.m_incr << "\n\t\t";
//   //   cout << endl;
//   // }
//   return;
// }

// template<typename MPTraits>
// void
// ObstacleBasedSamplerTest<MPTraits>::
// FindApproximateWitness(std::vector<Ray<CfgType> >& _rays, 
//             const CfgType& c1,
//             const Boundary* const _boundary,
//             std::vector<double>& min_distance_vec) {

//   // Define a validity checker, a distance metric, and a task
//   string callee = this->GetNameAndLabel() + "::Sampler()";
//   auto vc = this->GetValidityChecker(m_vcLabel);
//   auto dm = this->GetDistanceMetric(m_dmLabel);
//   auto robot = this->GetTask()->GetRobot();

//   // Old state that is generated by point selection strategies
//   bool c1BBox = c1.InBounds(_boundary);
//   bool c1Free = vc->IsValid(c1, callee);

//   for (auto r: rays){
//     std::vector<double> distance_v;

//     CfgType c2 = c1;
//     bool c2BBox = c1BBox;
//     bool c2Free = c1Free;

//     // Loop until the new state is outside the bounds or the validity changes
//     while(c2BBox && (c1Free == c2Free)) {
//       // Copy new data to old state
//       c1 = c2;
//       c1BBox = c2BBox;
//       c1Free = c2Free;
//       // Update new state
//       c2 += r;
//       c2BBox = c2.InBounds(_boundary);
//       c2Free = vc->IsValid(c2, callee);
//     }

//     // Construct Shells and generate _collision and _result
//     // If we treat the bounding box as an obstacle
//     if (!m_useBBX) { 
//       // If the new state (c2) is in the bounding box
//       if (c2BBox) { 
//         // If the old state (c1) is  in the free C space
//         if (c1Free) { 
//           distance_vec.push_back(dm->Distance(c1, c2));
//         }
//         // If the new state (c2) is in the free C space
//         else { 
//           distance_vec.push_back(dm->Distance(c1, c2));
//         }
//       }
//     } 
//     else {
//       if (c1BBox) {
//         if (c1Free) { // If the old state (c1) is in the free C space
//           distance_vec.push_back(dm->Distance(c1, c2));
//         }
//       }
//     }
//     double min_distance = std::min_element(distance_vec.begin(), distance_vec.end());
//     min_distance_vec.push_back(min_distance);
    
//   }
// }

// template <typename MPTraits>
// void
// ObstacleBasedSampler<MPTraits>::
// GenerateShells(const Boundary* const _boundary,
//     CfgType& _cFree, CfgType& _cColl, CfgType& _incr,
//     vector<CfgType>& _result, vector<CfgType>& _collision) {

//   string callee = this->GetNameAndLabel() + "::GenerateShells()";

//   // Call a validity checker
//   auto vc = this->GetValidityChecker(m_vcLabel);

//   // Add "free" shells
//   if(_cFree.InBounds(_boundary) and vc->IsValid(_cFree, callee))
//     _result.push_back(_cFree);  
//   // Get next shell
//   _cFree += _incr;

//   // Reverse direction of _incr
//   _incr = -_incr;

// }


// // template<typename MPTraits>
// // bool
// // ObstacleBasedSamplerTest<MPTraits>::
// // FindApproximateWitness(const std::size_t& _numRays,
// //             std::vector<Ray<CfgType> >& _rays, const CfgType& _sampledCfg,
// //             const bool& _initValidity, const Boundary* const _b,
// //             std::pair<size_t, CfgType>& _candidate,
// //             const bool& _useOppValidityWitness) {
// //   // if(this->m_debug)
// //   //   cout << "FindApproximateWitness: numRays = " << _numRays << endl;
// //   auto vcm = this->GetValidityChecker(m_vcLabel);
// //   string callee = this->GetNameAndLabel() + "::FindApproximateWitness()";

// //   // //Assume that the directions have been populated already if _rays isn't empty.
// //   // if(_rays.empty()) {
// //   //   //Reserve the number of rays that we need for faster push_back later
// //   //   _rays.reserve(_numRays);
// //   //   MakeRays(_sampledCfg, _numRays, _rays);
// //   // }

// //   // Step out along each direction to determine the candidate
// //   bool candidateFound = false;
// //   size_t iterations = 0;
// //   CfgType candidateCfg;

// //   if(this->m_debug)
// //     cout << "DEBUG:: stepping out along each direction to find candidates";

// //   //This loop goes until it has just one of its rays go from free->not free
// //   //or not free->free or iterates n times. Within it, it loops through all
// //   //rays each time, moving outward along these and checking that condition.
// //   //m_maxRayIterations is calculated by maxRayMagnitude/rayTickResolution
// //   while(!candidateFound && iterations++ < m_maxRayIterations) {
// //     // if(this->m_debug)
// //     //   cout << "\n\t" << iterations;

// //     for(auto rit = _rays.begin(); rit != _rays.end();) {
// //       //step out
// //       rit->m_tick += rit->m_incr;

// //       //determine new state
// //       CDInfo tmpInfo;

// //       bool currValidity{false};//will get overwritten
// //       const bool inBounds = rit->m_tick.InBounds(_b);

// //       // If doing the opposite validity witness, then it's the case that if
// //       // we are out of bounds and initially invalid, this ray can never lead
// //       // to a successful witness of opposite validity, so remove the ray.
// //       //Note: this is not taking into account whether or not m_useBBX is true.
// //       // If doing the same-validity witness, then we still will count the tick
// //       // before going out of bounds as the nearest witness, since it still must
// //       // correspond to an obstacle's boundary being crossed.
// //       if(!inBounds && !_initValidity && _useOppValidityWitness) {
// //         // if(this->m_debug)
// //         //   cout << "Ray is out of bounds at: " << rit->m_tick << endl;

// //         // only one ray
// //         if(_rays.size() == 1) {
// //           if(this->m_debug)
// //             cout << "Last valid ray just failed, Returning false" << endl;
// //           return false;
// //         }
// //         else {
// //           //Swap the ray leading to an invalid OOB cfg to the back of the vector
// //           // and then pop from back of vector:
// //           if(rit != _rays.end() - 1) { //Don't swap if it's already the last ray
// //             auto eit = _rays.end() - 1;
// //             swap(*rit, *eit);
// //           }

// //           //Remove the ray that cannot lead to a valid witness.
// //           _rays.pop_back();
// //           continue; //move to next ray (note: won't advance the iterator)
// //         }
// //       }
// //       //If we are doing the same validity witness, then the same triggers for a
// //       // witness still apply, except that the condition above should also
// //       // trigger a witness instead of disqualifying a ray.
// //       else if (!inBounds && !_initValidity && !_useOppValidityWitness) {
// //         // Force a change in validity, since we have found the edge of the
// //         // obstacle we are in and it happens to border the BBX boundary.
// //         currValidity = !_initValidity;
// //       }
// //       //So now we know that we have a ray tick that is either OOB and was an
// //       // initially valid sample, or we are in bounds and initial state is N/A.
// //       else {
// //         currValidity = vcm->IsValid(rit->m_tick, tmpInfo, callee);
// //         if(m_useBBX) // Only consider bounds here if using it as obstacle
// //           currValidity = inBounds && currValidity;
// //       }

// //       if(this->m_debug)
// //         cout << " (currValidity for direction " << distance(_rays.begin(), rit)
// //              << " = " << currValidity << ")";

// //       //if validity state has changed, save the candidate and stop searching:
// //       //Note that OOB is of the same invalid status here as in collision.
// //       if(currValidity != _initValidity) {
// //         candidateFound = true; // So the enveloping while loop will exit

// //         candidateCfg = rit->m_tick;
// //         //tick back the current cfg only if using the same validity witness:
// //         if(!_useOppValidityWitness)
// //           candidateCfg -= rit->m_incr;

// //         //Note: this is pushing back the index of the candidate ray (what
// //         // distance() returns) and the actual location that we have found. The
// //         // location's validity is determined by the type of witness desired.
// //         _candidate = make_pair(distance(_rays.begin(), rit), candidateCfg);

// //         //Quit the loop since the witness has been found.
// //         break;
// //       }
// //       //Must increment the iterator here due to the flow of the for loop:
// //       ++rit;
// //     }
// //   }//end while (!stateChangedFlag && iterations < max)

// //   //Check that we actually found a candidate
// //   if(!candidateFound) {
// //     if(this->m_debug)
// //       std::cout << callee + "Returning false after not finding a witness "
// //                 "candidate" << std::endl;
// //     return false;
// //   }

// //   if(this->m_debug) {
// //     cout << "\nDEBUG:: done stepping out along rays. Candidate = " << std::endl;
// //     cout << "\tray " << _candidate.first << ": " << _candidate.second;

// //     CDInfo tmpInfo;
// //     bool currValidity = vcm->IsValid(_candidate.second, tmpInfo, callee);
// //     if(m_useBBX)
// //       currValidity = (currValidity && _candidate.second.InBounds(_b));
// //     cout << " (currValidity = " << currValidity << ")" << endl;
// //   }

// //   // Ensure the candidate is valid:
// //   // Note: this is something we could remove for optimization
// //   if(!ValidateCandidate(_candidate, _rays, _b, _useOppValidityWitness))
// //     return false;

// //   // This check should be impossible to trigger, but it's a good thing to ensure
// //   if(candidateCfg == _sampledCfg) {
// //     if(this->m_debug)
// //       std::cout << callee + "returning false from _clrCfg == _cfg" << std::endl;
// //     return false;
// //   }

// //   CDInfo tmpInfo;
// //   bool currValidity = vcm->IsValid(candidateCfg, tmpInfo, callee);
// //   if(this->m_useBBX)
// //     currValidity = currValidity && candidateCfg.InBounds(_b);

// //   //Final check to make sure the witness is of correct validity.
// //   bool passed = _useOppValidityWitness ? (_initValidity != currValidity) :
// //                                          (_initValidity == currValidity);
// //   if(!passed) {
// //     if(this->m_debug)
// //       std::cout << callee + "returning false from candidate being of wrong "
// //                 "validity" << std::endl;
// //     return false;
// //   }

// //   //If we make it here, we have a successful witness.
// //   return true;
// // }





#endif
