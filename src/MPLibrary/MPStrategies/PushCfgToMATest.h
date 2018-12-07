#ifndef PUSH_CFG_TO_MA_TEST_H
#define PUSH_CFG_TO_MA_TEST_H

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test and benchmark the approximate pushing to the medial
///         axis (MA).
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PushCfgToMATest : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    PushCfgToMATest();
    PushCfgToMATest(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(std::ostream& _os) const;

  protected:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    const std::string m_strategyName = "PushCfgToMATest";

    //This will scale all samples so that in the 2D standard env, they initially
    // start in the narrow passage (used for some troubleshooting tests)
    bool m_forceNarrowPassageSamples2dEnv{false};

    //Can choose whether initially invalid samples are attempted to be pushed
    // to the MA (true) or not (false)
    bool m_attemptInvalidSamples{true};//false means throw away invalid

    //If using 2D.env, there are more tests (narrow passage count) that can be
    // activated using this flag.
    bool m_do2dEnvTests{false};

    //A choice of whether to perform the test using the canonical opposite-
    // validity witnesses (false), or same-validity witnesses. The same-validity
    // test is just going to find the actual witness, instead of pushing samples
    // to the medial axis, to serve as a simple functional verification.
    bool m_doSameValidityWitness{false};

    //The string keys of the samplers (Note: only 1 is supported currently)
    std::vector<std::string> m_samplerIndices;

    // Sampler labels mapped to number and attempts of sampler:
    std::map<std::string, std::pair<size_t, size_t> > m_samplerLabels;

    //Data that I accumulate in Iterate() and output in Finalize()
    size_t m_narrowPassageCount{0};
    size_t m_numSamplesFailed{0};
    size_t m_numInvalidPassedToPush{0};
    size_t m_numFailedSamplesThatWereInvalid{0};

    //A maximum number of samples to attempt to push:
    size_t m_maxSamplesToAttempt{0};
};

template <typename MPTraits>
PushCfgToMATest<MPTraits>::
PushCfgToMATest() {
  this->SetName(m_strategyName);
}

template <typename MPTraits>
PushCfgToMATest<MPTraits>::
PushCfgToMATest(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node),
  m_medialAxisUtility(_node) {
    this->SetName(m_strategyName);

    //This was stolen from line ~124 of MPStrategies/BasicPRM.h:
    for(auto& child : _node) {
        if(child.Name() == "Sampler") {
          std::string s = child.Read("method", true, "", "Sampler Label");
          size_t num = child.Read("number", true,
              1, 0, MAX_INT, "Number of samples");
          size_t attempts = child.Read("attempts", false,
              1, 0, MAX_INT, "Number of attempts per sample");
          m_samplerLabels[s] = std::make_pair(num, attempts);
          m_samplerIndices.push_back(s);
        }
    }

    //The default for this is true, as we want uniform sampling by default
    this->m_attemptInvalidSamples = _node.Read("attemptInvalidSamples", false,
      true, "Set this as false if only initially free samples are to be"
          "attempted to push to the MA for this test strategy");

    //For the next two, false is default as we don't want to assume 2D.env is
    // always being used
    this->m_do2dEnvTests = _node.Read("do2dEnvTests", false, false,
      "If using 2D.env, set this as true "
      "to perform additional narrow passage data collection");

    this->m_forceNarrowPassageSamples2dEnv = _node.Read(
      "forceNarrowPassageSamples2dEnv", false, false,
      "If using 2D.env, set this as true "
      "to force all samples to be in narrow passage initially");

    this->m_doSameValidityWitness = _node.Read(
      "sameValidityWitnessTest", false, false,
      "Performs a test to instead get the desired number of witness points");
  }

template <typename MPTraits>
void
PushCfgToMATest<MPTraits>::
Print(std::ostream& _os) const {
  m_medialAxisUtility.Print(_os);
}

template <typename MPTraits>
void
PushCfgToMATest<MPTraits>::
Initialize() {
  //Make sure all of the variables are squeaky clean:
  //Don't want to clear out the ones that are only initialized upon making
  // the object.
  m_narrowPassageCount = 0;
  m_numSamplesFailed = 0;
  m_numInvalidPassedToPush = 0;
  m_numFailedSamplesThatWereInvalid = 0;

  //Make the maximum number of samples to attempt be 100x the total number.
  // Not the ideal way to go, but it should to the trick.
  m_maxSamplesToAttempt = (size_t) 100 *
      m_samplerLabels[m_samplerIndices.at(0)].first;

  //Should probably reside in the constructor/Initialize() in the end:
  m_medialAxisUtility.SetMPLibrary(this->GetMPLibrary());
  m_medialAxisUtility.Initialize();//Very important!

  if(this->m_debug)
    std::cout << "PushCfgToMATest::Initialize()" << std::endl;
}

template <typename MPTraits>
void
PushCfgToMATest<MPTraits>::
Iterate() {
  if(this->m_debug)
    std::cout << "PushCfgToMATest::Iterate()" << std::endl << std::endl;

  if(m_samplerIndices.size() > 1) {
    RunTimeException(WHERE, "For this test strategy, only one sampler is supported.");
  }

  //Note that m_samplerIndices holds the STRING key to get the sampler.
  auto sampler = this->GetSampler(m_samplerIndices.at(0));
  auto samplerPair = m_samplerLabels[m_samplerIndices.at(0)];
  size_t numSamples = samplerPair.first;
  int maxAttempts = samplerPair.second;

  auto env = this->GetEnvironment();
  auto boundary = env->GetBoundary();
  std::string callee = this->GetNameAndLabel() + "::PushCfgToMATest";
  auto vcm =
      this->GetValidityChecker(m_medialAxisUtility.GetValidityCheckerLabel());

  this->m_numInvalidPassedToPush = 0;
  size_t numAdded = 0;
  this->m_numSamplesFailed = 0;

  //I'm kind of abusing pmpl by having this loop in Iterate(), but it provides
  // a lot of ease of control when doing it this way.
  std::vector<CfgType> samples;//not currently used except by sampler
  while(numAdded < numSamples
        && (numAdded + m_numSamplesFailed) < m_maxSamplesToAttempt){
    CfgType cfg;
    CDInfo tmpInfo;
    tmpInfo.ResetVars(true);
    StatClass* stats = this->GetStatClass();

    //Get 1 sample
    sampler->Sample(1, maxAttempts, boundary, std::back_inserter(samples));
    if(samples.empty())
      continue;
    cfg = samples.at(samples.size()-1);

    if(this->m_forceNarrowPassageSamples2dEnv) {
      //So random config will be in range of ([-15,15], [-15,15]), so then I can
      // multiply by (1/15, 5/15) to get a range of ([-1,1], [-5,5]).
      std::vector<double> scaledData(cfg.GetData());
      scaledData[0] *= 1./15.;
      scaledData[1] *= 5./15.;
      cfg.SetData(scaledData);
    }

    bool initiallyValid = vcm->IsValid(cfg, callee);

    if(!m_attemptInvalidSamples) {
      while(!initiallyValid) {
        cfg.GetRandomCfg(boundary);
        initiallyValid = vcm->IsValid(cfg, callee);
      }
    }
    if(!initiallyValid) {
      this->m_numInvalidPassedToPush++;
    }

    //It's not pretty, but if doing the same-validity witness test, don't push
    // to the MA and instead just try and get a witness.
    if(m_doSameValidityWitness) {
      //Don't push to the MA, just get a witness of same validity and add to map
      CfgType witness = cfg;
      if(!m_medialAxisUtility.CollisionInfo(cfg, witness, boundary, tmpInfo, false)){
        //Update failure stats:
        this->m_numSamplesFailed++;
        if(!initiallyValid)
          this->m_numFailedSamplesThatWereInvalid++;

        continue; // Try new sample
      }

      //Successful witness found, add to roadmap and update stats.
      //First, check if we have the case that the validity is NOT the same
      // (like it should be) and the only time that's okay is for an OOB witness
      //NOTE: this is assuming that useBBX is true!
      const bool witnessValid = vcm->IsValid(witness, callee),
                 witnessInBounds = witness.InBounds(boundary);
      const bool witnessValidity = witnessValid && witnessInBounds;
      if(witnessValidity != initiallyValid) {
        std::cout << "Error! Witness was wrong validity!!!" << std::endl;
        throw RunTimeException(WHERE, "Error in same-validity witness finding "
          "test: A witness of opposite-validity was found and it wansn't OOB.");
      }

      //add valid sample to roadmap
      this->GetRoadmap()->AddVertex(witness);
    }
    else {
      //Finally, push our free cfg to the MA:
      stats->StartClock("Push to MA Clock");
      if(!m_medialAxisUtility.PushToMedialAxis(cfg, boundary)) {
        stats->StopClock("Push to MA Clock");
        if(this->m_debug)
          std::cout << "Skipped a sample that couldn't be pushed to MA. "
                    << std::endl;

        //Pushing failed, so update the number of samples in total that failed:
        this->m_numSamplesFailed++;

        if(!initiallyValid) {
          //Useful to see if there is a disproportionate number of failing samples
          // that are initially invalid
          this->m_numFailedSamplesThatWereInvalid++;
        }

        continue;//Try a new sample
      }
      stats->StopClock("Push to MA Clock");
      //add valid sample to roadmap
      this->GetRoadmap()->AddVertex(cfg);
    }

    ++numAdded;
    if(this->m_debug)
      std::cout << numAdded << "/" << numSamples << std::endl;

    if(m_do2dEnvTests && !m_doSameValidityWitness) {
      //count if it's in the narrow passage (NOTE: assuming 2D.env is used!):
      //note it should be anything within x=[-1,1] and y=[-5,5] for the exact narrow passage
      const std::vector<double> dofData = cfg.GetData();
      if(dofData[0] >= -1. && dofData[0] <= 1. &&
         dofData[1] >= -5. && dofData[1] <= 5.)
      {
        this->m_narrowPassageCount++;
      }
    }
  }//end for (all samples)
}

template <typename MPTraits>
void
PushCfgToMATest<MPTraits>::
Finalize() {
  if(this->m_debug)
    std::cout << "PushCfgToMATest::Finalize()" << std::endl;

  // Output final map.
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map",
      this->GetEnvironment());

  StatClass* stats = this->GetStatClass();
  if(m_do2dEnvTests) {
    stats->SetStat("Number of narrow passage samples", m_narrowPassageCount);
  }

  //print the number of failed cfgs attempted to be pushed:
  stats->SetStat("Samples failed to push to MA", m_numSamplesFailed);

  //print the number of invalid cfgs attempted to be pushed:
  stats->SetStat("Number of initially invalid cfgs attempted",
                      m_numInvalidPassedToPush);

  //print the number of invalid samples that failed:
  stats->SetStat("Initially invalid samples that failed MA push",
                      m_numFailedSamplesThatWereInvalid);

  //Now output the position resolution used, since it's not fully defined by
  // the input from the xml:
  const double posRes = m_medialAxisUtility.GetPositionResolution();
  stats->SetStat("Environment's position resolution used", posRes);

  // Output stats.
  std::ofstream osStat(this->GetBaseFilename() + ".stat");
  stats->PrintAllStats(osStat, this->GetRoadmap());
}

#endif
