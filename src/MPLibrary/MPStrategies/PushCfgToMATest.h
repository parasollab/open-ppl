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
    virtual void Print(ostream& _os) const;

  protected:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    const string STRAT_NAME = "PushCfgToMATest";

    //This will scale all samples so that in the 2D standard env, they initially
    // start in the narrow passage (used for some troubleshooting tests)
    bool m_forceNarrowPassageSamples2dEnv{false};

    //Can choose whether initially invalid samples are attempted to be pushed
    // to the MA (true) or not (false)
    bool m_attemptInvalidSamples{true};//false means throw away invalid

    //If using 2D.env, there are more tests (narrow passage count) that can be
    // activated using this flag.
    bool m_do2dEnvTests{false};

    //The string keys of the samplers (Note: only 1 is supported currently)
    vector<string> m_samplerIndices;

    // Sampler labels mapped to number and attempts of sampler:
    map<string, pair<size_t, size_t> > m_samplerLabels;

    //Data that I accumulate in Iterate() and output in Finalize()
    size_t m_narrowPassageCount{0};
    size_t m_numSamplesFailed{0};
    size_t m_numInvalidPassedToPush{0};
    size_t m_numFailedSamplesThatWereInvalid{0};
};

template <typename MPTraits>
PushCfgToMATest<MPTraits>::
PushCfgToMATest() {
  this->SetName(STRAT_NAME);
}

template <typename MPTraits>
PushCfgToMATest<MPTraits>::
PushCfgToMATest(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node),
  m_medialAxisUtility(_node) {
    this->SetName(STRAT_NAME);

    //This was stolen from line ~124 of MPStrategies/BasicPRM.h:
    for(auto& child : _node) {
        if(child.Name() == "Sampler") {
          string s = child.Read("method", true, "", "Sampler Label");
          size_t num = child.Read("number", true,
              1, 0, MAX_INT, "Number of samples");
          size_t attempts = child.Read("attempts", false,
              1, 0, MAX_INT, "Number of attempts per sample");
          m_samplerLabels[s] = make_pair(num, attempts);
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
  }

template <typename MPTraits>
void
PushCfgToMATest<MPTraits>::
Print(ostream& _os) const {
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
    RunTimeException(WHERE, "For this MA Test, only one sampler is supported"
        " You can remove this exception, but only one sampler chosen"
        " will be used.");
  }
  //Note that m_samplerIndices holds the STRING key to get the sampler.
  auto s = this->GetSampler(m_samplerIndices.at(0));
  auto samplerPair = m_samplerLabels[m_samplerIndices.at(0)];
  size_t numSamples = samplerPair.first;
  int maxAttempts = samplerPair.second;

  auto env = this->GetEnvironment();
  auto boundary = env->GetBoundary();
  string callee = this->GetNameAndLabel() + "::PushToMedialAxis";
  auto vcm =
      this->GetValidityChecker(m_medialAxisUtility.GetValidityCheckerLabel());

  this->m_numInvalidPassedToPush = 0;
  size_t numAdded = 0;
  this->m_numSamplesFailed = 0;

  //I'm kind of abusing pmpl by having this loop in Iterate(), but it provides
  // a lot of ease of control when doing it this way.
  vector<CfgType> samples;//not currently used except by sampler
  while(numAdded < numSamples) {
    CfgType cfg;
    CDInfo tmpInfo;
    tmpInfo.ResetVars();
    tmpInfo.m_retAllInfo = true;
    StatClass* stats = this->GetStatClass();

    //Get 1 sample
    s->Sample(1, maxAttempts, boundary, back_inserter(samples));
    cfg = samples.at(samples.size()-1);//get it from the

    if(this->m_forceNarrowPassageSamples2dEnv) {
      //So random config will be in range of ([-15,15], [-15,15]), so then I can
      // multiply by (1/15, 5/15) to get a range of ([-1,1], [-5,5]).
      std::vector<double> scaledData(cfg.GetData());
      scaledData[0] *= 1./15.;
      scaledData[1] *= 5./15.;
      cfg.SetData(scaledData);
    }

    bool initiallyValid = vcm->IsValid(cfg, tmpInfo, callee);

    if(!m_attemptInvalidSamples) {
      while(!initiallyValid) {
        cfg.GetRandomCfg(env, boundary);
        initiallyValid = vcm->IsValid(cfg, tmpInfo, callee);
      }
    }
    if(!initiallyValid) {
      this->m_numInvalidPassedToPush++;
    }

    //Finally, push our free cfg to the MA:
    stats->StartClock("Push to MA Clock");
    if(!m_medialAxisUtility.PushToMedialAxis(cfg, boundary)) {
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

      stats->StopClock("Push to MA Clock");
      continue;//Try a new sample
    }
    stats->StopClock("Push to MA Clock");

    //add valid MA sample to roadmap
    this->GetRoadmap()->GetGraph()->AddVertex(cfg);

    if(this->m_debug)
      std::cout << ++numAdded << "/" << numSamples << std::endl;

    if(m_do2dEnvTests) {
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
  stats->SetStat("Position resolution used", posRes);

  // Output stats.
  ofstream osStat(this->GetBaseFilename() + ".stat");
  stats->PrintAllStats(osStat, this->GetRoadmap());
}

#endif
