#ifndef SIMILARSTRUCTURESAMPLER_H_
#define SIMILARSTRUCTURESAMPLER_H_

//
// Code adapted from Dr. Lydia Tapia's group at UNM
// All results using this sampler should cite their work.
//

#include "SamplerMethod.h"
#include "boost/dynamic_bitset.hpp"

class Environment;
class StatClass;
class CDInfo;
template <typename CFG> class RMSDDistance;


template <class CFG>
struct ActivityType {
  virtual boost::dynamic_bitset<> Active() = 0;
};

template <class CFG>
struct ActivityAll : public ActivityType<CFG> {
  const size_t m_jointCount;

  ActivityAll(const CFG& _cfg) : m_jointCount(_cfg.DOF()) {}

  virtual boost::dynamic_bitset<> Active() { return boost::dynamic_bitset<>(m_jointCount).set(); }
};

template <class CFG>
struct ActivityNumber : public ActivityType<CFG> {
  const size_t m_jointCount;
  const size_t m_number;

  ActivityNumber(const CFG& _cfg, size_t _number) : m_jointCount(_cfg.DOF()), m_number(min(_number, m_jointCount)) {}

  virtual boost::dynamic_bitset<> Active() {
    boost::dynamic_bitset<> bitset(m_jointCount);
    while(bitset.count() < m_number)
      bitset.set(LRand() % m_jointCount);
    return bitset;
  }
};

template <class CFG>
struct ActivityFraction : public ActivityNumber<CFG> {
  ActivityFraction(const CFG& _cfg, double _fraction) : ActivityNumber<CFG>(_cfg, _fraction * _cfg.DOF()) {}
};


struct DistributionType {
  virtual double Random() = 0;
  virtual ostream& Print(ostream& _os) const = 0;
};

struct DistributionConstant : public DistributionType {
  const double m_value;

  DistributionConstant(double _value) : m_value(_value) {}

  virtual double Random() { return m_value; }

  friend ostream& operator<<(ostream& _os, const DistributionConstant& _d);
  virtual ostream& Print(ostream& _os) const {
    _os << *this;
    return _os;
  }
};

struct DistributionUniform : public DistributionType {
  const double m_low, m_high;

  DistributionUniform(double _low, double _high) : m_low(min(_low,_high)), m_high(max(_low,_high)) {}

  virtual double Random() { return m_low + DRand() * m_high; }
  friend ostream& operator<<(ostream& _os, const DistributionUniform& _d);
  virtual ostream& Print(ostream& _os) const {
    _os << *this;
    return _os;
  }
};

struct DistributionGaussian : public DistributionType {
  const double m_mean, m_std;

  DistributionGaussian(double _mean, double _std) : m_mean(_mean), m_std(_std) {}

  virtual double Random() { return GaussianDistribution(m_mean, m_std); }
  friend ostream& operator<<(ostream& _os, const DistributionGaussian& _d);
  virtual ostream& Print(ostream& _os) const {
    _os << *this;
    return _os;
  }
};


template <typename MPTraits>
class SimilarStructureSampler : public SamplerMethod<MPTraits>
{
 protected:
   typedef typename MPTraits::CfgType CfgType;

  string m_vcLabel;

  shared_ptr<DistributionType> m_targetRangeDistribution;
  shared_ptr<DistributionType> m_subtargetDriftDistribution;

  shared_ptr<ActivityType<CfgType> > m_jointActivity;
  string m_jointActivityString;
  double m_jointActivityFractionActive;
  size_t m_jointActivityNumberActive;

  size_t m_samplesPerSeed;
  bool m_saveInvalid;
  bool m_doAlignment;
  bool m_ignoreValidity;

 public:
  SimilarStructureSampler(string _vcLabel = "") :
    SamplerMethod<MPTraits>(), m_vcLabel(_vcLabel),
    m_jointActivityFractionActive(0.0),
    m_jointActivityNumberActive(0),
    m_samplesPerSeed(1),
    m_saveInvalid(false),
    m_doAlignment(true),
    m_ignoreValidity(false) {
    this->SetName("SimilarStructureSampler");
  }

  SimilarStructureSampler(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) :
    SamplerMethod<MPTraits>(_problem, _node),
    m_jointActivityFractionActive(0.0),
    m_jointActivityNumberActive(0) {
    this->SetName("SimilarStructureSampler");
    ParseXML(_node);
  }

  virtual ~SimilarStructureSampler() {
  }

  void ParseXML(XMLNodeReader& _node) {
    m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");

    m_samplesPerSeed = floor(_node.numberXMLParameter("SamplesPerSeed", true, 1, 0, 32767, "Samples per Seed"));
    m_saveInvalid = _node.boolXMLParameter("SaveInvalid", false, false, "Should invalid nodes be added to the collisions output iterator?");
    m_doAlignment = _node.boolXMLParameter("DoAlignment", false, true, "Perform alignment when calculating half-rotation effects?");
    m_ignoreValidity = _node.boolXMLParameter("IgnoreValidity", false, false, "Perform alignment when calculating half-rotation effects?");

    m_jointActivityString = "";
    m_targetRangeDistribution = shared_ptr<DistributionType>();
    m_subtargetDriftDistribution = shared_ptr<DistributionType>();

    for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
      if(!ParseJointActivity(citr)) {

        //read target range string
        if(citr->getName() == "TargetRangeDistribution") {
          string type = citr->stringXMLParameter("type", true, "", "type of target range distribution: constant, uniform, gaussian");
          if(type == "constant") {
            const double value = citr->numberXMLParameter("value", true, 0.0, -MAX_DBL, MAX_DBL, "Value");
            m_targetRangeDistribution = shared_ptr<DistributionType>(new DistributionConstant(value));
          } else  if(type == "uniform") {
            const double low = citr->numberXMLParameter("min", true, 0.0, -MAX_DBL, MAX_DBL, "Min");
            const double high = citr->numberXMLParameter("max", true, 0.0, -MAX_DBL, MAX_DBL, "Max");
            m_targetRangeDistribution = shared_ptr<DistributionType>(new DistributionUniform(low,high));
          } else if(type == "gaussian") {
            const double mean = citr->numberXMLParameter("mean", true, 0.0, -MAX_DBL, MAX_DBL, "Mean");
            const double std = citr->numberXMLParameter("std", true, 0.0, -MAX_DBL, MAX_DBL, "Std");
            m_targetRangeDistribution = shared_ptr<DistributionType>(new DistributionGaussian(mean,std));
          } else {
            citr->warnUnknownNode();
            exit(-1);
          }

        } else if(citr->getName() == "SubtargetDriftDistribution") {
          //read subtarget drift string
          string type = citr->stringXMLParameter("type", true, "", "type of target range distribution: constant, uniform, gaussian");
          if(type == "constant") {
            const double value = citr->numberXMLParameter("value", true, 0.0, -MAX_DBL, MAX_DBL, "Value");
            m_subtargetDriftDistribution = shared_ptr<DistributionType>(new DistributionConstant(value));
          } else if(type == "uniform") {
            const double low = citr->numberXMLParameter("min", true, 0.0, -MAX_DBL, MAX_DBL, "Min");
            const double high = citr->numberXMLParameter("max", true, 0.0, -MAX_DBL, MAX_DBL, "Max");
            m_subtargetDriftDistribution = shared_ptr<DistributionType>(new DistributionUniform(low,high));
          } else if(type == "gaussian") {
            const double mean = citr->numberXMLParameter("mean", true, 0.0, -MAX_DBL, MAX_DBL, "Mean");
            const double std = citr->numberXMLParameter("std", true, 0.0, -MAX_DBL, MAX_DBL, "Std");
            m_subtargetDriftDistribution = shared_ptr<DistributionType>(new DistributionGaussian(mean,std));
          } else {
            citr->warnUnknownNode();
            exit(-1);
          }
        }
      }
    }

    if(m_jointActivityString == "") {
      cout << "\n\nError, you must specify a joint activity string: all, number, fraction\n\n";
      exit(-1);
    }
    if(m_targetRangeDistribution == shared_ptr<DistributionType>()) {
      cout << "\n\nError, you must specify a target range distribution type: constant, uniform, gaussian\n\n";
      exit(-1);
    }
    if(m_subtargetDriftDistribution == shared_ptr<DistributionType>()) {
      cout << "\n\nError, you must specify a subtarget drift distribution type: constant, uniform, gaussian\n\n";
      exit(-1);
    }
  }

  virtual void PrintOptions(ostream& _out) const {
    SamplerMethod<MPTraits>::PrintOptions(_out);
    _out << "\tvcLabel = " << m_vcLabel << endl;

    _out << "\ttargetRangeDistribution = "; m_targetRangeDistribution->Print(_out); _out << endl;
    _out << "\tsubtargetDriftDistribution = "; m_subtargetDriftDistribution->Print(_out); _out << endl;

    _out << "\tjointActivity = " << m_jointActivityString;
    if(m_jointActivityString == "fraction")
      _out << "(" << m_jointActivityFractionActive << ")";
    if(m_jointActivityString == "number")
      _out << "(" << m_jointActivityNumberActive << ")";
    _out << endl;

    _out << "\tsamplesPerSeed = " << m_samplesPerSeed << endl;
    _out << "\tsaveInvalid = " << m_saveInvalid << endl;
    _out << "\tdoAlignment = " << m_doAlignment << endl;
    _out << "\tignoreValidity = " << m_ignoreValidity << endl;
  }

 protected:

  virtual bool ParseJointActivity(XMLNodeReader::childiterator _citr) {
    if(_citr->getName() == "JointActivity") {
      m_jointActivityString = _citr->stringXMLParameter("type", false, "all", "which joints are active: all, number, fraction");

      if(m_jointActivityString == "all")
        return true;
      if(m_jointActivityString == "number") {
        m_jointActivityNumberActive = floor(_citr->numberXMLParameter("number", true, 0, 1, 65536, "Number of Joints Active"));
        return true;
      }
      if(m_jointActivityString == "fraction") {
        m_jointActivityFractionActive = _citr->numberXMLParameter("fraction", true, 1.0, 0.0, 1.0, "Fraction of Joints Active");
        return true;
      }

      _citr->warnUnknownNode();
      exit(-1);
    }

    return false;
  }


  // For a single joint, given a target rmsd change
  // and the maximum rmsd change possible on that joint,
  // (which is always associated with a half-revolution),
  // produces an angle change (in [0.0, 1.0] format) that will
  // for that joint that will approximately cause the desired target
  // rmsd change. This approximation does not take alignment into
  // account. It is based on the geometry of chords on the circular
  // path taken by atoms undergoing rotation about an axis.
  double TargetRotation(double _targetRMSD, double _fullRMSD) const {
    if(_fullRMSD <= 0.0)
      return 0.0;
    _targetRMSD = min(max(0.0, _targetRMSD), _fullRMSD);
    return asin(_targetRMSD / _fullRMSD) / PI;
  }

  // Store the effect (in terms of unaligned RMSD change) for each
  // joint of each seed node of rotating that joint by 180 degrees.
  vector<double> HalfRotationEffects(const CfgType& _cfg, Environment* _env) {
    RMSDDistance<MPTraits> rmsd;

    const vector<Vector3d> original_coords = rmsd.GetCoordinatesForRMSD(_cfg);
    const vector<double> original_joints = _cfg.GetData();

    vector<double> altered_joints = original_joints;

    vector<double> effects(_cfg.DOF());
    for(size_t j = _cfg.PosDOF(); j < _cfg.DOF(); ++j) {
      altered_joints[j] += 0.5;
      altered_joints[j] -= floor(altered_joints[j]);

      CfgType altered_cfg;
      altered_cfg.SetData(altered_joints);
      const vector<Vector3d> altered_coords = rmsd.GetCoordinatesForRMSD(altered_cfg);

      if(m_doAlignment) {
        effects[j] = rmsd.RMSD(original_coords, altered_coords, original_coords.size());
      } else {
        double accumulator = 0.0;
        for(size_t c = 0; c < original_coords.size(); ++c) {
          accumulator += (original_coords[c] - altered_coords[c]).normsqr();
        }
        effects[j] = sqrt(accumulator/double(original_coords.size()));
      }

      altered_joints[j] = original_joints[j];
    }
    return effects;
  }

  // Generate a single (possibly invalid) structurally similar cfg.
  CfgType SampleSimilar(const CfgType &_cfg, const vector<double> &_effects, std::fstream &_log) {
    vector<double> similar_joints = _cfg.GetData();
    if(this->m_debug)
      _log << similar_joints.size() << "\t";

    const boost::dynamic_bitset<> active_joints = m_jointActivity->Active();
    if(this->m_debug)
      _log << active_joints.count() << "\t";

    const double RMSD_TARGET = abs(m_targetRangeDistribution->Random());
    if(this->m_debug)
      _log << RMSD_TARGET << "\t";

    const double SCALED_TARGET = RMSD_TARGET / sqrt(double(active_joints.count()));
    if(this->m_debug)
      _log << SCALED_TARGET << "\t";

    for(size_t j = _cfg.PosDOF(); j < similar_joints.size(); ++j) {
      if(active_joints[j]) {
        const double DOF_TARGET = abs(SCALED_TARGET + m_subtargetDriftDistribution->Random());
        const double sign = (DRand() < 0.5) ? -1.0 : 1.0;
        const double change = TargetRotation(DOF_TARGET, _effects[j]);

        similar_joints[j] += (sign * change);
        similar_joints[j] -= floor(similar_joints[j]);
      }
    }
    CfgType return_cfg;
    return_cfg.SetData(similar_joints);
    return return_cfg;
  }

  virtual void InitializeJointActivity(const CfgType& _cfg, Environment* _env) {
    if(m_jointActivityString == "all") {
      m_jointActivity = shared_ptr<ActivityType<CfgType> >(new ActivityAll<CfgType>(_cfg));
    } else if(m_jointActivityString == "number") {
      m_jointActivity = shared_ptr<ActivityType<CfgType> >(new ActivityNumber<CfgType>(_cfg, m_jointActivityNumberActive));
    } else if(m_jointActivityString == "fraction") {
      m_jointActivity = shared_ptr<ActivityType<CfgType> >(new ActivityFraction<CfgType>(_cfg, m_jointActivityFractionActive));
    } else {
      cerr << "ERROR: Invalid ActivityType!" << endl;
      exit(-1);
    }
  }


 public:
  virtual bool Sampler(Environment* env, shared_ptr<Boundary> _bb,
        StatClass& Stat, CfgType& _cfgIn, vector<CfgType>& _cfgOut,
        vector<CfgType>& _cfgCol) {
    const size_t cfgOutSize = _cfgOut.size();

    typename MPTraits::MPProblemType::ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);

    const vector<CfgType> seed_nodes(1, _cfgIn);
    for(size_t seed_index = 0; seed_index < seed_nodes.size(); ++seed_index) {
      const CfgType original_cfg = seed_nodes[seed_index];
      if(this->m_debug)
        cout << "original_cfg:\t" << original_cfg << endl;

      InitializeJointActivity(original_cfg, env);

      const vector<double> effects = HalfRotationEffects(original_cfg, env);
      if(this->m_debug) {
        std::fstream log;
        std::stringstream log_name;
        log_name << "HalfRotationEffects." << seed_index << ".log";
        log.open(log_name.str().c_str(), std::fstream::out|std::fstream::trunc);
        log << "HalfRotationEffects" << endl;
        for(size_t x = 0; x < effects.size(); ++x)
          log << effects[x] << endl;
        log.close();
      }
      std::fstream log;
      std::stringstream log_name;
      log_name << "Similarity." << seed_index << ".log";
      if(this->m_debug) {
        log.open(log_name.str().c_str(), std::fstream::out | std::fstream::trunc);
        log << "JOINTS\tACTIVE\tTARGET\tSCALED\tRMSD\tVALID" << endl;
      }

      for(size_t sample = 0; sample < m_samplesPerSeed; ++sample) {
        if(this->m_debug)
          cout << "Seed " << seed_index << " Sample " << sample << "..." << endl;
        CfgType similar_cfg(SampleSimilar(original_cfg, effects, log));
        if(this->m_debug)
          cout << "\tsampled cfg:\t" << similar_cfg << endl;
        Stat.IncNodesAttempted(this->GetNameAndLabel());

        RMSDDistance<MPTraits> rmsd;
        if(this->m_debug)
          log << rmsd.Distance(original_cfg, similar_cfg) << "\t";
        string callee(this->GetNameAndLabel());callee+="::_BiasedSample()";
        if(m_ignoreValidity) {
           Stat.IncNodesGenerated(this->GetNameAndLabel());
           _cfgOut.push_back(similar_cfg);
           if(this->m_debug)
             log << "1" << endl;
        }
        else if(vcm->IsValid(similar_cfg, callee)) {
          Stat.IncNodesGenerated(this->GetNameAndLabel());
          _cfgOut.push_back(similar_cfg);
          if(this->m_debug)
            log << "1" << endl;
        }
        else {
          if(m_saveInvalid)
            _cfgCol.push_back(similar_cfg);
          if(this->m_debug)
            log << "0" << endl;
          continue;
        }
      }
      log.close();
    }
    return _cfgOut.size() > cfgOutSize;
  }

};

#endif
