#include "SamplerMethod.h"

#include <iostream>

/*------------------------------ Construction --------------------------------*/

SamplerMethod::
SamplerMethod(XMLNode& _node) : MPBaseObject(_node) {
}


SamplerMethod::
~SamplerMethod() = default;

/*-------------------- Individual Configuration Sampling ---------------------*/

void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    OutputIterator _valid, OutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  Cfg cfg(this->GetTask()->GetRobot());
  std::vector<Cfg> valid, invalid;
  valid.reserve(_numNodes);

  // Try to generate _numNodes samples, using up to _maxAttempts tries per
  // sample.
  for(size_t i = 0; i < _numNodes; ++i) {
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      cfg.GetRandomCfg(_boundary);
      if(this->Sampler(cfg, _boundary, valid, invalid))
        break;
    }
  }

  stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
  stats->IncNodesAttempted(this->GetNameAndLabel(),
      valid.size() + invalid.size());

  std::copy(valid.begin(), valid.end(), _valid);
  std::copy(invalid.begin(), invalid.end(), _invalid);
}


void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
    OutputIterator _valid) {
  std::vector<Cfg> invalid;

  Sample(_numNodes, _maxAttempts, _boundary, _valid, std::back_inserter(invalid));
}


void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
    OutputIterator _valid, OutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  Cfg cfg(this->GetTask()->GetRobot());
  std::vector<Cfg> valid, invalid;
  valid.reserve(_numNodes);

  // Try to generate _numNodes samples, using up to _maxAttempts tries per
  // sample.
  for(size_t i = 0; i < _numNodes; ++i) {
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      cfg.GetRandomCfg(_robotBoundary);
      if(this->Sampler(cfg, _robotBoundary, _eeBoundary, valid, invalid))
        break;
    }
  }

  stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
  stats->IncNodesAttempted(this->GetNameAndLabel(),
      valid.size() + invalid.size());

  std::copy(valid.begin(), valid.end(), _valid);
  std::copy(invalid.begin(), invalid.end(), _invalid);
}


void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
    OutputIterator _valid) {
  std::vector<Cfg> invalid;

  Sample(_numNodes, _maxAttempts, _robotBoundary, _eeBoundary, _valid,
      std::back_inserter(invalid));
}


void
SamplerMethod::
Filter(InputIterator _first, InputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    OutputIterator _valid, OutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Filter");

  std::vector<Cfg> valid, invalid;

  // Try to filter each configuration in the input range, using up to
  // _maxAttempts tries per sample.
  while(_first != _last) {
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      if(this->Sampler(*_first, _boundary, valid, invalid))
        break;
    }
    _first++;
  }

  stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
  stats->IncNodesAttempted(this->GetNameAndLabel(),
      valid.size() + invalid.size());

  std::copy(valid.begin(), valid.end(), _valid);
  std::copy(invalid.begin(), invalid.end(), _invalid);
}


void
SamplerMethod::
Filter(InputIterator _first, InputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    OutputIterator _valid) {
  std::vector<Cfg> invalid;
  Filter(_first, _last, _maxAttempts, _boundary, _valid,
      std::back_inserter(invalid));
}

/*----------------------- Group Configuration Sampling -----------------------*/

void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    GroupOutputIterator _valid, GroupOutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  GroupCfgType cfg(this->GetGroupRoadmap());
  std::vector<GroupCfgType> valid, invalid;

  // Try to generate _numNodes samples, using up to _maxAttempts tries per
  // sample.
  for(size_t i = 0; i < _numNodes; ++i) {
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      // Generate a random configuration for each robot.
      for(size_t i = 0; i < cfg.GetNumRobots(); ++i)
        cfg.GetRobotCfg(i).GetRandomCfg(_boundary);

      if(this->Sampler(cfg, _boundary, valid, invalid))
        break;
    }
  }

  stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
  stats->IncNodesAttempted(this->GetNameAndLabel(),
      valid.size() + invalid.size());

  std::copy(valid.begin(), valid.end(), _valid);
  std::copy(invalid.begin(), invalid.end(), _invalid);
}


void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
    GroupOutputIterator _valid) {
  std::vector<GroupCfgType> invalid;
  Sample(_numNodes, _maxAttempts, _boundary, _valid,
      std::back_inserter(invalid));
}


void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts, const BoundaryMap& _boundaryMap,
    GroupOutputIterator _valid, GroupOutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  const Boundary* const envBoundary = this->GetEnvironment()->GetBoundary();

  GroupCfgType cfg(this->GetGroupRoadmap());
  std::vector<GroupCfgType> valid, invalid;

  // Try to generate _numNodes samples, using up to _maxAttempts tries per
  // sample.
  for(size_t i = 0; i < _numNodes; ++i) {
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      // Generate a random configuration for each robot.
      for(size_t i = 0; i < cfg.GetNumRobots(); ++i) {
        // Determine the boundary to use.
        auto robot = cfg.GetRobot(i);
        auto boundary = _boundaryMap.count(robot) ? _boundaryMap.at(robot)
                                                  : envBoundary;
        cfg.GetRobotCfg(i).GetRandomCfg(boundary);
      }

      if(this->Sampler(cfg, _boundaryMap, valid, invalid))
        break;
    }
  }

  stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
  stats->IncNodesAttempted(this->GetNameAndLabel(),
      valid.size() + invalid.size());

  std::copy(valid.begin(), valid.end(), _valid);
  std::copy(invalid.begin(), invalid.end(), _invalid);
}


void
SamplerMethod::
Sample(size_t _numNodes, size_t _maxAttempts, const BoundaryMap& _boundaryMap,
    GroupOutputIterator _valid) {
  std::vector<GroupCfgType> invalid;
  Sample(_numNodes, _maxAttempts, _boundaryMap, _valid,
      std::back_inserter(invalid));
}


void
SamplerMethod::
Filter(GroupInputIterator _first, GroupInputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    GroupOutputIterator _valid, GroupOutputIterator _invalid) {
  throw NotImplementedException(WHERE) << "No default implementation is "
                                       << "provided.";
}


void
SamplerMethod::
Filter(GroupInputIterator _first, GroupInputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    GroupOutputIterator _valid) {
  std::vector<GroupCfgType> invalid;
  Filter(_first, _last, _maxAttempts, _boundary, _valid,
      std::back_inserter(invalid));
}

/*------------------------------- Sampler Rule -------------------------------*/

bool
SamplerMethod::
Sampler(Cfg& _cfg, const Boundary* const _boundary,
    std::vector<Cfg>& _valid, std::vector<Cfg>& _invalid) {
  throw NotImplementedException(WHERE) << "No default implementation is "
                                       << "provided.";
}


bool
SamplerMethod::
Sampler(Cfg& _cfg, const Boundary* const _robotBoundary,
    const Boundary* const _eeBoundary,
    std::vector<Cfg>& _valid, std::vector<Cfg>& _invalid) {
  throw NotImplementedException(WHERE) << "No default implementation is "
                                       << "provided.";
}


bool
SamplerMethod::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {
  throw NotImplementedException(WHERE) << "No default implementation is "
                                       << "provided.";
}


bool
SamplerMethod::
Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
    std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {
  throw NotImplementedException(WHERE) << "No default implementation is "
                                       << "provided.";
}

/*----------------------------------------------------------------------------*/
