#include "LPOutput.h"

#include <string>
#include <utility>
#include <vector>

/*----------------------------------------------------------------------------*/

LPOutput::
LPOutput() {
  Clear();
}


void
LPOutput::
Clear() {
  m_path.clear();
  m_intermediates.clear();
  m_edge.first.Clear();
  m_edge.second.Clear();
}


void
LPOutput::
SetLPLabel(const std::string& _label) {
  m_edge.first.SetLPLabel(_label);
  m_edge.second.SetLPLabel(_label);
}


void
LPOutput::
AddIntermediatesToWeights(const bool _saveIntermediates) {
  if(!_saveIntermediates)
    return;

  // Make a copy of the intermediates in reverse order for the backward edge.
  std::vector<Cfg> tmp;
  tmp.reserve(m_intermediates.size());
  std::copy(m_intermediates.rbegin(), m_intermediates.rend(),
            std::back_inserter(tmp));

  // Set both edges.
  m_edge.first.SetIntermediates(m_intermediates);
  m_edge.second.SetIntermediates(tmp);
}

/*----------------------------------------------------------------------------*/
