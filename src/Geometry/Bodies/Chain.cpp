#include "Chain.h"

#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"

#include "Utilities/PMPLExceptions.h"

#include <algorithm>
#include <queue>


/*-------------------------------- Generation --------------------------------*/

std::vector<Chain>
Chain::
Decompose(MultiBody* _mb) {
  // Composite multijoints are not supported.
  /// @todo We can probably support composite joints here, but it is not
  ///       immediately required. Need to test this separately.
  throw RunTimeException(WHERE) << "In progress...do not use!";

  if(_mb->IsComposite())
    throw RunTimeException(WHERE) << "Composite joints are not yet supported.";

  std::vector<Chain> chains;

  // Start from the base and scan outward. Repeat until the entire MultiBody is
  // decomposed.
  std::queue<Body*> roots;
  roots.push(_mb->GetBase());

  while(!roots.empty()) {
    // Get the next root.
    Body* current = roots.front();
    roots.pop();

    // Find the longest linear chain from the current root.
    chains.emplace_back(Chain::FindLongestLinearChain(_mb, current, true));

    // Get the last body in the chain.
    Body* last = chains.back().GetEnd()->GetNextBody();

    // If last has any forward connections, they are now roots for new
    // subchains.
    for(size_t i = 0; i < last->ForwardConnectionCount(); ++i)
      roots.push(last->GetForwardConnection(i).GetNextBody());
  }

  return chains;
}



std::pair<Chain, Chain>
Chain::
Bisect() const noexcept {
  //bisects the chain into two chains that have to same last joint
  const size_t halfSize = ceil(m_joints.size() / 2.0);

  JointList joints1, joints2;

  auto midpoint = m_joints.begin() + halfSize;
  for (auto it = m_joints.begin(); it != midpoint; it++) {
    joints1.push_back(*it);
  }

  for (midpoint = m_joints.begin() + halfSize - 1; midpoint != m_joints.end(); midpoint++) {
    joints2.push_back(*midpoint);
  }

  //properly split the chain based on the traversal direction of the chain at hand
  if (m_forward)
    return {Chain(m_multibody, std::move(joints1), this->GetBase(), nullptr, m_forward),
      Chain(m_multibody, std::move(joints2), nullptr, this->GetLastBody(), m_forward)};
  else
    return {Chain(m_multibody, std::move(joints2), nullptr, nullptr, m_forward),
      Chain(m_multibody, std::move(joints1), this->GetBase(), nullptr, m_forward)};
}

/*-------------------------------- Modifiers ---------------------------------*/


Chain&
Chain::
Reverse() noexcept {
  std::reverse(m_joints.begin(), m_joints.end());
  std::swap(m_base, m_lastBody);
  m_forward = !m_forward;

  return *this;
}


Chain&
Chain::
Append(const Chain& _other) noexcept {
  // Assert that the two chains have the same traversal direction.
  if(IsForward() != _other.IsForward())
    throw RunTimeException(WHERE) << "Cannot append two chains with opposite "
                                  << "traversal orders."
                                  << "\n\tChain 1: " << IsForward()
                                  << "\n\tChain 2: " << _other.IsForward();

  // Assert that the end of this chain is connected to the root of _other.
  if(!IsValidEnd(_other.GetBase()))
    throw RunTimeException(WHERE) << "Cannot append disconnected chains."
                                  << "\n\tChain 1: " << *this
                                  << "\n\tChain 2: " << _other;

  // We're still here, so the chains are compatible for joining.
  // Copy the joints in _other into this.
  std::copy(_other.m_joints.begin(), _other.m_joints.end(),
      std::back_inserter(m_joints));

  return *this;
}

bool
Chain::
IsSingleLink() const noexcept {
  //check for all single link cases
  const Body* lastBody = GetLastBody();
  const Body* base = GetBase();
  int size = Size(); //number of connection types stored in m_joints
  return (size == 2 && !base && !lastBody) ||
    (size == 1 && base && !lastBody) ||
    (size == 1 && !base && lastBody);
}

Chain&
Chain::
PopRoot() noexcept {
  m_joints.pop_front();
  return *this;
}

Chain&
Chain::
PopEnd() noexcept {
  m_joints.pop_back();
  return *this;
}


Chain&
Chain::
PushRoot(const Body* const _newRoot) noexcept {
  if(!IsValidRoot(_newRoot))
    throw RunTimeException(WHERE) << "Tried to push an invalid root into the "
                                  << "chain."
                                  << "\n\tConnection: " << _newRoot->GetIndex()
                                  << "\n\tChain: " << *this;

  m_joints.push_front(&(_newRoot->GetForwardConnection(0)));
  return *this;
}


Chain&
Chain::
PushEnd(const Body* const _newEnd) noexcept {
  if(!IsValidEnd(_newEnd))
    throw RunTimeException(WHERE) << "Tried to push an invalid end into the "
                                  << "chain."
                                  << "\n\tConnection: " << _newEnd->GetIndex()
                                  << "\n\tChain: " << *this;

  m_joints.push_back(&(_newEnd->GetBackwardConnection(0)));
  return *this;
}

/*-------------------------------- Iteration ---------------------------------*/

Chain::iterator
Chain::
begin() const noexcept {
  return m_joints.begin();
}


Chain::iterator
Chain::end() const noexcept {
  return m_joints.end();
}

/*--------------------------------- Queries ----------------------------------*/

const Connection*
Chain::
GetRoot() const noexcept {
  return m_joints.front();
}


Connection*
Chain::
GetEnd() const noexcept {
    return m_joints.back();
}


bool
Chain::
IsForward() const noexcept {
  return m_forward;
}


size_t
Chain::
Size() const noexcept {
  return m_joints.size();
}

//Body*
//Chain::
//GetBase() noexcept {
//  return m_base;
//}

const Body*
Chain::
GetBase() const noexcept {
  return m_base;
}

//Body*
//Chain::
//GetLastBody() noexcept {
//  return m_lastBody;
//}

const Body*
Chain::
GetLastBody() const noexcept {
  return m_lastBody;
}

bool
Chain::
IsValidRoot(const Body* const _newRoot) const noexcept {
  const Body* const root = GetBase();

  if(IsForward()) {
    // Check that _newRoot is the previous Connection of one of root's backward
    // connections.
    for(size_t i = 0; i < root->BackwardConnectionCount(); ++i)
      if(root->GetBackwardConnection(i).GetPreviousBody() == _newRoot)
        return true;
  }
  else {
    // Check that _newRoot is the next Connection of one of root's forward connections.
    for(size_t i = 0; i < root->ForwardConnectionCount(); ++i)
      if(root->GetForwardConnection(i).GetNextBody() == _newRoot)
        return true;
  }

  return false;
}


bool
Chain::
IsValidEnd(const Body* const _newEnd) const noexcept {
  const Body* const end = GetLastBody();

  if(IsForward()) {
    // Check that _newEnd is the next Connection of one of end's forward connections.
    for(size_t i = 0; i < end->ForwardConnectionCount(); ++i)
      if(end->GetForwardConnection(i).GetNextBody() == _newEnd)
        return true;
  }
  else {
    // Check that _newEnd is the previous Connection of one of end's backward
    // connections.
    for(size_t i = 0; i < end->BackwardConnectionCount(); ++i)
      if(end->GetBackwardConnection(i).GetPreviousBody() == _newEnd)
        return true;
  }

  return false;
}

/*---------------------------------- Helpers ---------------------------------*/

Chain
Chain::
FindLongestLinearChain(MultiBody* const _mb, Body* _root,
    const bool _forward) noexcept {
  JointList joints;

  // Start from the root body and traverse until we find a link with
  // more than one forward connection.
  Body* current = _root;
  while(true) {

    // Push the current joint into the chain.
    _forward ? joints.push_back(&(current->GetForwardConnection(0))) 
       : joints.push_back(&(current->GetBackwardConnection(0)));

    // If there is not exactly one connection from current to the next link(s)
    // in the traversal direction, we are finished.
    size_t numConnections = _forward ? current->ForwardConnectionCount()
                                           : current->BackwardConnectionCount();
    if(numConnections != 1)
      break;

    // Set current to the next Connection in this traversal direction.
    current = _forward ? current->GetForwardConnection(0).GetNextBody()
                       : current->GetBackwardConnection(0).GetPreviousBody();
  }

  //constructs a chain that stores the current (end) body as m_lastBody
  return Chain(_mb, std::move(joints), current, _forward);
}

/*------------------------------- Construction -------------------------------*/

Chain::
Chain(MultiBody* _mb, JointList&& _joints, const Body* _lastBody,
      const bool _forward)
  : m_multibody(_mb), m_joints(std::move(_joints)), m_lastBody(_lastBody),
    m_forward(_forward) {
  // There is no purpose to an empty chain; please do not make one.
  if(m_joints.empty())
    throw RunTimeException(WHERE) << "Cowardly refusing to instantiate an "
                                  << "empty chain.";
}

Chain::
Chain(MultiBody*  _mb, JointList&& _joints, const bool _forward)
  : m_multibody(_mb), m_joints(std::move(_joints)), m_forward(_forward) {
  // There is no purpose to an empty chain; please do not make one.
  if(m_joints.empty())
    throw RunTimeException(WHERE) << "Cowardly refusing to instantiate an "
                                  << "empty chain.";
}

Chain::
Chain(MultiBody* _mb, JointList&& _joints, const Body* _base,
      const Body* _lastBody, const bool _forward)
  : m_multibody(_mb), m_joints(_joints), m_base(_base),
    m_lastBody(_lastBody), m_forward(_forward) {
  // There is no purpose to an empty chain; please do not make one.
  if(m_joints.empty())
    throw RunTimeException(WHERE) << "Cowardly refusing to instantiate an "
                                  << "empty chain.";
}

/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Chain& _c) {
  _os << "{";
  for(auto Connection : _c)
    _os << " " << Connection->GetNextBodyIndex();
  return _os << " }";
}

/*----------------------------------------------------------------------------*/
